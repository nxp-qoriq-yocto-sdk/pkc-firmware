/*
 * Copyright 2004, 2007-2012 Freescale Semiconductor, Inc.
 * Copyright (C) 2003  Motorola,Inc.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/* U-Boot Startup Code for Motorola 85xx PowerPC based Embedded Boards
 *
 * The processor starts at 0xfffffffc and the code is first executed in the
 * last 4K page(0xfffff000-0xffffffff) in flash/rom.
 *
 */

#include <asm-offsets.h>
#include <config.h>
#include <mpc85xx.h>
#include <version.h>

#define _LINUX_CONFIG_H 1	/* avoid reading Linux autoconf.h file	*/

#include <ppc_asm.tmpl>
#include <ppc_defs.h>

#include <asm/cache.h>
#include <asm/mmu.h>

#undef	MSR_KERNEL
#define MSR_KERNEL ( MSR_ME )	/* Machine Check */
#define SAIF_DBG 0
/*
 * Set up GOT: Global Offset Table
 *
 * Use r12 to access the GOT
 */
	START_GOT
	GOT_ENTRY(_GOT2_TABLE_)
	GOT_ENTRY(_FIXUP_TABLE_)

	GOT_ENTRY(__init_end)
	GOT_ENTRY(__bss_end__)
	GOT_ENTRY(__bss_start)
	END_GOT

/*
 * e500 Startup -- after reset only the last 4KB of the effective
 * address space is mapped in the MMU L2 TLB1 Entry0. The .bootpg
 * section is located at THIS LAST page and basically does three
 * things: clear some registers, set up exception tables and
 * add more TLB entries for 'larger spaces'(e.g. the boot rom) to
 * continue the boot procedure.

 * Once the boot rom is mapped by TLB entries we can proceed
 * with normal startup.
 *
 */

	.section .bootpg,"ax"
	.globl _start_e500

_start_e500:
#if defined(CONFIG_E500_V1_V2)
/* Enable debug exception */
	li	r1,MSR_DE
	mtmsr 	r1
#endif

#ifdef CONFIG_SYS_FSL_ERRATUM_A004510
	mfspr	r3,SPRN_SVR
	rlwinm	r3,r3,0,0xff
	li	r4,CONFIG_SYS_FSL_ERRATUM_A004510_SVR_REV
	cmpw	r3,r4
	beq	1f

	/* Not a supported revision affected by erratum */
	li	r27,0
	b	2f

1:	li	r27,1	/* Remember for later that we have the erratum */
	/* Erratum says set bits 55:60 to 001001 */
	msync
	isync
	mfspr	r3,976
	li	r4,0x48
	rlwimi	r3,r4,0,0x1f8
	mtspr	976,r3
	isync
2:
#endif

/* clear registers/arrays not reset by hardware */

	/* L1 */
	li	r0,2
	mtspr	L1CSR0,r0	/* invalidate d-cache */
	mtspr	L1CSR1,r0	/* invalidate i-cache */

	mfspr	r1,DBSR
	mtspr	DBSR,r1		/* Clear all valid bits */

	/*
	 *	Enable L1 Caches early
	 *
	 */

#if defined(CONFIG_E500MC) && defined(CONFIG_SYS_CACHE_STASHING)
	/* set stash id to (coreID) * 2 + 32 + L1 CT (0) */
	li	r2,(32 + 0)
	mtspr	L1CSR2,r2
#endif

	/* Enable/invalidate the I-Cache */
	lis	r2,(L1CSR1_ICFI|L1CSR1_ICLFR)@h
	ori	r2,r2,(L1CSR1_ICFI|L1CSR1_ICLFR)@l
	mtspr	SPRN_L1CSR1,r2
1:
	mfspr	r3,SPRN_L1CSR1
	and.	r1,r3,r2
	bne	1b

	lis	r3,(L1CSR1_CPE|L1CSR1_ICE)@h
	ori	r3,r3,(L1CSR1_CPE|L1CSR1_ICE)@l
	mtspr	SPRN_L1CSR1,r3
	isync
2:
	mfspr	r3,SPRN_L1CSR1
	andi.	r1,r3,L1CSR1_ICE@l
	beq	2b

	/* Enable/invalidate the D-Cache */
	lis	r2,(L1CSR0_DCFI|L1CSR0_DCLFR)@h
	ori	r2,r2,(L1CSR0_DCFI|L1CSR0_DCLFR)@l
	mtspr	SPRN_L1CSR0,r2
1:
	mfspr	r3,SPRN_L1CSR0
	and.	r1,r3,r2
	bne	1b

	lis	r3,(L1CSR0_CPE|L1CSR0_DCE)@h
	ori	r3,r3,(L1CSR0_CPE|L1CSR0_DCE)@l
	mtspr	SPRN_L1CSR0,r3
	isync
2:
	mfspr	r3,SPRN_L1CSR0
	andi.	r1,r3,L1CSR0_DCE@l
	beq	2b

#if defined(CONFIG_E500_V1_V2) && !defined(CONFIG_NAND_SPL)
/*
 * TLB entry for debuggging in AS1
 * Create temporary TLB entry in AS0 to handle debug exception
 * As on debug exception MSR is cleared i.e. Address space is changed
 * to 0. A TLB entry (in AS0) is required to handle debug exception generated
 * in AS1.
 */

	lis     r6,FSL_BOOKE_MAS0(1,
			CONFIG_DEBUGGER_TEMP_TLB, 0)@h
	ori     r6,r6,FSL_BOOKE_MAS0(1,
			CONFIG_DEBUGGER_TEMP_TLB, 0)@l

#if !defined(CONFIG_SYS_RAMBOOT) && !defined(CONFIG_SECURE_BOOT)
/*
 * TLB entry is created for IVPR + IVOR15 to map on valid OP code address
 * bacause flash's virtual address maps to 0xff800000 - 0xffffffff.
 * and this window is outside of 4K boot window.
 */
	lis     r7,FSL_BOOKE_MAS1(1, 1, 0, 0, BOOKE_PAGESZ_4M)@h
	ori     r7,r7,FSL_BOOKE_MAS1(1, 1, 0, 0, BOOKE_PAGESZ_4M)@l

	lis     r8,FSL_BOOKE_MAS2(CONFIG_SYS_MONITOR_BASE & 0xffc00000,
							(MAS2_I|MAS2_G))@h
	ori     r8,r8,FSL_BOOKE_MAS2(CONFIG_SYS_MONITOR_BASE & 0xffc00000,
							(MAS2_I|MAS2_G))@l

	/* The 85xx has the default boot window 0xff800000 - 0xffffffff */
	lis     r9,FSL_BOOKE_MAS3(0xffc00000, 0, (MAS3_SX|MAS3_SW|MAS3_SR))@h
	ori     r9,r9,FSL_BOOKE_MAS3(0xffc00000, 0, (MAS3_SX|MAS3_SW|MAS3_SR))@l
#elif !defined(CONFIG_SYS_RAMBOOT) && defined(CONFIG_SECURE_BOOT)
	lis     r7,FSL_BOOKE_MAS1(1, 1, 0, 0, BOOKE_PAGESZ_1M)@h
	ori     r7,r7,FSL_BOOKE_MAS1(1, 1, 0, 0, BOOKE_PAGESZ_1M)@l

	lis     r8,FSL_BOOKE_MAS2(CONFIG_SYS_MONITOR_BASE,(MAS2_I|MAS2_G))@h
	ori     r8,r8,FSL_BOOKE_MAS2(CONFIG_SYS_MONITOR_BASE,(MAS2_I|MAS2_G))@l

	lis     r9,FSL_BOOKE_MAS3(CONFIG_SYS_PBI_FLASH_WINDOW, 0,
						(MAS3_SX|MAS3_SW|MAS3_SR))@h
	ori     r9,r9,FSL_BOOKE_MAS3(CONFIG_SYS_PBI_FLASH_WINDOW, 0,
						(MAS3_SX|MAS3_SW|MAS3_SR))@l
#else
/*
 * TLB entry is created for IVPR + IVOR15 to map on valid OP code address
 * because "nexti" will resize TLB to 4K
 */
	lis     r7,FSL_BOOKE_MAS1(1, 1, 0, 0, BOOKE_PAGESZ_256K)@h
	ori     r7,r7,FSL_BOOKE_MAS1(1, 1, 0, 0, BOOKE_PAGESZ_256K)@l

	lis     r8,FSL_BOOKE_MAS2(CONFIG_SYS_MONITOR_BASE, (MAS2_I))@h
	ori     r8,r8,FSL_BOOKE_MAS2(CONFIG_SYS_MONITOR_BASE,
							(MAS2_I))@l
	lis     r9,FSL_BOOKE_MAS3(CONFIG_SYS_MONITOR_BASE, 0,
						(MAS3_SX|MAS3_SW|MAS3_SR))@h
	ori     r9,r9,FSL_BOOKE_MAS3(CONFIG_SYS_MONITOR_BASE, 0,
						(MAS3_SX|MAS3_SW|MAS3_SR))@l
#endif
	mtspr   MAS0,r6
	mtspr   MAS1,r7
	mtspr   MAS2,r8
	mtspr   MAS3,r9
	tlbwe
	isync
#endif

/*
 * Ne need to setup interrupt vector for NAND SPL
 * because NAND SPL never compiles it.
 */
#if SAIF_DBG
#if !defined(CONFIG_NAND_SPL)
	/* Setup interrupt vectors */
	lis	r1,CONFIG_SYS_MONITOR_BASE@h
	mtspr	IVPR,r1

	lis	r3,(CONFIG_SYS_MONITOR_BASE & 0xffff)@h
	ori	r3,r3,(CONFIG_SYS_MONITOR_BASE & 0xffff)@l

	addi	r4,r3,CriticalInput - _start + _START_OFFSET
	mtspr	IVOR0,r4	/* 0: Critical input */
	addi	r4,r3,MachineCheck - _start + _START_OFFSET
	mtspr	IVOR1,r4	/* 1: Machine check */
	addi	r4,r3,DataStorage - _start + _START_OFFSET
	mtspr	IVOR2,r4	/* 2: Data storage */
	addi	r4,r3,InstStorage - _start + _START_OFFSET
	mtspr	IVOR3,r4	/* 3: Instruction storage */
	addi	r4,r3,ExtInterrupt - _start + _START_OFFSET
	mtspr	IVOR4,r4	/* 4: External interrupt */
	addi	r4,r3,Alignment - _start + _START_OFFSET
	mtspr	IVOR5,r4	/* 5: Alignment */
	addi	r4,r3,ProgramCheck - _start + _START_OFFSET
	mtspr	IVOR6,r4	/* 6: Program check */
	addi	r4,r3,FPUnavailable - _start + _START_OFFSET
	mtspr	IVOR7,r4	/* 7: floating point unavailable */
	addi	r4,r3,SystemCall - _start + _START_OFFSET
	mtspr	IVOR8,r4	/* 8: System call */
	/* 9: Auxiliary processor unavailable(unsupported) */
	addi	r4,r3,Decrementer - _start + _START_OFFSET
	mtspr	IVOR10,r4	/* 10: Decrementer */
	addi	r4,r3,IntervalTimer - _start + _START_OFFSET
	mtspr	IVOR11,r4	/* 11: Interval timer */
	addi	r4,r3,WatchdogTimer - _start + _START_OFFSET
	mtspr	IVOR12,r4	/* 12: Watchdog timer */
	addi	r4,r3,DataTLBError - _start + _START_OFFSET
	mtspr	IVOR13,r4	/* 13: Data TLB error */
	addi	r4,r3,InstructionTLBError - _start + _START_OFFSET
	mtspr	IVOR14,r4	/* 14: Instruction TLB error */
	addi	r4,r3,DebugBreakpoint - _start + _START_OFFSET
	mtspr	IVOR15,r4	/* 15: Debug */
#endif
#endif
	/* Clear and set up some registers. */
	li      r0,0x0000
	lis	r1,0xffff
	mtspr	DEC,r0			/* prevent dec exceptions */
	mttbl	r0			/* prevent fit & wdt exceptions */
	mttbu	r0
	mtspr	TSR,r1			/* clear all timer exception status */
	mtspr	TCR,r0			/* disable all */
	mtspr	ESR,r0			/* clear exception syndrome register */
	mtspr	MCSR,r0			/* machine check syndrome register */
	mtxer	r0			/* clear integer exception register */

#ifdef CONFIG_SYS_BOOK3E_HV
	mtspr	MAS8,r0			/* make sure MAS8 is clear */
#endif

	/* Enable Time Base and Select Time Base Clock */
	lis	r0,HID0_EMCP@h		/* Enable machine check */
#if defined(CONFIG_ENABLE_36BIT_PHYS)
	ori	r0,r0,HID0_ENMAS7@l	/* Enable MAS7 */
#endif
#ifndef CONFIG_E500MC
	ori	r0,r0,HID0_TBEN@l	/* Enable Timebase */
#endif
	mtspr	HID0,r0

#ifndef CONFIG_E500MC
	li	r0,(HID1_ASTME|HID1_ABE)@l	/* Addr streaming & broadcast */
	mfspr	r3,PVR
	andi.	r3,r3, 0xff
	cmpwi	r3,0x50@l	/* if we are rev 5.0 or greater set MBDD */
	blt 1f
	/* Set MBDD bit also */
	ori r0, r0, HID1_MBDD@l
1:
	mtspr	HID1,r0
#endif

#ifdef CONFIG_SYS_FSL_ERRATUM_CPU_A003999
	mfspr	r3,977
	oris	r3,r3,0x0100
	mtspr	977,r3
#endif

	/* Enable Branch Prediction */
#if defined(CONFIG_BTB)
	lis	r0,BUCSR_ENABLE@h
	ori	r0,r0,BUCSR_ENABLE@l
	mtspr	SPRN_BUCSR,r0
#endif


#if 0
/*
 * Search for the TLB that covers the code we're executing, and shrink it
 * so that it covers only this 4K page.  That will ensure that any other
 * TLB we create won't interfere with it.  We assume that the TLB exists,
 * which is why we don't check the Valid bit of MAS1.  We also assume
 * it is in TLB1.
 *
 * This is necessary, for example, when booting from the on-chip ROM,
 * which (oddly) creates a single 4GB TLB that covers CCSR and DDR.
 */
	bl	nexti		/* Find our address */
nexti:	mflr	r1		/* R1 = our PC */
	li	r2, 0
	mtspr	MAS6, r2	/* Assume the current PID and AS are 0 */
	isync
	msync
	tlbsx	0, r1		/* This must succeed */

	mfspr	r14, MAS0	/* Save ESEL for later */
	rlwinm	r14, r14, 16, 0xfff

	/* Set the size of the TLB to 4KB */
	mfspr	r3, MAS1
	li	r2, 0xF00
	andc	r3, r3, r2	/* Clear the TSIZE bits */
	ori	r3, r3, MAS1_TSIZE(BOOKE_PAGESZ_4K)@l
	oris	r3, r3, MAS1_IPROT@h
	mtspr	MAS1, r3

	/*
	 * Set the base address of the TLB to our PC.  We assume that
	 * virtual == physical.  We also assume that MAS2_EPN == MAS3_RPN.
	 */
	lis	r3, MAS2_EPN@h
	ori	r3, r3, MAS2_EPN@l	/* R3 = MAS2_EPN */

	and	r1, r1, r3	/* Our PC, rounded down to the nearest page */

	mfspr	r2, MAS2
	andc	r2, r2, r3
	or	r2, r2, r1
#ifdef CONFIG_SYS_FSL_ERRATUM_A004510
	cmpwi	r27,0
	beq	1f
	andi.	r15, r2, MAS2_I|MAS2_G /* save the old I/G for later */
	rlwinm	r2, r2, 0, ~MAS2_I
	ori	r2, r2, MAS2_G
1:
#endif
	mtspr	MAS2, r2	/* Set the EPN to our PC base address */

	mfspr	r2, MAS3
	andc	r2, r2, r3
	or	r2, r2, r1
	mtspr	MAS3, r2	/* Set the RPN to our PC base address */

	isync
	msync
	tlbwe
#endif

/*
 * Relocate CCSR, if necessary.  We relocate CCSR if (obviously) the default
 * location is not where we want it.  This typically happens on a 36-bit
 * system, where we want to move CCSR to near the top of 36-bit address space.
 *
 * To move CCSR, we create two temporary TLBs, one for the old location, and
 * another for the new location.  On CoreNet systems, we also need to create
 * a special, temporary LAW.
 *
 * As a general rule, TLB0 is used for short-term TLBs, and TLB1 is used for
 * long-term TLBs, so we use TLB0 here.
 */
#if (CONFIG_SYS_CCSRBAR_DEFAULT != CONFIG_SYS_CCSRBAR_PHYS)

#if !defined(CONFIG_SYS_CCSRBAR_PHYS_HIGH) || !defined(CONFIG_SYS_CCSRBAR_PHYS_LOW)
#error "CONFIG_SYS_CCSRBAR_PHYS_HIGH and CONFIG_SYS_CCSRBAR_PHYS_LOW) must be defined."
#endif

create_ccsr_new_tlb:
	/*
	 * Create a TLB for the new location of CCSR.  Register R8 is reserved
	 * for the virtual address of this TLB (CONFIG_SYS_CCSRBAR).
	 */
	lis	r8, CONFIG_SYS_CCSRBAR@h
	ori	r8, r8, CONFIG_SYS_CCSRBAR@l
	lis	r9, (CONFIG_SYS_CCSRBAR + 0x1000)@h
	ori	r9, r9, (CONFIG_SYS_CCSRBAR + 0x1000)@l
	lis     r0, FSL_BOOKE_MAS0(0, 0, 0)@h
	ori     r0, r0, FSL_BOOKE_MAS0(0, 0, 0)@l
	lis     r1, FSL_BOOKE_MAS1(1, 0, 0, 0, BOOKE_PAGESZ_4K)@h
	ori     r1, r1, FSL_BOOKE_MAS1(1, 0, 0, 0, BOOKE_PAGESZ_4K)@l
	lis     r2, FSL_BOOKE_MAS2(CONFIG_SYS_CCSRBAR, (MAS2_I|MAS2_G))@h
	ori     r2, r2, FSL_BOOKE_MAS2(CONFIG_SYS_CCSRBAR, (MAS2_I|MAS2_G))@l
	lis     r3, FSL_BOOKE_MAS3(CONFIG_SYS_CCSRBAR_PHYS_LOW, 0, (MAS3_SW|MAS3_SR))@h
	ori     r3, r3, FSL_BOOKE_MAS3(CONFIG_SYS_CCSRBAR_PHYS_LOW, 0, (MAS3_SW|MAS3_SR))@l
	lis	r7, CONFIG_SYS_CCSRBAR_PHYS_HIGH@h
	ori	r7, r7, CONFIG_SYS_CCSRBAR_PHYS_HIGH@l
	mtspr   MAS0, r0
	mtspr   MAS1, r1
	mtspr   MAS2, r2
	mtspr   MAS3, r3
	mtspr   MAS7, r7
	isync
	msync
	tlbwe
	
	/*
	 * Create a TLB for the current location of CCSR.  Register R9 is reserved
	 * for the virtual address of this TLB (CONFIG_SYS_CCSRBAR + 0x1000).
	 */
create_ccsr_old_tlb:
	lis     r0, FSL_BOOKE_MAS0(0, 1, 0)@h
	ori     r0, r0, FSL_BOOKE_MAS0(0, 1, 0)@l
	lis     r2, FSL_BOOKE_MAS2(CONFIG_SYS_CCSRBAR + 0x1000, (MAS2_I|MAS2_G))@h
	ori     r2, r2, FSL_BOOKE_MAS2(CONFIG_SYS_CCSRBAR + 0x1000, (MAS2_I|MAS2_G))@l
	lis     r3, FSL_BOOKE_MAS3(CONFIG_SYS_CCSRBAR_DEFAULT, 0, (MAS3_SW|MAS3_SR))@h
	ori     r3, r3, FSL_BOOKE_MAS3(CONFIG_SYS_CCSRBAR_DEFAULT, 0, (MAS3_SW|MAS3_SR))@l
	li	r7, 0	/* The default CCSR address is always a 32-bit number */
	mtspr   MAS0, r0
	/* MAS1 is the same as above */
	mtspr   MAS2, r2
	mtspr   MAS3, r3
	mtspr   MAS7, r7
	isync
	msync
	tlbwe
	
	/*
	 * We have a TLB for what we think is the current (old) CCSR.  Let's
	 * verify that, otherwise we won't be able to move it.
	 * CONFIG_SYS_CCSRBAR_DEFAULT is always a 32-bit number, so we only
	 * need to compare the lower 32 bits of CCSRBAR on CoreNet systems.
	 */
verify_old_ccsr:
	lis     r0, CONFIG_SYS_CCSRBAR_DEFAULT@h
	ori     r0, r0, CONFIG_SYS_CCSRBAR_DEFAULT@l
#ifdef CONFIG_FSL_CORENET
	lwz	r1, 4(r9)		/* CCSRBARL */
#else
	lwz	r1, 0(r9)		/* CCSRBAR, shifted right by 12 */
	slwi	r1, r1, 12
#endif

	cmpl	0, r0, r1

	/*
	 * If the value we read from CCSRBARL is not what we expect, then
	 * enter an infinite loop.  This will at least allow a debugger to
	 * halt execution and examine TLBs, etc.  There's no point in going
	 * on.
	 */
infinite_debug_loop:
	bne	infinite_debug_loop

#ifdef CONFIG_FSL_CORENET

#define CCSR_LAWBARH0	(CONFIG_SYS_CCSRBAR + 0x1000)
#define LAW_EN		0x80000000
#define LAW_SIZE_4K	0xb
#define CCSRBAR_LAWAR	(LAW_EN | (0x1e << 20) | LAW_SIZE_4K)
#define CCSRAR_C	0x80000000	/* Commit */

create_temp_law:
	/*
	 * On CoreNet systems, we create the temporary LAW using a special LAW
	 * target ID of 0x1e.  LAWBARH is at offset 0xc00 in CCSR.
	 */
	lis     r0, CONFIG_SYS_CCSRBAR_PHYS_HIGH@h
	ori     r0, r0, CONFIG_SYS_CCSRBAR_PHYS_HIGH@l
	lis     r1, CONFIG_SYS_CCSRBAR_PHYS_LOW@h
	ori     r1, r1, CONFIG_SYS_CCSRBAR_PHYS_LOW@l
	lis     r2, CCSRBAR_LAWAR@h
	ori     r2, r2, CCSRBAR_LAWAR@l

	stw     r0, 0xc00(r9)	/* LAWBARH0 */
	stw     r1, 0xc04(r9)	/* LAWBARL0 */
	sync
	stw     r2, 0xc08(r9)	/* LAWAR0 */

	/*
	 * Read back from LAWAR to ensure the update is complete.  e500mc
	 * cores also require an isync.
	 */
	lwz	r0, 0xc08(r9)	/* LAWAR0 */
	isync

	/*
	 * Read the current CCSRBARH and CCSRBARL using load word instructions.
	 * Follow this with an isync instruction. This forces any outstanding
	 * accesses to configuration space to completion.
	 */
read_old_ccsrbar:
	lwz	r0, 0(r9)	/* CCSRBARH */
	lwz	r0, 4(r9)	/* CCSRBARL */
	isync

	/*
	 * Write the new values for CCSRBARH and CCSRBARL to their old
	 * locations.  The CCSRBARH has a shadow register. When the CCSRBARH
	 * has a new value written it loads a CCSRBARH shadow register. When
	 * the CCSRBARL is written, the CCSRBARH shadow register contents
	 * along with the CCSRBARL value are loaded into the CCSRBARH and
	 * CCSRBARL registers, respectively.  Follow this with a sync
	 * instruction.
	 */
write_new_ccsrbar:
	lis	r0, CONFIG_SYS_CCSRBAR_PHYS_HIGH@h
	ori	r0, r0, CONFIG_SYS_CCSRBAR_PHYS_HIGH@l
	lis	r1, CONFIG_SYS_CCSRBAR_PHYS_LOW@h
	ori	r1, r1, CONFIG_SYS_CCSRBAR_PHYS_LOW@l
	lis	r2, CCSRAR_C@h
	ori	r2, r2, CCSRAR_C@l

	stw	r0, 0(r9)	/* Write to CCSRBARH */
	sync			/* Make sure we write to CCSRBARH first */
	stw	r1, 4(r9)	/* Write to CCSRBARL */
	sync

	/*
	 * Write a 1 to the commit bit (C) of CCSRAR at the old location.
	 * Follow this with a sync instruction.
	 */
	stw	r2, 8(r9)
	sync

	/* Delete the temporary LAW */
delete_temp_law:
	li	r1, 0
	stw	r1, 0xc08(r8)
	sync
	stw	r1, 0xc00(r8)
	stw	r1, 0xc04(r8)
	sync

#else /* #ifdef CONFIG_FSL_CORENET */

write_new_ccsrbar:
	/*
	 * Read the current value of CCSRBAR using a load word instruction
	 * followed by an isync. This forces all accesses to configuration
	 * space to complete.
	 */
	sync
	lwz	r0, 0(r9)
	isync

/* CONFIG_SYS_CCSRBAR_PHYS right shifted by 12 */
#define CCSRBAR_PHYS_RS12 ((CONFIG_SYS_CCSRBAR_PHYS_HIGH << 20) | \
			   (CONFIG_SYS_CCSRBAR_PHYS_LOW >> 12))

	/* Write the new value to CCSRBAR. */
	lis	r0, CCSRBAR_PHYS_RS12@h
	ori	r0, r0, CCSRBAR_PHYS_RS12@l
	stw	r0, 0(r9)
	sync

	/*
	 * The manual says to perform a load of an address that does not
	 * access configuration space or the on-chip SRAM using an existing TLB,
	 * but that doesn't appear to be necessary.  We will do the isync,
	 * though.
	 */
	isync

	/*
	 * Read the contents of CCSRBAR from its new location, followed by
	 * another isync.
	 */
	lwz	r0, 0(r8)
	isync

#endif  /* #ifdef CONFIG_FSL_CORENET */

	/* Delete the temporary TLBs */
delete_temp_tlbs:
	lis     r0, FSL_BOOKE_MAS0(0, 0, 0)@h
	ori     r0, r0, FSL_BOOKE_MAS0(0, 0, 0)@l
	li	r1, 0
	lis     r2, FSL_BOOKE_MAS2(CONFIG_SYS_CCSRBAR, (MAS2_I|MAS2_G))@h
	ori     r2, r2, FSL_BOOKE_MAS2(CONFIG_SYS_CCSRBAR, (MAS2_I|MAS2_G))@l
	mtspr   MAS0, r0
	mtspr   MAS1, r1
	mtspr   MAS2, r2
	isync
	msync
	tlbwe

	lis     r0, FSL_BOOKE_MAS0(0, 1, 0)@h
	ori     r0, r0, FSL_BOOKE_MAS0(0, 1, 0)@l
	lis     r2, FSL_BOOKE_MAS2(CONFIG_SYS_CCSRBAR + 0x1000, (MAS2_I|MAS2_G))@h
	ori     r2, r2, FSL_BOOKE_MAS2(CONFIG_SYS_CCSRBAR + 0x1000, (MAS2_I|MAS2_G))@l
	mtspr   MAS0, r0
	mtspr   MAS2, r2
	isync
	msync
	tlbwe
#endif /* #if (CONFIG_SYS_CCSRBAR_DEFAULT != CONFIG_SYS_CCSRBAR_PHYS) */

#ifdef CONFIG_SYS_FSL_ERRATUM_A004510
#define DCSR_LAWBARH0	(CONFIG_SYS_CCSRBAR + 0x1000)
#define LAW_SIZE_1M	0x13
#define DCSRBAR_LAWAR	(LAW_EN | (0x1d << 20) | LAW_SIZE_1M)

	cmpwi	r27,0
	beq	9f

	/*
	 * Create a TLB entry for CCSR
	 *
	 * We're executing out of TLB1 entry in r14, and that's the only
	 * TLB entry that exists.  To allocate some TLB entries for our
	 * own use, flip a bit high enough that we won't flip it again
	 * via incrementing.
	 */

	xori	r8, r14, 32
	lis	r0, MAS0_TLBSEL(1)@h
	rlwimi	r0, r8, 16, MAS0_ESEL_MSK
	lis	r1, FSL_BOOKE_MAS1(1, 1, 0, 0, BOOKE_PAGESZ_16M)@h
	ori	r1, r1, FSL_BOOKE_MAS1(1, 1, 0, 0, BOOKE_PAGESZ_16M)@l
	lis	r7, CONFIG_SYS_CCSRBAR@h
	ori	r7, r7, CONFIG_SYS_CCSRBAR@l
	ori	r2, r7, MAS2_I|MAS2_G
	lis	r3, FSL_BOOKE_MAS3(CONFIG_SYS_CCSRBAR_PHYS_LOW, 0, (MAS3_SW|MAS3_SR))@h
	ori	r3, r3, FSL_BOOKE_MAS3(CONFIG_SYS_CCSRBAR_PHYS_LOW, 0, (MAS3_SW|MAS3_SR))@l
	lis	r4, CONFIG_SYS_CCSRBAR_PHYS_HIGH@h
	ori	r4, r4, CONFIG_SYS_CCSRBAR_PHYS_HIGH@l
	mtspr	MAS0, r0
	mtspr	MAS1, r1
	mtspr	MAS2, r2
	mtspr	MAS3, r3
	mtspr	MAS7, r4
	isync
	tlbwe
	isync
	msync

	/* Map DCSR temporarily to physical address zero */
	li	r0, 0
	lis	r3, DCSRBAR_LAWAR@h
	ori	r3, r3, DCSRBAR_LAWAR@l

	stw	r0, 0xc00(r7)	/* LAWBARH0 */
	stw	r0, 0xc04(r7)	/* LAWBARL0 */
	sync
	stw	r3, 0xc08(r7)	/* LAWAR0 */

	/* Read back from LAWAR to ensure the update is complete. */
	lwz	r3, 0xc08(r7)	/* LAWAR0 */
	isync

	/* Create a TLB entry for DCSR at zero */

	addi	r9, r8, 1
	lis	r0, MAS0_TLBSEL(1)@h
	rlwimi	r0, r9, 16, MAS0_ESEL_MSK
	lis	r1, FSL_BOOKE_MAS1(1, 1, 0, 0, BOOKE_PAGESZ_1M)@h
	ori	r1, r1, FSL_BOOKE_MAS1(1, 1, 0, 0, BOOKE_PAGESZ_1M)@l
	li	r6, 0	/* DCSR effective address */
	ori	r2, r6, MAS2_I|MAS2_G
	li	r3, MAS3_SW|MAS3_SR
	li	r4, 0
	mtspr	MAS0, r0
	mtspr	MAS1, r1
	mtspr	MAS2, r2
	mtspr	MAS3, r3
	mtspr	MAS7, r4
	isync
	tlbwe
	isync
	msync

	/* enable the timebase */
#define CTBENR	0xe2084
	li	r3, 1
	addis	r4, r7, CTBENR@ha
	stw	r3, CTBENR@l(r4)
	lwz	r3, CTBENR@l(r4)
	twi	0,r3,0
	isync

	.macro	erratum_set_ccsr offset value
	addis	r3, r7, \offset@ha
	lis	r4, \value@h
	addi	r3, r3, \offset@l
	ori	r4, r4, \value@l
	bl	erratum_set_value
	.endm

	.macro	erratum_set_dcsr offset value
	addis	r3, r6, \offset@ha
	lis	r4, \value@h
	addi	r3, r3, \offset@l
	ori	r4, r4, \value@l
	bl	erratum_set_value
	.endm

	erratum_set_dcsr 0xb0e08 0xe0201800
	erratum_set_dcsr 0xb0e18 0xe0201800
	erratum_set_dcsr 0xb0e38 0xe0400000
	erratum_set_dcsr 0xb0008 0x00900000
	erratum_set_dcsr 0xb0e40 0xe00a0000
	erratum_set_ccsr 0x18600 CONFIG_SYS_FSL_CORENET_SNOOPVEC_COREONLY
	erratum_set_ccsr 0x10f00 0x415e5000
	erratum_set_ccsr 0x11f00 0x415e5000

	/* Make temp mapping uncacheable again, if it was initially */
	bl	2f
2:	mflr	r3
	tlbsx	0, r3
	mfspr	r4, MAS2
	rlwimi	r4, r15, 0, MAS2_I
	rlwimi	r4, r15, 0, MAS2_G
	mtspr	MAS2, r4
	isync
	tlbwe
	isync
	msync

	/* Clear the cache */
	lis	r3,(L1CSR1_ICFI|L1CSR1_ICLFR)@h
	ori	r3,r3,(L1CSR1_ICFI|L1CSR1_ICLFR)@l
	sync
	isync
	mtspr	SPRN_L1CSR1,r3
	isync
2:	sync
	mfspr	r4,SPRN_L1CSR1
	and.	r4,r4,r3
	bne	2b

	lis	r3,(L1CSR1_CPE|L1CSR1_ICE)@h
	ori	r3,r3,(L1CSR1_CPE|L1CSR1_ICE)@l
	sync
	isync
	mtspr	SPRN_L1CSR1,r3
	isync
2:	sync
	mfspr	r4,SPRN_L1CSR1
	and.	r4,r4,r3
	beq	2b

	/* Remove temporary mappings */
	lis	r0, MAS0_TLBSEL(1)@h
	rlwimi	r0, r9, 16, MAS0_ESEL_MSK
	li	r3, 0
	mtspr	MAS0, r0
	mtspr	MAS1, r3
	isync
	tlbwe
	isync
	msync

	li	r3, 0
	stw	r3, 0xc08(r7)	/* LAWAR0 */
	lwz	r3, 0xc08(r7)
	isync

	lis	r0, MAS0_TLBSEL(1)@h
	rlwimi	r0, r8, 16, MAS0_ESEL_MSK
	li	r3, 0
	mtspr	MAS0, r0
	mtspr	MAS1, r3
	isync
	tlbwe
	isync
	msync

	b	9f

	/* r3 = addr, r4 = value, clobbers r5, r11, r12 */
erratum_set_value:
	/* Lock two cache lines into I-Cache */
	sync
	mfspr	r11, SPRN_L1CSR1
	rlwinm	r11, r11, 0, ~L1CSR1_ICUL
	sync
	isync
	mtspr	SPRN_L1CSR1, r11
	isync

	mflr	r12
	bl	5f
5:	mflr	r5
	addi	r5, r5, 2f - 5b
	icbtls	0, 0, r5
	addi	r5, r5, 64

	sync
	mfspr	r11, SPRN_L1CSR1
3:	andi.	r11, r11, L1CSR1_ICUL
	bne	3b

	icbtls	0, 0, r5
	addi	r5, r5, 64

	sync
	mfspr	r11, SPRN_L1CSR1
3:	andi.	r11, r11, L1CSR1_ICUL
	bne	3b

	b	2f
	.align	6
	/* Inside a locked cacheline, wait a while, write, then wait a while */
2:	sync

	mfspr	r5, SPRN_TBRL
	addis	r11, r5, 0x10000@h /* wait 65536 timebase ticks */
4:	mfspr	r5, SPRN_TBRL
	subf.	r5, r5, r11
	bgt	4b

	stw	r4, 0(r3)

	mfspr	r5, SPRN_TBRL
	addis	r11, r5, 0x10000@h /* wait 65536 timebase ticks */
4:	mfspr	r5, SPRN_TBRL
	subf.	r5, r5, r11
	bgt	4b

	sync

	/*
	 * Fill out the rest of this cache line and the next with nops,
	 * to ensure that nothing outside the locked area will be
	 * fetched due to a branch.
	 */
	.rept 19
	nop
	.endr

	sync
	mfspr	r11, SPRN_L1CSR1
	rlwinm	r11, r11, 0, ~L1CSR1_ICUL
	sync
	isync
	mtspr	SPRN_L1CSR1, r11
	isync

	mtlr	r12
	blr

9:
#endif

create_init_ram_area:
	lis     r6,FSL_BOOKE_MAS0(1, 15, 0)@h
	ori     r6,r6,FSL_BOOKE_MAS0(1, 15, 0)@l

	/*
	 * create a temp mapping in AS=1 to the 1M CONFIG_SYS_MONITOR_BASE space, the main
	 * image has been relocated to CONFIG_SYS_MONITOR_BASE on the second stage.
	 */
	lis     r7,FSL_BOOKE_MAS1(1, 1, 0, 1, BOOKE_PAGESZ_1M)@h
	ori     r7,r7,FSL_BOOKE_MAS1(1, 1, 0, 1, BOOKE_PAGESZ_1M)@l

	lis     r8,FSL_BOOKE_MAS2(CONFIG_SYS_MONITOR_BASE, (MAS2_I|MAS2_G))@h
	ori     r8,r8,FSL_BOOKE_MAS2(CONFIG_SYS_MONITOR_BASE, (MAS2_I|MAS2_G))@l

	lis     r9,FSL_BOOKE_MAS3(CONFIG_SYS_MONITOR_BASE, 0, (MAS3_SX|MAS3_SW|MAS3_SR))@h
	ori     r9,r9,FSL_BOOKE_MAS3(CONFIG_SYS_MONITOR_BASE, 0, (MAS3_SX|MAS3_SW|MAS3_SR))@l

	mtspr   MAS0,r6
	mtspr   MAS1,r7
	mtspr   MAS2,r8
	mtspr   MAS3,r9
	isync
	msync
	tlbwe

	/* create a temp mapping in AS=1 to the stack */
	lis     r6,FSL_BOOKE_MAS0(1, 14, 0)@h
	ori     r6,r6,FSL_BOOKE_MAS0(1, 14, 0)@l

	lis     r7,FSL_BOOKE_MAS1(1, 1, 0, 1, BOOKE_PAGESZ_16K)@h
	ori     r7,r7,FSL_BOOKE_MAS1(1, 1, 0, 1, BOOKE_PAGESZ_16K)@l

	lis     r8,FSL_BOOKE_MAS2(CONFIG_SYS_INIT_RAM_ADDR, 0)@h
	ori     r8,r8,FSL_BOOKE_MAS2(CONFIG_SYS_INIT_RAM_ADDR, 0)@l

#if defined(CONFIG_SYS_INIT_RAM_ADDR_PHYS_LOW) && \
    defined(CONFIG_SYS_INIT_RAM_ADDR_PHYS_HIGH)
	lis     r9,FSL_BOOKE_MAS3(CONFIG_SYS_INIT_RAM_ADDR_PHYS_LOW, 0,
				(MAS3_SX|MAS3_SW|MAS3_SR))@h
	ori     r9,r9,FSL_BOOKE_MAS3(CONFIG_SYS_INIT_RAM_ADDR_PHYS_LOW, 0,
				(MAS3_SX|MAS3_SW|MAS3_SR))@l
	li      r10,CONFIG_SYS_INIT_RAM_ADDR_PHYS_HIGH
	mtspr	MAS7,r10
#else
	lis     r9,FSL_BOOKE_MAS3(CONFIG_SYS_INIT_RAM_ADDR, 0, (MAS3_SX|MAS3_SW|MAS3_SR))@h
	ori     r9,r9,FSL_BOOKE_MAS3(CONFIG_SYS_INIT_RAM_ADDR, 0, (MAS3_SX|MAS3_SW|MAS3_SR))@l

#endif

	mtspr   MAS0,r6
	mtspr   MAS1,r7
	mtspr   MAS2,r8
	mtspr   MAS3,r9
	isync
	msync
	tlbwe

	lis	r6,MSR_IS|MSR_DS|MSR_DE@h
	ori	r6,r6,MSR_IS|MSR_DS|MSR_DE@l
	lis	r7,switch_as@h
	ori	r7,r7,switch_as@l

	mtspr	SPRN_SRR0,r7
	mtspr	SPRN_SRR1,r6
	rfi

switch_as:
/* L1 DCache is used for initial RAM */

	/* Allocate Initial RAM in data cache.
	 */
	lis	r3,CONFIG_SYS_INIT_RAM_ADDR@h
	ori	r3,r3,CONFIG_SYS_INIT_RAM_ADDR@l
	mfspr	r2, L1CFG0
	andi.	r2, r2, 0x1ff
	/* cache size * 1024 / (2 * L1 line size) */
	slwi	r2, r2, (10 - 1 - L1_CACHE_SHIFT)
	mtctr	r2
	li	r0,0
1:
	dcbz	r0,r3
	dcbtls	0,r0,r3
	addi	r3,r3,CONFIG_SYS_CACHELINE_SIZE
	bdnz	1b

	/* Jump out the last 4K page and continue to 'normal' start */
#ifdef CONFIG_SYS_RAMBOOT
	b	_start_cont
#endif

	.text
	.globl	_start
_start:
	.long	0x27051956		/* U-BOOT Magic Number */
	.globl	version_string
version_string:
	.ascii U_BOOT_VERSION_STRING, "\0"

	.align	4
	.globl	_start_cont
_start_cont:
	/* Setup the stack in initial RAM,could be L2-as-SRAM or L1 dcache*/
	lis	r1,CONFIG_SYS_INIT_RAM_ADDR@h
	ori	r1,r1,CONFIG_SYS_INIT_SP_OFFSET@l

	li	r0,0
	stwu	r0,-4(r1)
	stwu	r0,-4(r1)		/* Terminate call chain */

	stwu	r1,-8(r1)		/* Save back chain and move SP */
	lis	r0,RESET_VECTOR@h	/* Address of reset vector */
	ori	r0,r0,RESET_VECTOR@l
	stwu	r1,-8(r1)		/* Save back chain and move SP */
	stw	r0,+12(r1)		/* Save return addr (underflow vect) */

	GET_GOT
	bl	c2x0_cpu_init_early_f 

	/* switch back to AS = 0 */
	lis	r3,(MSR_CE|MSR_ME|MSR_DE)@h
	ori	r3,r3,(MSR_CE|MSR_ME|MSR_DE)@l
	mtmsr	r3
	isync

	bl	c2x0_cpu_init_f
/*	bl	board_init_f  */
	bl c2x0_board_init_f 
	isync

	/* NOTREACHED - board_init_f() does not return */

    .globl get_svr
get_svr:
    mfspr   r3,SVR
    blr

/*------------------------------------------------------------------------------*/

/*
 * void write_tlb(mas0, mas1, mas2, mas3, mas7)
 */
	.globl	write_tlb
write_tlb:
	mtspr	MAS0,r3
	mtspr	MAS1,r4
	mtspr	MAS2,r5
	mtspr	MAS3,r6
#ifdef CONFIG_ENABLE_36BIT_PHYS
	mtspr	MAS7,r7
#endif
	li	r3,0
#ifdef CONFIG_SYS_BOOK3E_HV
	mtspr	MAS8,r3
#endif
	isync
	tlbwe
	msync
	isync
	blr

	.globl  getticks
getticks:
1:  mftbu   r3
    mftb    r4
    mftbu   r5
    cmp 0,r3,r5
    bne 1b
    blr

.globl  memcpy
memcpy:
    rlwinm. r7,r5,32-3,3,31     /* r0 = r5 >> 3 */
    addi    r6,r3,-4
    addi    r4,r4,-4
    beq 2f          /* if less than 8 bytes to do */
    andi.   r0,r6,3         /* get dest word aligned */
    mtctr   r7
    bne 5f
1:  lwz r7,4(r4)
    lwzu    r8,8(r4)
    stw r7,4(r6)
    stwu    r8,8(r6)
    bdnz    1b
    andi.   r5,r5,7
2:  cmplwi  0,r5,4
    blt 3f
    lwzu    r0,4(r4)
    addi    r5,r5,-4
    stwu    r0,4(r6)
3:  cmpwi   0,r5,0
    beqlr
    mtctr   r5
    addi    r4,r4,3
    addi    r6,r6,3
4:  lbzu    r0,1(r4)
    stbu    r0,1(r6)
    bdnz    4b
    blr
5:  subfic  r0,r0,4
    mtctr   r0
6:  lbz r7,4(r4)
    addi    r4,r4,1
    stb r7,4(r6)
    addi    r6,r6,1
    bdnz    6b
    subf    r5,r0,r5
    rlwinm. r7,r5,32-3,3,31
    beq 2b
    mtctr   r7
    b   1b

