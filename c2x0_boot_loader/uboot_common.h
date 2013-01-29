/*
* Copyright 2013 Freescale Semiconductor, Inc.
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
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston,
* MA 02111-1307 USA
*/

#ifndef __COMMON_H_
#define __COMMON_H_ 1

#undef  _LINUX_CONFIG_H
#define _LINUX_CONFIG_H 1   /* avoid reading Linux autoconf.h file  */

#ifndef __ASSEMBLY__        /* put C only stuff in this section */

typedef unsigned char       uchar;
typedef volatile unsigned long  vu_long;
typedef volatile unsigned short vu_short;
typedef volatile unsigned char  vu_char;

#include <config.h>
#include <asm-offsets.h>
#include <linux/bitops.h>
#include <linux/types.h>
#include <linux/string.h>
#include <asm/ptrace.h>
#include <stdarg.h>

#include <mpc85xx.h>
#include <asm/immap_85xx.h>

#endif /* __ASSEMBLY__ */

#endif  /* __COMMON_H_ */


#define MAS2_EPN    0xFFFFF000
#define MAS2_X0     0x00000040
#define MAS2_X1     0x00000020
#define MAS2_W      0x00000010
#define MAS2_I      0x00000008
#define MAS2_M      0x00000004
#define MAS2_G      0x00000002
#define MAS2_E      0x00000001

#define MAS3_RPN    0xFFFFF000
#define MAS3_U0     0x00000200
#define MAS3_U1     0x00000100
#define MAS3_U2     0x00000080
#define MAS3_U3     0x00000040
#define MAS3_UX     0x00000020
#define MAS3_SX     0x00000010
#define MAS3_UW     0x00000008
#define MAS3_SW     0x00000004
#define MAS3_UR     0x00000002
#define MAS3_SR     0x00000001


#define BOOKE_PAGESZ_1K         0
#define BOOKE_PAGESZ_4K         1
#define BOOKE_PAGESZ_16K        2
#define BOOKE_PAGESZ_64K        3
#define BOOKE_PAGESZ_256K       4
#define BOOKE_PAGESZ_1M         5
#define BOOKE_PAGESZ_4M         6
#define BOOKE_PAGESZ_16M        7
#define BOOKE_PAGESZ_64M        8
#define BOOKE_PAGESZ_256M       9
#define BOOKE_PAGESZ_1G     10
#define BOOKE_PAGESZ_4G     11
#define BOOKE_PAGESZ_16GB   12
#define BOOKE_PAGESZ_64GB   13
#define BOOKE_PAGESZ_256GB  14
#define BOOKE_PAGESZ_1TB    15

/*
 * The following data structure is placed in some memory wich is
 * available very early after boot (like DPRAM on MPC8xx/MPC82xx, or
 * some locked parts of the data cache) to allow for a minimum set of
 * global variables during system initialization (until we have set
 * up the memory controller so that we can use RAM).
 *
 * Keep it *SMALL* and remember to set GENERATED_GBL_DATA_SIZE > sizeof(gd_t)
 */

typedef	struct	c2x0_global_data {
	unsigned long	cie_idx;
	unsigned long	baudrate;
	unsigned long	cpu_clk;	/* CPU clock in Hz! */
	unsigned long	bus_clk;
#if defined(CONFIG_8xx)
	unsigned long	brg_clk;
#endif
#if defined(CONFIG_CPM2)
	/* There are many clocks on the MPC8260 - see page 9-5 */
	unsigned long	vco_out;
	unsigned long	cpm_clk;
	unsigned long	scc_clk;
	unsigned long	brg_clk;
#ifdef CONFIG_PCI
	unsigned long	pci_clk;
#endif
#endif
	unsigned long   mem_clk;
#if defined(CONFIG_MPC83xx)
	/* There are other clocks in the MPC83XX */
	u32 csb_clk;
#if defined(CONFIG_MPC8308) || defined(CONFIG_MPC831x) || \
	defined(CONFIG_MPC834x) || defined(CONFIG_MPC837x)
	u32 tsec1_clk;
	u32 tsec2_clk;
	u32 usbdr_clk;
#endif
#if defined (CONFIG_MPC834x)
	u32 usbmph_clk;
#endif /* CONFIG_MPC834x */
#if defined(CONFIG_MPC8315)
	u32 tdm_clk;
#endif
	u32 core_clk;
	u32 enc_clk;
	u32 lbiu_clk;
	u32 lclk_clk;
	u32 pci_clk;
#if defined(CONFIG_MPC8308) || defined(CONFIG_MPC831x) || \
	defined(CONFIG_MPC837x)
	u32 pciexp1_clk;
	u32 pciexp2_clk;
#endif
#if defined(CONFIG_MPC837x) || defined(CONFIG_MPC8315)
	u32 sata_clk;
#endif
#if defined(CONFIG_MPC8360)
	u32  mem_sec_clk;
#endif /* CONFIG_MPC8360 */
#endif
#if defined(CONFIG_FSL_ESDHC)
	u32 sdhc_clk;
#endif
#if defined(CONFIG_MPC85xx) || defined(CONFIG_MPC86xx)
	u32 lbc_clk;
	void *cpu;
#endif /* CONFIG_MPC85xx || CONFIG_MPC86xx */
#if defined(CONFIG_MPC83xx) || defined(CONFIG_MPC85xx) || defined(CONFIG_MPC86xx)
	u32 i2c1_clk;
	u32 i2c2_clk;
#endif
#if defined(CONFIG_QE)
	u32 qe_clk;
	u32 brg_clk;
	uint mp_alloc_base;
	uint mp_alloc_top;
#endif /* CONFIG_QE */
#if defined(CONFIG_FSL_LAW)
	u32 used_laws;
#endif
#if defined(CONFIG_E500)
	u32 used_tlb_cams[(CONFIG_SYS_NUM_TLBCAMS+31)/32];
#endif
#if defined(CONFIG_MPC5xxx)
	unsigned long	ipb_clk;
	unsigned long	pci_clk;
#endif
#if defined(CONFIG_MPC512X)
	u32 ips_clk;
	u32 csb_clk;
	u32 pci_clk;
#endif /* CONFIG_MPC512X */
#if defined(CONFIG_MPC8220)
	unsigned long   bExtUart;
	unsigned long   inp_clk;
	unsigned long   pci_clk;
	unsigned long   vco_clk;
	unsigned long   pev_clk;
	unsigned long   flb_clk;
#endif
	phys_size_t	ram_size;	/* RAM size */
	unsigned long	reset_status;	/* reset status register at boot	*/
#if defined(CONFIG_MPC83xx)
	unsigned long	arbiter_event_attributes;
	unsigned long	arbiter_event_address;
#endif
	unsigned long	env_addr;	/* Address  of Environment struct	*/
	unsigned long	env_valid;	/* Checksum of Environment valid?	*/
	unsigned long	have_console;	/* serial_init() was called		*/
#ifdef CONFIG_PRE_CONSOLE_BUFFER
	unsigned long	precon_buf_idx;	/* Pre-Console buffer index */
#endif
#if defined(CONFIG_SYS_ALLOC_DPRAM) || defined(CONFIG_CPM2)
	unsigned int	dp_alloc_base;
	unsigned int	dp_alloc_top;
#endif
#if defined(CONFIG_4xx)
	u32  uart_clk;
#endif /* CONFIG_4xx */
#if defined(CONFIG_SYS_GT_6426x)
	unsigned int	mirror_hack[16];
#endif
#if defined(CONFIG_A3000)	|| \
    defined(CONFIG_HIDDEN_DRAGON)  || \
    defined(CONFIG_MUSENKI)	||  \
    defined(CONFIG_SANDPOINT)
	void *		console_addr;
#endif
	unsigned long	next_avail_memory;	/* Start address of U-Boot in RAM */
#if defined(CONFIG_LCD) || defined(CONFIG_VIDEO)
	unsigned long	fb_base;	/* Base address of framebuffer memory	*/
#endif
#if defined(CONFIG_POST) || defined(CONFIG_LOGBUFFER)
	unsigned long	post_log_word;  /* Record POST activities */
	unsigned long	post_log_res; /* success of POST test */
	unsigned long	post_init_f_time;  /* When post_init_f started */
#endif
#ifdef CONFIG_BOARD_TYPES
	unsigned long	board_type;
#endif
#ifdef CONFIG_MODEM_SUPPORT
	unsigned long do_mdm_init;
	unsigned long be_quiet;
#endif
#if defined(CONFIG_LWMON) || defined(CONFIG_LWMON5)
	unsigned long kbd_status;
#endif
#ifdef CONFIG_SYS_FPGA_COUNT
	unsigned fpga_state[CONFIG_SYS_FPGA_COUNT];
#endif
#if defined(CONFIG_WD_MAX_RATE)
	unsigned long long wdt_last;	/* trace watch-dog triggering rate */
#endif
	void		**jt;		/* jump table */
	char		env_buf[32];	/* buffer for getenv() before reloc. */
} c2x0_gd_t;


register volatile c2x0_gd_t *gd asm ("r2");

extern void write_tlb(u32 _mas0, u32 _mas1, u32 _mas2, u32 _mas3, u32 _mas7);

#define C2X0_CONFIG_L2_SRAM_BASE 0xfff00000
#define COMMAND_SUPPORT 1

#define WATCHDOG_RESET() {}
