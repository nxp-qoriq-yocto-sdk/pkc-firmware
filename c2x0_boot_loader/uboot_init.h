/* Copyright 2013 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of Freescale Semiconductor nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE)ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

struct cpu_type {
    char name[15];
    u32 soc_ver;
    u32 num_cores;
    u32 mask;   /* which cpu(s) actually exist */
};

#define SVR_P4080	0x820000
#define SVR_P1010	0x80F100
#define SVR_C291	0x850000
#define SVR_C292	0x850020
#define SVR_C293	0x850030

#define C293
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

/*
 * LAW support
 */
enum law_size {
    LAW_SIZE_4K = 0xb,
    LAW_SIZE_8K,
    LAW_SIZE_16K,
    LAW_SIZE_32K,
    LAW_SIZE_64K,
    LAW_SIZE_128K,
    LAW_SIZE_256K,
    LAW_SIZE_512K,
    LAW_SIZE_1M,
    LAW_SIZE_2M,
    LAW_SIZE_4M,
    LAW_SIZE_8M,
    LAW_SIZE_16M,
    LAW_SIZE_32M,
    LAW_SIZE_64M,
    LAW_SIZE_128M,
    LAW_SIZE_256M,
    LAW_SIZE_512M,
    LAW_SIZE_1G,
    LAW_SIZE_2G,
    LAW_SIZE_4G,
    LAW_SIZE_8G,
    LAW_SIZE_16G,
    LAW_SIZE_32G,
};

enum law_trgt_if {
    LAW_TRGT_IF_PCIE_1 = 0x00,
    LAW_TRGT_IF_PCIE_2 = 0x01,
    LAW_TRGT_IF_PCIE_3 = 0x02,
    LAW_TRGT_IF_PCIE_4 = 0x03,
    LAW_TRGT_IF_RIO_1 = 0x08,
    LAW_TRGT_IF_RIO_2 = 0x09,

    LAW_TRGT_IF_DDR_1 = 0x10,
    LAW_TRGT_IF_DDR_2 = 0x11,   /* 2nd controller */
    LAW_TRGT_IF_DDR_INTRLV = 0x14,

    LAW_TRGT_IF_BMAN = 0x18,
    LAW_TRGT_IF_DCSR = 0x1d,
    LAW_TRGT_IF_LBC = 0x1f,
    LAW_TRGT_IF_QMAN = 0x3c,
};

#ifndef CONFIG_SYS_IMMR
#define CONFIG_SYS_IMMR CONFIG_SYS_CCSRBAR
#endif

#define FSL_HW_NUM_LAWS 32
#define LAW_EN  0x80000000

#define LAW_BASE (CONFIG_SYS_IMMR)
#define LAWAR_ADDR(x) (&((ccsr_local_t *)LAW_BASE)->law[x].lawar)
#define LAWBARH_ADDR(x) (&((ccsr_local_t *)LAW_BASE)->law[x].lawbarh)
#define LAWBARL_ADDR(x) (&((ccsr_local_t *)LAW_BASE)->law[x].lawbarl)
#define LAWBAR_SHIFT 0
/* 
 * TLB support
 */
//#define CONFIG_SYS_NUM_TLBCAMS 64

#define MAS0_TLBSEL_MSK 0x30000000
#define MAS0_TLBSEL(x)  (((x) << 28) & MAS0_TLBSEL_MSK)
#define MAS0_ESEL_MSK   0x0FFF0000
#define MAS0_ESEL(x)    (((x) << 16) & MAS0_ESEL_MSK)
#define MAS0_NV(x)  ((x) & 0x00000FFF)

#define MAS1_VALID  0x80000000
#define MAS1_IPROT  0x40000000
#define MAS1_TID(x) (((x) << 16) & 0x3FFF0000)
#define MAS1_TS     0x00001000
#define MAS1_TSIZE(x)   (((x) << 8) & 0x00000F00)
#define TSIZE_TO_BYTES(x) (1ULL << (((x) * 2) + 10))

#define FSL_BOOKE_MAS0(tlbsel,esel,nv) \
        (MAS0_TLBSEL(tlbsel) | MAS0_ESEL(esel) | MAS0_NV(nv))
#define FSL_BOOKE_MAS1(v,iprot,tid,ts,tsize) \
        ((((v) << 31) & MAS1_VALID)             |\
        (((iprot) << 30) & MAS1_IPROT)          |\
        (MAS1_TID(tid))             |\
        (((ts) << 12) & MAS1_TS)                |\
        (MAS1_TSIZE(tsize)))
#define FSL_BOOKE_MAS2(epn, wimge) \
        (((epn) & MAS3_RPN) | (wimge))
#define FSL_BOOKE_MAS3(rpn, user, perms) \
        (((rpn) & MAS3_RPN) | (user) | (perms))
#define FSL_BOOKE_MAS7(rpn) \
        (((u64)(rpn)) >> 32)

#define SET_TLB_ENTRY(_tlb, _epn, _rpn, _perms, _wimge, _ts, _esel, _sz, _iprot) \
    { .mas0 = FSL_BOOKE_MAS0(_tlb, _esel, 0), \
      .mas1 = FSL_BOOKE_MAS1(1, _iprot, 0, _ts, _sz), \
      .mas2 = FSL_BOOKE_MAS2(_epn, _wimge), \
      .mas3 = FSL_BOOKE_MAS3(_rpn, 0, _perms), \
      .mas7 = FSL_BOOKE_MAS7(_rpn), }

struct fsl_e_tlb_entry {
    u32 mas0;
    u32 mas1;
    u32 mas2; 
    u32 mas3; 
    u32 mas7; 
};  

struct fsl_e_tlb_entry c2x0_tlb_table[] = {
    /* TLB 0 - for temp stack in cache */
    SET_TLB_ENTRY(0, CONFIG_SYS_INIT_RAM_ADDR,
              CONFIG_SYS_INIT_RAM_ADDR,
              MAS3_SW|MAS3_SR, 0,
              0, 0, BOOKE_PAGESZ_4K, 0),
    SET_TLB_ENTRY(0, CONFIG_SYS_INIT_RAM_ADDR + 4 * 1024,
              CONFIG_SYS_INIT_RAM_ADDR + 4 * 1024,
              MAS3_SW|MAS3_SR, 0,
              0, 0, BOOKE_PAGESZ_4K, 0),
    SET_TLB_ENTRY(0, CONFIG_SYS_INIT_RAM_ADDR + 8 * 1024,
              CONFIG_SYS_INIT_RAM_ADDR + 8 * 1024,
              MAS3_SW|MAS3_SR, 0,
              0, 0, BOOKE_PAGESZ_4K, 0),
    SET_TLB_ENTRY(0, CONFIG_SYS_INIT_RAM_ADDR + 12 * 1024,
              CONFIG_SYS_INIT_RAM_ADDR + 12 * 1024,
              MAS3_SW|MAS3_SR, 0,
              0, 0, BOOKE_PAGESZ_4K, 0),
    /* TLB 1 */
    /* *I*** - Covers boot page */
#if 0
    /*
     * *I*G - L2SRAM. When L2 is used as 512KB SRAM, the address of the
     * SRAM TLB is at 0xfff00000, it covered the 0xfffff000.
     */
    SET_TLB_ENTRY(1, CONFIG_SYS_INIT_L2_ADDR, CONFIG_SYS_INIT_L2_ADDR,
            MAS3_SX|MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
            0, 0, BOOKE_PAGESZ_1M, 1),

    /* *I*G* - CCSRBAR */
    SET_TLB_ENTRY(1, CONFIG_SYS_CCSRBAR, CONFIG_SYS_CCSRBAR_PHYS,
              MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
              0, 1, BOOKE_PAGESZ_1M, 1),
#else
	/* TLB 1 */
	/* *I*** - Covers boot page */
	SET_TLB_ENTRY(1, 0xfffff000, 0xfffff000,
			MAS3_SX|MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
			0, 0, BOOKE_PAGESZ_4K, 1),

	/* *I*G* - CCSRBAR */
	SET_TLB_ENTRY(1, CONFIG_SYS_CCSRBAR, CONFIG_SYS_CCSRBAR_PHYS,
			MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
			0, 1, BOOKE_PAGESZ_1M, 1),

	/* OB window */
#if 0
	SET_TLB_ENTRY(1, 0xff000000, 0xff000000,
			MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
			0, 3, BOOKE_PAGESZ_1M, 1),
#endif

#if 0
#if 1
	SET_TLB_ENTRY(1, 0x80000000, 0x800000000,
            MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
            0, 3, BOOKE_PAGESZ_1G, 1),

	SET_TLB_ENTRY(1, 0xc0000000, 0x8fee00000,
            MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
            0, 4, BOOKE_PAGESZ_1M, 1),

#else
/*
	SET_TLB_ENTRY(1, 0x80000000, 0x80000000,
            MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
            0, 3, BOOKE_PAGESZ_1M, 1),
*/
	SET_TLB_ENTRY(1, 0x80000000, 0x80000000,
            MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
            0, 3, BOOKE_PAGESZ_1G, 1),

	SET_TLB_ENTRY(1, 0x9000000, 0x90000000,
            MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
            0, 4, BOOKE_PAGESZ_1M, 1),
#endif
#endif
	/* L2 SRAM */
	SET_TLB_ENTRY(1, 0xfff00000, 0xfff00000,
		      MAS3_SX|MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
		      0, 8, BOOKE_PAGESZ_256K, 1),
	SET_TLB_ENTRY(1, 0xfff40000, 0xfff40000,
		      MAS3_SX|MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
		      0, 9, BOOKE_PAGESZ_256K, 1),

	/* *I*G - PSRAM */
	SET_TLB_ENTRY(1, 0xfff80000, 0xfff80000,
		      MAS3_SX|MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
		      0, 10, BOOKE_PAGESZ_256K, 1),
	SET_TLB_ENTRY(1, 0xfffc0000, 0xfffc0000,
		      MAS3_SX|MAS3_SW|MAS3_SR, MAS2_I|MAS2_G,
		      0, 11, BOOKE_PAGESZ_256K, 1),
#endif

};

int c2x0_num_tlb_entries = ARRAY_SIZE(c2x0_tlb_table);
#define CPU_TYPE_ENTRY(n, v, nc) \
    { .name = #n, .soc_ver = SVR_##v, .num_cores = (nc), \
      .mask = (1 << (nc)) - 1 }

struct cpu_type c2x0_cpu_type_list [] = {
    CPU_TYPE_ENTRY(P4080, P4080, 8),
    CPU_TYPE_ENTRY(P1010, P1010, 1),
    CPU_TYPE_ENTRY(C293, C293, 1),
};

/* 
 * Prompt Support
 */

#define C2X0_CONFIG_SYS_PROMPT   "=> "       /* Monitor Command Prompt */
#define C2X0_CONFIG_SYS_CBSIZE   1024        /* Console I/O Buffer Size */
#define C2X0_FLAG_PARSE_SEMICOLON (1 << 1)     /* symbol ';' is special for parser */
#define EOF -1
#define b_getch(input) ((input)->get(input))
#define getcmd_putch(ch)    putc(ch)
#define CTL_BACKSPACE       ('\b')
#define getcmd_cbeep()      c2x0_putc('\a')
struct c2x0_in_str {
    const char *p;
    int __promptme;
    int promptmode;
    int (*get) (struct c2x0_in_str *);
};  

int fsl_c2x0_fw(void );
void c2x0_set_tlb(u8 tlb, u32 epn, u64 rpn,
        u8 perms, u8 wimge,
        u8 ts, u8 esel, u8 tsize, u8 iprot);
