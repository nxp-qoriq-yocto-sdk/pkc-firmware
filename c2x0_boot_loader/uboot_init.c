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

#include "uboot_common.h"
#include "uboot_init.h"
#include "uboot_print.h"

void
c2x0_CritcalInputException(struct pt_regs *regs)
{
	return;
}

/*****************************************************************************************/
    /****************    LAW support   ******************************/
/*****************************************************************************************/
int c2x0_set_next_law(phys_addr_t addr, enum law_size sz, enum law_trgt_if id)
{
	u32 idx = gd->used_laws;
	gd->used_laws++;
    out_be32(LAWAR_ADDR(idx), 0);
    out_be32(LAWBARL_ADDR(idx), addr & 0xffffffff);
    out_be32(LAWBARH_ADDR(idx), (uint32_t)((u64)addr >> 32));
    out_be32(LAWAR_ADDR(idx), LAW_EN | ((u32)id << 20) | (u32)sz);
    in_be32(LAWAR_ADDR(idx));

    return idx;
}

/*****************************************************************************************/
    /****************    TLB support   ******************************/
/*****************************************************************************************/
void c2x0_init_tlbs(void)
{
    int i;

    for (i = 0; i < c2x0_num_tlb_entries; i++) {
    	write_tlb(c2x0_tlb_table[i].mas0,
              c2x0_tlb_table[i].mas1,
              c2x0_tlb_table[i].mas2,
              c2x0_tlb_table[i].mas3,
              c2x0_tlb_table[i].mas7);
    }

    return ;
}

/* We run cpu_init_early_f in AS = 1 */
void c2x0_cpu_init_early_f(void)
{
    u32 mas0, mas1, mas2, mas3, mas7;
    int i;

    /* Pointer is writable since we allocated a register for it */
    gd = (c2x0_gd_t *) (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_GBL_DATA_OFFSET);
    /*
     * Clear initial global data
     *   we don't use memset so we can share this code with NAND_SPL
     */
    for (i = 0; i < sizeof(c2x0_gd_t); i++)
        ((char *)gd)[i] = 0;

	gd->cie_idx = 100;

    mas0 = MAS0_TLBSEL(1) | MAS0_ESEL(13);
    mas1 = MAS1_VALID | MAS1_TID(0) | MAS1_TS | MAS1_TSIZE(BOOKE_PAGESZ_1M);
    mas2 = FSL_BOOKE_MAS2(CONFIG_SYS_CCSRBAR, MAS2_I|MAS2_G);
    mas3 = FSL_BOOKE_MAS3(CONFIG_SYS_CCSRBAR_PHYS, 0, MAS3_SW|MAS3_SR);
    mas7 = FSL_BOOKE_MAS7(CONFIG_SYS_CCSRBAR_PHYS);

    write_tlb(mas0, mas1, mas2, mas3, mas7);

    gd->used_laws = 0;

    for (i = 0; i < c2x0_num_tlb_entries; i++) {
        write_tlb(c2x0_tlb_table[i].mas0,
              c2x0_tlb_table[i].mas1,
              c2x0_tlb_table[i].mas2,
              c2x0_tlb_table[i].mas3,
              c2x0_tlb_table[i].mas7);
    }

	return;
}

void c2x0_cpu_init_f(void)
{
	return; 
}

int c2x0_get_clocks (void)
{
	volatile ccsr_gur_t *gur = (void *)(CONFIG_SYS_MPC85xx_GUTS_ADDR);
	u32 plat_ratio;

	plat_ratio = in_be32(&gur->porpllsr) & MPC85xx_PORPLLSR_PLAT_RATIO;
	plat_ratio >>= 1;
	gd->bus_clk = CONFIG_SYS_CLK_FREQ * plat_ratio;
	gd->baudrate = CONFIG_BAUDRATE;
	return (1);
}

int c2x0_probecpu (void)
{
 #ifdef P1010 
    gd->cpu = &(c2x0_cpu_type_list[1]);
 #else
    /* default to be C2X0 */
    gd->cpu = &(c2x0_cpu_type_list[2]);
 #endif

    return 0;
}

void c2x0_set_tlb(u8 tlb, u32 epn, u64 rpn,
         u8 perms, u8 wimge,
         u8 ts, u8 esel, u8 tsize, u8 iprot)
{
    u32 _mas0, _mas1, _mas2, _mas3, _mas7;
	int i = esel / 32;
    int bit = esel % 32;

    if (tlb == 1)
	{
    	gd->used_tlb_cams[i] |= (1 << bit);
	}

    _mas0 = FSL_BOOKE_MAS0(tlb, esel, 0);
    _mas1 = FSL_BOOKE_MAS1(1, iprot, 0, ts, tsize);
    _mas2 = FSL_BOOKE_MAS2(epn, wimge);
    _mas3 = FSL_BOOKE_MAS3(rpn, 0, perms);
    _mas7 = FSL_BOOKE_MAS7(rpn);

    write_tlb(_mas0, _mas1, _mas2, _mas3, _mas7);

}

void c2x0_board_init_f(int dumy)
{
    /* Pointer is writable since we allocated a register for it */
    gd = (c2x0_gd_t *) (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_GBL_DATA_OFFSET);
    /* compiler optimization barrier needed for GCC >= 3.4 */
    __asm__ __volatile__("":::"memory");
   
	c2x0_probecpu();
	c2x0_get_clocks();

#ifdef PRINT_DEBUG
	c2x0_serial_init();
#endif
	print_debug("\t\t\t Uboot UP !!!!! Loading firmware.......\n");

	fsl_c2x0_fw( );
}

u64 get_tbclk (void)
{
#define	CONFIG_SYS_FSL_TBCLK_DIV	16

    u64 tbclk_div = CONFIG_SYS_FSL_TBCLK_DIV;

    return (gd->bus_clk + (tbclk_div >> 1)) / tbclk_div;
}

u64 usec2ticks(u64 usec)
{
    u64 ticks;

    if (usec < 1000) {
        ticks = ((usec * (get_tbclk()/1000)) + 500) / 1000;
    } else {
        ticks = ((usec / 10) * (get_tbclk() / 100000));
    }
    
    return (ticks);
}
