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


static int c2x0_cread_line(const char *const prompt, char *buf, unsigned int *len)
{
	unsigned long num = 0;
	unsigned long cwlen = 0;
	unsigned long eol_num = 0;
	unsigned long wlen;
	char ichar;
	int esc_len = 0;
/*	char esc_save[8];   */
	while (1) {
		ichar = c2x0_getc();
		if ((ichar == '\n') || (ichar == '\r')) {
			*len = eol_num;
			buf[eol_num] = '\0';	/* lose the newline */
			do_command(buf);
			c2x0_putc('\n');
			break;
		}
        switch (ichar) {
        case 0x1b:
            if (esc_len == 0) {
                /* esc_save[esc_len] = ichar;  */
                esc_len = 1;
            } else {
                esc_len = 0;
            }
            break;

        case 8:
            if (num) {
                wlen = eol_num - num;
                num--;
                c2x0_putc(CTL_BACKSPACE);
                c2x0_putc(' ');
                do {
                    c2x0_putc(CTL_BACKSPACE);
                } while (wlen--);
                eol_num--;
            }
            break;
		default:
			/*
		 	 * handle standard linux xterm esc sequences for arrow key, etc.
		 	 */
    		/* room ??? */
		    if (num == eol_num) {
        		if (eol_num > (unsigned long)(len - 1)) {
            		getcmd_cbeep();
            		return 0;
        		}
        		(eol_num)++;
    		}
    		cwlen = eol_num - num;
    		buf[num] = ichar;
    		c2x0_putc(ichar);
    		(num)++;
    		while (--cwlen) {
        		c2x0_putc(CTL_BACKSPACE);
    		}
			break;
		}
	}
	return 0;
}

static int  c2x0_get_user_input(struct c2x0_in_str *i)
{
    i->__promptme = 1;
	char console_buffer[C2X0_CONFIG_SYS_CBSIZE + 1];  /* console I/O buffer   */
    unsigned int len = C2X0_CONFIG_SYS_CBSIZE;
    int rc;
       
	console_buffer[0] = '\0'; 
	/* TODO : Nothing called puts here. need to define own */
    c2x0_puts(C2X0_CONFIG_SYS_PROMPT);
    rc = c2x0_cread_line(C2X0_CONFIG_SYS_PROMPT, console_buffer, &len);
    return rc < 0 ? rc : len;
}   
 
/* This is the magic location that prints prompts
 * and gets data back from the user */
static int c2x0_file_get(struct c2x0_in_str *i)
{
    int ch;

    ch = 0;
    /* If there is data waiting, eat it up */
    if (i->p && *i->p) {
        ch = *i->p++;
    } else {
        /* need to double check i->file because we might be doing something
         * more complicated by now, like sourcing or substituting. */
            while(! i->p ) {
                c2x0_get_user_input(i);
            }
            i->promptmode=2;
            if (i->p && *i->p) {
                ch = *i->p++;
            }
    }
    return ch;
}


static void c2x0_setup_file_in_str(struct c2x0_in_str *i)
{   
    i->get = c2x0_file_get;
    i->__promptme=1;
    i->promptmode=1;
    i->p = NULL;
}
/* most recursion does not come through here, the exeception is
 * from builtin_source() */
int c2x0_parse_stream_outer(struct c2x0_in_str *inp, int flag)
{

	unsigned int ch;
	inp->promptmode=1;
	while ((ch=b_getch(inp))!=EOF);

	return 0;
}


int c2x0_parse_file_outer(void)
{
    int rcode = 0;
    struct c2x0_in_str input; 
    c2x0_setup_file_in_str(&input);
    rcode = c2x0_parse_stream_outer(&input, C2X0_FLAG_PARSE_SEMICOLON);
    return rcode;
}

void c2x0_loop(void )
{
	unsigned int *addr = (unsigned int *)0xfff80000;
	unsigned int i = 0;
	char str[ ] = "One Convergence - C2X0";
#if 0
	unsigned int val;
	*addr = 0x44444444;
//	*addr = 0x12345678;
//	*(addr + 1) = 0x87654321;
	*(addr + 1) = 0x55555555;
	do {
//		*addr = 0x12345678;
//		*(addr + 1) = 0x87654321;
		val = *addr;

		*addr = 0x44444444;
		*(addr + 1) = 0x55555555;

	} while (1);
#else
	do {
		for(i=0; i<sizeof(str); i++)
			*((char*)addr + i) = str[i];
	}while(1);
#endif
	
}

void c2x0_code(void )
{
#if 0
	unsigned char *baddr = CONFIG_SYS_CCSRBAR;
	unsigned int *addr = NULL;
	unsigned int tlb_idx = 2;

	volatile unsigned char *obaddr  = 0xa0000000;
	volatile unsigned char *sramaddr = (unsigned int *)0xfff80000;
	unsigned int val;
	unsigned int i = 0;
	char str[ ] = "Ossommmmmm";
	char initstr[ ] = "hello";

	for(i=0;i<100;i++)
		*((char *)sramaddr + i) = 0;

	*((char *)sramaddr + i) = initstr[i]; ++i;

	/* Set tlb to PCIe 1 ob */
	//c2x0_set_tlb(1, 0xc0000000, 0xc00000000, MAS3_SW|MAS3_SR, MAS2_I|MAS2_G, 0, tlb_idx, BOOKE_PAGESZ_1G, 1);

	*((char *)sramaddr + i) = initstr[i]; ++i;

	/* Set law to PCIe 1 */
	addr = ((char *)baddr + 0xc28);
//	*addr = 0x00c00000;
	*addr = 0x000f0000;
	addr = ((char *)baddr + 0xc30);
	*addr = 0x802000B;

	*((char *)sramaddr + i) = initstr[i]; ++i;
#if 1
	/* Outbound window setting */
	addr = ((char *)baddr + 0xac20);
	*addr = 0;
	addr = ((char *)baddr + 0xac24);
	*addr = 0;
	addr = ((char *)baddr + 0xac28);
//	*addr = 0x00c00000;
//	*addr = 0x00c00000;
	addr = ((char *)baddr + 0xac30);
	*addr = 0x80044021;
#endif
	*((char *)sramaddr + i) = initstr[i]; ++i;

	sramaddr += sizeof(initstr);

	do {
		for(i=0; i<sizeof(str); i++) {
			*((char*)sramaddr + i) = str[i];
			*((char*)obaddr + i) = str[i];
		}

	}while(1);
#endif


/*	unsigned char *obaddr = 0xff000000; */
	unsigned char *obaddr = (unsigned char *)0x80000000; 
	unsigned char *sramaddr = (unsigned char *)0xfff80000;
	unsigned int *hsmem = (unsigned int *)0xfff00000 + ((1*1024*1024) - (28*1024) - (64));
	void *msi_addr = NULL;
	char str[ ] = "SUCCESS";
	int  i = 0;

	for(i=0; i<100; i++)
		*(sramaddr + i) = 0;

/*	sramaddr += sizeof(str); */

    do {
        for(i=0; i<sizeof(str); i++) {
            *((char*)sramaddr + i) = str[i];
            *((char*)obaddr + i) = str[i];
        }

    }while(0);


	/* Print the MSI address and MSI data */
	print_debug("H obmem L offset: %0x \n", *hsmem++);
	print_debug("H obmem H offset: %0x \n", *hsmem++);

	msi_addr = (void *) (0x90000000 + (*hsmem & 0x000fffff));
	print_debug("MSI Low  offset:%0x \n", *hsmem++);
	print_debug("MSI High offset:%0x \n", *hsmem++);
#define ASSIGN16_PTR(x,y)                out_le16(x,y);
#define ASSIGN16_PTR1(x,y)               out_be16(x,y);
	print_debug("MSI Addr: %0x \n", msi_addr);
	ASSIGN16_PTR(msi_addr, 0x4191);
	ASSIGN16_PTR1(msi_addr, 0x4191);
}

void c2x0_board_init_f(int dumy)
{
    /* Pointer is writable since we allocated a register for it */
    gd = (c2x0_gd_t *) (CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_GBL_DATA_OFFSET);
    /* compiler optimization barrier needed for GCC >= 3.4 */
    __asm__ __volatile__("":::"memory");
   
	c2x0_probecpu();
	c2x0_get_clocks();

/*	c2x0_loop(); */

	c2x0_serial_init();
	print_debug("\t\t\t Uboot UP !!!!! Loading firmware.......\n");

	fsl_c2x0_fw( );
/*
	print_debug("Debug value... :%d \n", gd->cie_idx);
	c2x0_code();
*/	
#if 0
	c2x0_parse_file_outer();
#endif
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
