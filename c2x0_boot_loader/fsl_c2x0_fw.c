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

#include <linux/compiler.h>
#include "uboot_common.h"
#include "uboot_print.h"
#include "fsl_c2x0_fw.h"

#define CCSR_VIRT_ADDR		0xffe00000
#define JRREGS_OFFSET		0x1000	/* Input Ring Base Address for JR 0 */
#define RNG_OFFSET		0x600	/* RNG TRNG Miscellaneous Control */
#define KEK_OFFSET		0x400	/* Key Encryption Key */
#define SCFG_OFFSET		0xC	/* Security Configuration */
#define RDSTA_OFFSET		0x6c0	/* RNG DRNG Status Register */
#define FAULTS_OFFSET		0xfc0	/* fault registers */

/*
 * To access a register in a specific SEC engine, use the following SEC
 * register base addresses:
 * SEC Register Descriptions
 * 8_0000h-9_FFFCh : SEC 1
 * A_0000h-B_FFFCh : SEC 2 (SEC 2 is not implemented in C291)
 * C_0000h-D_FFFCh : SEC 3 (SEC 3 is not implemented in C291 or C292)
 */
#define SEC0_OFFSET			0x80000
#define SEC0_BASE_ADDR		(CCSR_VIRT_ADDR + SEC0_OFFSET)
#define SEC0_JR0_ADDR		(SEC0_BASE_ADDR + JRREGS_OFFSET)
#define SEC0_RNG_ADDR		(SEC0_BASE_ADDR + RNG_OFFSET)
#define SEC0_KEK_ADDR		(SEC0_BASE_ADDR + KEK_OFFSET)
#define SEC0_SCFG_ADDR		(SEC0_BASE_ADDR + SCFG_OFFSET)
#define SEC0_RDSTA_ADDR		(SEC0_BASE_ADDR + RDSTA_OFFSET)
#define SEC0_FAULTS_ADDR	(SEC0_BASE_ADDR + FAULTS_OFFSET)

#define SEC1_OFFSET			0xa0000
#define SEC1_BASE_ADDR		(CCSR_VIRT_ADDR + SEC1_OFFSET)
#define SEC1_JR0_ADDR		(SEC1_BASE_ADDR + JRREGS_OFFSET)
#define SEC1_RNG_ADDR		(SEC1_BASE_ADDR + RNG_OFFSET)
#define SEC1_KEK_ADDR		(SEC1_BASE_ADDR + KEK_OFFSET)
#define SEC1_SCFG_ADDR		(SEC1_BASE_ADDR + SCFG_OFFSET)
#define SEC1_RDSTA_ADDR		(SEC1_BASE_ADDR + RDSTA_OFFSET)
#define SEC1_FAULTS_ADDR	(SEC1_BASE_ADDR + FAULTS_OFFSET)

#define SEC2_OFFSET			0xc0000
#define SEC2_BASE_ADDR		(CCSR_VIRT_ADDR + SEC2_OFFSET)
#define SEC2_JR0_ADDR		(SEC2_BASE_ADDR + JRREGS_OFFSET)
#define SEC2_RNG_ADDR		(SEC2_BASE_ADDR + RNG_OFFSET)
#define SEC2_KEK_ADDR		(SEC2_BASE_ADDR + KEK_OFFSET)
#define SEC2_SCFG_ADDR		(SEC2_BASE_ADDR + SCFG_OFFSET)
#define SEC2_RDSTA_ADDR		(SEC2_BASE_ADDR + RDSTA_OFFSET)
#define SEC2_FAULTS_ADDR	(SEC2_BASE_ADDR + FAULTS_OFFSET)

#define GUTS_SVR		(CCSR_VIRT_ADDR + 0xe0000 + 0xa4)

#define MCFGR_PS_SHIFT		16
#define MCFGR_PS_MASK		(1 << MCFGR_PS_SHIFT)
#define MCFGR_LARGE_BURST	0x4
#define JR_INTMASK			0x00000001

#define OB_TLB_SIZE_MASK    ((~(0)) << 30)
#define MSI_TLB_SIZE_MASK    ((~((u32)0)) << 20)

#define MIN(a,b) ((a)<(b) ? (a):(b))

#define BITS_PER_BYTE 8

#define RESET_REG_ADDR      (CCSR_VIRT_ADDR + 0Xe0000 + 0xb0)
#define RESET_REG_VALUE     0x2
#define RESET_PIC_PIR_ADDR  (CCSR_VIRT_ADDR + 0x40000 + 0x1090)
#define RESET_PIC_PIR_VALUE 0x1

/*
 * Note: all global variables must be initialized; any uninitialized variable
 * or variables initialized by zero are automatically placed in the .bss section
 * of the object file which currently is placed at load time in a memory
 * location that is not accessible via the LAW windows.
 * Remove this note when .bss section is moved to a memory area visible by LAWs.
 */

/*
 * Stack Pointer - start right below the firmware and is used to allocate
 * memory in a stack like manner; grows from top to bottom
 */
static u32 stack_ptr = PLATFORM_SRAM_VIRT_ADDR + PLATFORM_SRAM_SIZE
			- FIRMWARE_SIZE;

static inline void rng_processing(struct c_mem_layout *c_mem);
static inline void copy_kek_and_set_scr(struct c_mem_layout *c_mem);

static void Memset(u8 *ptr, u8 val, u32 size)
{
	u32 i = 0;
	for (i = 0; i < size; i++)
		*ptr++ = val;
}

void *c2alloc(size_t size)
{
	stack_ptr = ALIGN_TO_L1_CACHE_LINE(stack_ptr);
	stack_ptr -= size;
	stack_ptr = ALIGN_TO_L1_CACHE_LINE(stack_ptr);

	return (void *)stack_ptr;
}

void *c2zalloc(size_t size)
{
	void *addr;

	addr = c2alloc(size);
	Memset(addr, 0, size);

	return addr;
}

static void firmware_up(struct c_mem_layout *mem)
{
	mem->h_hs_mem->data.device.p_ib_mem_base_l = (u32) mem->p_ib_mem;
	mem->h_hs_mem->data.device.p_ib_mem_base_h = (u32) (mem->p_ib_mem >> 32);
	mem->h_hs_mem->data.device.p_ob_mem_base_l = (u32) mem->p_pci_mem;
	mem->h_hs_mem->data.device.p_ob_mem_base_h = (u32) (mem->p_pci_mem >> 32);
	mem->h_hs_mem->data.device.no_secs = (u32) mem->rsrc_mem->sec_eng_cnt;

	mem->h_hs_mem->state = FIRMWARE_UP;
	print_debug("\n\n\nFIRMWARE UP\n");

	SYNC_MEM;
}

static void init_ring_pairs(struct c_mem_layout *mem)
{
	u8 i;
	u8 num_of_rps = mem->rsrc_mem->num_of_rps;
	app_ring_pair_t *rps = mem->rsrc_mem->rps;

	mem->rsrc_mem->idxs_mem = c2zalloc(sizeof(indexes_mem_t) * num_of_rps);
	mem->rsrc_mem->cntrs_mem = c2zalloc(sizeof(counters_mem_t));
	mem->rsrc_mem->s_c_cntrs_mem = c2zalloc(sizeof(counters_mem_t));
	mem->rsrc_mem->r_cntrs_mem = c2zalloc(
			sizeof(ring_counters_mem_t) * num_of_rps);
	mem->rsrc_mem->r_s_c_cntrs_mem = c2zalloc(
			sizeof(ring_counters_mem_t) * num_of_rps);

	print_debug("Init Ring Pairs:\n");
	print_debug("Indexes mem       :%10p\n", mem->rsrc_mem->idxs_mem);
	print_debug("Counters mem      :%10p\n", mem->rsrc_mem->cntrs_mem);
	print_debug("S C Counters mem  :%10p\n", mem->rsrc_mem->s_c_cntrs_mem);
	print_debug("R counters mem    :%10p\n", mem->rsrc_mem->r_cntrs_mem);
	print_debug("R S C counters mem:%10p\n", mem->rsrc_mem->r_s_c_cntrs_mem);

	for (i = 0; i < num_of_rps; i++) {
		rps[i].req_r = NULL;
		rps[i].msi_addr = NULL;
		rps[i].sec = NULL;
		rps[i].r_s_cntrs = NULL;
		rps[i].idxs = &(mem->rsrc_mem->idxs_mem[i]);
		rps[i].r_cntrs = &(mem->rsrc_mem->r_cntrs_mem[i]);
		rps[i].r_s_c_cntrs = &(mem->rsrc_mem->r_s_c_cntrs_mem[i]);
		rps[i].ip_pool = mem->rsrc_mem->ip_pool;
		rps[i].intr_ctrl_flag = 0;
		rps[i].next = &rps[(i+1) % num_of_rps];

		print_debug("\nRing %d details\n", i);
		print_debug("\tIdxs addr  : %10p\n", rps[i].idxs);
		print_debug("\tR cntrs    : %10p\n", rps[i].r_cntrs);
		print_debug("\tR S C cntrs: %10p\n", rps[i].r_s_c_cntrs);
		print_debug("\tIp pool    : %10p\n", rps[i].ip_pool);
	}
}

static void init_shadow_counters(struct c_mem_layout *mem)
{
	u32 i;
	app_ring_pair_t *rps = mem->rsrc_mem->rps;

	print_debug("Init R S mem....\n");
	for (i = 0; i < mem->rsrc_mem->num_of_rps; i++) {
		rps[i].r_s_cntrs = &(mem->rsrc_mem->r_s_cntrs_mem[i]);
		print_debug("Ring %d        R S Cntrs: %0x\n", i, rps[i].r_s_cntrs);
	}
}

int hs_complete(struct c_mem_layout *mem)
{
	int hs_comp;

	mem->c_hs_mem->state = DEFAULT;
	mem->rsrc_mem->rps = mem->rsrc_mem->rps->next;

	print_debug("\nHS_COMPLETE:\n");

	if (in_be32(mem->rsrc_mem->sec->rdsta) & 0x1) {
		mem->h_hs_mem->result = RESULT_OK;
		mem->h_hs_mem->state = FW_RNG_COMPLETE;
		hs_comp = 1;
	} else {
		mem->h_hs_mem->result = RESULT_OK;
		mem->h_hs_mem->state = FW_INIT_RNG;
		hs_comp = 0;
	}
	return hs_comp;
}

uint32_t hs_fw_init_ring_pair(struct c_mem_layout *mem, uint32_t r_offset)
{

	u32 offset;
	u32 rid = mem->c_hs_mem->data.ring.rid;
	u32 msi_addr_l = mem->c_hs_mem->data.ring.msi_addr_l;
	app_ring_pair_t *rp = &(mem->rsrc_mem->rps[rid]);

	mem->c_hs_mem->state = DEFAULT;
	print_debug("\nFW_INIT_RING_PAIR\n");

	rp->id = rid;
	rp->depth = mem->c_hs_mem->data.ring.depth;
	rp->msi_data = mem->c_hs_mem->data.ring.msi_data;
	rp->msi_addr = (void*)(((u8 *)mem->v_msi_mem + ((mem->p_pci_mem + msi_addr_l) -
			mem->p_msi_mem)));
	rp->req_r = mem->rsrc_mem->req_mem + r_offset;
	r_offset += (rp->depth * sizeof(req_ring_t));
	rp->resp_r = (resp_ring_t *)((u8 *)mem->h_hs_mem + mem->c_hs_mem->data.ring.resp_ring_offset);

	print_debug("Rid:	%d\n", rid);
	print_debug("Depth:	%d\n", rp->depth);
	print_debug("MSI Data:	%0x\n",rp->msi_data);
	print_debug("MSI addr:	%0x\n", rp->msi_addr);
	print_debug("Req r addr: %0x\n", rp->req_r);
	print_debug("Resp r addr:%0x\n", rp->resp_r);

	offset = (u8 *) rp->req_r - (u8 *) mem->v_ib_mem;
	mem->h_hs_mem->data.ring.req_r = offset;

	offset = (u8 *) &(rp->intr_ctrl_flag) - (u8 *) mem->v_ib_mem;
	mem->h_hs_mem->data.ring.intr_ctrl_flag = offset;

	mem->h_hs_mem->result = RESULT_OK;
	mem->h_hs_mem->state = FW_INIT_RING_PAIR_COMPLETE;

	return r_offset;
}

void hs_fw_init_config(struct c_mem_layout *mem)
{
	u8 num_of_rps;
	u32 req_mem_size, r_s_cntrs, offset;

	mem->c_hs_mem->state = DEFAULT;
	print_debug("\nFW_INIT_CONFIG\n");

	num_of_rps = mem->c_hs_mem->data.config.num_of_rps;
	mem->rsrc_mem->num_of_rps = num_of_rps;
	print_debug("Number of ring pairs: %d\n", num_of_rps);

	mem->rsrc_mem->rps = c2alloc(num_of_rps * sizeof(app_ring_pair_t));
	init_ring_pairs(mem);

	req_mem_size = mem->c_hs_mem->data.config.req_mem_size;
	print_debug("Req mem size: %d\n", req_mem_size);

	mem->rsrc_mem->req_mem = c2alloc(req_mem_size);
	print_debug("Req mem addr: %0p\n", mem->rsrc_mem->req_mem);

	r_s_cntrs = mem->c_hs_mem->data.config.r_s_cntrs;
	mem->rsrc_mem->r_s_cntrs_mem = (ring_shadow_counters_mem_t *)
		((u8 *)mem->h_hs_mem + r_s_cntrs);

	print_debug("Shadow counters details from Host.\n");
	print_debug("R S CNTRS OFFSET: %10x\n", r_s_cntrs);
	print_debug("The same counters mapped in local addresses:\n");
	print_debug("R S CNTRS       : %10p\n\n", mem->rsrc_mem->r_s_cntrs_mem);

	init_shadow_counters(mem);

	/* communicate to the host the offsets of the counters allocated earlier */
	print_debug("\nSENDING FW_INIT_CONFIG_COMPLETE\n");
	offset = (u8 *)mem->rsrc_mem->r_s_c_cntrs_mem - (u8 *)mem->v_ib_mem;
	mem->h_hs_mem->data.config.r_s_c_cntrs = offset;
	print_debug("S R CNTRS OFFSET: %10x\n", offset);

	offset = (u8 *) mem->rsrc_mem->s_c_cntrs_mem - (u8 *) mem->v_ib_mem;
	mem->h_hs_mem->data.config.s_c_cntrs = offset;
	print_debug("S_CNTRS OFFSET  : %10x\n", offset);

	offset = (u8 *) mem->rsrc_mem->ip_pool - (u8 *) mem->v_ib_mem;
	mem->h_hs_mem->data.config.ip_pool = offset;
	print_debug("ip_pool         : %10x\n", offset);

	offset = (u8 *) &(mem->rsrc_mem->rps[0].intr_ctrl_flag) - (u8 *) mem->v_ib_mem;
	mem->h_hs_mem->data.config.resp_intr_ctrl_flag = offset;
	print_debug("intr_ctrl_flag  : %10x\n", offset);

	mem->h_hs_mem->result = RESULT_OK;
	mem->h_hs_mem->state = FW_INIT_CONFIG_COMPLETE;
}

static void handshake(struct c_mem_layout *mem)
{
	uint32_t r_offset = 0;
	print_debug("\nHANDSHAKE\n");
	print_debug("State address: %0x\n", &(mem->c_hs_mem->state));

	/* Mark the firmware up to the driver */
	firmware_up(mem);

	while (true) {
		/* wait for a notification from the host driver */
		while (mem->c_hs_mem->state == DEFAULT) {
			SYNC_MEM;
		}
		print_debug("State updated by driver: %d\n", mem->c_hs_mem->state);

		switch (mem->c_hs_mem->state) {
		case FW_INIT_CONFIG:
			hs_fw_init_config(mem);
			break;

		case FW_INIT_RING_PAIR:
			r_offset = hs_fw_init_ring_pair(mem, r_offset);
			break;

		case FW_HS_COMPLETE:
			if (hs_complete(mem))
				return;
			break;

		case FW_WAIT_FOR_RNG:
			/* print_debug("\n FW_WAIT_FOR_RNG\n"); */
			rng_processing(mem);
			break;

		case FW_RNG_DONE:
/*			print_debug("\n FW_RNG_DONE\n"); */
			copy_kek_and_set_scr(mem);
			return; 
		}
	}
}

int32_t sec_reset(ccsr_sec_t *sec)
{
	/*ccsr_sec_t *sec = (void *)CONFIG_SYS_FSL_SEC_ADDR;*/
	u32 mcfgr = in_be32(&sec->mcfgr);
	u32 timeout = 100000;

	mcfgr |= MCFGR_SWRST;
	out_be32(&sec->mcfgr, mcfgr);

	mcfgr |= MCFGR_DMA_RST;
	out_be32(&sec->mcfgr, mcfgr);
	do {
		mcfgr = in_be32(&sec->mcfgr);
	} while ((mcfgr & MCFGR_DMA_RST) == MCFGR_DMA_RST && --timeout);

	if (timeout == 0)
		return -1;

	timeout = 100000;
	do {
		mcfgr = in_be32(&sec->mcfgr);
	} while ((mcfgr & MCFGR_SWRST) == MCFGR_SWRST && --timeout);

	return ( timeout ? 0 : -1 );
}

static void sec_eng_hw_init(struct sec_engine *sec)
{
	u32 jrcfg;
	u32 mcr;

	phys_addr_t ip_r_base = 0;
	phys_addr_t op_r_base = 0;

	ip_r_base |= (u32) (sec->jr.i_ring);
	op_r_base |= (u32) (sec->jr.o_ring);

	sec->jr.size = SEC_JR_DEPTH;

	/* set the pointer size to be 36 bits */
	mcr = in_be32(&sec->info->mcfgr);
	mcr |= MCFGR_PS_MASK;
	mcr |= MCFGR_LARGE_BURST;
	out_be32(&sec->info->mcfgr, mcr);

	/* Initialising the jr regs */
	out_be32(&sec->jr.regs->irba_h, ip_r_base >> 32);
	out_be32(&sec->jr.regs->irba_l, (u32) ip_r_base);
	out_be32(&sec->jr.regs->orba_h, op_r_base >> 32);
	out_be32(&sec->jr.regs->orba_l, (u32) op_r_base);
	out_be32(&sec->jr.regs->ors, SEC_JR_DEPTH);
	out_be32(&sec->jr.regs->irs, SEC_JR_DEPTH);

	out_be32(&(sec->jr.regs->irja), 0);
	out_be32(&(sec->jr.regs->irsa), 0);
	out_be32(&(sec->jr.regs->orsf), 0);
	out_be32(&(sec->jr.regs->orjr), 0);

	sec->jr.head = 0;
	sec->jr.tail = 0;

	SYNC_MEM;

	/* disabling interrupt from SEC */
	jrcfg = in_be32(&sec->jr.regs->jrcfg1);
	jrcfg |= JR_INTMASK;
	out_be32(&sec->jr.regs->jrcfg1, jrcfg);

	SYNC_MEM;
	print_debug("Sec hw eng init done for id: %d\n\n", sec->id);

	return; 
}

static void make_sec_circ_list(struct sec_engine *sec, u8 count)
{
	u8 i;
	for (i = 0; i < count; i++) {
		sec[i].next = &(sec[(i + 1) % count]);
	}
}

static inline void copy_kek_and_set_scr(struct c_mem_layout *c_mem)
{
	u32 k_value, i, j;
	u8 sec_cnt;
	struct sec_engine *sec[3];
	struct kek_regs *kek[3];

	sec_cnt = c_mem->rsrc_mem->sec_eng_cnt;

	for (i = 0; i < sec_cnt; i += 1) {
		sec[i] = c_mem->rsrc_mem->sec + i;
		kek[i] = (struct kek_regs *)sec[i]->kek;
	}

	if ( 1 < sec_cnt) {
		for (i = 0; i < 24; i += 1) {
			k_value = in_be32((u32 *)(kek[0] + i));
			for (j = 1; j < sec_cnt; j +=1) 
				out_be32((u32 *)(kek[j] + i), k_value);
		}
	}
	
	for (i = 0; i < sec_cnt; i += 1)
		out_be32(sec[i]->scfg, 0x00000703);
}

static void init_rng(struct sec_engine *sec)
{
	u32 x;
	u32 delay;

	struct rng_regs *regs = (struct rng_regs *)sec->rng;

	/*
	 * Put TRNG (entropy generator or True RNG) into program mode
	 *  reset to default values
	 */
	out_be32(&regs->rtmctl, 0x00010000u);

	/*
	 * Set the entropy delay values to the three CAAMs
	 *  (Leave the SAMP_SIZE field set to 2500)
	 */
	switch (sec->id) {
	case SEC_ENG_1:
		delay = RNG4_ENT_DLY0;
		break;
	case SEC_ENG_2:
		delay = RNG4_ENT_DLY1;
		break;
	case SEC_ENG_3:
		delay = RNG4_ENT_DLY2;
		break;
	default:
		delay = RNG4_ENT_DLY0;
		break;

	}

	/* Set entropy delay */
	x = delay * ACTUAL_CLOCK / DEFAULT_CLOCK;
	out_be32(&regs->rtsdctl, ((x << 16) | 0x09C4u));
	/*
	 * Set the Frequency maximum
	 */
	out_be32(&regs->rtfreqmax, (delay * 8));
	/*
	 * Set the Frequency minimum
	 */
	out_be32(&regs->rtfreqmin, (delay / 2));

	/*
	 * Put the TRNG back into run mode
	 *  Clear any error that may have occured, just in case
	 *
	 * Note that putting the TRNG back into run mode will
	 *  cause it to automatically start generating entropy
	 *
	 * It will take about 30,000,000 clock cycles to generate entropy,
	 *  so this should be done early in the boot process.
	 */
	out_be32(&regs->rtmctl, 0x00001000u);

}

static void init_sec_regs_offset(struct sec_engine *sec)
{
	int32_t id = sec->id;

	switch (id) {
	case SEC_ENG_1:
		sec->jr.regs = (struct sec_jr_regs *) SEC0_JR0_ADDR;
		sec->info = (ccsr_sec_t *) SEC0_BASE_ADDR;
		sec->rng = (struct rng_regs *) SEC0_RNG_ADDR;
		sec->kek = (struct kek_regs *) SEC0_KEK_ADDR;
		sec->scfg = (u32 *) SEC0_SCFG_ADDR;
		sec->rdsta = (u32 *) SEC0_RDSTA_ADDR;
		sec->faults = (struct fault_regs *) SEC0_FAULTS_ADDR;
		break;

	case SEC_ENG_2:
		sec->jr.regs = (struct sec_jr_regs *) SEC1_JR0_ADDR;
		sec->info = (ccsr_sec_t *) SEC1_BASE_ADDR;
		sec->rng = (struct rng_regs *) SEC1_RNG_ADDR;
		sec->kek = (struct kek_regs *) SEC1_KEK_ADDR;
		sec->scfg = (u32 *) SEC1_SCFG_ADDR;
		sec->rdsta = (u32 *) SEC1_RDSTA_ADDR;
		sec->faults = (struct fault_regs *) SEC1_FAULTS_ADDR;
		break;

	case SEC_ENG_3:
		sec->jr.regs = (struct sec_jr_regs *) SEC2_JR0_ADDR;
		sec->info = (ccsr_sec_t *) SEC2_BASE_ADDR;
		sec->rng = (struct rng_regs *) SEC2_RNG_ADDR;
		sec->kek = (struct kek_regs *) SEC2_KEK_ADDR;
		sec->scfg = (u32 *) SEC2_SCFG_ADDR;
		sec->rdsta = (u32 *) SEC2_RDSTA_ADDR;
		sec->faults = (struct fault_regs *) SEC2_FAULTS_ADDR;
		break;

	default:
		print_error("\n Invalid Sec Id... :%d\n", id);
	}

	init_rng(sec);
}

static void init_rsrc_sec(struct sec_engine *sec)
{
	sec->jr.id = sec->id;
	sec->jr.i_ring = c2zalloc(SEC_JR_DEPTH * sizeof(struct sec_ip_ring));
	sec->jr.o_ring = c2zalloc(SEC_JR_DEPTH * sizeof(struct sec_op_ring));

	print_debug("sec ip ring: %10x\n", sec->jr.i_ring);
	print_debug("sec op ring: %10x\n", sec->jr.o_ring);

	/* Call for hardware init of sec engine */
	init_sec_regs_offset(sec);
	sec_eng_hw_init(sec);
}

static void alloc_rsrc_mem(struct c_mem_layout *c_mem)
{
	struct sec_engine *sec;
	int32_t i;
	u32 sec_nums;

	sec_nums = in_be32((u32 *)GUTS_SVR);
	sec_nums = (sec_nums & 0xF000) >> 12;
	if (sec_nums == 0) {
		sec_nums = 1;
	}

	c_mem->rsrc_mem = c2zalloc(sizeof(struct resource));
	c_mem->rsrc_mem->sec_eng_cnt = sec_nums;
	c_mem->rsrc_mem->sec = c2zalloc(sec_nums * sizeof(struct sec_engine));

	print_debug("\nalloc_rsrc_mem\n");
	print_debug("rsrc addr: %10p\n", c_mem->rsrc_mem);
	print_debug("sec addr : %10p\n\n", c_mem->rsrc_mem->sec);

	make_sec_circ_list(c_mem->rsrc_mem->sec, sec_nums);

	/* Call for hardware init of sec engine */
	sec = c_mem->rsrc_mem->sec;
	for (i = 0; i < sec_nums; i++) {
		sec->id = (i + 1);
		init_rsrc_sec(sec);
		sec = sec->next;
	}

	c_mem->free_mem = TOTAL_CARD_MEMORY - (0x100000000ull - stack_ptr);

#ifdef COMMON_IP_BUFFER_POOL
	c_mem->rsrc_mem->ip_pool = (void *)L2_SRAM_VIRT_ADDR;
	Memset(c_mem->rsrc_mem->ip_pool, 0, DEFAULT_POOL_SIZE);
	c_mem->free_mem -= DEFAULT_POOL_SIZE;
	print_debug("ip pool addr: %0x\n", c_mem->rsrc_mem->ip_pool);
#endif
}

#ifdef TIMED_WAIT_FOR_JOBS
#define WAIT_FOR_DRIVER_JOBS conditional_timed_wait_for_driver_jobs
static inline u32 conditional_timed_wait_for_driver_jobs(u32 *x, u32 *y)
{
	u64 start_ticks;
	u64 timeout_ticks;

	start_ticks = getticks();
	timeout_ticks = usec2ticks(HOST_JOB_WAIT_TIME_OUT);

	while ((getticks() - start_ticks < timeout_ticks)
	       && ((*x - *y) < BUDGET_NO_OF_TOT_JOBS)) {
		SYNC_MEM;
	}

	return *x - *y;
}
#else
#define WAIT_FOR_DRIVER_JOBS(x, y)	\
	while (BUDGET_NO_OF_TOT_JOBS > (x-y))	\
		 SYNC_MEM;

#endif

static inline void Enq_Cpy(struct sec_ip_ring *sec_i, req_ring_t *req_r,
		u32 count)
{
	while (count--)
		*(u64 *) sec_i++ = *(u64 *) req_r++;
}

static inline void Deq_Cpy(resp_ring_t *resp_r, struct sec_op_ring *sec_o,
			   u32 count)
{
	memcpy(resp_r, sec_o, (sizeof(resp_ring_t) * count));
}

static inline u32 sel_sec_enqueue(struct c_mem_layout *c_mem,
		struct sec_engine **psec, app_ring_pair_t *rp,  u32 *todeq)
{
	struct sec_engine *sec	= NULL;
	struct sec_jr *jr		= NULL;
	dma_addr_t desc		= 0;
	u64 sec_sel		= 0;
	u32 secroom		= 0;
	u32 wi			= 0;

	u32 ri			= rp->idxs->r_index;
	u32 sec_cnt		= c_mem->rsrc_mem->sec_eng_cnt;

	print_debug("%s( ): rp: %d ri: %d\n", __FUNCTION__, rp->id, rp->idxs->r_index);
	desc = rp->req_r[ri].desc;
	print_debug("%s( ): DESC: %0llx SEC number :%d\n", __FUNCTION__, desc, (desc & (u64) 0x03));

	sec_sel = (desc & (u64) 0x03);
	if (sec_cnt < sec_sel)
		sec_sel = 0; 

	switch (sec_sel) {
	case SEC_ENG_1:
		sec = c_mem->rsrc_mem->sec;
		break;
	case SEC_ENG_2:
		sec = c_mem->rsrc_mem->sec + 1;
		break;
	case SEC_ENG_3:
		sec = c_mem->rsrc_mem->sec + 2;
		break;
	default:
		sec = *psec;
		*psec = sec->next;
		break;
	}	

	jr = &(sec->jr);
	secroom = in_be32(&(jr->regs->irsa));
	if(!secroom)
		goto RET;

	wi = jr->tail;
	rp->req_r[ri].desc = desc & ~((u64) 0x03);
	jr->i_ring[wi].desc = rp->req_r[ri].desc;

	jr->enq_cnt += 1;

	jr->tail = MOD_ADD(wi, 1, jr->size);
	rp->idxs->r_index = MOD_ADD(ri, 1, rp->depth);

	rp->r_cntrs->jobs_processed += 1;
	rp->r_s_cntrs->req_jobs_processed = rp->r_cntrs->jobs_processed;
	out_be32(&(jr->regs->irja), 1);

RET:
	*todeq += 1;
	return 1;
}

static inline u32 sec_dequeue(struct c_mem_layout *c_mem,
		struct sec_engine **deq_sec, app_ring_pair_t *rp, u32 *todeq)
{
	struct sec_jr *jr = &(*deq_sec)->jr;
	u32 cnt = in_be32(&jr->regs->orsf);
	u32 room = 0;
	u32 wi = 0;
	u32 ri;
	u32 ret_cnt = 0;

	if (!cnt) {
		*deq_sec = (*deq_sec)->next;
		return 0;
	}

	ri = jr->head;

	while (cnt) {
		room = rp->depth -
			(rp->r_cntrs->jobs_added - rp->r_s_c_cntrs->jobs_processed);
		if (!room)
			return ret_cnt;

		wi = rp->idxs->w_index;
		Deq_Cpy(&(rp->resp_r[wi]), &jr->o_ring[ri], 1);
		rp->idxs->w_index = MOD_ADD(wi, 1, rp->depth);
		rp->r_cntrs->jobs_added += 1;

		ri = MOD_ADD(ri, 1, jr->size);
		jr->head = ri;
		jr->deq_cnt += 1;

		rp->r_s_cntrs->resp_jobs_added = rp->r_cntrs->jobs_added;

		out_be32(&jr->regs->orjr, 1);
		*todeq -= 1;

		--cnt;
		++ret_cnt;
	}

	return ret_cnt;
}

static inline void raise_intr(app_ring_pair_t *r)
{
	r->intr_ctrl_flag = 1;
	out_le32(r->msi_addr, r->msi_data);
}

inline void Enq_Circ_Cpy(struct sec_jr *jr, app_ring_pair_t *rp, uint32_t count)
{
	uint32_t i;
	uint32_t ri = rp->idxs->r_index;
	uint32_t wi = jr->tail;
	uint32_t rpdepth = rp->depth;
	uint32_t jrdepth = jr->size;

	for(i = 0; i < count; i++) {
		print_debug("%s: desc: %0llx from ri: %d to sec wi: %d \n",
				__func__, rp->req_r[ri].desc, ri, wi);
		jr->i_ring[wi].desc   = rp->req_r[ri].desc;
		ri = (ri+1) &~ rpdepth;
		wi = (wi+1) &~ jrdepth;
	}
	rp->idxs->r_index = ri;
	jr->tail          = wi;
}

static inline uint32_t enqueue_to_sec(struct sec_engine *sec,
		app_ring_pair_t *rp, uint32_t in_jobs)
{
	uint32_t secroom = in_be32(&(sec->jr.regs->irsa));
	uint32_t count = MIN(secroom, in_jobs);

	if(count > 0) {
		Enq_Circ_Cpy(&(sec->jr), rp, count);
		out_be32(&(sec->jr.regs->irja), count);
		rp->r_cntrs->jobs_processed += count;
		rp->r_s_cntrs->req_jobs_processed = rp->r_cntrs->jobs_processed;
	}
	return count;
}

inline void Deq_Circ_Cpy(struct sec_jr *jr, app_ring_pair_t *rp, uint32_t count)
{
	uint32_t i;
	uint32_t wi = rp->idxs->w_index;
	uint32_t ri = jr->head;
	uint32_t rdepth = rp->depth;
	uint32_t jrdepth = jr->size;

	for(i = 0; i < count; i++) {
		print_debug("%s: desc: %0llx from ri: %d to host wi: %d \n",
				__func__, jr->o_ring[ri].desc, ri, wi);
		rp->resp_r[wi].desc   = jr->o_ring[ri].desc;
		rp->resp_r[wi].result = jr->o_ring[ri].status;
		wi = (wi + 1) &~(rdepth);
		ri = (ri + 1) &~(jrdepth);
	}
	rp->idxs->w_index = wi;
	jr->head          = ri;
}

static inline uint32_t dequeue_from_sec(struct sec_engine *sec,
		app_ring_pair_t *rp)
{
	uint32_t out_jobs  = in_be32(&(sec->jr.regs->orsf));
	uint32_t hostroom = rp->depth - (rp->r_cntrs->jobs_added - rp->r_s_c_cntrs->jobs_processed);
	uint32_t count = MIN(out_jobs, hostroom);

	print_debug("%s: out_jobs: %d, hostroom: %d, count: %d \n",
			__func__, out_jobs, hostroom, count);

	if (count) {
		Deq_Circ_Cpy(&(sec->jr), rp, count);
		rp->r_cntrs->jobs_added += count;
		rp->r_s_cntrs->resp_jobs_added = rp->r_cntrs->jobs_added;
		out_be32(&(sec->jr.regs->orjr), count);
	}
	return count;
}

static inline void raise_intr_app_ring(app_ring_pair_t *rp)
{
	print_debug("%s: MSI addr: %0x, MSI data:%0x \n",
			__func__, rp->msi_addr, rp->msi_data);
	rp->intr_ctrl_flag = 1;
	out_le32(rp->msi_addr, rp->msi_data);
}

static inline uint32_t irq_is_due(uint32_t deq_cnt, app_ring_pair_t *rp,
				uint32_t irq_timeout)
{
	uint32_t raise_irq;
	uint32_t host_jobs = rp->r_cntrs->jobs_added - rp->r_s_c_cntrs->jobs_processed;

	raise_irq = (deq_cnt > 0) && (!rp->intr_ctrl_flag);
	raise_irq |= (host_jobs != 0 ) && (irq_timeout == 0);

	return raise_irq;
}

static void ring_processing_perf(struct c_mem_layout *c_mem)
{
	app_ring_pair_t *recv_r = c_mem->rsrc_mem->rps;
	app_ring_pair_t *resp_r = c_mem->rsrc_mem->rps;
	struct sec_engine *enq_sec = c_mem->rsrc_mem->sec;
	struct sec_engine *deq_sec = c_mem->rsrc_mem->sec;
	uint32_t deq_cnt;
	uint32_t enq_cnt;
	uint32_t irq_timeout = IRQ_TIMEOUT;
	uint32_t in_jobs;
	int32_t in_flight = 0;

	while (1) {
		deq_cnt = dequeue_from_sec(deq_sec, resp_r);
		in_flight -= deq_cnt;

		if (irq_is_due(deq_cnt, resp_r, irq_timeout)) {
			raise_intr_app_ring(resp_r);
			resp_r->intr_ctrl_flag = 1;
			irq_timeout = IRQ_TIMEOUT;
		} else if (irq_timeout > 0) {
			irq_timeout -= 1;
		} else {
			irq_timeout = 0;
		}

		in_jobs = recv_r->r_s_c_cntrs->jobs_added - recv_r->r_cntrs->jobs_processed;
		if (in_jobs > 0) {
			enq_cnt = enqueue_to_sec(enq_sec, recv_r, in_jobs);
			in_flight += enq_cnt;
		}

		/* change sec and rings */
		recv_r = recv_r->next;
		resp_r = resp_r->next;
		enq_sec = enq_sec->next;
		deq_sec = deq_sec->next;
	}
}

static inline void rng_processing(struct c_mem_layout *c_mem)
{
	u32     cnt =   0,  ring_jobs   =   0;

	app_ring_pair_t     *rp         =   c_mem->rsrc_mem->rps;
	struct sec_engine   *sec        =   c_mem->rsrc_mem->sec;
	u32                 *r_deq_cnt  =   NULL;
	u32                 deq         =   0;

	r_deq_cnt   =   &(rp->r_cntrs->jobs_processed);
	cnt = WAIT_FOR_DRIVER_JOBS(&(rp->r_s_c_cntrs->jobs_added), r_deq_cnt);
	if (cnt == 0) {
		return;
	}

	ring_jobs    =  sel_sec_enqueue(c_mem, &sec, rp, &deq);

DEQ:
	ring_jobs   =  sec_dequeue(c_mem, &sec, rp, &deq);
	if (ring_jobs) {
		raise_intr(&(c_mem->rsrc_mem->rps[0]));
	} else {
		goto DEQ;
	}
}

int32_t fsl_c2x0_fw(void)
{
	phys_addr_t p_addr;
	phys_addr_t p_aligned_addr;

	struct c_mem_layout *c_mem;
	struct dev_handshake_mem *c_hs_mem;

	print_debug("Allocation starts 28K below top: %x\n", stack_ptr);

	/* One cache line for handshake (HS) memory */
	c_hs_mem = c2alloc(L1_CACHE_LINE_SIZE);
	print_debug("\nDevice handshake memory: %p\n", c_hs_mem);

	c_mem = c2alloc(sizeof(struct c_mem_layout));
	c_mem->c_hs_mem = c_hs_mem;
	c_mem->v_ib_mem = L2_SRAM_VIRT_ADDR;
	c_mem->p_ib_mem = L2_SRAM_VIRT_ADDR;	/* Phy addr is same as v addr */
	/*PCIE1 controller physical address-outbound window will be set to 16G*/
	c_mem->p_pci_mem = CONFIG_SYS_PCIE1_MEM_PHYS;

	print_debug("\nc_mem_layout\n", c_mem);
	print_debug("c_mem    : %10p\n", c_mem);
	print_debug("c_hs_mem : %10p\n", c_mem->c_hs_mem);
	print_debug("v_ib_mem : %10x\n", c_mem->v_ib_mem);
	print_debug("p_ib_mem : %10llx\n", c_mem->p_ib_mem);
	print_debug("p_pci_mem: %10llx\n", c_mem->p_pci_mem);

	/* OB MEM
	 * Driver would have updated the offset of its created memory inside
	 * the outbound window. Calculate the address valid in our domain
	 * The offset will definitely be 1G aligned so simple addition should
	 * give the 1G aligned address
	 */
	p_addr = (phys_addr_t)c_mem->c_hs_mem->h_ob_mem_h << 32;
	p_addr |= c_mem->c_hs_mem->h_ob_mem_l;
	/* Since we have 1G TLB open for rings - get the 1G aligned
	 * address for this physical address */
	p_aligned_addr = p_addr & OB_TLB_SIZE_MASK;
	c_mem->p_ob_mem = c_mem->p_pci_mem + p_aligned_addr;
	c_mem->v_ob_mem = CONFIG_SYS_PCIE1_MEM_VIRT; /* TLB exist only for 1G */
	c_mem->h_hs_mem = (struct host_handshake_mem *)
			((u8 *)c_mem->v_ob_mem + (p_addr - p_aligned_addr));

	print_debug("\nOB MEM DETAILS\n");
	print_debug("Host ob mem l          : %10x\n", c_mem->c_hs_mem->h_ob_mem_l);
	print_debug("Host ob mem h          : %10x\n", c_mem->c_hs_mem->h_ob_mem_h);
	print_debug("Host ob mem 64 bit addr: %10llx\n", p_addr);
	print_debug("Host ob mem aligned 1G : %10llx\n", p_aligned_addr);
	print_debug("p_ob_mem               : %10llx\n", c_mem->p_ob_mem);
	print_debug("v_ob_mem               : %10x\n", c_mem->v_ob_mem);
	print_debug("h_hs_mem               : %10x\n", c_mem->h_hs_mem);

	/* MSI details */
	p_addr = (phys_addr_t)c_mem->c_hs_mem->h_msi_mem_h << 32;
	p_addr |= c_mem->c_hs_mem->h_msi_mem_l;
	/* Since we have 1M TLB open for MSI window -
	 * get the 1M aligned address for this physical address */
	p_aligned_addr = p_addr & MSI_TLB_SIZE_MASK;
	/* Physical address should be within 16G window */
	c_mem->p_msi_mem = c_mem->p_pci_mem + p_aligned_addr;
	c_mem->v_msi_mem = CONFIG_SYS_PCIE1_MSI_MEM_VIRT;

	print_debug("\nMSI DETAILS\n");
	print_debug("MSI mem l          : %10x\n", c_mem->c_hs_mem->h_msi_mem_l);
	print_debug("MSI mem h          : %10x\n", c_mem->c_hs_mem->h_msi_mem_h);
	print_debug("MSI mem 64 bit addr: %10llx\n", p_addr);
	print_debug("MSI aligned 1M     : %10llx\n", p_aligned_addr);
	print_debug("p_msi_mem          : %10llx\n", c_mem->p_msi_mem);
	print_debug("v_msi_mem          : %10x\n", c_mem->v_msi_mem);

	/* Set the TLB here for the OB mem 1G - Using TLB 3 */
	c2x0_set_tlb(1, c_mem->v_ob_mem, c_mem->p_ob_mem,
			MAS3_SW | MAS3_SR, MAS2_I | MAS2_G, 0, 3,
			BOOKE_PAGESZ_1G, 1);
	/* Set the TLB here for MSI 1M - Using TLB 2 */
	c2x0_set_tlb(1, c_mem->v_msi_mem, c_mem->p_msi_mem,
			MAS3_SX | MAS3_SW | MAS3_SR, MAS2_I | MAS2_G, 0, 2,
			BOOKE_PAGESZ_1M, 1);

	alloc_rsrc_mem(c_mem);

	/* Default the state */
	c_mem->c_hs_mem->state = DEFAULT;

	print_debug("\nTOTAL memory:\t%8d bytes\n", TOTAL_CARD_MEMORY);
	print_debug("FREE memory:\t%8d bytes\n", c_mem->free_mem);

	/* Start the handshake */
	handshake(c_mem);

	print_debug("\n\n\nFirmware up\n");
	ring_processing_perf(c_mem);
	return 0;
}
