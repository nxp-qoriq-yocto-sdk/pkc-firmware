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
#define JRREGS_OFFSET		0x1000
#define RNG_OFFSET			0x00600
#define KEK_OFFSET			0x00400
#define SCFG_OFFSET			0x0000C
#define RDSTA_OFFSET		0x006c0

#define SEC0_OFFSET			0x80000
#define SEC0_BASE_ADDR		(CCSR_VIRT_ADDR + SEC0_OFFSET)
#define SEC0_JR0_ADDR		(SEC0_BASE_ADDR + JRREGS_OFFSET)
#define SEC0_RNG_ADDR		(SEC0_BASE_ADDR + RNG_OFFSET)
#define SEC0_KEK_ADDR		(SEC0_BASE_ADDR + KEK_OFFSET)
#define SEC0_SCFG_ADDR		(SEC0_BASE_ADDR + SCFG_OFFSET)
#define SEC0_RDSTA_ADDR		(SEC0_BASE_ADDR + RDSTA_OFFSET)

#define SEC1_OFFSET			0xa0000
#define SEC1_BASE_ADDR		(CCSR_VIRT_ADDR + SEC1_OFFSET)
#define SEC1_JR0_ADDR		(SEC1_BASE_ADDR + JRREGS_OFFSET)
#define SEC1_RNG_ADDR		(SEC1_BASE_ADDR + RNG_OFFSET)
#define SEC1_KEK_ADDR		(SEC1_BASE_ADDR + KEK_OFFSET)
#define SEC1_SCFG_ADDR		(SEC1_BASE_ADDR + SCFG_OFFSET)
#define SEC1_RDSTA_ADDR		(SEC1_BASE_ADDR + RDSTA_OFFSET)

#define SEC2_OFFSET			0xc0000
#define SEC2_BASE_ADDR		(CCSR_VIRT_ADDR + SEC2_OFFSET)
#define SEC2_JR0_ADDR		(SEC2_BASE_ADDR + JRREGS_OFFSET)
#define SEC2_RNG_ADDR		(SEC2_BASE_ADDR + RNG_OFFSET)
#define SEC2_KEK_ADDR		(SEC2_BASE_ADDR + KEK_OFFSET)
#define SEC2_SCFG_ADDR		(SEC2_BASE_ADDR + SCFG_OFFSET)
#define SEC2_RDSTA_ADDR		(SEC2_BASE_ADDR + RDSTA_OFFSET)

#define MCFGR_PS_SHIFT		16
#define MCFGR_PS_MASK		(1 << MCFGR_PS_SHIFT)
#define MCFGR_LARGE_BURST	0x4
#define JR_INTMASK			0x00000001


#define WAIT_FOR_STATE_CHANGE(x)	{ while (DEFAULT == x) SYNC_MEM; }
#define MIN(a,b) ((a)<(b) ? (a):(b))
#define LINEAR_ROOM(wi, depth, room)     MIN((depth-wi), room)
#define MOD_INC(x, size)                 ((++x) & (size-1))

#define BITS_PER_BYTE 8

#ifdef P4080_EP_TYPE
#define RESET_REG_ADDR      0xfe0e00b0
#define RESET_REG_VALUE     0x2
#define RESET_PIC_PIR_ADDR  0xfe041090
#define RESET_PIC_PIR_VALUE 0x1
#elif C293_EP_TYPE
#define RESET_REG_ADDR      (CCSR_VIRT_ADDR + 0Xe0000 + 0xb0)
#define RESET_REG_VALUE     0x2
#define RESET_PIC_PIR_ADDR  (CCSR_VIRT_ADDR + 0x40000 + 0x1090)
#define RESET_PIC_PIR_VALUE 0x1
#endif

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

#if 0
/* Helper function which can be used to measure cpu ticks.
 * Not used now but may be in future for firmware
 * profiling.
 */
static u64 readtb(void)
{
	u32 tbl = 0, tbh = 0;
	u64 tb = 0;

	asm volatile ("mfspr %0, 526" : "=r" (tbl));
	asm volatile ("mfspr %0, 527" : "=r" (tbh));

	SYNC_MEM;
	tb = ((u64) tbh << 32) | tbl;

	return tb;
}
#endif

static void Memset(u8 *ptr, u8 val, u32 size)
{
	u32 i = 0;
	for (i = 0; i < size; i++)
		*ptr++ = val;
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

static void init_p_q(priority_q_t *p_q, u8 num)
{
	u8 i = 0;
	for (i = 0; i < (num - 1); i++) {
		p_q[i].ring = NULL;
		p_q[i].next = &(p_q[i + 1]);
	}
	p_q[i].ring = NULL;
	p_q[i].next = NULL;
}

static app_ring_pair_t *next_ring(app_ring_pair_t *rp)
{
    rp->c_link = ((rp->c_link + 1) % rp->max_next_link);
    return rp->rp_links[rp->c_link];
}

static inline void init_order_mem(struct c_mem_layout *mem)
{
	app_ring_pair_t *rp = mem->rsrc_mem->rps;
	app_ring_pair_t *rp_head = mem->rsrc_mem->rps;
/*	u32 total_rps = mem->rsrc_mem->ring_count; */

	print_debug("init_order_mem \n");
	print_debug("rp_head : %x, rp: %x\n", rp_head, rp);
	while(NULL != rp)
	{
		print_debug("First rp: %d\n", rp->id);
		/* Check if the rp is ordered */
		if(!((rp->props & APP_RING_PROP_ORDER_MASK)
                            >> APP_RING_PROP_ORDER_SHIFT))
		{
			print_debug("Order bit is not set for ring: %d\n", rp->id);
			goto NEXT_RP;
		}

		print_debug("Order bit is set for ring: %d\n", rp->id);
		rp->order_j_d_index = 0;
		stack_ptr -= (rp->depth / BITS_PER_BYTE);
		rp->resp_j_done_flag = (u8 *) stack_ptr;
		print_debug("Resp job done flag: %x\n", rp->resp_j_done_flag);
		Memset(rp->resp_j_done_flag, 0, (rp->depth/BITS_PER_BYTE));
NEXT_RP:
		print_debug("rp_head: %x, rp: %x\n", rp_head, rp);
		rp = next_ring(rp);
		if( rp_head == rp )
			rp = NULL;
		print_debug("rp_head: %x, next rp: %x\n", rp_head, rp);
	} 

	return;
}

static void init_rps(struct c_mem_layout *mem, u8 num, u8 respringcount)
{
	u8 i = 0;
	u8 j = 0;
/*	u8	*idxs = NULL, *cntrs = NULL, *s_c_cntrs = NULL; */
	app_ring_pair_t *rps = mem->rsrc_mem->rps;

	print_debug("Init Ring Pairs:\n");
	stack_ptr -=  (sizeof(indexes_mem_t) * (num+respringcount));
	mem->rsrc_mem->idxs_mem = (indexes_mem_t *) stack_ptr;
	Memset((u8 *)mem->rsrc_mem->idxs_mem, 0, (sizeof(indexes_mem_t) * (num + respringcount)));
	print_debug("Indexes mem: %0x\n", mem->rsrc_mem->idxs_mem);

	stack_ptr -=  (sizeof(counters_mem_t));
	mem->rsrc_mem->cntrs_mem = (counters_mem_t *) stack_ptr;
	Memset((u8 *)mem->rsrc_mem->cntrs_mem, 0, sizeof(counters_mem_t));
	print_debug("Counters mem: %0x\n", mem->rsrc_mem->cntrs_mem);

	stack_ptr -=  (sizeof(counters_mem_t));
	mem->rsrc_mem->s_c_cntrs_mem = (counters_mem_t *) stack_ptr;
	Memset((u8 *)mem->rsrc_mem->s_c_cntrs_mem, 0, sizeof(counters_mem_t));
	print_debug("S C Counters mem: %0x\n", mem->rsrc_mem->s_c_cntrs_mem);

	stack_ptr -=  (sizeof(ring_counters_mem_t) * (num+respringcount));
	mem->rsrc_mem->r_cntrs_mem = (ring_counters_mem_t *) stack_ptr;
	Memset((u8 *)mem->rsrc_mem->r_cntrs_mem, 0,
	       (sizeof(ring_counters_mem_t) * (num + respringcount)));
	print_debug("R counters mem: %0x\n", mem->rsrc_mem->r_cntrs_mem);

	stack_ptr -= (sizeof(ring_counters_mem_t) * (num+respringcount));
	mem->rsrc_mem->r_s_c_cntrs_mem = (ring_counters_mem_t *) stack_ptr;
	Memset((u8 *)mem->rsrc_mem->r_s_c_cntrs_mem, 0,
	       (sizeof(ring_counters_mem_t) * (num + respringcount)));
	print_debug("R S C counters mem: %0x\n", mem->rsrc_mem->r_s_c_cntrs_mem);

	for (i = 0; i < num; i++) {
		rps[i].req_r = NULL;
		rps[i].msi_addr = NULL;
		rps[i].sec = NULL;
		rps[i].r_s_cntrs = NULL;

		rps[i].idxs = &(mem->rsrc_mem->idxs_mem[i]);
		rps[i].cntrs = &(mem->rsrc_mem->r_cntrs_mem[i]);
		rps[i].r_s_c_cntrs = &(mem->rsrc_mem->r_s_c_cntrs_mem[i]);
		rps[i].ip_pool = mem->rsrc_mem->ip_pool;
		rps[i].intr_ctrl_flag = 0;

		rps[i].next = NULL;

		for(j=0; j<FSL_CRYPTO_MAX_RING_PAIRS; j++)
			rps[i].rp_links[j] = NULL;

		print_debug("Ring: %d details\n", i);
		print_debug("\tIdxs addr: %0x\n", rps[i].idxs);
		print_debug("\tCntrs: %0x\n", rps[i].cntrs);
		print_debug("\tR S C cntrs: %0x\n", rps[i].r_s_c_cntrs);
		print_debug("\tIp pool: %0x\n", rps[i].ip_pool);
	}
}

static void init_drv_resp_ring(struct c_mem_layout *mem, u32 offset, u32 depth, u8 count)
{
    u8 loc = mem->rsrc_mem->ring_count;
    drv_resp_ring_t *ring   =   NULL;
    uint8_t i;

    for(i=0; i<count; i++) {
        ring    =   &(mem->rsrc_mem->drv_resp_ring[i]);

        ring->id            =   i;
        ring->msi_data      =   0;
        ring->intr_ctrl_flag=   0;
        ring->msi_addr      =   ring->r_s_cntrs     =   NULL;
        ring->depth         =   depth;
        ring->resp_r        =   (resp_ring_t *)((u8 *)mem->h_hs_mem + offset);
        ring->idxs          =   &(mem->rsrc_mem->idxs_mem[loc + i]);
        ring->r_cntrs       =   &(mem->rsrc_mem->r_cntrs_mem[loc + i]);
        ring->r_s_c_cntrs   =   &(mem->rsrc_mem->r_s_c_cntrs_mem[loc + i]);

        print_debug("Init Drv Resp Ring:\n");
        print_debug("\tResp ring addr:	%0x\n", ring->resp_r);
        print_debug("\tIndexes addr:	%0x\n", ring->idxs);
        print_debug("\tR Cntrs addr:	%0x\n", ring->r_cntrs);
        print_debug("\tR S C cntrs addr:%0x\n", ring->r_s_c_cntrs);
        print_debug("\tDepth:		%d\n", ring->depth);

        offset += (depth * sizeof(resp_ring_t));
    }
}

static void make_drv_resp_ring_circ_list(struct c_mem_layout *mem, u32 count)
{
    i32 i = 0;

    for(i=1; i<count; i++) {
        mem->rsrc_mem->drv_resp_ring[i-1].next = &mem->rsrc_mem->drv_resp_ring[i];
    }
    mem->rsrc_mem->drv_resp_ring[i-1].next = &mem->rsrc_mem->drv_resp_ring[0];
}

static void init_scs(struct c_mem_layout *mem)
{
	u32 i = 0;
	app_ring_pair_t *rps = mem->rsrc_mem->rps;
	print_debug("Init R S mem....\n");
	for (i = 0; i < mem->rsrc_mem->ring_count; i++) {
		rps[i].r_s_cntrs = &(mem->rsrc_mem->r_s_cntrs_mem[i]);
		print_debug("Ring: %d	R S Cntrs: %0x\n", i, rps[i].r_s_cntrs);
	}

	mem->rsrc_mem->drv_resp_ring->r_s_cntrs = &(mem->rsrc_mem->r_s_cntrs_mem[i]);
	print_debug("Driver resp ring R S Cntrs: %0x\n", mem->rsrc_mem->drv_resp_ring->r_s_cntrs);
}

static void add_ring_to_pq(priority_q_t *p_q, app_ring_pair_t *rp, u8 pri)
{
	app_ring_pair_t *cursor = p_q[pri].ring;
	print_debug("Pri:%d\tp_q: %0x\trp: %0x\tcursor: %0x\n",
			pri, p_q, rp, cursor);

	if (!rp->id)
		return;

	if (!cursor) {
		p_q[pri].ring = rp;
	} else {
		while (cursor->next)
			cursor = cursor->next;
		cursor->next = rp;
		while (cursor->rp_links[0])
			cursor = cursor->rp_links[0];
		cursor->rp_links[0] = rp;
	}
	rp->prio = pri;
}

static void make_rp_circ_list(struct c_mem_layout *mem)
{
	priority_q_t *p_q = mem->rsrc_mem->p_q;
	app_ring_pair_t *r = NULL;

	while (p_q) {
		r = p_q->ring;
		while (r->next)
			r = r->next;
		p_q = p_q->next;
		if (p_q)
			r->next = p_q->ring;
	}
	r->next = mem->rsrc_mem->p_q->ring;
	mem->rsrc_mem->rps = r;
}

static void make_rp_prio_links(struct c_mem_layout *mem)
{
	u8 i = 0;
	u8 max_pri = 0;
	priority_q_t *p_q = mem->rsrc_mem->p_q;
	priority_q_t *p_q_n = mem->rsrc_mem->p_q;
	app_ring_pair_t *r = NULL;
	app_ring_pair_t *r_next = NULL;
	app_ring_pair_t *r_head = mem->rsrc_mem->p_q->ring;

	while( NULL != p_q_n ) {
		max_pri++;
		p_q_n = p_q_n->next;
	}

	while (p_q) {
		r = p_q->ring;
		p_q_n = p_q->next;
		while(r) {
			r_next = r->rp_links[0];
			if( NULL != r_next ) {
				for(i=0; i<max_pri; i++)
					r->rp_links[i] = r_next;
				r->max_next_link = max_pri;
			} else {
				for(i=0; i<max_pri; i++) {
					if( 0 == i ) {
						r->rp_links[i] = r_head;
					} else {
						if(p_q_n)
							r->rp_links[i] = p_q_n->ring;
						else
							r->rp_links[i] = r_head;
					}
				}
				r->max_next_link = max_pri;
				max_pri--;
			}
			r = r_next;
		}
		p_q = p_q->next;
	}
	mem->rsrc_mem->rps = r_head;
}

int hs_complete(struct c_mem_layout *mem)
{
	int hs_comp;

	mem->c_hs_mem->state = DEFAULT;
	mem->rsrc_mem->drv_resp_ring->msi_addr = mem->rsrc_mem->rps[0].msi_addr;
	mem->rsrc_mem->drv_resp_ring->msi_data = mem->rsrc_mem->rps[0].msi_data;

	mem->rsrc_mem->cmdrp = mem->rsrc_mem->rps;
	mem->rsrc_mem->rps = mem->rsrc_mem->rps->next;
	make_rp_prio_links(mem);
	make_rp_circ_list(mem);

	stack_ptr = ALIGN_TO_L1_CACHE_LINE_REV(stack_ptr);
	init_order_mem(mem);

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
	mem->c_hs_mem->state = DEFAULT;
	print_debug("\nFW_INIT_RING_PAIR\n");
	{
	u32 rid = mem->c_hs_mem->data.ring.rid;
	u32 prio = (mem->c_hs_mem->data.ring.props & APP_RING_PROP_PRIO_MASK)
			>> APP_RING_PROP_PRIO_SHIFT;
	u32 msi_addr_l = mem->c_hs_mem->data.ring.msi_addr_l;
	app_ring_pair_t *rp = &(mem->rsrc_mem->rps[rid]);

	rp->id = rid;
	rp->props = mem->c_hs_mem->data.ring.props;
	rp->depth = mem->c_hs_mem->data.ring.depth;
	rp->msi_data = mem->c_hs_mem->data.ring.msi_data;
	rp->msi_addr = (void*)(((u8 *)mem->v_msi_mem + ((mem->p_pci_mem + msi_addr_l) -
			mem->p_msi_mem)));
	rp->req_r = mem->rsrc_mem->req_mem + r_offset;
	r_offset += (rp->depth * sizeof(req_ring_t));
	rp->resp_r = (resp_ring_t *)((u8 *)mem->h_hs_mem + mem->c_hs_mem->data.ring.resp_ring_offset);

	print_debug("Rid:	%d\n", rid);
	print_debug("Order:	%d\n",
		(mem->c_hs_mem->data.ring.props & APP_RING_PROP_ORDER_MASK) >>
		APP_RING_PROP_ORDER_SHIFT);
	print_debug("Prio:	%d\n", prio);
	print_debug("Depth:	%d\n", rp->depth);
	print_debug("MSI Data:	%0x\n",rp->msi_data);
	print_debug("MSI addr:	%0x\n", rp->msi_addr);
	print_debug("Req r addr: %0x\n", rp->req_r);
	print_debug("Resp r addr:%0x\n", rp->resp_r);

	add_ring_to_pq(mem->rsrc_mem->p_q, rp, (prio - 1));

	{
	u32 offset = 0;

	offset = (u8 *) rp->req_r - (u8 *) mem->v_ib_mem;
	mem->h_hs_mem->data.ring.req_r = offset;
	offset = (u8 *) &(rp->intr_ctrl_flag) - (u8 *) mem->v_ib_mem;
	mem->h_hs_mem->data.ring.intr_ctrl_flag = offset;
	}
	}
	mem->h_hs_mem->result = RESULT_OK;
	mem->h_hs_mem->state = FW_INIT_RING_PAIR_COMPLETE;
	/*c2x0_getc();*/

	return r_offset;
}

void hs_fw_init_config(struct c_mem_layout *mem)
{
	u8 max_pri, max_rps, respr_count, count;
	u32 req_mem_size, resp_ring_off, depth, s_cntrs, r_s_cntrs, offset;

	mem->c_hs_mem->state = DEFAULT;
	print_debug("\nFW_INIT_CONFIG\n");

	mem->rsrc_mem->ring_count = mem->c_hs_mem->data.config.num_of_rps;
	max_pri = mem->c_hs_mem->data.config.max_pri;
	print_debug("Max pri: %d\n", max_pri);

	stack_ptr = ALIGN_TO_L1_CACHE_LINE_REV(stack_ptr);
	stack_ptr -= max_pri * sizeof(priority_q_t);

	/* Alloc memory for prio q first */
	mem->rsrc_mem->p_q = (priority_q_t *) stack_ptr;
	init_p_q(mem->rsrc_mem->p_q, max_pri);


	max_rps = mem->c_hs_mem->data.config.num_of_rps;
	mem->rsrc_mem->ring_count = max_rps;
	print_debug("Max rps: %d\n", max_rps);

	stack_ptr = ALIGN_TO_L1_CACHE_LINE_REV(stack_ptr);
	stack_ptr -= (max_rps * sizeof(app_ring_pair_t));

	mem->rsrc_mem->rps = (app_ring_pair_t *) stack_ptr;
	mem->rsrc_mem->orig_rps = mem->rsrc_mem->rps;
	respr_count = mem->c_hs_mem->data.config.num_of_fwresp_rings;
	init_rps(mem, max_rps, respr_count);


	req_mem_size = mem->c_hs_mem->data.config.req_mem_size;
	print_debug("Req mem size: %d\n", req_mem_size);

	stack_ptr = ALIGN_TO_L1_CACHE_LINE_REV(stack_ptr);
	stack_ptr -=  req_mem_size;

	mem->rsrc_mem->req_mem = (void *) stack_ptr;
	print_debug("Req mem addr: %0x\n", mem->rsrc_mem->req_mem);


	resp_ring_off = mem->c_hs_mem->data.config.fw_resp_ring;
	depth = mem->c_hs_mem->data.config.fw_resp_ring_depth;
	count = mem->c_hs_mem->data.config.num_of_fwresp_rings;
	print_debug("Resp ring off: %0x\n", resp_ring_off);

	stack_ptr = ALIGN_TO_L1_CACHE_LINE_REV(stack_ptr);
	stack_ptr -= count * sizeof(drv_resp_ring_t);

	mem->rsrc_mem->drv_resp_ring_count = count;
	mem->rsrc_mem->drv_resp_ring = (drv_resp_ring_t *) stack_ptr;

	init_drv_resp_ring(mem, resp_ring_off, depth, count);
	make_drv_resp_ring_circ_list(mem, count);

	s_cntrs = mem->c_hs_mem->data.config.s_cntrs;
	r_s_cntrs = mem->c_hs_mem->data.config.r_s_cntrs;
	mem->rsrc_mem->s_cntrs_mem = (shadow_counters_mem_t *)
		((u8 *)mem->h_hs_mem + s_cntrs);
	mem->rsrc_mem->r_s_cntrs_mem = (ring_shadow_counters_mem_t *)
		((u8 *)mem->h_hs_mem + r_s_cntrs);

	print_debug("Shadow counters details from Host.\n");
	print_debug("S CNTRS OFFSET: %0x\n", s_cntrs);
	print_debug("R S CNTRS OFFSET: %0x\n", r_s_cntrs);
	init_scs(mem);

	print_debug("\nSENDING FW_INIT_CONFIG_COMPLETE\n");
	offset = (u8 *)mem->rsrc_mem->r_s_c_cntrs_mem - (u8 *)mem->v_ib_mem;
	mem->h_hs_mem->data.config.s_r_cntrs = offset;
	print_debug("S R CNTRS OFFSET: %0x\n", offset);

	offset = (u8 *) mem->rsrc_mem->s_c_cntrs_mem - (u8 *) mem->v_ib_mem;
	mem->h_hs_mem->data.config.s_cntrs = offset;
	print_debug("CNTRS OFFSET: %0x\n", offset);

	offset = (u8 *) mem->rsrc_mem->ip_pool - (u8 *) mem->v_ib_mem;
	mem->h_hs_mem->data.config.ip_pool = offset;

	offset = (u8 *) &(mem->rsrc_mem->drv_resp_ring->intr_ctrl_flag) - (u8 *) mem->v_ib_mem;
	mem->h_hs_mem->data.config.resp_intr_ctrl_flag = offset;

	mem->h_hs_mem->result = RESULT_OK;
	mem->h_hs_mem->state = FW_INIT_CONFIG_COMPLETE;

	/*c2x0_getc();*/
}

static void handshake(struct c_mem_layout *mem)
{
	uint32_t r_offset = 0;
	print_debug("\nHANDSHAKE\n");
	print_debug("State address: %0x\n", &(mem->c_hs_mem->state));

	/* Mark the firmware up to the driver */
	firmware_up(mem);

	while (true) {
		WAIT_FOR_STATE_CHANGE(mem->c_hs_mem->state);
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

i32 sec_reset(ccsr_sec_t *sec)
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
	print_debug("\nSec hw eng init done for id: %d....\n", sec->id);

	return; 
}

static void make_sec_circ_list(struct sec_engine *sec, u8 count)
{
	i32 i = 0;
	for (i = 0; i < (count - 1); i++)
		sec[i].next = &(sec[i + 1]);
	sec[i].next = &sec[0];
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
	i32 id = sec->id;

	switch (id) {
	case SEC_ENG_1:
		sec->jr.regs = (struct sec_jr_regs *) SEC0_JR0_ADDR;
		sec->info = (ccsr_sec_t *) SEC0_BASE_ADDR;
		sec->rng = (struct rng_regs *) SEC0_RNG_ADDR;
		sec->kek = (struct kek_regs *) SEC0_KEK_ADDR;
		sec->scfg = (u32 *) SEC0_SCFG_ADDR;
		sec->rdsta = (u32 *) SEC0_RDSTA_ADDR;
		break;

	case SEC_ENG_2:
		sec->jr.regs = (struct sec_jr_regs *) SEC1_JR0_ADDR;
		sec->info = (ccsr_sec_t *) SEC1_BASE_ADDR;
		sec->rng = (struct rng_regs *) SEC1_RNG_ADDR;
		sec->kek = (struct kek_regs *) SEC1_KEK_ADDR;
		sec->scfg = (u32 *) SEC1_SCFG_ADDR;
		sec->rdsta = (u32 *) SEC1_RDSTA_ADDR;
		break;

	case SEC_ENG_3:
		sec->jr.regs = (struct sec_jr_regs *) SEC2_JR0_ADDR;
		sec->info = (ccsr_sec_t *) SEC2_BASE_ADDR;
		sec->rng = (struct rng_regs *) SEC2_RNG_ADDR;
		sec->kek = (struct kek_regs *) SEC2_KEK_ADDR;
		sec->scfg = (u32 *) SEC2_SCFG_ADDR;
		sec->rdsta = (u32 *) SEC2_RDSTA_ADDR;
		break;

	default:
		print_error("\n Invalid Sec Id... :%d\n", id);
	}

	init_rng(sec);
}

static u32 init_rsrc_sec(struct sec_engine *sec)
{
	u32 mem = 0;

	sec->jr.id = sec->id;
	stack_ptr = ALIGN_TO_L1_CACHE_LINE_REV(stack_ptr);
	stack_ptr -=  ALIGN_TO_L1_CACHE_LINE((SEC_JR_DEPTH *
			sizeof(struct sec_ip_ring)));
	sec->jr.i_ring = (struct sec_ip_ring *) stack_ptr;

	mem += SEC_JR_DEPTH * sizeof(struct sec_ip_ring);
	Memset((u8 *)sec->jr.i_ring, 0, (SEC_JR_DEPTH *
			sizeof(struct sec_ip_ring)));
	print_debug("sec ip ring: %0x\n", sec->jr.i_ring);

	stack_ptr = ALIGN_TO_L1_CACHE_LINE_REV(stack_ptr);
	stack_ptr -=  ALIGN_TO_L1_CACHE_LINE((SEC_JR_DEPTH *
			sizeof(struct sec_op_ring)));
	sec->jr.o_ring = (struct sec_op_ring *)stack_ptr;
	mem += SEC_JR_DEPTH * sizeof(struct sec_op_ring);
	Memset((u8 *)sec->jr.o_ring, 0, (SEC_JR_DEPTH * sizeof(struct sec_op_ring)));
	print_debug("sec op ring: %0x\n", sec->jr.o_ring);

	/* Call for hardware init of sec engine */
	init_sec_regs_offset(sec);
	sec_eng_hw_init(sec);

	return mem;
}

static void alloc_rsrc_mem(struct c_mem_layout *c_mem)
{
	struct resource *rsrc  = c_mem->rsrc_mem;
	struct sec_engine *sec = NULL;
	i32 i            = 0;
	u32 sec_nums     = 0;
	u32 *dev_id_addr = NULL;

	print_debug("\nalloc_rsrc_mem\n");
	print_debug("rsrc addr: %0x\n", rsrc);

	Memset((u8 *)rsrc, 0, sizeof(struct resource));

	dev_id_addr = (u32 *) (CCSR_VIRT_ADDR + 0xe0000 + 0xa4);
	sec_nums = in_be32(dev_id_addr);
	sec_nums = (sec_nums & 0xF000) >> 12;
	if (!sec_nums)
		sec_nums = 1;

	rsrc->sec_eng_cnt = sec_nums;

	/* Initialize the SEC engine
	 * All the required memory for SEC engine will be allocated in L2 SRAM
	 * Max we may need = 3sec engines * (sizeof(struct sec_engine)) --
	 * Given 128 as depth of rings the max size required is
	 * approx 2624 bytes.
	 */
	stack_ptr -= ALIGN_TO_L1_CACHE_LINE(
					(sec_nums * sizeof(struct sec_engine)));
	rsrc->sec = (struct sec_engine *) (stack_ptr);
	Memset((u8 *)rsrc->sec, 0, sizeof(struct sec_engine) * sec_nums);
	print_debug("sec addr: %0x\n", rsrc->sec);

	c_mem->free_mem -= sec_nums * sizeof(struct sec_engine);
	make_sec_circ_list(rsrc->sec, sec_nums);

	/* Call for hardware init of sec engine */
	sec = rsrc->sec;
	for (i = 0; i < sec_nums; i++) {
		sec->id = (i + 1);
		c_mem->free_mem -= init_rsrc_sec(sec);
		sec = sec->next;
	}

#ifdef COMMON_IP_BUFFER_POOL
	rsrc->ip_pool = (void *)(L2_SRAM_VIRT_ADDR);
	Memset(rsrc->ip_pool, 0, DEFAULT_POOL_SIZE);
	c_mem->free_mem -= (DEFAULT_POOL_SIZE);
	print_debug("ip pool addr: %0x\n", rsrc->ip_pool);
#endif
}

#ifdef P4080_EP_TYPE
static void fix_p4080_reg_settings(void)
{
	volatile u32 *ccsr = (u32 *)0xfe000000;

	/* It is observed that P4080 device u-boot is over-riding the
	 * following settings :-
	 * Modifies the size of LAW for PCIe controller 1 to be 512MB
	 * - This should be 1G
	 * Creates a new LAW for PCIe controller 2 - This should be disabled
	 *  Disables inbound address translation address registers.
	 *  Disables outbound address translation address registers.
	 */
	/* Hence to make our driver and firmware work properly -
	 * Setting back those registers to proper value. */
	/* 1) Disabling PCIe controller 2 LAW */
	ccsr = (u32 *) (0xfe000c88);
	*ccsr = 0;
	SYNC_MEM;
	/* 2) Modifying the PCIe controller 1 LAW to be of 16G size
	 * PCIe controller 1 LAW will start from 0XA00000000
	 * instead of known 0XC00000000 */
	ccsr = (u32 *) (0xfe000c60);
	*ccsr = 0X0000000c;
	ccsr = (u32 *) (0xfe000c68);
	*ccsr = 0x80000021;
	SYNC_MEM;
	/* Enabling the inbound address translation register */
	ccsr = (u32 *) (0xfe200de0);
	*ccsr = 0x000fff00;
	SYNC_MEM;
	ccsr = (u32 *) (0xfe200df0);
	*ccsr = 0xa0f55013;
	SYNC_MEM;
	/* Enabling the outbound translation address register */
	ccsr = (u32 *) (0xfe200c20);
	*ccsr = 0x00;
	SYNC_MEM;
	ccsr = (u32 *) (0xfe200c28);
	*ccsr = 0X00c00000;
	SYNC_MEM;
	ccsr = (u32 *) (0xfe200c30);
	*ccsr = 0x80044021;
	SYNC_MEM;
}
#endif

/* Switch controls */
#define TIMED_WAIT_FOR_JOBS

#define BUDGET_NO_OF_TOT_JOBS                   50

#ifndef TIMED_WAIT_FOR_JOBS
#define WAIT_FOR_DRIVER_JOBS(x, y)	\
	while (BUDGET_NO_OF_TOT_JOBS > (x-y))	\
		 SYNC_MEM;
#else
#define WAIT_FOR_DRIVER_JOBS		\
	conditional_timed_wait_for_driver_jobs
#endif

#define MOD_ADD(x, value, size)		((x+value) & (size-1))

#ifdef TIMED_WAIT_FOR_JOBS
static inline u32 wait_for_timeout(u64 usecs)
{
	u64 start_ticks = 0;
	u64 timeout_ticks = 0;

	start_ticks = getticks();
	timeout_ticks = usec2ticks(usecs);

	while (getticks() - start_ticks < timeout_ticks)
		;

	return 0;
}

static inline u32 conditional_timed_wait_for_driver_jobs(u32 *x, u32 *y)
{
	u64 start_ticks = 0;
	u64 timeout_ticks = 0;

	start_ticks = getticks();
#define HOST_JOB_WAIT_TIME_OUT  100000ull
	timeout_ticks = usec2ticks(HOST_JOB_WAIT_TIME_OUT);

	while ((getticks() - start_ticks < timeout_ticks)
	       && ((*x - *y) < BUDGET_NO_OF_TOT_JOBS)) {
		SYNC_MEM;
	}

	return *x - *y;
}

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

inline i32 circ_room(u32 wi, u32 ri, u32 w_depth, u32 r_depth, u32 count)
{
	i32 val1 = LINEAR_ROOM(wi, w_depth, count);
	i32 val2 = LINEAR_ROOM(ri, r_depth, count);

	return MIN(val1, val2);
}

static inline void irja_signal_caam(struct sec_jr *jr, u32 cnt)
{
#define IRJA_CAAM_SIGNAL_THRESHOLD  10
	if (jr->enq_cnt >= IRJA_CAAM_SIGNAL_THRESHOLD) {
		out_be32(&(jr->regs->irja), cnt);
		jr->enq_cnt = 0;
	}
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
	sec->tot_req_cnt += 1;

	jr->tail = MOD_ADD(wi, 1, jr->size);
	rp->idxs->r_index = MOD_ADD(ri, 1, rp->depth);

	rp->cntrs->jobs_processed += 1;
	rp->r_s_cntrs->req_jobs_processed = rp->cntrs->jobs_processed;
	out_be32(&(jr->regs->irja), 1);

RET:
	*todeq += 1;
	return 1;
}

static inline void loop_inorder(app_ring_pair_t *resp_ring)
{
	u32 flag = 0;
	u32 byte_pos = 0;
	u8  *pos_ptr = NULL;
	u32 bit_pos = 0;

	do {
		print_debug("Ordered job done idx: %d\n", resp_ring->order_j_d_index);
		/* Checking whether next ordered response bit is set  */
		byte_pos = resp_ring->order_j_d_index / BITS_PER_BYTE;
		bit_pos = resp_ring->order_j_d_index % BITS_PER_BYTE;				
		pos_ptr = resp_ring->resp_j_done_flag + byte_pos;
		print_debug("Ordered byte pos: %d, bit pos: %d, addr: %x, value: %x\n",
				byte_pos, bit_pos, pos_ptr, *pos_ptr);
		flag = 0x1 & ( *pos_ptr >> (bit_pos));
		print_debug("Flag value: %x\n", flag);
		if (0x1 == flag) {
			*pos_ptr &= ~(1 << bit_pos);
			resp_ring->cntrs->jobs_added += 1;
			resp_ring->order_j_d_index = 
				(resp_ring->order_j_d_index + 1) % resp_ring->depth;
		}
	} while(flag);
}

static inline void inorder_dequeue(app_ring_pair_t *resp_ring,
		struct sec_jr *jr, u32 ri, u32 wi)
{
	u8  *pos_ptr = NULL;
	u32 byte_pos = 0;
	u32 bit_pos = 0;

	/* Setting the proper bit position for this response */
	byte_pos = (wi - 1) / BITS_PER_BYTE;
	pos_ptr = resp_ring->resp_j_done_flag + byte_pos;
	bit_pos = (wi - 1) % (BITS_PER_BYTE);

	print_debug("Job byte pos: %d, bit pos: %d, addr: %x, value: %x\n",
				byte_pos, bit_pos, pos_ptr, *pos_ptr);
	*pos_ptr |= ( 1 << bit_pos);
	print_debug("Addr value after set bit: %x\n", *pos_ptr);

	memcpy(&(resp_ring->resp_r[wi - 1]), &jr->o_ring[ri], sizeof(resp_ring_t));
	print_debug("Index: %d, Desc: %0llx\n", wi - 1, resp_ring->resp_r[wi - 1].desc);

	loop_inorder(resp_ring);
}

static inline u32 sec_dequeue(struct c_mem_layout *c_mem,
		struct sec_engine **deq_sec, u32 *todeq)
{
	struct sec_jr *jr = &(*deq_sec)->jr;
	u32 cnt = in_be32(&jr->regs->orsf);
	u32 room = 0;
	u32 wi = 0;
	u32 ri = jr->head;
	app_ring_pair_t *resp_ring = NULL;
	dev_ctx_t *ctx = NULL;
	u32 rid = 0;
	u32 ret_cnt = 0;

	if (!cnt) {
		*deq_sec = (*deq_sec)->next;
		return 0;
	}

	while (cnt) {
		ctx = (dev_ctx_t *) ((u32)jr->o_ring[ri].desc - 32);
		rid = ctx->r_id;

		resp_ring = &(c_mem->rsrc_mem->orig_rps[rid]);

		room =
		    resp_ring->depth - (resp_ring->cntrs->jobs_added -
					resp_ring->r_s_c_cntrs->jobs_processed);
		if (!room)
			return ret_cnt;

		if (ctx->wi) {
			/* For order response driver will request with a write index id. 
			 * If an unorder response comes from sec we need to wait till 
			 * next order response is comming
			 */
			inorder_dequeue(resp_ring, jr, ri, ctx->wi);	
		} else {
			wi = resp_ring->idxs->w_index;
			Deq_Cpy(&(resp_ring->resp_r[wi]), &jr->o_ring[ri], 1);
			resp_ring->idxs->w_index = MOD_ADD(wi, 1, resp_ring->depth);
			resp_ring->cntrs->jobs_added += 1;
		}

		ri = MOD_ADD(ri, 1, jr->size);
		jr->head = ri;
		jr->deq_cnt += 1;

		resp_ring->r_s_cntrs->resp_jobs_added =
		    resp_ring->cntrs->jobs_added;

		out_be32(&jr->regs->orjr, 1);
		*todeq -= 1;

		//out_le16(resp_ring->msi_addr, resp_ring->msi_data);
		--cnt;
		++ret_cnt;
	}

	(*deq_sec)->tot_resp_cnt += ret_cnt;
	return ret_cnt;
}

static inline void raise_intr(drv_resp_ring_t *r)
{
	r->intr_ctrl_flag = 1;
	out_le16(r->msi_addr, r->msi_data);
/*	print_debug("MSI Addr :%0x   MSI Data :%d\n",
 *	r->msi_addr, r->msi_data); */
}

#ifndef HIGH_PERF
void invalidate_pending_app_reqs(struct c_mem_layout *c_mem)
{
	indexes_mem_t *ring_indexes = NULL;
	ring_counters_mem_t *ring_counters = NULL;
	ring_shadow_counters_mem_t *s_ring_counters = NULL;
	priority_q_t *p_q_cursor = c_mem->rsrc_mem->p_q;
	app_ring_pair_t *ring_cursor = NULL;
	app_ring_pair_t *ring_cursor_head = NULL;
	drv_resp_ring_t *drv_r = c_mem->rsrc_mem->drv_resp_ring;

	u32 ri = 0;
	u32 wi = 0;

	print1_debug(c_mem,
		     "\n Invalidating the pending reqs on all the app rings\n");

	while (p_q_cursor) {
		ring_cursor_head = ring_cursor = p_q_cursor->ring;
		while (ring_cursor) {

			ring_counters = ring_cursor->cntrs;
			ring_indexes = ring_cursor->idxs;
			s_ring_counters = ring_cursor->r_s_cntrs;
			ri = ring_indexes->r_index;
			wi = drv_r->idxs->w_index;

			print1_debug(c_mem, "Read index %d, Write index : %d\n",
				     ri, wi);
			print1_debug(c_mem,
				     "Jobs added  :%d Jobs Processed  :%d\n",
				     ring_cursor->r_s_c_cntrs->jobs_added,
				     ring_counters->jobs_processed);
			print1_debug(c_mem,
				     "Jobs pending    :%d on Ring     :%d\n",
				     ring_cursor->r_s_c_cntrs->jobs_added -
				     ring_counters->jobs_processed,
				     ring_cursor->id);

			while (0 !=
			       (ring_cursor->r_s_c_cntrs->jobs_added -
				ring_counters->jobs_processed)) {
				drv_r->resp_r[wi].desc =
				    ring_cursor->req_r[ri].desc;
#define JOB_DISACRDED   -1
				drv_r->resp_r[wi].result = JOB_DISACRDED;
				wi = MOD_INC(wi, drv_r->depth);
				ri = MOD_INC(ri, ring_cursor->depth);

				ring_counters->jobs_processed += 1;
				drv_r->r_cntrs->jobs_added += 1;
			}
			ring_indexes->r_index = ri;
			drv_r->idxs->w_index = wi;

			print1_debug(c_mem,
				     "Updated read %d and write %d index for ring %d,"
				     "ring_counters->jobs_processed %d,"
				     "ring_counters->jobs_added %d\n",
				     ri, wi, ring_cursor->id,
				     ring_counters->jobs_processed,
				     ring_counters->jobs_added);

			drv_r->r_s_cntrs->resp_jobs_added =
			    drv_r->r_cntrs->jobs_added;
			s_ring_counters->req_jobs_processed =
			    ring_counters->jobs_processed;
			print1_debug(c_mem,
				     "Giving interrupt for ring :%d\n",
				     ring_cursor->id);
			out_le16(ring_cursor->msi_addr, ring_cursor->msi_data);
			ring_cursor = ring_cursor->next;
			print1_debug(c_mem,
				     "ring_cursor : %0x, ring_cursor_head : %0x\n",
				     ring_cursor, ring_cursor_head);
			if (ring_cursor_head == ring_cursor)
				ring_cursor = NULL;
		}
		p_q_cursor = p_q_cursor->next;
	}
}

void resetcounters(struct c_mem_layout *mem, u32 sec_id)
{
	u32 i = 0;

	indexes_mem_t *ring_indexes = NULL;
	ring_counters_mem_t *ring_counters = NULL;
	priority_q_t *p_q_cursor = mem->rsrc_mem->p_q;
	app_ring_pair_t *ring_cursor = NULL;
	app_ring_pair_t *ring_cursor_head = NULL;

	print1_debug(mem, "\n Reset counters .............\n");
	mem->rsrc_mem->sec[sec_id].tot_req_cnt =
	    mem->rsrc_mem->sec[sec_id].tot_resp_cnt = 0;
	mem->rsrc_mem->cntrs_mem->tot_jobs_added =
	    mem->rsrc_mem->cntrs_mem->tot_jobs_processed = 0;
	for (i = 1; i < mem->rsrc_mem->ring_count; ++i) {
		ring_indexes = &(mem->rsrc_mem->idxs_mem[i]);
		ring_counters = &(mem->rsrc_mem->r_cntrs_mem[i]);
		print1_debug(mem,
			     "Updates for ring %d, ring_indexes : %0x "
			     "ring_counters : %0x\n",
			     i, ring_indexes, ring_counters);
		ring_indexes->w_index = ring_indexes->r_index = 0;
		print1_debug(mem, "index update  Finished\n");
		ring_counters->jobs_added = ring_counters->jobs_processed = 0;
		print1_debug(mem, "jobs_added Finished\n");
	}
	while (p_q_cursor) {
		ring_cursor_head = ring_cursor = p_q_cursor->ring;
		while (ring_cursor) {
			ring_cursor->cntrs->jobs_added =
			    ring_cursor->cntrs->jobs_processed = 0;
			ring_cursor->r_s_c_cntrs->jobs_added = 0;
			/*ring_cursor->response_count = 0;*/
			ring_cursor = ring_cursor->next;
			print1_debug(mem,
				     "ring_cursor : %0x, ring_cursor_head : %0x\n",
				     ring_cursor, ring_cursor_head);
			if (ring_cursor_head == ring_cursor)
				ring_cursor = NULL;
		}
		p_q_cursor = p_q_cursor->next;
		print1_debug(mem, "Going for Next priority queue\n");
	}
	print1_debug(mem,
		     "resetting counters for response ring r_cntrs %0x,"
		     "r_s_c_cntrs %0x\n",
		     mem->rsrc_mem->drv_resp_ring->r_cntrs,
		     mem->rsrc_mem->drv_resp_ring->r_s_c_cntrs);
	mem->rsrc_mem->drv_resp_ring->r_cntrs->jobs_added =
	    mem->rsrc_mem->drv_resp_ring->r_cntrs->jobs_processed = 0;
	mem->rsrc_mem->drv_resp_ring->idxs->w_index =
	    mem->rsrc_mem->drv_resp_ring->idxs->r_index = 0;
}

int check_addr_value(int addr)
{
	if ((addr < CONFIG_SYS_INIT_L3_ADDR) ||
	    (addr > (CONFIG_SYS_INIT_L3_ADDR + TOTAL_CARD_MEMORY - 1))) {
		print_debug("Trying to access a non-accessable memory, return: %d\n",
			     NON_ACCESS_MEM);
		return NON_ACCESS_MEM;
	}
	return 0;
}

int process_debug_cmd_md(struct c_mem_layout *mem, cmd_ring_req_desc_t *cmd_req)
{
	cmd_op_t *cmd_op;
	int addr, i, err;

	cmd_op = (cmd_op_t *) ((u8 *)mem->v_ob_mem + (cmd_req->cmd_op - mem->p_ob_mem));
	print1_debug(mem, "Cmd op buffer address... :%0x\n", cmd_op);

	addr = cmd_req->ip_info.dgb.address;
	err = check_addr_value(addr);
	if (err)
		return err;
	/* memcpy(cmd_op->buffer.debug_op, (void *)cmd_req->ip_info.dgb.address, 5);*/
	for (i = 0; i < 64; ++i) {
		cmd_op->buffer.debug_op[i] = *((u32 *)addr + i);
		print1_debug(mem, "DUMPING AT %x : %x\n",
			(u32 *)addr + i, *((u32 *)addr + i));
	}
	return 0;
}

int process_debug_cmd_mw(struct c_mem_layout *mem, cmd_ring_req_desc_t *cmd_req)
{
	int addr, err;

	addr = cmd_req->ip_info.dgb.address;
	err = check_addr_value(addr);
	if (err)
		return err;

	*(u32 *) (addr) = cmd_req->ip_info.dgb.val;
	print1_debug(mem, "WRITING AT ADDRESS : %x\n", addr);
	return 0;
}

int process_debug_cmd(struct c_mem_layout *mem, cmd_ring_req_desc_t *cmd_req)
{
	int err = 0;
	print1_debug(mem, "DEBUGGING IN FW\n");
	print1_debug(mem, "GOT THE COMMAND: %d\n", cmd_req->ip_info.dgb.cmd_id);
	print1_debug(mem, "THE ADDRESS: %u\n", cmd_req->ip_info.dgb.address);
	print1_debug(mem, "GOT THE VALUE: %x\n", cmd_req->ip_info.dgb.val);

	switch (cmd_req->ip_info.dgb.cmd_id) {
	case MD:
		err = process_debug_cmd_md(cmd_req);
		break;
	case MW:
		err = process_debug_cmd_mw(cmd_req);
		break;
	case PRINT1_DEBUG:
		print1_debug(mem, "DEBUG PRINTS ARE ENABLED\n");
		mem->dgb_print = cmd_req->ip_info.dgb.val;
		print1_debug(mem, "DEBUG PRINTS ARE ENABLED\n");
		break;
	case PRINT1_ERROR:
		mem->err_print = cmd_req->ip_info.dgb.val;
		break;
	default:
		err = 1;
	}
	return err;
}

void process_resetsec_cmd(struct c_mem_layout *mem, cmd_ring_req_desc_t *cmd_req)
{
	print1_debug(mem, "Resetting sec engine     :%d\n",
			cmd_req->ip_info.sec_id);
	sec_reset(mem->rsrc_mem->sec[cmd_req->ip_info.sec_id].info);
	{
	struct sec_jr *sec_jr = &(mem->rsrc_mem->sec->jr);
	print1_debug(mem, "Before SEC i/p ring virtual address	:%0x\n",
			sec_jr->i_ring);
	print1_debug(mem, "SEC Output ring virtual address         :%0x\n",
			sec_jr->o_ring);
	}
	sec_eng_hw_init(&(mem->rsrc_mem->sec[cmd_req->ip_info.sec_id]));
	{
	struct sec_jr *sec_jr = &(mem->rsrc_mem->sec->jr);
	print1_debug(mem, "After  fsl_sec_init SEC Input ring virtual address   :%0x\n",
			sec_jr->i_ring);
	print1_debug(mem, "SEC Output ring virtual address     :%0x\n",
			sec_jr->o_ring);
	}
	resetcounters(mem, cmd_req->ip_info.sec_id);
	{
	i32 secid = cmd_req->ip_info.sec_id;
	struct sec_jr *sec_jr = &(mem->rsrc_mem->sec[secid].jr);
	print1_debug(mem, "After resetcounters SEC Input ring virtual address         :%0x\n",
			sec_jr->i_ring);
	print1_debug(mem, "SEC Output ring virtual address         :%0x\n",
			sec_jr->o_ring);
	}
}

void process_devstat_cmd(struct c_mem_layout *mem, cmd_ring_req_desc_t *cmd_req)
{
	cmd_op_t *cmd_op;
	app_ring_pair_t *ring_cursor;
	u32 total_job_added = 0;
	u32 total_job_processed = 0;
	u32 i;

	print1_debug(mem, "Device stats\n");

	cmd_op = (cmd_op_t *) ((u8 *)mem->v_ob_mem +
			(cmd_req->cmd_op - mem->p_ob_mem));

	print1_debug(mem, "Cmd op buffer address... :%0x\n", cmd_op);

	cmd_op->buffer.dev_stat_op.fwversion = FW_VERSION;
	cmd_op->buffer.dev_stat_op.totalmem = TOTAL_CARD_MEMORY;
	cmd_op->buffer.dev_stat_op.codemem = FIRMWARE_SIZE;
	cmd_op->buffer.dev_stat_op.heapmem = TOTAL_CARD_MEMORY - FIRMWARE_SIZE;
	cmd_op->buffer.dev_stat_op.freemem = mem->free_mem;
	cmd_op->buffer.dev_stat_op.num_of_sec_engine =
			mem->rsrc_mem->sec_eng_cnt;
	cmd_op->buffer.dev_stat_op.no_of_app_rings =
			mem->rsrc_mem->ring_count;
	cmd_op->buffer.dev_stat_op.total_jobs_rx =
			mem->rsrc_mem->cntrs_mem->tot_jobs_added;
	cmd_op->buffer.dev_stat_op.total_jobs_pending =
			mem->rsrc_mem->cntrs_mem->tot_jobs_processed;

	for (i = 1 ; i < mem->rsrc_mem->ring_count; ++i) {
		ring_cursor = &(mem->rsrc_mem->orig_rps[i]);
		total_job_added += ring_cursor->r_s_c_cntrs->jobs_added;
		total_job_processed += ring_cursor->cntrs->jobs_processed;
	}

	cmd_op->buffer.dev_stat_op.total_jobs_rx = total_job_added;
	cmd_op->buffer.dev_stat_op.total_jobs_pending = total_job_processed;
}

void process_ringstat_cmd(struct c_mem_layout *mem, cmd_ring_req_desc_t *cmd_req)
{
	priority_q_t *p_q_cursor = NULL;
	app_ring_pair_t *ring_cursor = NULL;
	u32 pending_cnt, r_id, prop;

	print1_debug(mem, "Ring Statistics\n");

	cmd_op = (cmd_op_t *) ((u8 *)mem->v_ob_mem + (cmd_req->cmd_op - mem->p_ob_mem));
	p_q_cursor = mem->rsrc_mem->p_q;

	while (p_q_cursor) {
		ring_cursor = p_q_cursor->ring;
		if (0 == cmd_req->ip_info.ring_id)
			ring_cursor = mem->rsrc_mem->cmdrp;

		while (ring_cursor) {
			r_id = ring_cursor->id;
			if (cmd_req->ip_info.ring_id == r_id) {
				cmd_op->buffer.ring_stat_op.depth = ring_cursor->depth;
#ifdef COMMON_IP_BUFFER_POOL
				cmd_op->buffer.ring_stat_op.tot_size =
				    ring_cursor->depth * sizeof(app_ring_pair_t);
#else
				cmd_op->buffer.ring_stat_op.tot_size = DEFAULT_POOL_SIZE +
					ring_cursor->depth * sizeof(app_ring_pair_t);
#endif
				prop = (ring_cursor->props & APP_RING_PROP_PRIO_MASK) >>
						APP_RING_PROP_PRIO_SHIFT;
				print1_debug(mem, "priority : %d\n", prop);
				cmd_op->buffer.ring_stat_op.priority = prop;
				prop = (ring_cursor-> props & APP_RING_PROP_AFFINE_MASK) >>
						APP_RING_PROP_AFFINE_SHIFT;
				print1_debug(mem, "affinity : %d\n", prop);
				cmd_op->buffer.ring_stat_op.affinity = prop;
				prop = (ring_cursor->props & APP_RING_PROP_ORDER_MASK) >>
						APP_RING_PROP_ORDER_SHIFT;
				print1_debug(mem, "order : %d\n", prop);
				cmd_op->buffer.ring_stat_op.order = prop;
				print1_debug(mem,"Ring : %d, Job added : %d\n",
					ring_cursor->id,
					ring_cursor->r_s_c_cntrs->jobs_added);
				print1_debug(mem, "Ring : %d, Job processed : %d\n",
					ring_cursor->id,
					ring_cursor->cntrs->jobs_processed);
				pending_cnt = ring_cursor->r_s_c_cntrs->jobs_added -
				    ring_cursor->cntrs->jobs_processed;
				/* FREE SPACE IN RING */
				cmd_op->buffer.ring_stat_op.free_count =
				    (ring_cursor->depth - pending_cnt);
				/* TOT JOBS PROCESSED BY RING */
				cmd_op->buffer.ring_stat_op.jobs_processed =
				    ring_cursor->cntrs->jobs_processed;
				/* TOTAL JOBS PENDING */
				cmd_op->buffer.ring_stat_op.jobs_pending = pending_cnt;

				return;
			}
			ring_cursor = ring_cursor->next;
		}
		p_q_cursor = p_q_cursor->next;
	}
}

void process_pingdev_cmd(struct c_mem_layout *mem, cmd_ring_req_desc_t *cmd_req)
{
	cmd_op_t *cmd_op;

	print1_debug(mem, "Changed Ping Dev command.....\n");
	cmd_op = (cmd_op_t *) ((u8 *)mem->v_ob_mem + (cmd_req->cmd_op - mem->p_ob_mem));
	cmd_op->buffer.ping_op.resp = 556;
	print1_debug(mem, "Resp : %d\n", cmd_op->buffer.ping_op.resp);
	print1_debug(mem, "Sending Resp : %d to driver\n", cmd_op->buffer.ping_op.resp);
}

void process_secstat_cmd(struct c_mem_layout *mem, cmd_ring_req_desc_t *cmd_req)
{
	cmd_op_t *cmd_op;
	int i;

	print1_debug(mem, "SENDING THE SEC STATISTICS\n");

	cmd_op = (cmd_op_t *) ((u8 *)mem->v_ob_mem + (cmd_req->cmd_op - mem->p_ob_mem));

	cmd_op->buffer.sec_op.sec_ver = mem->rsrc_mem->sec->info->secvid_ms;
	cmd_op->buffer.sec_op.cha_ver = mem->rsrc_mem->sec->info->chavid_ls;
	cmd_op->buffer.sec_op.no_of_sec_jr = mem->rsrc_mem->sec_eng_cnt;
	cmd_op->buffer.sec_op.jr_size = mem->rsrc_mem->sec->jr.size;
	cmd_op->buffer.sec_op.no_of_sec_engines = mem->rsrc_mem->sec_eng_cnt;

	for (i = 0; i < mem->rsrc_mem->sec_eng_cnt; ++i) {
		cmd_op->buffer.sec_op.sec[i].sec_tot_req_jobs =
				mem->rsrc_mem->sec[i].tot_req_cnt;
		cmd_op->buffer.sec_op.sec[i].sec_tot_resp_jobs =
				mem->rsrc_mem->sec[i].tot_resp_cnt;
	}

	print1_debug(mem, "SEC STATISTIC SENT\n");
}

void process_resetdev_cmd()
{
	u32 reset_val;
	volatile u32 *rreg;

#ifdef P4080_EP_TYPE
	rreg = (u32 *)RESET_REG_ADDR;
	reset_val = RESET_REG_VALUE;
#elif C293_EP_TYPE
	rreg = (u32 *)RESET_PIC_PIR_ADDR;
	reset_val = RESET_PIC_PIR_VALUE;
#endif

	print1_debug(mem, "Resetting Device\n");
	*rreg = reset_val;
}

/*******************************************************************************
 * Function     : process_command
 *
 * Arguments    : mem: Pointer to the cards memory where all the resources start
 *                cmd_req: Dequeue command req ring desc
 *
 * Return Value : u32
 *
 * Description  : Process the command from the host driver
 *
 ******************************************************************************/
u32 process_command(struct c_mem_layout *mem, cmd_ring_req_desc_t *cmd_req)
{
	print1_debug(mem, "   ---- Command Ring Processing ----\n");
	switch (cmd_req->cmd_type) {
	case BLOCK_APP_JOBS:
		/* This command is sent by driver to block the jobs on app rings
		 * Driver may use this feature for smooth exits for some of the
		 * commands like RESET SEC, RESET DEV etc.
		 */

		/* Invalidate the current pending app reqs
		 * on all the request rings */
		invalidate_pending_app_reqs(mem);
		/* This return value is useful for the callee to block jobs */
		return BLOCK_APP_JOBS;
		break;

	case DEBUG:
		return process_debug_cmd(mem, cmd_req);

	case RESETDEV:
		process_resetdev_cmd();
		break;

	case RESETSEC:
		process_resetsec_cmd(mem, cmd_req);
		/* Driver as part of protocol would have sent the block
		 * command earlier. Since now the RESET is done, we can
		 * unblock the rings and accept the jobs.
		 */
		return UNBLOCK_APP_JOBS;

	case DEVSTAT:
		process_devstat_cmd(mem, cmd_req);
		break;

	case RINGSTAT:
		process_ringstat_cmd(mem, cmd_req);
		break;

	case PINGDEV:
		process_pingdev_cmd(mem, cmd_req);
		break;

	case REHANDSHAKE:
		print1_debug(mem, "SETTING DEVICE IN REHANDSHAKE\n");
		mem->c_hs_mem->state = DEFAULT;
/*            sec_reset();*/
		return REHANDSHAKE;

	case SECSTAT:
		process_secstat_cmd(mem, cmd_req);
		break;

	default:
		print_error("Invalid Command  !!\n");
		return 1;
	}
	return 0;
}

static i32 cmd_ring_processing(struct c_mem_layout *mem)
{
	app_ring_pair_t *cmdrp = mem->rsrc_mem->cmdrp;
	u32 ri = cmdrp->idxs->r_index;
	u32 wi = cmdrp->idxs->w_index;
	u64 desc = 0;
	u32 res = 0;

	if (cmdrp->r_s_c_cntrs->jobs_added - cmdrp->cntrs->jobs_processed) {
		desc = cmdrp->req_r[ri].desc;
		print1_debug(mem,
			     "GOT THE DESC : %0llx - - -  AT : %u - - - "
			     "DEPTH OF RING: %u\n",
			     desc, ri, cmdrp->depth);
		res =
		    process_command(mem, (cmd_ring_req_desc_t *) ((u32) desc));
	} else
		goto out;

	ri = (ri + 1) % cmdrp->depth;
	cmdrp->idxs->r_index = ri;	/* MOD_ADD(ri, 1, cmdrp->depth); */
	cmdrp->cntrs->jobs_processed += 1;

	/* Shadow counter */
	cmdrp->r_s_cntrs->req_jobs_processed = cmdrp->cntrs->jobs_processed;

	/* Add the response */
	print1_debug(mem,
		     "SENDING DESC TO DRIVER : %0llx AT WI : %u -----[%0lx]\n",
		     desc, wi, &(cmdrp->resp_r[wi].desc));
	cmdrp->resp_r[wi].desc = desc;
	cmdrp->resp_r[wi].result = res;
	wi = (wi + 1) % cmdrp->depth;
	cmdrp->idxs->w_index = wi;	/* MOD_ADD(wi, 1, cmdrp->depth); */
	cmdrp->cntrs->jobs_added += 1;

	/* Shadow counter */
	cmdrp->r_s_cntrs->resp_jobs_added = cmdrp->cntrs->jobs_added;

	out_le16(cmdrp->msi_addr, cmdrp->msi_data);
	print1_debug(mem, "INTERRUPTED DRIVER\n");
out:
	return res;
}
#endif

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
		rp->cntrs->jobs_processed += count;
		rp->r_s_cntrs->req_jobs_processed = rp->cntrs->jobs_processed;
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
	uint32_t hostroom = rp->depth - (rp->cntrs->jobs_added - rp->r_s_c_cntrs->jobs_processed);
	uint32_t count = MIN(out_jobs, hostroom);

	print_debug("%s: out_jobs: %d, hostroom: %d, count: %d \n",
			__func__, out_jobs, hostroom, count);

	if (count) {
		Deq_Circ_Cpy(&(sec->jr), rp, count);
		rp->cntrs->jobs_added += count;
		rp->r_s_cntrs->resp_jobs_added = rp->cntrs->jobs_added;
		out_be32(&(sec->jr.regs->orjr), count);
	}
	return count;
}

static inline void raise_intr_app_ring(app_ring_pair_t *rp)
{
	print_debug("%s: MSI addr: %0x, MSI data:%0x \n",
			__func__, rp->msi_addr, rp->msi_data);
	rp->intr_ctrl_flag = 1;
	out_le16(rp->msi_addr, rp->msi_data);
}

static inline uint32_t irq_is_due(uint32_t deq_cnt, app_ring_pair_t *rp,
				uint32_t irq_timeout)
{
	uint32_t raise_irq;
	uint32_t host_jobs = rp->cntrs->jobs_added - rp->r_s_c_cntrs->jobs_processed;

	raise_irq = (deq_cnt > 0) && (!rp->intr_ctrl_flag);
	raise_irq |= (host_jobs != 0 ) && (irq_timeout == 0);

	return raise_irq;
}

#ifdef HIGH_PERF
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

		in_jobs = recv_r->r_s_c_cntrs->jobs_added - recv_r->cntrs->jobs_processed;
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

#else
static inline void check_intr(app_ring_pair_t *r, u32 deq, u32 *processedcount)
{
	if(deq) {
		print_debug("%s( ): Dequeued: %d\n", __FUNCTION__, deq);
	}
	if(deq && r->intr_ctrl_flag) {
		print_debug("%s( ): Dequeued: %d intr ctrl flag: %d\n",
				__FUNCTION__, deq, r->intr_ctrl_flag);
	}

	if(deq && (!r->intr_ctrl_flag)) {
		raise_intr_app_ring(r);
	} else {
		if(r->cntrs->jobs_added - r->r_s_c_cntrs->jobs_processed) {
			if(*processedcount != r->r_s_c_cntrs->jobs_processed) {
				*processedcount = r->r_s_c_cntrs->jobs_processed;
				raise_intr_app_ring(r);
			}
		}
	}
}

static void ring_processing(struct c_mem_layout *c_mem)
{
	u32 deq = 0, cnt = 0, ring_jobs = 0, processedcount = 0;
	i32 res = 0, block_app_jobs = 0;
	u32 block_req = 0;

	struct sec_engine *enq_sec = c_mem->rsrc_mem->sec;
	struct sec_engine *deq_sec = c_mem->rsrc_mem->sec;
	app_ring_pair_t *rp = c_mem->rsrc_mem->rps;

RP_START:
	res = cmd_ring_processing(c_mem);

	if (0 == res)
		goto APP_RING;

	if (BLOCK_APP_JOBS == res) {
		print1_debug(c_mem,
                 "------------> Stopped processing app jobs....\n");
        /* Wait for some timeout inorder driver
         * to get the ACK and process it..
         * Can wait for long as anyways RESET operation is in progress..
         */
		wait_for_timeout(50000000ull);
		block_app_jobs = 1;
	}

	if (UNBLOCK_APP_JOBS == res) {
		print1_debug(c_mem, "Releasing the block condition on app rings...\n");
		block_app_jobs = 0;
	}

	if (REHANDSHAKE == res) {
		print1_debug(c_mem, "Going for rehandshake.WAITING FOR FLAG TO SET\n");
		WAIT_FOR_STATE_CHANGE(c_mem->c_hs_mem->state);
		block_app_jobs = 0;
		print1_debug(c_mem, "FLAG HAS BEEN SET GOOING TO START\n");
		return ;
	}

APP_RING:
	if (block_app_jobs)
		goto RP_START;

	if((rp->r_s_c_cntrs->jobs_added - rp->cntrs->jobs_added) >= rp->depth - 1)
		block_req = 1;
	else
		block_req = 0;

	cnt = rp->r_s_c_cntrs->jobs_added - rp->cntrs->jobs_processed;
	
	if(!cnt || block_req)
		goto DEQ;

	sel_sec_enqueue(c_mem, &enq_sec, rp, &deq);

DEQ:
	if (!deq)
		goto NEXTRING;

	ring_jobs = sec_dequeue(c_mem, &deq_sec, &deq);

	/* CHECK INTERRUPTS */
	check_intr(rp, ring_jobs, &processedcount);

NEXTRING:
/*	rp = rp->next; */
	rp = next_ring(rp);
	goto RP_START;
}
#endif

static inline void rng_processing(struct c_mem_layout *c_mem)
{
	u32     cnt =   0,  ring_jobs   =   0;

	app_ring_pair_t     *rp         =   c_mem->rsrc_mem->rps;
	struct sec_engine   *sec        =   c_mem->rsrc_mem->sec;
	u32                 *r_deq_cnt  =   NULL;
	u32                 deq         =   0;

	/*	We need to check the jobs only in ring pair 1 because
	 *	for RNG Instantiation the driver is using only the
	 *	ring pair 1 to send the job.
	 */

	do {
		rp = rp->next;
	} while (1 != rp->id);
	
	r_deq_cnt   =   &(rp->cntrs->jobs_processed);
	cnt = WAIT_FOR_DRIVER_JOBS(&(rp->r_s_c_cntrs->jobs_added), r_deq_cnt);
	if (cnt == 0) {
		return;
	}

	ring_jobs    =  sel_sec_enqueue(c_mem, &sec, rp, &deq);

DEQ:
	ring_jobs   =  sec_dequeue(c_mem, &sec, &deq);
	if (!ring_jobs)
		goto DEQ;

	if (ring_jobs)
		raise_intr(c_mem->rsrc_mem->drv_resp_ring);
}

i32 fsl_c2x0_fw(void)
{
	phys_addr_t p_addr = 0;
	phys_addr_t p_aligned_addr = 0;

	struct c_mem_layout *c_mem = NULL;

#ifdef P4080_EP_TYPE
	/* Not required for C2X0 */
	fix_p4080_reg_settings();
#endif
#ifndef HIGH_PERF
START:
#endif
	stack_ptr = ALIGN_TO_L1_CACHE_LINE_REV(stack_ptr);

	/* One cache line for handshake (HS) memory */
	stack_ptr -= L1_CACHE_LINE_SIZE;

	print_debug("\nMemory Pointers\n");
	c_mem = (struct c_mem_layout *)(stack_ptr - sizeof(struct c_mem_layout));
	print_debug("c_mem: %0x\n", c_mem);

	c_mem->dgb_print = c_mem->err_print = 0;
	c_mem->free_mem = TOTAL_CARD_MEMORY - FIRMWARE_SIZE;

	/* Allocating top cache line size number of bytes
	 * for handshake - so that it can get re-used
	 * Ideally the hs mem should be in the platform SRAM
	 * but for now FW size is 512K, hence it will be
	 * in lower part of L2 SRAM.
	 */
	c_mem->c_hs_mem = (struct crypto_c_hs_mem *)stack_ptr;
	stack_ptr -= sizeof(struct c_mem_layout);
	c_mem->free_mem -= sizeof(struct c_mem_layout);
	print_debug("c_hs_mem: %0x\n", c_mem->c_hs_mem);

	c_mem->v_ib_mem = L2_SRAM_VIRT_ADDR;
	c_mem->p_ib_mem = L2_SRAM_VIRT_ADDR;	/* Phy addr is same as v addr */
	print_debug("v_ib_mem: %0x\n", c_mem->v_ib_mem);
	print_debug("p_ib_mem: %0llx\n", c_mem->p_ib_mem);

	/*PCIE1 controller physical address-outbound window will be set to 16G*/
	c_mem->p_ob_mem = CONFIG_SYS_PCIE1_MEM_PHYS;
	c_mem->p_pci_mem = CONFIG_SYS_PCIE1_MEM_PHYS;
	print_debug("p_ob_mem: %0llx\n", c_mem->p_ob_mem);
	print_debug("p_pci_mem: %0llx\n", c_mem->p_pci_mem);

	/* TLB exist only for 1G  */
	c_mem->v_ob_mem = CONFIG_SYS_PCIE1_MEM_VIRT;
	print_debug("v_ob_mem: %0x\n", c_mem->v_ob_mem);

	c_mem->v_msi_mem = CONFIG_SYS_PCIE1_MSI_MEM_VIRT;
	print_debug("v_msi_mem: %0x\n", c_mem->v_msi_mem);

	print_debug("\nOB MEM DETAILS\n");
	/* Driver would have updated the offset of its
	 * created memory inside the outbound window
	 * Calculate the address valid in our domain -
	 * The offset will definitely be 1G aligned so simple
	 * addition should give the 1G aligned address.
	 */
	print_debug("Host ob mem l: %0x\n", c_mem->c_hs_mem->h_ob_mem_l);
	print_debug("Host ob mem h: %0x\n", c_mem->c_hs_mem->h_ob_mem_h);
	/* Calculate one 36bit address from H & L parts of it */
	p_addr = c_mem->c_hs_mem->h_ob_mem_h;
	p_addr = (p_addr << 32) | c_mem->c_hs_mem->h_ob_mem_l;
	print_debug("Host ob 64 Bit address: %0llx\n", p_addr);

	/* Since we have 1G TLB open for rings - get the 1G aligned
	 * address for this physical address */
#define OB_TLB_SIZE_MASK    ((~(0)) << 30)
	p_aligned_addr = (p_addr & OB_TLB_SIZE_MASK);
	c_mem->p_ob_mem = (CONFIG_SYS_PCIE1_MEM_PHYS + p_aligned_addr);
	print_debug("1G Aligned host ob mem addr: %0llx\n", c_mem->p_ob_mem);

	/* Set the TLB here for this 1G */
	/* Using TLB 3 */
	c2x0_set_tlb(1, c_mem->v_ob_mem, c_mem->p_ob_mem, MAS3_SW | MAS3_SR,
		     MAS2_I | MAS2_G, 0, 3, BOOKE_PAGESZ_1G, 1);

	c_mem->h_hs_mem =
	    (struct fsl_h_mem_handshake *) (((u8 *)c_mem->v_ob_mem + (p_addr - p_aligned_addr)));
	print_debug("h_hs_mem: %0x\n", c_mem->h_hs_mem);

	print_debug("MSI DETAILS\n");
	print_debug("MSI mem l: %0x\n", c_mem->c_hs_mem->h_msi_mem_l);
	print_debug("MSI mem h: %0x\n", c_mem->c_hs_mem->h_msi_mem_h);

	/* Form the 64 bit address */
	p_addr = c_mem->c_hs_mem->h_msi_mem_h;
	p_addr = (p_addr << 32) | c_mem->c_hs_mem->h_msi_mem_l;
	print_debug("MSI mem 64 bit address: %0llx\n", p_addr);

	/* Since we have 1M TLB open for MSI window -
	 * get the 1M aligned address for this physical address */
#define MSI_TLB_SIZE_MASK    ((~((u32)0)) << 20)
	p_aligned_addr = (p_addr & MSI_TLB_SIZE_MASK);

	/* Physical address should be within 16G window */
	c_mem->p_msi_mem = c_mem->p_pci_mem + p_aligned_addr;
	print_debug("p_msi_mem: %0llx\n", c_mem->p_msi_mem);

	/* Set the TLB here for this 1M */
	/* Using TLB 2 */
	c2x0_set_tlb(1, c_mem->v_msi_mem, c_mem->p_msi_mem,
		     MAS3_SX | MAS3_SW | MAS3_SR, MAS2_I | MAS2_G, 0, 2,
		     BOOKE_PAGESZ_1M, 1);

	stack_ptr = ALIGN_TO_L1_CACHE_LINE_REV(stack_ptr);

	stack_ptr -= sizeof(struct resource);
	c_mem->rsrc_mem = (struct resource *) stack_ptr;

	/* From here allocations will start on L2 part of the cache */

	alloc_rsrc_mem(c_mem);

	/* Default the state */
	c_mem->c_hs_mem->state = DEFAULT;

	/* Init the intr time counters */
	c_mem->intr_ticks = 0;
	c_mem->intr_timeout_ticks = usec2ticks(10);

	/* Start the handshake */
	handshake(c_mem);

	if ((NULL == c_mem->rsrc_mem->p_q) ||
	    (NULL == c_mem->rsrc_mem->p_q->ring)) {
		print_error("Nothing to process.....\n");
		return -1;
	}

	/* Separate the command ring */
	/*
	c_mem->rsrc_mem->cmdrp = c_mem->rsrc_mem->rps;
	c_mem->rsrc_mem->rps = c_mem->rsrc_mem->rps->next;
	make_rp_circ_list(c_mem);
	*/
	print_debug("\n\n\nFirmware up\n");
#ifdef HIGH_PERF
	ring_processing_perf(c_mem);
#else
	ring_processing(c_mem);
	goto START;
#endif
	return 0;
}
