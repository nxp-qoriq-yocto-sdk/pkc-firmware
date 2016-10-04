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

#ifndef __FW_H_
#define __FW_H_

/*      FIRMWARE VERSION FORMAT
 *--------------------------------------
 *  31  27  23  19  15  11  7   3   0
 *  |   |   |   |   |   |   |   |   |
 *  |  DAY  | M |    YEAR   | MI| MR|
 *--------------------------------------
 */
#define FW_VERSION          0x0f300d11
#define TOTAL_CARD_MEMORY   (1024*1024)	/* 1 MB */
#define FIRMWARE_SIZE		(28*1024)

#define L2_SRAM_SIZE            (512*1024)
#define PLATFORM_SRAM_SIZE      (512*1024)

/* Address space specific macros */
#define CONFIG_SYS_INIT_L3_ADDR             0XFFF00000

/* The lower 512KB block in 1MB is L2 SRAM and higher block is platform SRAM */
#define L2_SRAM_TLB_ADDR	CONFIG_SYS_INIT_L3_ADDR
#define PLATFORM_SRAM_TLB_ADDR	(L2_SRAM_TLB_ADDR + L2_SRAM_SIZE)

/* MSI TLB ADDR is CONFIG_SYS_PCIE1_MEM_VIRT + 1G */
#define MSI_TLB_ADDR                        0xC0000000

#define PLATFORM_SRAM_VIRT_ADDR             PLATFORM_SRAM_TLB_ADDR
#define L2_SRAM_VIRT_ADDR                   L2_SRAM_TLB_ADDR
#define CONFIG_SYS_PCIE1_MSI_MEM_VIRT       MSI_TLB_ADDR

/* Physical address ranges used */
#define CONFIG_SYS_PCIE1_MSI_MEM_PHYS       0X840000000ull
#define CONFIG_SYS_PCIE1_MEM_PHYS           0X800000000ull
#define CONFIG_SYS_PCIE1_MEM_VIRT           0X80000000

#define OUTBOUND_WINDOW_TLB_SIZE            (1*1024*1024*1024ull)
#define MSI_WINDOW_TLB_SIZE                 (1*1024*1024)

/* Sec engine related macros */
#define SEC_JR_DEPTH                    (128)
#define DEFAULT_POOL_SIZE               (512*1024)

/* Setting the same input buffer pool for all the rings */
#define COMMON_IP_BUFFER_POOL

/* Device cache line related definitions */
#define L1_CACHE_LINE_SIZE_SHIFT        6
#define L1_CACHE_LINE_SIZE          (1 << L1_CACHE_LINE_SIZE_SHIFT)
#define __cache_line_aligned__		\
	__attribute__((__aligned__(L1_CACHE_LINE_SIZE)))

#define ALIGN_TO_L1_CACHE_LINE(x)   (x & ~(L1_CACHE_LINE_SIZE-1))

/* Watchdog counter to signal the host with an interrupt if it looks stuck */
#define IRQ_TIMEOUT 10000

/* General Macros */
#define true            1
#define RESULT_OK       1
#define INTR_FLAG_SET   1
#define SEC_ENG_1 1
#define SEC_ENG_2 2
#define SEC_ENG_3 3

#define SYNC_MEM        asm volatile("sync" : : : "memory")
#define MOD_ADD(x, value, size)	((x+value) & (size-1))

#define MCFGR_SWRST       ((u32)(1)<<31)	/* Software Reset */
#define MCFGR_DMA_RST     ((u32)(1)<<28)	/* DMA Reset */

#define FSL_CRYPTO_MAX_RING_PAIRS   6
#define BUDGET_NO_OF_TOT_JOBS                   50
#define HOST_JOB_WAIT_TIME_OUT  100000ull

/* Switch controls */
#define TIMED_WAIT_FOR_JOBS

/* 32 bit virtual address */
typedef u32 va_addr_t;

struct sec_jr_regs {
	u32 irba_h;
	u32 irba_l;
	u32 rsvd1;
	u32 irs;
	u32 rsvd2;
	u32 irsa;
	u32 rsvd3;
	u32 irja;
	u32 orba_h;
	u32 orba_l;
	u32 rsvd4;
	u32 ors;
	u32 rsvd5;
	u32 orjr;
	u32 rsvd6;
	u32 orsf;
	u32 rsvd7;
	u32 jrsta;
	u32 rsvd8;
	u32 jrint;
	u32 jrcfg0;
	u32 jrcfg1;
	u32 rsvd9;
	u32 irri;
	u32 rsvd10;
	u32 orwi;
	u32 rsvd11;
	u32 jrcr;
};

struct fault_regs {
	u32 far_h;	/* Fault Address high  */
	u32 far_l;	/* Fault Address low */
	u32 famr;	/* Fault Address MID */
	u32 fadr;	/* Fault Address Detail */
	u32 reserved1;
	u32 ssta;	/* SEC Status */
};

struct sec_ip_ring {
	dma_addr_t desc;
};

struct sec_op_ring {
	dma_addr_t desc;
	u32 status;
} __packed;

struct sec_jr {
	struct sec_jr_regs *regs;
	u32 size;
	u32 head;
	u32 tail;
	u32 enq_cnt;
	u32 deq_cnt;
	u32 id;
	struct sec_ip_ring *i_ring;
	struct sec_op_ring *o_ring;
};

#define DEFAULT_CLOCK 400
#define ACTUAL_CLOCK 400
#define RNG4_ENT_DLY0  2400
#define RNG4_ENT_DLY1  2600
#define RNG4_ENT_DLY2  2800

struct rng_regs {
	uint32_t rtmctl;
	uint32_t rtscmisc;
	uint32_t rtpkrrng;
	uint32_t rtpkrmax;
	uint32_t rtsdctl;
	uint32_t rtsblim;
	uint32_t rtfreqmin;
	uint32_t rtfreqmax;
	uint32_t rtscml;
};

struct kek_regs {
	uint32_t jdkek;
	uint32_t tdkek;
	uint32_t tdsk;
};

/* The container data structure be in platform SRAM */
struct sec_engine {
	u8 id;
	ccsr_sec_t *info;
	struct rng_regs *rng;
	struct kek_regs *kek;
	struct fault_regs *faults;
	u32 *scfg;
	u32 *rdsta;
	struct sec_jr jr;
	struct sec_engine *next;
};

/*******************************************************************************
Description : Defines the application ring request entry
Fields      : sec_desc  : DMA address of the sec desc
*******************************************************************************/
struct req_ring {
	dma_addr_t desc;
} __packed;

typedef struct req_ring req_ring_t;

/*******************************************************************************
Description :	Defines the application ring response entry
Fields      :	sec_desc  : DMA address of the sec desc
		result    : Result of crypto op from sec engine
*******************************************************************************/
/* typedef struct resp_ring { */
struct resp_ring {
	dma_addr_t desc;
	volatile uint32_t result;
} __packed;

typedef struct resp_ring resp_ring_t;

/*******************************************************************************
Description :	Defines the container structure for ring indexes.
Fields      :	w_index   : Write index of the ring
		r_index   : Read index of the ring
*******************************************************************************/
typedef struct indexes_mem {
	va_addr_t w_index;
	va_addr_t r_index;
} indexes_mem_t;

/*******************************************************************************
Description : Contains the counters per job ring. There will two copies one for
		local usage and one shadowed for firmware
Fields	:Local memory
		jobs_added: Count of number of resp jobs added
		jobs_processed: Count of number of req jobs processed
	Shadow copy memory
		jobs_added: Count of number of req jobs added by driver
		jobs_processed: Count of number of resp jobs processed by driver
*******************************************************************************/
typedef struct ring_counters_mem {
	u32 jobs_added;
	u32 jobs_processed;
} ring_counters_mem_t;

/*******************************************************************************
Description :	Defines the container structure for total counters.
Fields      :	req_tot_jobs_added: Total no of req jobs added across ring pairs
		req_tot_jobs_processed: Total no of req jobs processed across
					ring pairs
		ring_counters: Pointer to the array of ring pair counters.
*******************************************************************************/
typedef struct counters_mem {
	u32 tot_jobs_added;
	u32 tot_jobs_processed;
} counters_mem_t;
typedef counters_mem_t shadow_counters_mem_t;

/*******************************************************************************
Description :	Defines the container structure for counters of a ring pair.
		This memory is alloc on host and is shared with firmware.
Fields      :	resp_jobs_added: Number of jobs added in the resp ring by fw
		resp_jobs_processed: Number of jobs processed from resp ring
					by drv
*******************************************************************************/
typedef struct ring_shadow_counters_mem {
	u32 resp_jobs_added;
	u32 req_jobs_processed;
} ring_shadow_counters_mem_t;

/*******************************************************************************
Description :	Defines the container structure for application ring pair
Fields      :	req_r	: Request ring
		resp_r	: Response ring
		ip_pool	: Input pool
		msi_addr: Device domain valid msi address. FW uses to gen intr
		id	: Id of the ring (1,2,3...)
		depth	: Depth of the ring
		response_count	: Count of number of responses added in the ring
		msi_data	: Data to be written at MSI address
		intr_ctrl_flag	: Used to control whether intr has to be gen for
					this ring or not. Driver sets and resets
					this field to indicate the same.
		p_req_addr	: Physical address of the req ring
		p_resp_addr	: Physical address of the resp ring
		p_ip_pool_addr	: Physical address of the input pool
		p_msi_addr	: Physical address of the MSI address
		sec		: Pointer to the affined sec engine
		next		: Points to the next ring pair in the same
					prio level.
*******************************************************************************/
typedef struct app_ring_pair {
	req_ring_t *req_r;
	resp_ring_t *resp_r;
	void *ip_pool;
	void *msi_addr;
	struct sec_engine *sec;
	indexes_mem_t *idxs;
	ring_counters_mem_t *r_cntrs;
	ring_counters_mem_t *r_s_c_cntrs;
	ring_shadow_counters_mem_t *r_s_cntrs;

	u8 id;
	u16 msi_data;
	u32 depth;
	u32 intr_ctrl_flag;

	struct app_ring_pair *next;
} app_ring_pair_t;

/*******************************************************************************
Description :	Defines the container structure for resources in the device
Fields      :	sec_eng_cnt	: Number of sec engines
		sec	: Array of sec engine rsrc structures
		ring_count: Number of rings created in the device (App+cmd)
		p_q	: Linked list of priority queues created

*******************************************************************************/
struct resource {
	u8 sec_eng_cnt;
	u8 num_of_rps;  /* number of ring pairs communicated by the host */

	void *req_mem;
	void *ip_pool;
	struct sec_engine *sec;
	app_ring_pair_t *rps;

	indexes_mem_t *idxs_mem;
	ring_counters_mem_t *r_cntrs_mem;
	ring_counters_mem_t *r_s_c_cntrs_mem;
	counters_mem_t *cntrs_mem;
	counters_mem_t *s_c_cntrs_mem;
	ring_shadow_counters_mem_t *r_s_cntrs_mem;
};

/* Handshake related data structures */
struct dev_handshake_mem {
	u32 h_ob_mem_l;
	u32 h_ob_mem_h;

	u32 h_msi_mem_l;
	u32 h_msi_mem_h;

	u8 state;
	u8 data_len;
	u8 pad;

	union cmd_data {
		struct c_config_data {
			u8 num_of_rps;
			u8 padding;
			u32 req_mem_size;
			u32 r_s_cntrs;
		} config;
		struct c_ring_data {
			u8 rid;
			u8 padding;
			u16 msi_data;
			u32 depth;
			u32 resp_ring_offset;
			u32 msi_addr_l;
			u32 msi_addr_h;
		} ring;
	} data;
};

/*******************************************************************************
Description : Defines the handshake memory layout on the host
Fields      :
*******************************************************************************/
struct host_handshake_mem {
	u8 state;
	u8 result;

	u32 dev_avail_mem;

	union resp_data {
		struct fw_up_data {
			u32 p_ib_mem_base_l;
			u32 p_ib_mem_base_h;
			u32 p_ob_mem_base_l;
			u32 p_ob_mem_base_h;
			u32 no_secs;
		} device;
		struct config_data {
			u32 r_s_c_cntrs;
			u32 s_c_cntrs;
			u32 ip_pool;    /* ip_pool allocated in firmware SRAM */
			u32 resp_intr_ctrl_flag;
		} config;
		struct ring_data {
			u32 req_r;
			u32 intr_ctrl_flag;
		} ring;
	} data;
};

#define JR_SIZE_SHIFT   0
#define JR_SIZE_MASK    0xffff0000
#define JR_NO_SHIFT     16
#define JR_NO_MASK      0xff00ffff
#define SEC_NO_SHIFT    24
#define SEC_NO_MASK     0x00ffffff

/* Identifies different states of the device */
typedef enum handshake_state {
	DEFAULT = 255,
	FIRMWARE_UP = 10,
	FW_INIT_CONFIG_COMPLETE,
	FW_GET_SEC_INFO_COMPLETE,
	FW_INIT_RING_PAIR_COMPLETE,
	FW_INIT_MSI_INFO_COMPLETE,
	FW_INIT_IDX_MEM_COMPLETE,
	FW_INIT_COUNTERS_MEM_COMPLETE,
	FW_INIT_RNG,
	FW_RNG_COMPLETE
} handshake_state_t;

/* Identifies different commands to be sent to the firmware */
typedef enum h_handshake_commands {
	HS_GET_SEC_INFO,
	HS_INIT_CONFIG,
	HS_INIT_RING_PAIR,
	HS_INIT_MSI_INFO,
	HS_INIT_IDX_MEM,
	HS_INIT_COUNTERS_MEM,
	HS_COMPLETE,
	WAIT_FOR_RNG,
	RNG_DONE
} h_handshake_commands_t;

/* Identifies different commands to be sent to the firmware */
typedef enum fw_handshake_commands {
	FW_GET_SEC_INFO,
	FW_INIT_CONFIG,
	FW_INIT_RING_PAIR,
	FW_INIT_MSI_INFO,
	FW_INIT_IDX_MEM,
	FW_INIT_COUNTERS_MEM,
	FW_HS_COMPLETE,
	FW_WAIT_FOR_RNG,
	FW_RNG_DONE
} fw_handshake_commands_t;

/*******************************************************************************
Description :	Defines the application ring job context. This is helpful in
		data processing to have a mapping between sec desc and the
		application ring which gave that request.
Fields      : ring_offset: Offset of the ring pair in which sec desc was posted
*******************************************************************************/
typedef struct job_ctx {
	phys_addr_t ring_offset;
} job_ctx_t;

/*******************************************************************************
Description :	Defines the memory layout on the device. Since the memory on the
		device is plain, it is accessed by typecasting it to this
		structure so that accessing is easy.
Fields      :
*******************************************************************************/
struct c_mem_layout {
	va_addr_t v_ib_mem;
	va_addr_t v_ob_mem;
	va_addr_t v_msi_mem;

	phys_addr_t p_ib_mem;
	phys_addr_t p_ob_mem;
	phys_addr_t p_msi_mem;
	phys_addr_t p_pci_mem;

	/* Addresses of common pool for all the ring pairs */
	phys_addr_t p_buf_pool_mem;
	va_addr_t v_buf_pool_mem;

	struct host_handshake_mem __iomem *h_hs_mem;
	struct dev_handshake_mem *c_hs_mem;

	struct resource *rsrc_mem;

	u32 free_mem;

	int dgb_print;
	int err_print;
};

/*                  COMMAND RING STRUCTURE                  */

typedef enum commands {
	DEBUG,
	DEVSTAT,
	REHANDSHAKE,
	PINGDEV,
	RESETDEV,
	RESETSEC,
	RINGSTAT,
	SECSTAT,
	BLOCK_APP_JOBS,
	UNBLOCK_APP_JOBS,
	NON_ACCESS_MEM
} commands_t;

typedef enum debug_commands {
	MD,
	MW,
	PRINT1_DEBUG,
	PRINT1_ERROR,
} dgb_cmd_type_t;

#define MAX_SEC_NO 3
/* SEC STAT */
typedef struct fsl_sec_stat {
	u32 sec_ver;
	u32 cha_ver;
	u32 no_of_sec_engines;
	u32 no_of_sec_jr;
	u32 jr_size;
	struct sec_ctrs_t {
		u32 sec_tot_req_jobs;
		u32 sec_tot_resp_jobs;
	} sec[MAX_SEC_NO];
} fsl_sec_stat_t;

/* DEVICES STAT */
typedef struct fsl_dev_stat_op {
	u32 fwversion;
	u32 totalmem;
	u32 codemem;
	u32 heapmem;
	u32 freemem;
	u32 num_of_sec_engine;	/* ALREADY IN crypto_dev_config_t */
	u32 no_of_app_rings;	/* ALREADY IN crypto_dev_config_t */
	u32 total_jobs_rx;	/* ALREADY IN struct crypto_h_mem_layout */
	u32 total_jobs_pending;	/* ALREADY IN struct crypto_h_mem_layout */
} fsl_dev_stat_op_t;

/* RESOURCE STAT */
struct fsl_ring_stat_op {
	u32 depth;		/* ALREADY IN struct ring_info  */
	u32 tot_size;		/* REALLY NEED THIS ???         */
	u32 priority;		/* PRIORITY OF RING  */
	u32 affinity;		/* AFFINITY OF RING  */
	u32 order;		/* ORDER OF RING    */
	u32 free_count;		/* DEPTH - CURRENT JOBS */
	u32 jobs_processed;	/* ALREADY IN struct  ring_counters_mem */
	u32 jobs_pending;	/* ALREADY IN struct  ring_counters_mem */
	u32 budget;
} __packed;

typedef struct fsl_ring_stat_op fsl_ring_stat_op_t;

/* DEBUG */
typedef struct debug_op {
	u32 total_ticks;
	u32 pcie_data_consume_ticks; /* WRITE HOST TO CARD + CARD TO HOST */
	u32 job_wait_ticks;	/* WAIT TIME IN JOB QUEUE */
	u32 job_process_ticks;	/* PROCESS TIME */
	u32 sec_job_ticks;	/* TICKS FOR SEC TO COMPLETE JOB */
} debug_op_t;

/* PING */
typedef struct ping_op {
	u32 resp;
} ping_op_t;

typedef struct debug_ip {
	dgb_cmd_type_t cmd_id;
	unsigned int address;
	unsigned int val;
} debug_ip_t;

/*******************************************************************************
Description :	output from the device in repsonse
Fields      :	ping_op   -
		debug_op  -
		ring_stat_op  -
		dev_stat_op   -
*******************************************************************************/
union op_buffer {
	ping_op_t ping_op;
	/* debug_op_t          debug_op;    */
	u32 debug_op[64];
	fsl_ring_stat_op_t ring_stat_op;
	fsl_dev_stat_op_t dev_stat_op;
	fsl_sec_stat_t sec_op;
} __packed;

typedef union op_buffer op_buffer_t;

/*******************************************************************************
Description : output from the device in repsonse of command
Fields      : buffer     : output buffer
*******************************************************************************/
struct cmd_op {
	op_buffer_t buffer;
} __packed;

typedef struct cmd_op cmd_op_t;

/*******************************************************************************
Description :	prepares the command descriptor for command ring
Fields      :	cmd_type      : type of command
		ip_info       : input to the command
		cmd_op        : output buffer
*******************************************************************************/
/* COMMAND RING DESC */
struct cmd_ring_req_desc {
	commands_t cmd_type;

	union __ip_info {
		u32 ring_id;	/* RING ID */
		u32 sec_id;	/* SEC ENGINE ID */
		u32 count;	/* COUNT VAR TO CKECK LIVELENESS */
		debug_ip_t dgb;
	} ip_info;

	dma_addr_t cmd_op;
} __packed;

typedef struct cmd_ring_req_desc cmd_ring_req_desc_t;

/* ASSEMBLY FUNCTIONS */
extern u64 getticks(void );
/* EXTERN FUNCTIONS */
extern u64 usec2ticks(u64 );
extern void c2x0_set_tlb(u8 , u32 , u64 ,
            u8 , u8 , u8 , u8 , u8 , u8 );

#endif
