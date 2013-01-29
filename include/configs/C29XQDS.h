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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * C29XQDS board configuration file
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef CONFIG_36BIT
#define CONFIG_PHYS_64BIT
#endif

#define CONFIG_SYS_CCSR_DO_NOT_RELOCATE
#define CONFIG_ENABLE_36BIT_PHYS

#ifdef CONFIG_C293QDS
#define CONFIG_C293
#endif

#ifdef CONFIG_SDCARD
#define CONFIG_RAMBOOT_SDCARD
#define CONFIG_SYS_RAMBOOT
#define CONFIG_SYS_TEXT_BASE		0xffff9000
#define CONFIG_RESET_VECTOR_ADDRESS	0xfffffffc
#define CONFIG_MIN_UBOOT
#endif

#define CONFIG_SYS_MONITOR_BASE CONFIG_SYS_TEXT_BASE

/* High Level Configuration Options */
#define CONFIG_BOOKE			/* BOOKE */
#define CONFIG_E500			/* BOOKE e500 family */
#define CONFIG_MPC85xx

#define CONFIG_FSL_LAW			/* Use common FSL init code */

#define CONFIG_ICS307_REFCLK_HZ		25000000
#define CONFIG_DDR_CLK_FREQ	100000000
#define CONFIG_SYS_CLK_FREQ	66666666

#define CONFIG_MISC_INIT_R
#define CONFIG_HWCONFIG

/*
 * These can be toggled for performance analysis, otherwise use default.
 */
#define CONFIG_L2_CACHE			/* toggle L2 cache */
#define CONFIG_BTB			/* toggle branch predition */

#define CONFIG_ADDR_STREAMING		/* toggle addr streaming */


#define CONFIG_SYS_MEMTEST_START	0x00200000
#define CONFIG_SYS_MEMTEST_END		0x00400000
#define CONFIG_PANIC_HANG

#define CONFIG_SYS_CCSRBAR		0xffe00000
#define CONFIG_SYS_CCSRBAR_PHYS_LOW	CONFIG_SYS_CCSRBAR

/* Platform SRAM setting  */
#define CONFIG_SYS_PLATFORM_SRAM_BASE	0xffb00000
#ifdef CONFIG_PHYS_64BIT
#define CONFIG_SYS_PLATFORM_SRAM_BASE_PHYS \
			(0xf00000000ull | CONFIG_SYS_PLATFORM_SRAM_BASE)
#else
#define CONFIG_SYS_PLATFORM_SRAM_BASE_PHYS CONFIG_SYS_PLATFORM_SRAM_BASE
#endif
#define CONFIG_SYS_PLATFORM_SRAM_SIZE	(512 << 10)

#define CONFIG_SYS_NO_FLASH
#define CONFIG_SYS_INIT_RAM_LOCK
#define CONFIG_SYS_INIT_RAM_ADDR	0xffd00000
#define CONFIG_SYS_INIT_RAM_END		0x00004000

#define CONFIG_SYS_GBL_DATA_OFFSET	(CONFIG_SYS_INIT_RAM_END \
						- GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_OFFSET	CONFIG_SYS_GBL_DATA_OFFSET

#define CONFIG_SYS_MONITOR_LEN		(512 * 1024)
#define CONFIG_SYS_MALLOC_LEN		(1024 * 1024)

/* Serial Port */
#define CONFIG_CONS_INDEX	1
#define CONFIG_SYS_NS16550
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	1
#define CONFIG_SYS_NS16550_CLK		get_bus_freq(0)

#define CONFIG_SERIAL_MULTI		/* Enable both serial ports */
#define CONFIG_SYS_CONSOLE_IS_IN_ENV

#define CONFIG_SYS_BAUDRATE_TABLE	\
	{300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200}

#define CONFIG_SYS_NS16550_COM1	(CONFIG_SYS_CCSRBAR+0x4500)
#define CONFIG_SYS_NS16550_COM2	(CONFIG_SYS_CCSRBAR+0x4600)


/*
 * Environment
 */
#define CONFIG_ENV_SIZE			0x2000

/*
 * Miscellaneous configurable options
 */
#define CONFIG_SYS_LONGHELP			/* undef to save memory	*/
#define CONFIG_CMDLINE_EDITING			/* Command-line editing */
#define CONFIG_AUTO_COMPLETE			/* add autocompletion support */
#define CONFIG_SYS_LOAD_ADDR	0x2000000	/* default load address */
#define CONFIG_SYS_PROMPT	"=> "		/* Monitor Command Prompt */

#ifdef CONFIG_CMD_KGDB
#define CONFIG_SYS_CBSIZE	1024		/* Console I/O Buffer Size */
#else
#define CONFIG_SYS_CBSIZE	256		/* Console I/O Buffer Size */
#endif
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE+sizeof(CONFIG_SYS_PROMPT)+16)
						/* Print Buffer Size */
#define CONFIG_SYS_MAXARGS	16		/* max number of command args */
#define CONFIG_SYS_BARGSIZE	CONFIG_SYS_CBSIZE/* Boot Argument Buffer Size */
#define CONFIG_SYS_HZ		1000		/* dec freq: 1ms ticks */

/*
 * Internal Definitions
 *
 * Boot Flags
 */
#define BOOTFLAG_COLD	0x01		/* Normal Power-On: Boot from FLASH */
#define BOOTFLAG_WARM	0x02		/* Software reboot */

/*
 * Environment Configuration
 */

#define CONFIG_BAUDRATE		115200


#endif	/* __CONFIG_H */
