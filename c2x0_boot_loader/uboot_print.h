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

void c2x0_serial_init (void);

#define CONFIG_SYS_NS16550_REG_SIZE 1

#define UART_REG(x)                        \
    unsigned char prepad_##x[CONFIG_SYS_NS16550_REG_SIZE - 1]; \
    unsigned char x;


#define C2X0_UART_LSR_DR 0x01        /* Data ready */


struct c2x0_NS16550 {
    UART_REG(rbr);      /* 0 */
    UART_REG(ier);      /* 1 */
    UART_REG(fcr);      /* 2 */
    UART_REG(lcr);      /* 3 */
    UART_REG(mcr);      /* 4 */
    UART_REG(lsr);      /* 5 */
    UART_REG(msr);      /* 6 */
    UART_REG(spr);      /* 7 */
    UART_REG(mdr1);     /* 8 */
    UART_REG(reg9);     /* 9 */
    UART_REG(regA);     /* A */
    UART_REG(regB);     /* B */
    UART_REG(regC);     /* C */
    UART_REG(regD);     /* D */
    UART_REG(regE);     /* E */
    UART_REG(uasr);     /* F */
    UART_REG(scr);      /* 10*/
    UART_REG(ssr);      /* 11*/
    UART_REG(reg12);    /* 12*/
    UART_REG(osc_12m_sel);  /* 13*/
};
    
#define thr rbr
#define iir fcr
#define dll rbr
#define dlm ier

typedef struct c2x0_NS16550 *C2X0_NS16550_t;

//#define c2x0_serial_out(x, y)    outb(x, (ulong)y)

//#define CONFIG_SYS_NS16550_CLK (gd->bus_clk/2)

#define CONFIG_SYS_NS16550_IER  0x00
#define UART_LCR_BKSE   0x80        /* Bank select enable */
/* useful defaults for LCR */
#define UART_LCR_8N1    0x03
#define UART_MCR_DTR    0x01        /* DTR   */
#define UART_MCR_RTS    0x02        /* RTS   */
#define UART_FCR_FIFO_EN    0x01 /* Fifo enable */
#define UART_FCR_RXSR       0x02 /* Receiver soft reset */
#define UART_FCR_TXSR       0x04 /* Transmitter soft reset */

#define UART_LCRVAL UART_LCR_8N1        /* 8 data, 1 stop, no parity */

#define C2X0_UART_LSR_THRE   0x20        /* Xmit holding register empty */
#define C2X0_CONFIG_CONS_INDEX   1
#define UART_MCRVAL (UART_MCR_DTR | \
             UART_MCR_RTS)      /* RTS/DTR */
#define UART_FCRVAL (UART_FCR_FIFO_EN | \
             UART_FCR_RXSR |    \
             UART_FCR_TXSR)     /* Clear & enable FIFOs */
#define c2x0_serial_out(x, y)    writeb(x, y)
#define c2x0_serial_in(y)        readb(y)
#define MODE_X_DIV 16


struct cmd_tbl_s {
    char        *name;      /* Command Name         */
    int     maxargs;    /* maximum number of arguments  */
    int     repeatable; /* autorepeat allowed?      */
                    /* Implementation function  */
    int     (*cmd)(struct cmd_tbl_s *, int, int, char * const []);
    char        *usage;     /* Usage message    (short) */
#ifdef  CONFIG_SYS_LONGHELP
    char        *help;      /* Help  message    (long)  */
#endif
#ifdef CONFIG_AUTO_COMPLETE
    /* do auto completion on the arguments */
    int     (*complete)(int argc, char * const argv[], char last_char, int maxv, char *cmdv[]);
#endif
};
    
typedef struct cmd_tbl_s    cmd_tbl_t; 
char c2x0_getc(void);
void c2x0_puts(const char *s);
void do_command( char *buf );
void c2x0_putc(const char c);
int c2x0_printf(const char *, ...);
int vsprintf(char *, const char *, va_list );
unsigned long simple_strtoul(const char *,char **,unsigned int );

#ifdef PRINT_DEBUG
#define print_debug c2x0_printf
#else
#define print_debug(fmt, ...)
#endif

#ifdef PRINT_ERROR
#define print_error c2x0_printf
#else
#define print_error(fmt, ...)
#endif
