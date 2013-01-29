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
#include "uboot_print.h"

int c2x0_serial_init (void)
{
    int clock_divisor;

    clock_divisor = (( (gd->bus_clk) + (gd->baudrate * (MODE_X_DIV / 2))) / (MODE_X_DIV * gd->baudrate));
    c2x0_serial_out(UART_LCR_BKSE | UART_LCRVAL, (ulong)&serial_ports[0]->lcr);
    c2x0_serial_out(0, &serial_ports[0]->dll);
    c2x0_serial_out(UART_LCRVAL, &serial_ports[0]->lcr);
    c2x0_serial_out(UART_MCRVAL, &serial_ports[0]->mcr);
    c2x0_serial_out(UART_LCR_BKSE | UART_LCRVAL, &serial_ports[0]->lcr);
    c2x0_serial_out(clock_divisor & 0xff, &serial_ports[0]->dll);
    c2x0_serial_out(UART_LCRVAL, &serial_ports[0]->lcr);
	
    gd->have_console = 1;
    return (0);
}
char c2x0_getc(void)
{ 
	C2X0_NS16550_t com_port = C2X0_PORT;	
    while ((c2x0_serial_in(&com_port->lsr) & C2X0_UART_LSR_DR) == 0) {
        WATCHDOG_RESET();
    }   
    return c2x0_serial_in(&com_port->rbr);
}

void c2x0_NS16550_putc(C2X0_NS16550_t com_port, char c)
{   
    while ((c2x0_serial_in(&com_port->lsr) & C2X0_UART_LSR_THRE) == 0)
        ;
    c2x0_serial_out(c, &com_port->thr);

    /*
     * Call watchdog_reset() upon newline. This is done here in putc
     * since the environment code uses a single puts() to print the complete
     * environment upon "printenv". So we can't put this watchdog call
     * in puts().
     */
    if (c == '\n')
        WATCHDOG_RESET();
} 

void
c2x0_putc(const char c)
{       
    if (c == '\n')
        c2x0_NS16550_putc(C2X0_PORT, '\r');
    
    c2x0_NS16550_putc(C2X0_PORT, c);
}
 
void c2x0_puts(const char *s)
{
    while (*s) {
        c2x0_putc (*s++);
    }
}
#if COMMAND_SUPPORT
#define vscnprintf(buf, size, fmt, args...) vsprintf(buf, fmt, ##args)
int c2x0_printf(const char *fmt, ...)
{
    va_list args;
    uint i;
    char printbuffer[CONFIG_SYS_PBSIZE];

    va_start(args, fmt);

    /* For this to work, printbuffer must be larger than
     * anything we ever want to print.
     */
    i = vscnprintf(printbuffer, sizeof(printbuffer), fmt, args);
    va_end(args);

    /* Print the string */
    c2x0_puts(printbuffer);
    return i;
}
#define MAX_LINE_LENGTH_BYTES (64)
#define DEFAULT_LINE_LENGTH_BYTES (16)
int c2x0_print_buffer (ulong addr, void* data, uint width, uint count, uint linelen)
{
    int i;
    uint32_t x;
        
    c2x0_printf("%08lx:", addr);
        /* Copy from memory into linebuf and print hex values */
    for (i = 0; i < linelen; i++) {
        x = *(volatile uint32_t *)data;
        c2x0_printf(" %0*x", width * 2, x); 
        data += width;
    }   
    return 0;
}
#define DISP_LINE_LEN   16
int c2x0_do_mem_md ( cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    ulong   addr, length = 0x40;
    int size = 4;
    int rc = 0;

	c2x0_printf("Argv :%s\n", argv[1]);
    addr = simple_strtoul(argv[1], NULL, 16);
	c2x0_printf("Addr :%x\n", addr);
        /* Print the lines. */
    c2x0_print_buffer(addr, (void*)addr, size, length, DISP_LINE_LEN/size);

    return (rc);
}

int c2x0_strcmp(const char * cs,const char * ct)
{   
    register signed char __res;

    while (1) {
        if ((__res = *cs - *ct++) != 0 || !*cs++)
            break;
    }

    return __res;
}

int c2x0_do_mem_mw ( cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    ulong   addr, writeval;
    /* Address is specified since argc > 1
    */
    addr = simple_strtoul(argv[1], NULL, 16);
    /* Get the value to write.
    */
    writeval = simple_strtoul(argv[2], NULL, 16);
    *((ulong  *)addr) = (ulong )writeval;
    return 0;
}

void do_command( char *buf )
{
    char command[10], addr[10], data[10];
    int i=0, j=0;
	char *argv[3];

	if ( buf[0] != 'm' )
		return ;
    
	while( buf[i] != ' ' )
        command[j++] = buf[i++];
    command[j] = '\0';

	if( !c2x0_strcmp( command, "md" ) )
	{
    	j=0;
		i++;
    	while( buf[i] != '\0' )
        	addr[j++] = buf[i++];
    	addr[j] = '\0';
		argv[0] = command;
		argv[1] = addr;
        c2x0_printf("\nCommand %s\n", argv[0]);
        c2x0_printf("Addr to read%s\n", argv[1]);
		c2x0_do_mem_md(0, 0, 2, argv);
	}
	else if( !c2x0_strcmp( command, "mw" ) )
	{
    	j=0;
		i++;
    	while( buf[i] != ' ' )
        	addr[j++] = buf[i++];
		addr[j] = '\0';
		j=0;
		i++;
    	while( buf[i] != '\0' )
        	data[j++] = buf[i++];
    	data[j] = '\0';
        argv[0] = command;
        argv[1] = addr;
		argv[2] = data; 
        c2x0_printf("\nCommand %s\n", argv[0]);
        c2x0_printf("Addr to write%s\n", argv[1]);
        c2x0_printf("Data to write %s\n", argv[2]);
        c2x0_do_mem_mw(0, 0, 3, argv);
	}
	else
		c2x0_printf("\nUnknown Command\n");
    return ;

}
#endif 
