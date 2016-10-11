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

/* Note: The port number specified in the functions is 1 based.
 *	 the array is 0 based.
 */
static C2X0_NS16550_t serial_ports[1] = {
	(C2X0_NS16550_t)CONFIG_SYS_NS16550_COM1
};

#if defined PRINT_DEBUG
void c2x0_serial_init (void)
{
	int clock_divisor;

	clock_divisor = (gd->bus_clk + (gd->baudrate * (MODE_X_DIV / 2))) / (MODE_X_DIV * gd->baudrate);
	c2x0_serial_out(UART_LCR_BKSE | UART_LCRVAL, (ulong)&serial_ports[0]->lcr);
	c2x0_serial_out(0, &serial_ports[0]->dll);
	c2x0_serial_out(UART_LCRVAL, &serial_ports[0]->lcr);
	c2x0_serial_out(UART_MCRVAL, &serial_ports[0]->mcr);
	c2x0_serial_out(UART_LCR_BKSE | UART_LCRVAL, &serial_ports[0]->lcr);
	c2x0_serial_out(clock_divisor & 0xff, &serial_ports[0]->dll);
	c2x0_serial_out(UART_LCRVAL, &serial_ports[0]->lcr);

	gd->have_console = 1;
}
#else
void c2x0_serial_init (void)
{return;}
#endif

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

void c2x0_putc(const char c)
{       
	if (c == '\n')
		c2x0_NS16550_putc(serial_ports[0], '\r');
    
	c2x0_NS16550_putc(serial_ports[0], c);
}
 
void c2x0_puts(const char *s)
{
	while (*s)
		c2x0_putc (*s++);
}

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
