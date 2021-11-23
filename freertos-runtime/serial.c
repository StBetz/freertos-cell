
/* {{{1 License
    FreeRTOS V8.2.0 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

	***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
	***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
	the FAQ page "My application does not run, what could be wrong?".  Have you
	defined configASSERT()?

	http://www.FreeRTOS.org/support - In return for receiving this top quality
	embedded software for free we request you assist our global community by
	participating in the support forum.

	http://www.FreeRTOS.org/training - Investing in training allows your team to
	be as productive as possible as early as possible.  Now you can receive
	FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
	Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!

    This main activate and test the GPIO and UART IRQ 
}}} */

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "serial.h"

 
#define UART_BASE 	0xFE215000
#define IRQ_BASE 	0xFF841000

#define AUX_IRQ 		0x00 //Interrupt Controll
#define AUX_ENABLES		0x04 //UART,SPI activate
#define AUX_MU_IO_REG		0x40 //FIFO read and write
#define AUX_MU_IER_REG		0x44 //Interrupt Register
#define AUX_MU_IIR_REG		0x48 //Interrupt Register show
#define AUX_MU_LCR_REG		0x4c //Controll Register
#define AUX_MU_MCR_REG		0x50 //ignor
#define AUX_MU_LSR_REG		0x54 //Data Status 
#define AUX_MU_MSR_REG		0x58 //Pin high ir low
#define AUX_MU_SCRATCH		0x5c //temp Storage
#define AUX_MU_CNTL_REG		0x60 //Features
#define AUX_MU_STAT_REG		0x64 //Information
#define AUX_MU_BAUD_REG		0x68 //Baudrate

#define UART_CLK   (250*1000*1000) 
#define UART_BAUDRATE  115200

/* Code is from linux kernel: drivers/tty/serial/8250/8250_early.c */
#define DIV_ROUND_CLOSEST(x, divisor)(      \
{             \
  typeof(x) __x = x;        \
  typeof(divisor) __d = divisor;      \
  (((typeof(x))-1) > 0 ||       \
   ((typeof(divisor))-1) > 0 || (__x) > 0) ?  \
    (((__x) + ((__d) / 2)) / (__d)) : \
    (((__x) - ((__d) / 2)) / (__d));  \
}            \
)

static uint32_t mmio_read32(void *addr)
{
  return *((volatile uint32_t*)addr);
}

static void mmio_write32(void *addr, uint32_t val)
{
  *((volatile uint32_t*)addr) = val;
}

sio_fd_t mini_uart_open(void)
{
	unsigned divisor = DIV_ROUND_CLOSEST(UART_CLK, 8 * UART_BAUDRATE);
  sio_fd_t uart_base = (void*)UART_BASE;

  //mmio_write32(UART_CLOCK_REG, mmio_read32(UART_CLOCK_REG) | (1 << UART_GATE_NR));

	mmio_write32(uart_base + AUX_IRQ , 0 );/* IRQ off */
	mmio_write32(uart_base + AUX_ENABLES, 1); /* Mini UART activate */
	mmio_write32(uart_base + AUX_MU_IER_REG , 0); /* IER off */
	mmio_write32(uart_base + AUX_MU_IIR_REG , 6); /* clear FIFO */
	mmio_write32(uart_base + AUX_MU_LCR_REG , 1); /* set Data size 8-bit mode*/
	mmio_write32(uart_base + AUX_MU_CNTL_REG , 47);/*Auto* flow controll*/

	
	/* Program baudrate */
	mmio_write32(uart_base + AUX_MU_BAUD_REG, divisor); 	
  return uart_base;
}

void mini_uart_irq_rx_enable(void)
{
 sio_fd_t uart_base = (void*)UART_BASE; 
 uint32_t irq_reg ;
 mmio_write32(uart_base + AUX_MU_IIR_REG , 2); /* clear receive FIFO (second bit, Respi Doku is wrong) */
 mmio_write32(uart_base+ AUX_MU_IER_REG , 1 ); /*receive Interrupt*/
 //irq_reg = mmio_read32(uart_base+ AUX_IRQ);
   
 /* printf("Interrupt aktive: %x \n",irq_reg);
  printf("AUX Enable: %x \n",mmio_read32(uart_base+ AUX_IRQ+ 0x04));
  for(int i =0x40;i<=0x74;i= i + 0x04){
    printf("REgister %x : %x \n",i,mmio_read32(uart_base+i));
  }*/
}

static int serial_ready(void) 
{
  uint32_t a;
   sio_fd_t uart_base = (void*)UART_BASE;
    a = (mmio_read32(uart_base + AUX_MU_LSR_REG) & 0x00000020);
    return a;
}

void mini_uart_putchar(uint32_t c)
{
  sio_fd_t uart_base = (void*)UART_BASE;
  uint32_t *uart_tx = uart_base + AUX_MU_IO_REG;
  while(!serial_ready());
   /* Wait for empty transmit */
  mmio_write32(uart_tx, c);
}

int mini_uart_getchar(void){
 sio_fd_t uart_base = (void*)UART_BASE;
 if(mmio_read32(uart_base  + AUX_MU_LSR_REG)& 0x00000001 ){  /*Receive FIFO recieve with at least one Symbol */
  return mmio_read32(uart_base + AUX_MU_IO_REG);
 }else{
  printf("recieve FIFO is empty \n");
  return 0x000;
 }
}

int mini_uart_irq_getchar(void)
{
  sio_fd_t uart_base = (void*)UART_BASE;
  return mmio_read32(uart_base + AUX_MU_IO_REG);
}

sio_fd_t serial_open(void){
 return mini_uart_open();
}

void serial_irq_rx_enable(sio_fd_t fd){
 mini_uart_irq_rx_enable();
}

void serial_putchar(sio_fd_t fd, uint32_t c){
 mini_uart_putchar(c);
}

int serial_irq_getchar(sio_fd_t fd){
 return mini_uart_irq_getchar();
}

