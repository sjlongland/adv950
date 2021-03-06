/*
 *  Driver for 8250/16550-type serial ports
 *
 *  Based on drivers/char/serial.c, by Linus Torvalds, Theodore Ts'o.
 *
 *  Copyright (C) 2001 Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/serial_8250.h>
void adv950_uart_write_wakeup(struct uart_port *port);
int adv950_uart_resume_port(struct uart_driver *drv, struct uart_port *uport);
void adv950_resume_port(int line);
int adv950_uart_suspend_port(struct uart_driver *drv, struct uart_port *uport);
void adv950_suspend_port(int line);
void adv950_uart_configure_port(struct uart_driver *drv, struct uart_state *state,
		    struct uart_port *port);
int adv950_uart_match_port(struct uart_port *port1, struct uart_port *port2);
int adv950_uart_register_port(struct uart_port *port);
int __init adv950_uart_init(void);
int adv950_uart_register_driver(struct uart_driver *drv);
int adv950_uart_remove_one_port(struct uart_driver *drv, struct uart_port *uport);
int adv950_uart_register_driver(struct uart_driver *drv);
int adv950_uart_add_one_port(struct uart_driver *drv, struct uart_port *uport);
void __exit adv950_uart_exit(void);
void adv950_uart_unregister_driver(struct uart_driver *drv);
void adv950_uart_unregister_port(int line);
unsigned int adv950_uart_get_divisor(struct uart_port *port, unsigned int baud);
void adv950_uart_configure_port(struct uart_driver *drv, struct uart_state *state,
		    struct uart_port *port);

struct old_serial_port {
	unsigned int uart;
	unsigned int baud_base;
	unsigned int port;
	unsigned int irq;
	unsigned int flags;
	unsigned char hub6;
	unsigned char io_type;
	unsigned char *iomem_base;
	unsigned short iomem_reg_shift;
	unsigned long irqflags;
};

/*
 * This replaces serial_uart_config in include/linux/serial.h
 */
struct adv950_uart_config {
	const char	*name;
	unsigned short	fifo_size;
	unsigned short	tx_loadsz;
	unsigned char	fcr;
	unsigned int	flags;
};
/////////////
#define TX_BUF_TOT_LEN 128
#define RX_BUF_TOT_LEN 128
//DMA registers
#define DMAADL (0x100 + 0x00)		//DMA Address Low
#define DMAADH (0x100 + 0x04)		//DMA Address High
#define DMATL  (0x100 + 0x08)		//DMA Transfer Length
#define DMASTA (0x100 + 0x0c)  		//DMA Status

//DMA Status
#define DMAREAD	(1UL <<  31)
#define DMAACT	(1UL << 0)
#define DMAERR	(1UL << 1)
#define DMADONE	(1UL << 2) 
//////////////
#define UART_CAP_FIFO	(1 << 8)	/* UART has FIFO */
#define UART_CAP_EFR	(1 << 9)	/* UART has EFR */
#define UART_CAP_SLEEP	(1 << 10)	/* UART has IER sleep */
#define UART_CAP_AFE	(1 << 11)	/* MCR-based hw flow control */
#define UART_CAP_UUE	(1 << 12)	/* UART needs IER bit 6 set (Xscale) */
#define UART_CAP_RTOIE	(1 << 13)	/* UART needs IER bit 4 set (Xscale, Tegra) */

#define UART_BUG_QUOT	(1 << 0)	/* UART has buggy quot LSB */
#define UART_BUG_TXEN	(1 << 1)	/* UART has buggy TX IIR status */
#define UART_BUG_NOMSR	(1 << 2)	/* UART has buggy MSR status bits (Au1x00) */
#define UART_BUG_THRE	(1 << 3)	/* UART has buggy THRE reassertion */

#define PROBE_RSA	(1 << 0)
#define PROBE_ANY	(~0)

#define HIGH_BITS_OFFSET ((sizeof(long)-sizeof(int))*8)

#ifdef CONFIG_SERIAL_8250_SHARE_IRQ
#define SERIAL8250_SHARE_IRQS 1
#else
#define SERIAL8250_SHARE_IRQS 0
#endif

#if defined(__alpha__) && !defined(CONFIG_PCI)
/*
 * Digital did something really horribly wrong with the OUT1 and OUT2
 * lines on at least some ALPHA's.  The failure mode is that if either
 * is cleared, the machine locks up with endless interrupts.
 */
#define ALPHA_KLUDGE_MCR  (UART_MCR_OUT2 | UART_MCR_OUT1)
#elif defined(CONFIG_SBC8560)
/*
 * WindRiver did something similarly broken on their SBC8560 board. The
 * UART tristates its IRQ output while OUT2 is clear, but they pulled
 * the interrupt line _up_ instead of down, so if we register the IRQ
 * while the UART is in that state, we die in an IRQ storm. */
#define ALPHA_KLUDGE_MCR (UART_MCR_OUT2)
#else
#define ALPHA_KLUDGE_MCR 0
#endif
