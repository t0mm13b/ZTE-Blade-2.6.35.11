/*
 * u_smd.c - utilities for USB gadget serial over smd
 *
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This code also borrows from drivers/usb/gadget/u_serial.c, which is
 * Copyright (C) 2000 - 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 * Copyright (C) 1999 - 2002 Greg Kroah-Hartman (greg@kroah.com)
 * Copyright (C) 2000 Peter Berger (pberger@brimson.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/termios.h>
#include <mach/usb_gadget_fserial.h>
#include <mach/msm_smd.h>

#include "u_serial.h"

#define RX_QUEUE_SIZE		8
#define RX_BUF_SIZE		2048

#define TX_QUEUE_SIZE		8
#define TX_BUF_SIZE		2048

static struct workqueue_struct *gsmd_wq;

#define N_PORTS	2
#define CH_OPENED	0
struct smd_port_info {
	struct smd_channel	*ch;
	char			*name;
	unsigned long		flags;
	wait_queue_head_t	wait;
};

struct smd_port_info smd_pi[N_PORTS] = {
	{
		.name = "DS",
	},
	{
		.name = "UNUSED",
	},
};

struct gsmd_port {
	unsigned		port_num;
	spinlock_t		port_lock;

	unsigned		n_read;
	struct list_head	read_pool;
	struct list_head	read_queue;
	struct work_struct	push;

	struct list_head	write_pool;
	struct work_struct	pull;

	struct gserial		*port_usb;

	struct smd_port_info	*pi;
	struct work_struct	connect_work;

	/* At present, smd does not notify
	 * control bit change info from modem
	 */
	struct work_struct	update_modem_ctrl_sig;

#define ACM_CTRL_DTR		0x01
#define ACM_CTRL_RTS		0x02
	unsigned		cbits_to_modem;

#define ACM_CTRL_DCD		0x01
#define ACM_CTRL_DSR		0x02
#define ACM_CTRL_BRK		0x04
#define ACM_CTRL_RI		0x08
	unsigned		cbits_to_laptop;
};

static struct portmaster {
	struct mutex lock;
	struct gsmd_port *port;
} ports[N_PORTS];
static unsigned n_ports;

static void gsmd_free_req(struct usb_ep *ep, struct usb_request *req)
{
	kfree(req->buf);
	usb_ep_free_request(ep, req);
}

static void gsmd_free_requests(struct usb_ep *ep, struct list_head *head)
{
	struct usb_request	*req;

	while (!list_empty(head)) {
		req = list_entry(head->next, struct usb_request, list);
		list_del(&req->list);
		gsmd_free_req(ep, req);
	}
}

static struct usb_request *
gsmd_alloc_req(struct usb_ep *ep, unsigned len, gfp_t flags)
{
	struct usb_request *req;

	req = usb_ep_alloc_request(ep, flags);
	if (!req) {
		pr_err("%s: usb alloc request failed\n", __func__);
		return 0;
	}

	req->length = len;
	req->buf = kmalloc(len, flags);
	if (!req->buf) {
		pr_err("%s: request buf allocation failed\n", __func__);
		usb_ep_free_request(ep, req);
		return 0;
	}

	return req;
}

static int gsmd_alloc_requests(struct usb_ep *ep, struct list_head *head,
		int num, int size,
		void (*cb)(struct usb_ep *ep, struct usb_request *))
{
	int i;
	struct usb_request *req;

	pr_debug("%s: ep:%p head:%p num:%d size:%d cb:%p", __func__,
			ep, head, num, size, cb);

	for (i = 0; i < num; i++) {
		req = gsmd_alloc_req(ep, size, GFP_ATOMIC);
		if (!req) {
			pr_debug("%s: req allocated:%d\n", __func__, i);
			return list_empty(head) ? -ENOMEM : 0;
		}
		req->complete = cb;
		list_add(&req->list, head);
	}

	return 0;
}

static void gsmd_start_rx(struct gsmd_port *port)
{
	struct list_head	*pool;
	struct usb_ep		*out;
	int ret;

	if (!port) {
		pr_err("%s: port is null\n", __func__);
		return;
	}

	spin_lock_irq(&port->port_lock);

	if (!port->port_usb) {
		pr_debug("%s: USB disconnected\n", __func__);
		goto start_rx_end;
	}

	pool = &port->read_pool;
	out = port->port_usb->out;

	while (!list_empty(pool)) {
		struct usb_request	*req;

		req = list_entry(pool->next, struct usb_request, list);
		list_del(&req->list);
		req->length = RX_BUF_SIZE;

		spin_unlock_irq(&port->port_lock);
		ret = usb_ep_queue(out, req, GFP_KERNEL);
		spin_lock_irq(&port->port_lock);
		if (ret) {
			pr_err("%s: usb ep out queue failed"
					"port:%p, port#%d\n",
					 __func__, port, port->port_num);
			list_add_tail(&req->list, pool);
			break;
		}
	}
start_rx_end:
	spin_unlock_irq(&port->port_lock);
}

static void gsmd_rx_push(struct work_struct *w)
{
	struct gsmd_port *port = container_of(w, struct gsmd_port, push);
	struct list_head *q;

	pr_debug("%s: port:%p port#%d", __func__, port, port->port_num);

	spin_lock_irq(&port->port_lock);

	q = &port->read_queue;
	while (!list_empty(q)) {
		struct usb_request *req;
		int avail;
		struct smd_port_info *pi = port->pi;

		req = list_first_entry(q, struct usb_request, list);

		switch (req->status) {
		case -ESHUTDOWN:
			pr_debug("%s: req status shutdown portno#%d port:%p\n",
					__func__, port->port_num, port);
			goto rx_push_end;
		default:
			pr_warning("%s: port:%p port#%d"
					" Unexpected Rx Status:%d\n", __func__,
					port, port->port_num, req->status);
		case 0:
			/* normal completion */
			break;
		}

		avail = smd_write_avail(pi->ch);
		if (!avail)
			goto rx_push_end;

		if (req->actual) {
			char		*packet = req->buf;
			unsigned	size = req->actual;
			unsigned	n;
			unsigned	count;

			n = port->n_read;
			if (n) {
				packet += n;
				size -= n;
			}

			count = smd_write(pi->ch, packet, size);
			if (count != size) {
				port->n_read += count;
				goto rx_push_end;
			}
		}

		list_move(&req->list, &port->read_pool);
	}

rx_push_end:
	spin_unlock_irq(&port->port_lock);

	gsmd_start_rx(port);
}

static void gsmd_read_pending(struct gsmd_port *port)
{
	int avail;

	if (!port || !port->pi->ch)
		return;

	/* passing null buffer discards the data */
	while ((avail = smd_read_avail(port->pi->ch)))
		smd_read(port->pi->ch, 0, avail);

	return;
}

static void gsmd_tx_pull(struct work_struct *w)
{
	struct gsmd_port *port = container_of(w, struct gsmd_port, pull);
	struct list_head *pool = &port->write_pool;

	pr_debug("%s: port:%p port#%d pool:%p\n", __func__,
			port, port->port_num, pool);

	if (!port->port_usb) {
		pr_debug("%s: usb is disconnected\n", __func__);
		gsmd_read_pending(port);
		return;
	}

	spin_lock_irq(&port->port_lock);
	while (!list_empty(pool)) {
		struct usb_request *req;
		struct usb_ep *in = port->port_usb->in;
		struct smd_port_info *pi = port->pi;
		int avail;
		int ret;

		avail = smd_read_avail(pi->ch);
		if (!avail)
			break;

		avail = avail > TX_BUF_SIZE ? TX_BUF_SIZE : avail;

		req = list_entry(pool->next, struct usb_request, list);
		list_del(&req->list);
		req->length = smd_read(pi->ch, req->buf, avail);

		spin_unlock_irq(&port->port_lock);
		ret = usb_ep_queue(in, req, GFP_KERNEL);
		spin_lock_irq(&port->port_lock);
		if (ret) {
			pr_err("%s: usb ep out queue failed"
					"port:%p, port#%d err:%d\n",
					__func__, port, port->port_num, ret);
			/* could be usb disconnected */
			if (!port->port_usb)
				gsmd_free_req(in, req);
			else
				list_add(&req->list, pool);
			goto tx_pull_end;
		}
	}

tx_pull_end:
	/* TBD: Check how code behaves on USB bus suspend */
	if (port->port_usb && smd_read_avail(port->pi->ch) && !list_empty(pool))
		queue_work(gsmd_wq, &port->pull);

	spin_unlock_irq(&port->port_lock);

	return;
}

static void gsmd_read_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct gsmd_port *port = ep->driver_data;
	unsigned long flags;

	pr_debug("%s: ep:%p port:%p\n", __func__, ep, port);

	if (!port) {
		pr_err("%s: port is null\n", __func__);
		return;
	}

	spin_lock_irqsave(&port->port_lock, flags);
	list_add_tail(&req->list, &port->read_queue);
	queue_work(gsmd_wq, &port->push);
	spin_unlock_irqrestore(&port->port_lock, flags);

	return;
}

static void gsmd_write_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct gsmd_port *port = ep->driver_data;
	unsigned long flags;

	pr_debug("%s: ep:%p port:%p\n", __func__, ep, port);

	if (!port) {
		pr_err("%s: port is null\n", __func__);
		return;
	}

	spin_lock_irqsave(&port->port_lock, flags);
	list_add(&req->list, &port->write_pool);

	switch (req->status) {
	default:
		pr_warning("%s: port:%p port#%d unexpected %s status %d\n",
				__func__, port, port->port_num,
				ep->name, req->status);
		/* FALL THROUGH */
	case 0:
		queue_work(gsmd_wq, &port->pull);
		break;

	case -ESHUTDOWN:
		/* disconnect */
		pr_debug("%s: %s shutdown\n", __func__, ep->name);
		gsmd_free_req(ep, req);
		break;
	}

	spin_unlock_irqrestore(&port->port_lock, flags);

	return;
}

static void gsmd_start_io(struct gsmd_port *port)
{
	int ret;

	pr_debug("%s: port: %p\n", __func__, port);

	ret = gsmd_alloc_requests(port->port_usb->out,
				&port->read_pool,
				RX_QUEUE_SIZE, RX_BUF_SIZE,
				gsmd_read_complete);
	if (ret) {
		pr_err("%s: unable to allocate out requests\n",
				__func__);
		return;
	}

	ret = gsmd_alloc_requests(port->port_usb->in,
				&port->write_pool,
				TX_QUEUE_SIZE, TX_BUF_SIZE,
				gsmd_write_complete);
	if (ret) {
		pr_err("%s: unable to allocate IN requests\n",
				__func__);
		gsmd_free_requests(port->port_usb->out, &port->read_pool);
		return;
	}

	gsmd_start_rx(port);
}

static unsigned int convert_uart_sigs_to_acm(unsigned uart_sig)
{
	unsigned int acm_sig = 0;

	/* should this needs to be in calling functions ??? */
	uart_sig &= (TIOCM_RI | TIOCM_CD | TIOCM_DSR);

	if (uart_sig & TIOCM_RI)
		acm_sig |= ACM_CTRL_RI;
	if (uart_sig & TIOCM_CD)
		acm_sig |= ACM_CTRL_DCD;
	if (uart_sig & TIOCM_DSR)
		acm_sig |= ACM_CTRL_DSR;

	return acm_sig;
}

static unsigned int convert_acm_sigs_to_uart(unsigned acm_sig)
{
	unsigned int uart_sig = 0;

	/* should this needs to be in calling functions ??? */
	acm_sig &= (ACM_CTRL_DTR | ACM_CTRL_RTS);

	if (acm_sig & ACM_CTRL_DTR)
		uart_sig |= TIOCM_DTR;
	if (acm_sig & ACM_CTRL_RTS)
		uart_sig |= TIOCM_RTS;

	return uart_sig;
}

static void gsmd_notify(void *priv, unsigned event)
{
	struct gsmd_port *port = priv;
	struct smd_port_info *pi = port->pi;
	int i;

	switch (event) {
	case SMD_EVENT_DATA:
		pr_debug("%s: Event data\n", __func__);
		if (smd_read_avail(pi->ch))
			queue_work(gsmd_wq, &port->pull);
		if (smd_write_avail(pi->ch))
			queue_work(gsmd_wq, &port->push);
		break;
	case SMD_EVENT_OPEN:
		pr_debug("%s: Event Open\n", __func__);
		set_bit(CH_OPENED, &pi->flags);
		wake_up(&pi->wait);
		break;
	case SMD_EVENT_CLOSE:
		pr_debug("%s: Event Close\n", __func__);
		clear_bit(CH_OPENED, &pi->flags);
		break;
	case SMD_EVENT_STATUS:
		i = smd_tiocmget(port->pi->ch);
		port->cbits_to_laptop = convert_uart_sigs_to_acm(i);
		if (port->port_usb && port->port_usb->send_modem_ctrl_bits)
			port->port_usb->send_modem_ctrl_bits(port->port_usb,
						port->cbits_to_laptop);
		break;
	}
}

#define MAX_SMD_RETRY_CNT	20
static void gsmd_connect_work(struct work_struct *w)
{
	struct gsmd_port *port;
	struct smd_port_info *pi;
	int ret;
	int retry_cnt = 0;

	port = container_of(w, struct gsmd_port, connect_work);
	pi = port->pi;

	pr_debug("%s: port:%p port#%d\n", __func__, port, port->port_num);

	/* SMD driver comes online gets initialized and loads modem
	 * 10 seconds after boot up. If USB cable is connected at boot-up,
	 * this might result smd open failure. To work-around, retry
	 * opening multiple times.
	 */
	do {
		if (!port->port_usb)
			return;

		ret = smd_named_open_on_edge(pi->name, SMD_APPS_MODEM,
					&pi->ch, port, gsmd_notify);
		if (!ret)
			break;

		retry_cnt++;
		msleep(1000);
	} while (retry_cnt < MAX_SMD_RETRY_CNT);

	if (ret) {
		pr_err("%s: unable to open smd port:%s err:%d\n",
				__func__, pi->name, ret);
		return;
	}

	pr_debug("%s: SMD port open successful retrycnt:%d\n",
			__func__, retry_cnt);

	wait_event(pi->wait, test_bit(CH_OPENED, &pi->flags));

	if (!port->port_usb)
		return;

	/* update usb control signals to modem */
	if (port->cbits_to_modem)
		smd_tiocmset(port->pi->ch,
			port->cbits_to_modem,
			~port->cbits_to_modem);

	gsmd_start_io(port);
}

static void gsmd_notify_modem(struct gserial *gser, u8 portno, int ctrl_bits)
{
	struct gsmd_port *port;
	int temp;

	if (portno >= n_ports) {
		pr_err("%s: invalid portno#%d\n", __func__, portno);
		return;
	}

	if (!gser) {
		pr_err("%s: gser is null\n", __func__);
		return;
	}

	port = ports[portno].port;

	temp = convert_acm_sigs_to_uart(ctrl_bits);

	if (temp == port->cbits_to_modem)
		return;

	port->cbits_to_modem = temp;

	/* usb could send control signal before smd is ready */
	if (!test_bit(CH_OPENED, &port->pi->flags))
		return;

	/* if DTR is high, update latest modem info to laptop */
	if (port->cbits_to_modem & TIOCM_DTR) {
		unsigned i;

		i = smd_tiocmget(port->pi->ch);
		port->cbits_to_laptop = convert_uart_sigs_to_acm(i);

		if (gser->send_modem_ctrl_bits)
			gser->send_modem_ctrl_bits(
					port->port_usb,
					port->cbits_to_laptop);
	}

	smd_tiocmset(port->pi->ch,
			port->cbits_to_modem,
			~port->cbits_to_modem);
}

int gsmd_connect(struct gserial *gser, u8 portno)
{
	unsigned long flags;
	int ret;
	struct gsmd_port *port;

	pr_debug("%s: gserial:%p portno:%u\n", __func__, gser, portno);

	if (portno >= n_ports) {
		pr_err("%s: Invalid port no#%d", __func__, portno);
		return -EINVAL;
	}

	if (!gser) {
		pr_err("%s: gser is null\n", __func__);
		return -EINVAL;
	}

	port = ports[portno].port;

	spin_lock_irqsave(&port->port_lock, flags);
	port->port_usb = gser;
	gser->notify_modem = gsmd_notify_modem;
	spin_unlock_irqrestore(&port->port_lock, flags);

	ret = usb_ep_enable(gser->in, gser->in_desc);
	if (ret) {
		pr_err("%s: usb_ep_enable failed eptype:IN ep:%p",
				__func__, gser->in);
		port->port_usb = 0;
		return ret;
	}
	gser->in->driver_data = port;

	ret = usb_ep_enable(gser->out, gser->out_desc);
	if (ret) {
		pr_err("%s: usb_ep_enable failed eptype:OUT ep:%p",
				__func__, gser->out);
		port->port_usb = 0;
		gser->in->driver_data = 0;
		return ret;
	}
	gser->out->driver_data = port;

	queue_work(gsmd_wq, &port->connect_work);

	return 0;
}

void gsmd_disconnect(struct gserial *gser, u8 portno)
{
	unsigned long flags;
	struct gsmd_port *port;

	pr_debug("%s: gserial:%p portno:%u\n", __func__, gser, portno);

	if (portno >= n_ports) {
		pr_err("%s: invalid portno#%d\n", __func__, portno);
		return;
	}

	if (!gser) {
		pr_err("%s: gser is null\n", __func__);
		return;
	}

	port = ports[portno].port;

	spin_lock_irqsave(&port->port_lock, flags);
	port->port_usb = 0;
	spin_unlock_irqrestore(&port->port_lock, flags);

	/* disable endpoints, aborting down any active I/O */
	usb_ep_disable(gser->out);
	gser->out->driver_data = NULL;

	usb_ep_disable(gser->in);
	gser->in->driver_data = NULL;

	spin_lock_irqsave(&port->port_lock, flags);
	gsmd_free_requests(gser->out, &port->read_pool);
	gsmd_free_requests(gser->out, &port->read_queue);
	gsmd_free_requests(gser->in, &port->write_pool);
	spin_unlock_irqrestore(&port->port_lock, flags);

	if (!test_bit(CH_OPENED, &port->pi->flags))
		return;

	/* lower the dtr */
	port->cbits_to_modem = 0;
	smd_tiocmset(port->pi->ch,
			port->cbits_to_modem,
			~port->cbits_to_modem);

	smd_close(port->pi->ch);
	port->pi->flags = 0;
}

static void gsmd_port_free(int portno)
{
	struct gsmd_port *port = ports[portno].port;

	if (!port)
		kfree(port);
}

static int gsmd_port_alloc(int portno, struct usb_cdc_line_coding *coding)
{
	struct gsmd_port *port;

	port = kzalloc(sizeof(struct gsmd_port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	port->port_num = portno;
	port->pi = &smd_pi[portno];

	spin_lock_init(&port->port_lock);

	INIT_LIST_HEAD(&port->read_pool);
	INIT_LIST_HEAD(&port->read_queue);
	INIT_WORK(&port->push, gsmd_rx_push);

	INIT_LIST_HEAD(&port->write_pool);
	INIT_WORK(&port->pull, gsmd_tx_pull);

	INIT_WORK(&port->connect_work, gsmd_connect_work);
	init_waitqueue_head(&port->pi->wait);

	ports[portno].port = port;

	pr_debug("%s: port:%p portno:%d\n", __func__, port, portno);

	return 0;
}

int gsmd_setup(struct usb_gadget *g, unsigned count)
{
	struct usb_cdc_line_coding	coding;
	int ret;
	int i;

	pr_debug("%s: g:%p count: %d\n", __func__, g, count);

	if (!count || count > N_PORTS) {
		pr_err("%s: Invalid num of ports count:%d gadget:%p\n",
				__func__, count, g);
		return -EINVAL;
	}

	coding.dwDTERate = cpu_to_le32(9600);
	coding.bCharFormat = 8;
	coding.bParityType = USB_CDC_NO_PARITY;
	coding.bDataBits = USB_CDC_1_STOP_BITS;

	gsmd_wq = create_singlethread_workqueue("k_gsmd");
	if (!gsmd_wq) {
		pr_err("%s: Unable to create workqueue gsmd_wq\n",
				__func__);
		return -ENOMEM;
	}

	for (i = 0; i < count; i++) {
		mutex_init(&ports[i].lock);
		ret = gsmd_port_alloc(i, &coding);
		if (ret) {
			pr_err("%s: Unable to alloc port:%d\n", __func__, i);
			goto free_ports;
		}
		n_ports++;
	}
	return 0;
free_ports:
	for (i = 0; i < n_ports; i++)
		gsmd_port_free(i);

	destroy_workqueue(gsmd_wq);

	return ret;
}

void gsmd_cleanup(struct usb_gadget *g, unsigned count)
{
	/* TBD */
}
