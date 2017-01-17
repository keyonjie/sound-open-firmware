/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Intel Corporation nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
 *         Keyon Jie <yang.jie@linux.intel.com>
 */

#include <reef/debug.h>
#include <reef/timer.h>
#include <reef/interrupt.h>
#include <reef/ipc.h>
#include <reef/mailbox.h>
#include <reef/reef.h>
#include <reef/stream.h>
#include <reef/dai.h>
#include <reef/dma.h>
#include <reef/alloc.h>
#include <reef/wait.h>
#include <reef/trace.h>
#include <reef/ssp.h>
#include <platform/interrupt.h>
#include <platform/mailbox.h>
#include <platform/shim.h>
#include <platform/dma.h>
#include <platform/platform.h>
#include <reef/audio/component.h>
#include <reef/audio/pipeline.h>
#include <uapi/intel-ipc.h>
#include <reef/intel-ipc.h>

extern struct ipc *_ipc;

static void do_notify(void)
{
	uint32_t flags;
	struct ipc_msg *msg;

	tracev_ipc("Not");

	spin_lock_irq(&_ipc->lock, flags);
	msg = _ipc->dsp_msg;
	if (msg == NULL)
		goto out;

	/* copy the data returned from DSP - TODO: this is copying data to address 0  */
	if (msg->rx_size && msg->rx_size < MSG_MAX_SIZE)
		mailbox_inbox_read(0, msg->rx_data, msg->rx_size);

	/* any callback ? */
	if (msg->cb)
		msg->cb(msg->cb_data, msg->rx_data);

	list_item_append(&msg->list, &_ipc->empty_list);

out:
	spin_unlock_irq(&_ipc->lock, flags);
	/* clear DONE bit - tell Host we have completed */
	shim_write(SHIM_IPCDH, shim_read(SHIM_IPCDH) & ~SHIM_IPCDH_DONE);

	/* unmask Done interrupt */
	shim_write(SHIM_IMRD, shim_read(SHIM_IMRD) & ~SHIM_IMRD_DONE);
}

/* test code to check working IRQ */
static void irq_handler(void *arg)
{
	uint32_t isr;

	tracev_ipc("IRQ");

	/* Interrupt arrived, check src */
	isr = shim_read(SHIM_ISRD);

	if (isr & SHIM_ISRD_DONE) {

		/* Mask Done interrupt before return */
		shim_write(SHIM_IMRD, shim_read(SHIM_IMRD) | SHIM_IMRD_DONE);
		interrupt_clear(PLATFORM_IPC_INTERUPT);
		do_notify();
	}

	if (isr & SHIM_ISRD_BUSY) {

		/* Mask Busy interrupt before return */
		shim_write(SHIM_IMRD, shim_read(SHIM_IMRD) | SHIM_IMRD_BUSY);
		interrupt_clear(PLATFORM_IPC_INTERUPT);

		/* TODO: place message in Q and process later */
		/* It's not Q ATM, may overwrite */
		if (_ipc->host_pending)
			trace_ipc_error("Pen");
		_ipc->host_msg = shim_read(SHIM_IPCXL);
		_ipc->host_pending = 1;
	}
}

void ipc_platform_do_cmd(struct ipc *ipc)
{
	struct intel_ipc_data *iipc = ipc_get_drvdata(ipc);
	uint32_t ipcxh, status;

	trace_ipc("Cmd");
	//trace_value(_ipc->host_msg);

	status = ipc_cmd();
	ipc->host_pending = 0;

	/* clear BUSY bit and set DONE bit - accept new messages */
	ipcxh = shim_read(SHIM_IPCXH);
	ipcxh &= ~SHIM_IPCXH_BUSY;
	ipcxh |= SHIM_IPCXH_DONE | status;
	shim_write(SHIM_IPCXH, ipcxh);

	// TODO: signal audio work to enter D3 in normal context
	/* are we about to enter D3 ? */
	if (iipc->pm_prepare_D3) {
		while (1)
			wait_for_interrupt(0);
	}

	/* unmask busy interrupt */
	shim_write(SHIM_IMRD, shim_read(SHIM_IMRD) & ~SHIM_IMRD_BUSY);
}

void ipc_platform_send_msg(struct ipc *ipc)
{
	struct ipc_msg *msg;
	uint32_t flags;

	spin_lock_irq(&ipc->lock, flags);

	/* any messages to send ? */
	if (list_is_empty(&ipc->msg_list)) {
		ipc->dsp_pending = 0;
		goto out;
	}

	/* can't send nofication when one is in progress */
	if (shim_read(SHIM_IPCDH) & (SHIM_IPCDH_BUSY | SHIM_IPCDH_DONE))
		goto out;

	/* now send the message */
	msg = list_first_item(&ipc->msg_list, struct ipc_msg, list);
	mailbox_outbox_write(0, msg->tx_data, msg->tx_size);
	list_item_del(&msg->list);
	ipc->dsp_msg = msg;
	tracev_ipc("Msg");

	/* now interrupt host to tell it we have message sent */
	shim_write(SHIM_IPCDL, msg->header);
	shim_write(SHIM_IPCDH, SHIM_IPCDH_BUSY);

out:
	spin_unlock_irq(&ipc->lock, flags);
}

int platform_ipc_init(struct ipc *ipc)
{
	struct intel_ipc_data *iipc;
	uint32_t imrd;
	int i;

	_ipc = ipc;

	/* init ipc data */
	iipc = rzalloc(RZONE_DEV, RMOD_SYS, sizeof(struct intel_ipc_data));
	ipc_set_drvdata(_ipc, iipc);
	_ipc->dsp_msg = NULL;
	list_init(&ipc->empty_list);
	list_init(&ipc->msg_list);
	spinlock_init(&ipc->lock);
	for (i = 0; i < MSG_QUEUE_SIZE; i++)
		list_item_prepend(&ipc->message[i].list, &ipc->empty_list);

	/* allocate page table buffer */
	iipc->page_table = rballoc(RZONE_DEV, RMOD_SYS,
		IPC_INTEL_PAGE_TABLE_SIZE);
	if (iipc->page_table)
		bzero(iipc->page_table, IPC_INTEL_PAGE_TABLE_SIZE);

	/* dma */
	iipc->dmac0 = dma_get(DMA_ID_DMAC0);

	/* PM */
	iipc->pm_prepare_D3 = 0;

	/* configure interrupt */
	interrupt_register(PLATFORM_IPC_INTERUPT, irq_handler, NULL);
	interrupt_enable(PLATFORM_IPC_INTERUPT);

	/* Unmask Busy and Done interrupts */
	imrd = shim_read(SHIM_IMRD);
	imrd &= ~(SHIM_IMRD_BUSY | SHIM_IMRD_DONE);
	shim_write(SHIM_IMRD, imrd);

	return 0;
}

