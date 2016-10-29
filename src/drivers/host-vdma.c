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
 */

#include <reef/debug.h>
#include <reef/reef.h>
#include <reef/dma.h>
#include <reef/host-vdma.h>
#include <reef/stream.h>
#include <reef/timer.h>
#include <reef/alloc.h>
#include <reef/interrupt.h>
#include <reef/work.h>
#include <reef/lock.h>
#include <reef/trace.h>
#include <reef/wait.h>
#include <platform/dma.h>
#include <platform/platform.h>
#include <platform/interrupt.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>


/* tracing */
#define trace_dma(__e)	trace_event(TRACE_CLASS_DMA, __e)
#define trace_dma_error(__e)	trace_error(TRACE_CLASS_DMA, __e)
#define tracev_dma(__e)	tracev_event(TRACE_CLASS_DMA, __e)


/* data for each DMA channel */
struct dma_chan_data {
	uint32_t status;
	uint32_t direction;

	struct vdma_lli *lli;
	struct vdma_lli *lli_current;
	uint32_t desc_count;

	struct dma *dma;
	int32_t channel;
	pthread_t thread;

	uint32_t dest_dev;
	uint32_t src_dev;

	void (*cb)(void *data, uint32_t type);	/* client callback function */
	void *cb_data;		/* client callback data */
	int cb_type;		/* callback type */
};

#define DW_MAX_CHAN	8

/* private data for DW DMA engine */
struct dma_pdata {
	struct dma_chan_data chan[DW_MAX_CHAN];
	uint32_t class;		/* channel class - set for controller atm */
};

/* allocate next free DMA channel */
static int host_vdma_channel_get(struct dma *dma)
{
	struct dma_pdata *p = dma_get_drvdata(dma);
	uint32_t flags;
	int i;

	spin_lock_irq(&dma->lock, flags);

	trace_dma("Dgt");

	/* find first free non draining channel */
	for (i = 0; i < DW_MAX_CHAN; i++) {

		/* use channel if it's free */
		if (p->chan[i].status != DMA_STATUS_FREE)
			continue;

		p->chan[i].status = DMA_STATUS_IDLE;

		/* unmask block, transfer and error interrupts for channel */

		/* return channel */
		spin_unlock_irq(&dma->lock, flags);
		return i;
	}

	/* DMAC has no free channels */
	spin_unlock_irq(&dma->lock, flags);
	trace_dma_error("eDg");
	return -ENODEV;
}

/* channel must not be running when this is called */
static void host_vdma_channel_put_unlocked(struct dma *dma, int channel)
{
	struct dma_pdata *p = dma_get_drvdata(dma);

	trace_dma("Dpt");

	/* channel can only be freed if it's not still draining */
	if (p->chan[channel].status == DMA_STATUS_DRAINING) {
		p->chan[channel].status = DMA_STATUS_CLOSING;
		return;
	}

	/* mask block, transfer and error interrupts for channel */

	/* free the lli allocated by set_config*/
	if (p->chan[channel].lli) {
		rfree(RZONE_MODULE, RMOD_SYS, p->chan[channel].lli);
		p->chan[channel].lli = NULL;
	}

	/* set new state */
	p->chan[channel].status = DMA_STATUS_FREE;
	p->chan[channel].cb = NULL;
	p->chan[channel].desc_count = 0;
}

/* channel must not be running when this is called */
static void host_vdma_channel_put(struct dma *dma, int channel)
{
	uint32_t flags;

	spin_lock_irq(&dma->lock, flags);
	host_vdma_channel_put_unlocked(dma, channel);
	spin_unlock_irq(&dma->lock, flags);
}

static void *copier_thread(void *data)
{
	struct dma_chan_data *chan = data;
	//struct vdma_lli *lli_desc = chan->lli;
	//int ret = 0;
	void *buf;

	buf = calloc(1, 1024 * 16);
	if (buf == NULL) {
		//ret = -ENOMEM;
		goto out;
	}

	while (chan->status == DMA_STATUS_RUNNING) {

	}

out:
	return NULL;
}

static int host_vdma_start(struct dma *dma, int channel)
{
	struct dma_pdata *p = dma_get_drvdata(dma);
	void *(*copier)(void *) = NULL;
	uint32_t flags;
	int ret = 0;

	spin_lock_irq(&dma->lock, flags);

	tracev_dma("DEn");

	/* valid stream ? */
	if (p->chan[channel].lli == NULL) {
		ret = -EINVAL;
		trace_dma_error("eDv");
		goto out;
	}

	p->chan[channel].status = DMA_STATUS_RUNNING;

	switch (p->chan[channel].direction) {
	case DMA_DIR_LMEM_TO_HMEM:
		// HOST device WAV or PCM defined by HOST address
		//if (p->chan[channel].dest_dev == )
		break;
	case DMA_DIR_HMEM_TO_LMEM:
		break;
	case DMA_DIR_MEM_TO_MEM:
		break;
	case DMA_DIR_MEM_TO_DEV:
		break;
	case DMA_DIR_DEV_TO_MEM:
		break;
	case DMA_DIR_DEV_TO_DEV:
		break;
	default:
		break;
	}

	/* start copier thread for this channel */
	ret = pthread_create(&p->chan[channel].thread, NULL,
			copier, &p->chan[channel]);

out:
	spin_unlock_irq(&dma->lock, flags);
	return ret;
}

static int host_vdma_release(struct dma *dma, int channel)
{
	struct dma_pdata *p = dma_get_drvdata(dma);
	uint32_t flags;

	spin_lock_irq(&dma->lock, flags);

	trace_dma("Dpr");

	/* resume and reload DMA */
	p->chan[channel].status = DMA_STATUS_RUNNING;

	spin_unlock_irq(&dma->lock, flags);
	return 0;
}

static int host_vdma_pause(struct dma *dma, int channel)
{
	struct dma_pdata *p = dma_get_drvdata(dma);
	uint32_t flags;

	spin_lock_irq(&dma->lock, flags);

	trace_dma("Dpa");

	if (p->chan[channel].status != DMA_STATUS_RUNNING)
		goto out;

	/* pause the channel */
	p->chan[channel].status = DMA_STATUS_PAUSING;

out:
	spin_unlock_irq(&dma->lock, flags);
	return 0;
}


static int host_vdma_stop(struct dma *dma, int channel, int drain)
{
	struct dma_pdata *p = dma_get_drvdata(dma);
	int ret = 0;
	uint32_t flags;

	spin_lock_irq(&dma->lock, flags);

	trace_dma("DDi");

	/* stop the copier thread and wait for completion */
	p->chan[channel].status = DMA_STATUS_STOPPING;
	ret = pthread_join(p->chan[channel].thread, NULL);

	spin_unlock_irq(&dma->lock, flags);

	return ret;
}

/* fill in "status" with current DMA channel state and position */
static int host_vdma_status(struct dma *dma, int channel,
	struct dma_chan_status *status, uint8_t direction)
{
	struct dma_pdata *p = dma_get_drvdata(dma);

	status->state = p->chan[channel].status;
	//status->r_pos = dw_read(dma, DW_SAR(channel));
	//status->w_pos = dw_read(dma, DW_DAR(channel));
	status->timestamp = timer_get_system(NULL);

	return 0;
}

/* set the DMA channel configuration, source/target address, buffer sizes */
static int host_vdma_set_config(struct dma *dma, int channel,
	struct dma_sg_config *config)
{
	struct dma_pdata *p = dma_get_drvdata(dma);
	struct list_item *plist;
	struct dma_sg_elem *sg_elem;
	struct vdma_lli *lli_desc, *lli_desc_head, *lli_desc_tail;
	uint32_t desc_count = 0, flags;

	spin_lock_irq(&dma->lock, flags);

	tracev_dma("Dsc");

	/* get number of SG elems */
	list_for_item(plist, &config->elem_list)
		desc_count++;

	if (desc_count == 0) {
		trace_dma_error("eDC");
		return -EINVAL;
	}

	/* set source/dest devices */
	p->chan[channel].src_dev = config->src_dev;
	p->chan[channel].dest_dev = config->dest_dev;

	/* do we need to realloc descriptors */
	if (desc_count != p->chan[channel].desc_count) {

		p->chan[channel].desc_count = desc_count;

		/* allocate descriptors for channel */
		if (p->chan[channel].lli)
			rfree(RZONE_MODULE, RMOD_SYS, p->chan[channel].lli);
		p->chan[channel].lli = rzalloc(RZONE_MODULE, RMOD_SYS,
			sizeof(struct vdma_lli) * p->chan[channel].desc_count);
		if (p->chan[channel].lli == NULL) {
			trace_dma_error("eDm");
			return -ENOMEM;
		}
	}

	/* initialise descriptors */
	bzero(p->chan[channel].lli, sizeof(struct vdma_lli) *
		p->chan[channel].desc_count);
	lli_desc = lli_desc_head = p->chan[channel].lli;
	lli_desc_tail = p->chan[channel].lli + p->chan[channel].desc_count - 1;

	/* fill in lli for the elem in the list */
	list_for_item(plist, &config->elem_list) {

		sg_elem = container_of(plist, struct dma_sg_elem, list);
		lli_desc->direction = config->direction;
		lli_desc->sar = sg_elem->src;
		lli_desc->dar = sg_elem->dest;
		lli_desc->size = sg_elem->size;

		/* set next descriptor in list */
		lli_desc->llp = (lli_desc + 1);

		/* next descriptor */
		lli_desc++;
	}

	/* end of list or cyclic buffer ? */
	if (config->cyclic) {
		lli_desc_tail->llp = lli_desc_head;
	} else {
		lli_desc_tail->llp = 0x0;

	}

	spin_unlock_irq(&dma->lock, flags);

	return 0;
}

static void host_vdma_set_cb(struct dma *dma, int channel, int type,
		void (*cb)(void *data, uint32_t type), void *data)
{
	struct dma_pdata *p = dma_get_drvdata(dma);
	uint32_t flags;

	spin_lock_irq(&dma->lock, flags);
	p->chan[channel].cb = cb;
	p->chan[channel].cb_data = data;
	p->chan[channel].cb_type = type;
	spin_unlock_irq(&dma->lock, flags);
}

/* this will probably be called at the end of every period copied */
static void host_vdma_irq_handler(void *data)
{
#if 0
	struct dma *dma = (struct dma *)data;
	struct dma_pdata *p = dma_get_drvdata(dma);
	uint32_t status_tfr = 0, status_block = 0, status_err = 0, status_intr;
	uint32_t mask, pmask;
	int i;

	tracev_dma("DIr");

	for (i = 0; i < DW_MAX_CHAN; i++) {

		/* skip if channel is not running or pausing*/
		if (p->chan[i].status != DMA_STATUS_RUNNING &&
			p->chan[i].status != DMA_STATUS_PAUSING)
			continue;

		mask = 0x1 << i;

		/* end of a transfer */
		if (status_tfr & mask &&
			p->chan[i].cb_type & DMA_IRQ_TYPE_LLIST) {

			if (p->chan[i].status == DMA_STATUS_PAUSING) {
				p->chan[i].status = DMA_STATUS_PAUSED;
				continue;
			}

			if (p->chan[i].cb)
				p->chan[i].cb(p->chan[i].cb_data,
					DMA_IRQ_TYPE_LLIST);

			/* check for reload channel */
			//host_vdma_chan_reload(dma, i);
		}

	}
#endif
}

static int host_vdma_probe(struct dma *dma)
{
	struct dma_pdata *dw_pdata;
	int i;

	/* allocate private data */
	dw_pdata = rzalloc(RZONE_DEV, RMOD_SYS, sizeof(*dw_pdata));
	dma_set_drvdata(dma, dw_pdata);

	spinlock_init(&dma->lock);

	/* init work */
	for (i = 0; i < DW_MAX_CHAN; i++) {
		dw_pdata->chan[i].dma = dma;
		dw_pdata->chan[i].channel = i;
	}

	/* register our IRQ handler */
	interrupt_register(dma_irq(dma), host_vdma_irq_handler, dma);
	interrupt_enable(dma_irq(dma));

	return 0;
}

const struct dma_ops host_vdma_ops = {
	.channel_get	= host_vdma_channel_get,
	.channel_put	= host_vdma_channel_put,
	.start		= host_vdma_start,
	.stop		= host_vdma_stop,
	.pause		= host_vdma_pause,
	.release	= host_vdma_release,
	.status		= host_vdma_status,
	.set_config	= host_vdma_set_config,
	.set_cb		= host_vdma_set_cb,
	.probe		= host_vdma_probe,
};
