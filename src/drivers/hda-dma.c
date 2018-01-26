/*
 * Copyright (c) 2018, Intel Corporation
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
 * Author: Keyon Jie <yang.jie@linux.intel.com>
 *         Liam Girdwood <liam.r.girdwood@linux.intel.com>
 */

#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <reef/reef.h>
#include <reef/lock.h>
#include <reef/list.h>
#include <reef/stream.h>
#include <reef/alloc.h>
#include <reef/trace.h>
#include <reef/dma.h>
#include <reef/io.h>
#include <reef/ipc.h>
#include <reef/wait.h>
#include <platform/dma.h>
#include <arch/cache.h>
#include <uapi/ipc.h>

#define trace_host(__e)	trace_event(TRACE_CLASS_HOST, __e)
#define tracev_host(__e)	tracev_event(TRACE_CLASS_HOST, __e)
#define trace_host_error(__e)	trace_error(TRACE_CLASS_HOST, __e)

/* Gateway Stream Registers */
#define DGCS		0x00
#define DGBBA		0x04
#define DGBS		0x08
#define DGBFPI		0x0c /* firmware need to update this when DGCS.FWCB=1 */
#define DGBRP		0x10 /* Read Only, read pointer */
#define DGBWP		0x14 /* Read Only, write pointer */
#define DGBSP		0x18
#define DGMBS		0x1c
#define DGLLPI		0x24
#define DGLPIBI		0x28

/* DGCS */
#define DGCS_SCS	(1 << 31)
#define DGCS_GEN	(1 << 26)
#define DGCS_BSC	(1 << 11)
#define DGCS_BF		(1 << 9) /* buffer full */
#define DGCS_BNE	(1 << 8) /* buffer not empty */
#define DGCS_FIFORDY	(1 << 5) /* enable FIFO */

/* DGBBA */
#define DGBBA_MASK	0xffff80

/* DGBS */
#define DGBS_MASK	0xfffff0

#define HDA_DMA_MAX_CHANS		8

struct hda_chan_data {
	uint32_t stream_id;
	uint32_t status;
	uint32_t desc_count;
	uint32_t desc_avail;
	uint32_t direction;
};

struct dma_pdata {
	struct dma *dma;
	int32_t num_channels;
	struct hda_chan_data chan[HDA_DMA_MAX_CHANS];
};

static inline uint32_t host_dma_reg_read(struct dma *dma, uint32_t chan,
	uint32_t reg)
{
	return io_reg_read(dma_chan_base(dma, chan)  + reg);
}

static inline void host_dma_reg_write(struct dma *dma, uint32_t chan,
	uint32_t reg, uint32_t value)
{
	io_reg_write(dma_chan_base(dma, chan) + reg, value);
}

static inline void hda_update_bits(struct dma *dma, uint32_t chan,
	uint32_t reg, uint32_t mask, uint32_t value)
{
	io_reg_update_bits(dma_chan_base(dma, chan)  + reg,  mask, value);
}

/* notify DMA to copy bytes */
static int hda_dma_copy(struct dma *dma, int channel, int bytes)
{
	struct dma_pdata *p = dma_get_drvdata(dma);
	uint32_t avail = p->chan[channel].desc_avail;

	trace_host("GwU");

	/* do we have any free desc/periods to copy intio ? */
	if (avail == p->chan[channel].desc_count) {

		/* no, so consume one and return */
		p->chan[channel].desc_avail--;
		if (p->chan[channel].desc_avail < 0)
			p->chan[channel].desc_avail = 0;
		return 0;
	}

	/* reset BSC before start next copy */
	hda_update_bits(dma, channel, DGCS, DGCS_BSC, DGCS_BSC);

	/* set BFPI to let host gateway knows we have read size */
	/* which will trigger next copy start */
	host_dma_reg_write(dma, channel, DGBFPI, bytes);

	host_dma_reg_write(dma, channel, DGLLPI, bytes);
	host_dma_reg_write(dma, channel, DGLPIBI, bytes);

	return 0;
}

/* allocate next free DMA channel */
static int hda_dma_channel_get(struct dma *dma, int channel)
{
	struct dma_pdata *p = dma_get_drvdata(dma);
	uint32_t flags;

	spin_lock_irq(&dma->lock, flags);

	trace_host("Dgt");

	/* use channel if it's free */
	if (p->chan[channel].status == COMP_STATE_INIT) {
		p->chan[channel].status = COMP_STATE_READY;

		/* return channel */
		spin_unlock_irq(&dma->lock, flags);
		return channel;
	}

	/* DMAC has no free channels */
	spin_unlock_irq(&dma->lock, flags);
	trace_host_error("eG0");
	return -ENODEV;
}

/* channel must not be running when this is called */
static void hda_dma_channel_put_unlocked(struct dma *dma, int channel)
{
	struct dma_pdata *p = dma_get_drvdata(dma);

	/* set new state */
	p->chan[channel].status = COMP_STATE_INIT;
}

/* channel must not be running when this is called */
static void hda_dma_channel_put(struct dma *dma, int channel)
{
	uint32_t flags;

	spin_lock_irq(&dma->lock, flags);
	hda_dma_channel_put_unlocked(dma, channel);
	spin_unlock_irq(&dma->lock, flags);
}

static int hda_dma_start(struct dma *dma, int channel)
{
	struct dma_pdata *p = dma_get_drvdata(dma);
	uint32_t flags;
	uint32_t dgcs;
	int ret = 0;

	spin_lock_irq(&dma->lock, flags);

	trace_host("DEn");

	/* is channel idle, disabled and ready ? */
	dgcs = host_dma_reg_read(dma, channel, DGCS);
	if (p->chan[channel].status != COMP_STATE_PREPARE ||
		(dgcs & DGCS_GEN)) {
		ret = -EBUSY;
		trace_host_error("eS0");
		trace_value(dgcs);
		trace_value(p->chan[channel].status);
		goto out;
	}

	/* enable the channel */
	hda_update_bits(dma, channel, DGCS,  DGCS_GEN | DGCS_FIFORDY,
		DGCS_GEN | DGCS_FIFORDY);

	/* full buffer is copied at startup */
	p->chan[channel].desc_avail = p->chan[channel].desc_count;
out:
	spin_unlock_irq(&dma->lock, flags);
	return ret;
}

static int hda_dma_release(struct dma *dma, int channel)
{
	struct dma_pdata *p = dma_get_drvdata(dma);
	uint32_t flags;

	spin_lock_irq(&dma->lock, flags);

	trace_host("Dpr");

	/* resume and reload DMA */
	p->chan[channel].status = COMP_STATE_ACTIVE;

	spin_unlock_irq(&dma->lock, flags);
	return 0;
}

static int hda_dma_pause(struct dma *dma, int channel)
{
	struct dma_pdata *p = dma_get_drvdata(dma);
	uint32_t flags;

	spin_lock_irq(&dma->lock, flags);

	trace_host("Dpa");

	if (p->chan[channel].status != COMP_STATE_ACTIVE)
		goto out;

	/* pause the channel */
	p->chan[channel].status = COMP_STATE_PAUSED;

out:
	spin_unlock_irq(&dma->lock, flags);
	return 0;
}

static int hda_dma_stop(struct dma *dma, int channel)
{
	struct dma_pdata *p = dma_get_drvdata(dma);
	uint32_t flags;

	spin_lock_irq(&dma->lock, flags);

	trace_host("DDi");

	/* disable the channel */
	hda_update_bits(dma, channel, DGCS,  DGCS_GEN | DGCS_FIFORDY, 0);
	p->chan[channel].status = COMP_STATE_PREPARE;

	spin_unlock_irq(&dma->lock, flags);
	return 0;
}

/* fill in "status" with current DMA channel state and position */
static int hda_dma_status(struct dma *dma, int channel,
	struct dma_chan_status *status, uint8_t direction)
{
	struct dma_pdata *p = dma_get_drvdata(dma);

	status->state = p->chan[channel].status;
	status->r_pos =  host_dma_reg_read(dma, channel, DGBRP);
	status->w_pos = host_dma_reg_read(dma, channel, DGBWP);
	status->timestamp = timer_get_system(platform_timer);

	return 0;
}

/* set the DMA channel configuration, source/target address, buffer sizes */
static int hda_dma_set_config(struct dma *dma, int channel,
	struct dma_sg_config *config)
{
	struct dma_pdata *p = dma_get_drvdata(dma);
	struct list_item *plist;
	struct dma_sg_elem *sg_elem;
	uint32_t buffer_addr = 0;
	uint32_t period_bytes = 0;
	uint32_t buffer_bytes = 0;
	uint32_t desc_count = 0;
	uint32_t flags;
	uint32_t addr;
	int ret = 0;

	spin_lock_irq(&dma->lock, flags);

	trace_host("Dsc");

	/* HDA only supports cyclic buffers */
	if (!config->cyclic) {
		trace_host_error("eD0");
		ret = -EINVAL;
		goto out;
	}

	/* get number of SG elems */
	list_for_item(plist, &config->elem_list)
		desc_count++;

	if (desc_count == 0) {
		trace_host_error("eD1");
		ret = -EINVAL;
		goto out;
	}

	/* default channel config */
	p->chan[channel].direction = config->direction;
	p->chan[channel].desc_count = desc_count;


	/* validate - HDA only supports continuous elems of same size  */
	list_for_item(plist, &config->elem_list) {

		sg_elem = container_of(plist, struct dma_sg_elem, list);

		if (config->direction == SOF_IPC_STREAM_PLAYBACK)
			addr = sg_elem->dest;
		else
			addr = sg_elem->src;

		/* make sure elem is continuous */
		if (buffer_addr && buffer_addr + buffer_bytes != addr) {
			trace_host_error("eD2");
			ret = -EINVAL;
			goto out;
		}

		/* make sure period_bytes are constant */
		if (period_bytes && period_bytes != sg_elem->size) {
			trace_host_error("eD3");
			ret = -EINVAL;
			goto out;
		}

		/* update counters */
		period_bytes = sg_elem->size;
		buffer_bytes += period_bytes;

		if (buffer_addr == 0)
			buffer_addr = addr;
	}

	/* init channel in HW */
	host_dma_reg_write(dma, channel, DGCS, DGCS_SCS);
	host_dma_reg_write(dma, channel, DGBBA,  buffer_addr);
	host_dma_reg_write(dma, channel, DGBS,  buffer_bytes);
	host_dma_reg_write(dma, channel, DGBFPI,  0);
	host_dma_reg_write(dma, channel, DGBSP,  period_bytes);
	host_dma_reg_write(dma, channel, DGMBS,  period_bytes);
	host_dma_reg_write(dma, channel, DGLLPI,  0);
	host_dma_reg_write(dma, channel, DGLPIBI,  0);

	p->chan[channel].status = COMP_STATE_PREPARE;
out:
	spin_unlock_irq(&dma->lock, flags);
	return ret;
}

/* restore DMA conext after leaving D3 */
static int hda_dma_pm_context_restore(struct dma *dma)
{
	return 0;
}

/* store DMA conext after leaving D3 */
static int hda_dma_pm_context_store(struct dma *dma)
{
	return 0;
}

static int hda_dma_set_cb(struct dma *dma, int channel, int type,
	void (*cb)(void *data, uint32_t type, struct dma_sg_elem *next),
	void *data)
{
	return -EINVAL;
}

static int hda_dma_probe(struct dma *dma)
{
	struct dma_pdata *hda_pdata;
	int i;

	/* allocate private data */
	hda_pdata = rzalloc(RZONE_SYS, RFLAGS_NONE, sizeof(*hda_pdata));
	dma_set_drvdata(dma, hda_pdata);

	spinlock_init(&dma->lock);

	/* init work */
	for (i = 0; i < HDA_DMA_MAX_CHANS; i++)
		hda_pdata->chan[i].status = COMP_STATE_INIT;

	return 0;
}

const struct dma_ops hda_host_dma_ops = {
	.channel_get	= hda_dma_channel_get,
	.channel_put	= hda_dma_channel_put,
	.start		= hda_dma_start,
	.stop		= hda_dma_stop,
	.copy		= hda_dma_copy,
	.pause		= hda_dma_pause,
	.release	= hda_dma_release,
	.status		= hda_dma_status,
	.set_config	= hda_dma_set_config,
	.set_cb		= hda_dma_set_cb,
	.pm_context_restore		= hda_dma_pm_context_restore,
	.pm_context_store		= hda_dma_pm_context_store,
	.probe		= hda_dma_probe,
};

const struct dma_ops hda_link_dma_ops = {
	.channel_get	= hda_dma_channel_get,
	.channel_put	= hda_dma_channel_put,
	.start		= hda_dma_start,
	.stop		= hda_dma_stop,
	.copy		= hda_dma_copy,
	.pause		= hda_dma_pause,
	.release	= hda_dma_release,
	.status		= hda_dma_status,
	.set_config	= hda_dma_set_config,
	.set_cb		= hda_dma_set_cb,
	.pm_context_restore		= hda_dma_pm_context_restore,
	.pm_context_store		= hda_dma_pm_context_store,
	.probe		= hda_dma_probe,
};

