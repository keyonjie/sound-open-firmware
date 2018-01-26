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

#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <reef/reef.h>
#include <reef/lock.h>
#include <reef/list.h>
#include <reef/dai.h>
#include <reef/alloc.h>
#include <reef/dma.h>
#include <reef/wait.h>
#include <reef/stream.h>
#include <reef/audio/component.h>
#include <reef/audio/pipeline.h>
#include <platform/dma.h>
#include <arch/cache.h>

#define DAI_PLAYBACK_STREAM	0
#define DAI_CAPTURE_STREAM	1

/* tracing */
#define trace_dai(__e) trace_event(TRACE_CLASS_DAI, __e)
#define trace_dai_error(__e)   trace_error(TRACE_CLASS_DAI, __e)
#define tracev_dai(__e)        tracev_event(TRACE_CLASS_DAI, __e)


struct dai_data {
	/* local DMA config */
	int chan;
	struct dma_sg_config config;

	struct dai *dai;
	struct dma *dma;
	uint32_t period_bytes;
	completion_t complete;
	int xrun;		/* true if we are doing xrun recovery */

	uint32_t last_bytes;    /* the last bytes(<period size) it copies. */
	uint32_t dai_pos_blks;	/* position in bytes (nearest block) */

	volatile uint64_t *dai_pos; /* host can read back this value without IPC */
	uint64_t wallclock;	/* wall clock at stream start */
};

static int dai_cmd(struct comp_dev *dev, int cmd, void *data);

/* this is called by DMA driver every time descriptor has completed */
static void dai_dma_cb(void *data, uint32_t type, struct dma_sg_elem *next)
{
	struct comp_dev *dev = (struct comp_dev *)data;
	struct dai_data *dd = comp_get_drvdata(dev);
	struct comp_buffer *dma_buffer;
	uint32_t copied_size;

	tracev_dai("irq");

	/* is stream stopped or paused and we are not handling XRUN ? */
	if (dev->state != COMP_STATE_ACTIVE && dd->xrun == 0) {

		/* stop the DAI */
		dai_trigger(dd->dai, COMP_CMD_STOP, dev->params.direction);

		/* tell DMA not to reload */
		next->size = DMA_RELOAD_END;

		/* inform waiters */
		wait_completed(&dd->complete);
	}

	/* is our pipeline handling an XRUN ? */
	if (dd->xrun) {

		/* make sure we only playback silence during an XRUN */
		if (dev->params.direction == SOF_IPC_STREAM_PLAYBACK) {

			dma_buffer = list_first_item(&dev->bsource_list,
				struct comp_buffer, sink_list);

			/* fill buffer with silence */
			bzero(dma_buffer->addr, dma_buffer->size);

			/* writeback buffer contents from cache */
			dcache_writeback_region(dma_buffer->addr,
				dma_buffer->size);
		}

		/* inform waiters */
		wait_completed(&dd->complete);
		return;
	}

	if (dev->params.direction == SOF_IPC_STREAM_PLAYBACK) {
		dma_buffer = list_first_item(&dev->bsource_list,
			struct comp_buffer, sink_list);

		copied_size = dd->last_bytes ? dd->last_bytes : dd->period_bytes;

		/* recalc available buffer space */
		comp_update_buffer_consume(dma_buffer, copied_size);

		/* writeback buffer contents from cache */
		dcache_writeback_region(dma_buffer->r_ptr, copied_size);

		/* update host position(in bytes offset) for drivers */
		dev->position += copied_size;
		if (dd->dai_pos) {
			dd->dai_pos_blks += copied_size;
			*dd->dai_pos = dd->dai_pos_blks +
				dma_buffer->r_ptr - dma_buffer->addr;
		}

		/* make sure there is availble bytes for next period */
		if (dma_buffer->avail < dd->period_bytes) {
			trace_dai_error("xru");
			comp_underrun(dev, dma_buffer, copied_size, 0);
		}

	} else {
		dma_buffer = list_first_item(&dev->bsink_list,
			struct comp_buffer, source_list);

		/* invalidate buffer contents */
		dcache_invalidate_region(dma_buffer->w_ptr, dd->period_bytes);

		/* recalc available buffer space */
		comp_update_buffer_produce(dma_buffer, dd->period_bytes);

		/* update positions */
		dev->position += dd->period_bytes;
		if (dd->dai_pos) {
			dd->dai_pos_blks += dd->period_bytes;
			*dd->dai_pos = dd->dai_pos_blks +
				dma_buffer->w_ptr - dma_buffer->addr;
		}

		/* make sure there is free bytes for next period */
		if (dma_buffer->free < dd->period_bytes) {
			trace_dai_error("xro");
			comp_overrun(dev, dma_buffer, dd->period_bytes, 0);
		}
	}

	/* notify pipeline that DAI needs its buffer processed */
	if (dev->state == COMP_STATE_ACTIVE)
		pipeline_schedule_copy(dev->pipeline, 0);
}

static struct comp_dev *dai_new(struct sof_ipc_comp *comp)
{
	struct comp_dev *dev;
	struct sof_ipc_comp_dai *dai;
	struct sof_ipc_comp_dai *ipc_dai = (struct sof_ipc_comp_dai *)comp;
	struct dai_data *dd;

	trace_dai("new");

	dev = rzalloc(RZONE_RUNTIME, RFLAGS_NONE,
		COMP_SIZE(struct sof_ipc_comp_dai));
	if (dev == NULL)
		return NULL;

	dai = (struct sof_ipc_comp_dai *)&dev->comp;
	memcpy(dai, ipc_dai, sizeof(struct sof_ipc_comp_dai));

	dd = rzalloc(RZONE_RUNTIME, RFLAGS_NONE, sizeof(*dd));
	if (dd == NULL) {
		rfree(dev);
		return NULL;
	}

	comp_set_drvdata(dev, dd);

	dd->dai = dai_get(dai->type, dai->index);
	if (dd->dai == NULL) {
		trace_dai_error("eDg");
		goto error;
	}

	dd->dma = dma_get(ipc_dai->dmac_id);
	if (dd->dma == NULL) {
		trace_dai_error("eDd");
		goto error;
	}

	list_init(&dd->config.elem_list);
	dd->dai_pos = NULL;
	dd->dai_pos_blks = 0;
	dd->last_bytes = 0;
	dd->xrun = 0;

	/* get DMA channel from DMAC1 */
	dd->chan = dma_channel_get(dd->dma, 0);
	if (dd->chan < 0){
		trace_dai_error("eDc");
		goto error;
	}

	/* set up callback */
	dma_set_cb(dd->dma, dd->chan, DMA_IRQ_TYPE_LLIST, dai_dma_cb, dev);
	dev->state = COMP_STATE_READY;
	return dev;

error:
	rfree(dd);
	rfree(dev);
	return NULL;
}

static void dai_free(struct comp_dev *dev)
{
	struct dai_data *dd = comp_get_drvdata(dev);

	dma_channel_put(dd->dma, dd->chan);

	rfree(dd);
	rfree(dev);
}

/* set component audio SSP and DMA configuration */
static int dai_playback_params(struct comp_dev *dev)
{
	struct dai_data *dd = comp_get_drvdata(dev);
	struct dma_sg_config *config = &dd->config;
	struct sof_ipc_comp_config *source_config;
	struct dma_sg_elem *elem;
	struct comp_buffer *dma_buffer;
	struct list_item *elist;
	struct list_item *tlist;
	int i;
	int err;
	uint32_t buffer_size;

	/* set up DMA configuration */
	config->direction = DMA_DIR_MEM_TO_DEV;
	config->src_width = comp_sample_bytes(dev);
	config->dest_width = comp_sample_bytes(dev);
	config->cyclic = 1;
	config->dest_dev = dd->dai->plat_data.fifo[0].handshake;

	/* set up local and host DMA elems to reset values */
	dma_buffer = list_first_item(&dev->bsource_list,
		struct comp_buffer, sink_list);
	source_config = COMP_GET_CONFIG(dma_buffer->source);
	buffer_size = source_config->periods_sink * dd->period_bytes;

	/* resize the buffer if space is available to align with period size */
	err = buffer_set_size(dma_buffer, buffer_size);
	if (err < 0) {
		trace_dai_error("ep1");
		trace_value(source_config->periods_sink);
		trace_value(dd->period_bytes);
		trace_value(buffer_size);
		trace_value(dma_buffer->alloc_size);
		return err;
	}

	if (list_is_empty(&config->elem_list)) {
		/* set up cyclic list of DMA elems */
		for (i = 0; i < source_config->periods_sink; i++) {

			elem = rzalloc(RZONE_RUNTIME, RFLAGS_NONE, sizeof(*elem));
			if (elem == NULL)
				goto err_unwind;

			elem->size = dd->period_bytes;
			elem->src = (uintptr_t)(dma_buffer->r_ptr) +
				i * dd->period_bytes;

			elem->dest = dai_fifo(dd->dai, SOF_IPC_STREAM_PLAYBACK);

			list_item_append(&elem->list, &config->elem_list);
		}
	}

	return 0;

err_unwind:
	trace_dai_error("ep3");
	list_for_item_safe(elist, tlist, &config->elem_list) {
		elem = container_of(elist, struct dma_sg_elem, list);
		list_item_del(&elem->list);
		rfree(elem);
	}
	return -ENOMEM;
}

static int dai_capture_params(struct comp_dev *dev)
{
	struct dai_data *dd = comp_get_drvdata(dev);
	struct dma_sg_config *config = &dd->config;
	struct sof_ipc_comp_config *sink_config;
	struct dma_sg_elem *elem;
	struct comp_buffer *dma_buffer;
	struct list_item *elist;
	struct list_item *tlist;
	int i;
	int err;
	uint32_t buffer_size;

	/* set up DMA configuration */
	config->direction = DMA_DIR_DEV_TO_MEM;
	config->src_width = comp_sample_bytes(dev);
	config->dest_width = comp_sample_bytes(dev);
	config->cyclic = 1;
	config->src_dev = dd->dai->plat_data.fifo[1].handshake;

	/* set up local and host DMA elems to reset values */
	dma_buffer = list_first_item(&dev->bsink_list,
		struct comp_buffer, source_list);
	sink_config = COMP_GET_CONFIG(dma_buffer->sink);
	buffer_size = sink_config->periods_source * dd->period_bytes;

	/* resize the buffer if space is available to align with period size */
	err = buffer_set_size(dma_buffer, buffer_size);
	if (err < 0) {
		trace_dai_error("ec1");
		trace_value(sink_config->periods_sink);
		trace_value(dd->period_bytes);
		trace_value(buffer_size);
		trace_value(dma_buffer->alloc_size);
		return err;
	}

	if (list_is_empty(&config->elem_list)) {
		/* set up cyclic list of DMA elems */
		for (i = 0; i < sink_config->periods_source; i++) {

			elem = rzalloc(RZONE_RUNTIME, RFLAGS_NONE, sizeof(*elem));
			if (elem == NULL)
				goto err_unwind;

			elem->size = dd->period_bytes;
			elem->dest = (uintptr_t)(dma_buffer->w_ptr) +
				i * dd->period_bytes;
			elem->src = dai_fifo(dd->dai, SOF_IPC_STREAM_CAPTURE);
			list_item_append(&elem->list, &config->elem_list);
		}
	}

	return 0;

err_unwind:
	trace_dai_error("ec3");
	list_for_item_safe(elist, tlist, &config->elem_list) {
		elem = container_of(elist, struct dma_sg_elem, list);
		list_item_del(&elem->list);
		rfree(elem);
	}
	return -ENOMEM;
}

static int dai_params(struct comp_dev *dev)
{
	struct dai_data *dd = comp_get_drvdata(dev);
	struct comp_buffer *dma_buffer;

	trace_dai("par");

	/* can set params on only init state */
	if (dev->state != COMP_STATE_READY) {
		trace_dai_error("wdp");
		return -EINVAL;
	}

	/* calculate period size based on config */
	dev->frame_bytes = comp_frame_bytes(dev);
	if (dev->frame_bytes == 0) {
		trace_dai_error("ed1");
		return -EINVAL;
	}

	dd->period_bytes = dev->frames * dev->frame_bytes;
	if (dd->period_bytes == 0) {
		trace_dai_error("ed2");
		return -EINVAL;
	}

	if (dev->params.direction == SOF_IPC_STREAM_PLAYBACK) {
		dma_buffer = list_first_item(&dev->bsource_list,
			struct comp_buffer, sink_list);
		dma_buffer->r_ptr = dma_buffer->addr;

		return dai_playback_params(dev);
	} else {
		dma_buffer = list_first_item(&dev->bsink_list,
			struct comp_buffer, source_list);
		dma_buffer->w_ptr = dma_buffer->addr;

		return dai_capture_params(dev);
	}
}

static int dai_prepare(struct comp_dev *dev)
{
	struct dai_data *dd = comp_get_drvdata(dev);
	struct comp_buffer *dma_buffer;
	int ret = 0;

	trace_dai("pre");

	ret = comp_set_state(dev, COMP_CMD_PREPARE);
	if (ret < 0)
		return ret;

	dev->position = 0;

	if (list_is_empty(&dd->config.elem_list)) {
		trace_dai_error("wdm");
		comp_set_state(dev, COMP_CMD_RESET);
		return -EINVAL;
	}

	/* initialise buffers */
	if (dev->params.direction == SOF_IPC_STREAM_PLAYBACK) {

		/* write back buffer contents from cache for playback */
		dma_buffer = list_first_item(&dev->bsource_list,
			struct comp_buffer, sink_list);

		dcache_writeback_region(dma_buffer->addr, dma_buffer->size);
	}

	/* dma reconfig not required if XRUN handling */
	if (dd->xrun)
		return ret;

	ret = dma_set_config(dd->dma, dd->chan, &dd->config);
	if (ret < 0)
		comp_set_state(dev, COMP_CMD_RESET);

	return ret;
}

static int dai_reset(struct comp_dev *dev)
{
	struct dai_data *dd = comp_get_drvdata(dev);
	struct dma_sg_config *config = &dd->config;
	struct list_item *elist;
	struct list_item *tlist;
	struct dma_sg_elem *elem;

	trace_dai("res");

	list_for_item_safe(elist, tlist, &config->elem_list) {
		elem = container_of(elist, struct dma_sg_elem, list);
		list_item_del(&elem->list);
		rfree(elem);
	}

	dd->dai_pos_blks = 0;
	if (dd->dai_pos)
		*dd->dai_pos = 0;
	dd->dai_pos = NULL;
	dd->last_bytes = 0;
	dd->wallclock = 0;
	dev->position = 0;
	dd->xrun = 0;
	comp_set_state(dev, COMP_CMD_RESET);

	return 0;
}

/* The playback source pipeline must be advanced by one period so that it
 * does not write to the period that DMA is reading. The configuration of the
 * upstream pipeline is unknown to the DAI but we can check if the source buffer
 * is shared with another DMA engine (which preloads the buffer by one period)
 * and only advance the write pointer when source component is not another
 * DMA engine.
 */
static void dai_pointer_init(struct comp_dev *dev)
{
	struct comp_buffer *dma_buffer;
	struct dai_data *dd = comp_get_drvdata(dev);

	/* not reuquired for capture streams */
	if (dev->params.direction == SOF_IPC_STREAM_PLAYBACK) {
		dma_buffer = list_first_item(&dev->bsource_list,
			struct comp_buffer, sink_list);

		switch (dma_buffer->source->comp.type) {
		case SOF_COMP_HOST:
		case SOF_COMP_SG_HOST:
			/* buffer is preloaded and advanced by host DMA engine */
			break;
		default:
			/* advance source pipeline w_ptr by one period
			 * this places pipeline w_ptr in period before DAI r_ptr */
			comp_update_buffer_produce(dma_buffer, dd->period_bytes);
			break;
		}
	}
}

/* used to pass standard and bespoke command (with data) to component */
static int dai_cmd(struct comp_dev *dev, int cmd, void *data)
{
	struct dai_data *dd = comp_get_drvdata(dev);
	int ret;

	trace_dai("cmd");
	tracev_value(cmd);

	wait_init(&dd->complete);

	ret = comp_set_state(dev, cmd);
	if (ret < 0)
		return ret;

	switch (cmd) {
	case COMP_CMD_START:
		dai_pointer_init(dev);
		/* fall through */
	case COMP_CMD_RELEASE:

		/* only start the DAI if we are not XRUN handling */
		if (dd->xrun == 0) {

			/* start the DAI */
			ret = dma_start(dd->dma, dd->chan);
			if (ret < 0)
				return ret;
			dai_trigger(dd->dai, cmd, dev->params.direction);
		} else {
			dd->xrun = 0;
		}

		/* update starting wallclock */
		platform_dai_wallclock(dev, &dd->wallclock);
		break;
	case COMP_CMD_XRUN:
		dd->xrun = 1;
		/* fall through */
	case COMP_CMD_PAUSE:
	case COMP_CMD_STOP:
		wait_init(&dd->complete);

		/* wait for DMA to complete */
		dd->complete.timeout = dev->pipeline->ipc_pipe.deadline;
		ret = wait_for_completion_timeout(&dd->complete);
		if (ret < 0) {
			trace_dai_error("ed0");
			trace_value(cmd);
		}
		break;
	default:
		break;
	}

	return ret;
}

/* copy and process stream data from source to sink buffers */
static int dai_copy(struct comp_dev *dev)
{
	return 0;
}

static int dai_position(struct comp_dev *dev, struct sof_ipc_stream_posn *posn)
{
	struct dai_data *dd = comp_get_drvdata(dev);

	/* TODO: improve accuracy by adding current DMA position */
	posn->dai_posn = dev->position;

	/* set stream start wallclock */
	posn->wallclock = dd->wallclock;

	return 0;
}

static int dai_config(struct comp_dev *dev, struct sof_ipc_dai_config *config)
{
	struct dai_data *dd = comp_get_drvdata(dev);

	/* set dma burst elems to slot number */
	dd->config.burst_elems = config->num_slots;

	/* calc frame bytes */
	switch (config->sample_valid_bits) {
	case 16:
		dev->frame_bytes = 2 * config->num_slots;
		break;
	case 17 ... 32:
		dev->frame_bytes = 4 * config->num_slots;
		break;
	default:
		break;
	}

	if (dev->frame_bytes == 0) {
		trace_dai_error("de1");
		return -EINVAL;
	}

	return 0;
}

static struct comp_driver comp_dai = {
	.type	= SOF_COMP_DAI,
	.ops	= {
		.new		= dai_new,
		.free		= dai_free,
		.params		= dai_params,
		.cmd		= dai_cmd,
		.copy		= dai_copy,
		.prepare	= dai_prepare,
		.reset		= dai_reset,
		.dai_config	= dai_config,
		.position	= dai_position,
	},
};

void sys_comp_dai_init(void)
{
	comp_register(&comp_dai);
}
