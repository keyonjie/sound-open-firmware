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
#include <uapi/ipc.h>
#include <reef/reef.h>
#include <reef/debug.h>
#include <reef/trace.h>
#include <reef/ipc.h>
#include <reef/dma.h>
#include <reef/wait.h>
#include <platform/dma.h>

/* tracing */
#define trace_dma(__e)	trace_event(TRACE_CLASS_DMA, __e)
#define trace_dma_error(__e)	trace_error(TRACE_CLASS_DMA, __e)
#define tracev_dma(__e)	tracev_event(TRACE_CLASS_DMA, __e)

static struct dma_sg_elem *sg_get_elem_at(struct dma_sg_config *host_sg,
	int32_t *offset)
{
	struct dma_sg_elem *host_sg_elem;
	struct list_item *plist;
	int32_t _offset = *offset;
 
	/* find host element with host_offset */
	list_for_item(plist, &host_sg->elem_list) {

		host_sg_elem = container_of(plist, struct dma_sg_elem, list);

		/* is offset in this elem ? */
		if (_offset >= 0 && _offset < host_sg_elem->size) {
			*offset = _offset;
			return host_sg_elem;
		}

		_offset -= host_sg_elem->size;
	}

	/* host offset in beyond end of SG buffer */
	trace_dma_error("ex0");
	return NULL;
}

static void dma_complete(void *data, uint32_t type, struct dma_sg_elem *next)
{
	completion_t *comp = (completion_t *)data;

	if (type == DMA_IRQ_TYPE_LLIST)
		wait_completed(comp);

#if defined(CONFIG_DMA_TRACE)
	ipc_dma_trace_send_position();
#endif

	next->size = DMA_RELOAD_END;
}

/* Copy DSP memory to host memory.
 * copies DSP memory to host in PAGE_SIZE or smaller blocks and waits/sleeps
 * between blocks. Can't be used in IRQ context.
 */
int dma_copy_to_host(struct dma_copy *dc, struct dma_sg_config *host_sg,
	int32_t host_offset, void *local_ptr, int32_t size)
{
	struct dma_sg_config config;
	struct dma_sg_elem *host_sg_elem;
	struct dma_sg_elem local_sg_elem;
	int32_t err;
	int32_t offset = host_offset;
	int32_t bytes_copied = 0;

	if (size <= 0)
		return 0;

	/* find host element with host_offset */
	host_sg_elem = sg_get_elem_at(host_sg, &offset);
	if (host_sg_elem == NULL)
		return -EINVAL;

	/* set up DMA configuration */
	config.direction = DMA_DIR_LMEM_TO_HMEM;
	config.src_width = sizeof(uint32_t);
	config.dest_width = sizeof(uint32_t);
	config.cyclic = 0;
	list_init(&config.elem_list);

	/* configure local DMA elem */
	local_sg_elem.dest = host_sg_elem->dest + offset;
	local_sg_elem.src = (uint32_t)local_ptr;
	if (size >= HOST_PAGE_SIZE - offset)
		local_sg_elem.size = HOST_PAGE_SIZE - offset;
	else
		local_sg_elem.size = size;

	list_item_prepend(&local_sg_elem.list, &config.elem_list);

	/* transfer max PAGE size at a time to SG buffer */
	while (size > 0) {

		/* start the DMA */
		wait_init(&dc->complete);
		err = dma_set_config(dc->dmac, dc->chan, &config);
		if (err < 0)
			return err;

		err = dma_start(dc->dmac, dc->chan);
		if (err < 0)
			return err;
	
		/* wait for DMA to complete */
		err = wait_for_completion_timeout(&dc->complete);
		if (err < 0) {
			trace_dma_error("ex1");
			return -EIO;
		}

		/* update offset and bytes remaining */
		size -= local_sg_elem.size;
		host_offset += local_sg_elem.size;

		/* next dest host address is in next host elem */

		host_sg_elem = list_next_item(host_sg_elem, list);
		local_sg_elem.dest = host_sg_elem->dest;

		/* local address is continuous */
		local_sg_elem.src = (uint32_t)local_ptr + local_sg_elem.size;

		bytes_copied += local_sg_elem.size;

		/* do we have less than 1 PAGE to copy ? */
		if (size >= HOST_PAGE_SIZE)
			local_sg_elem.size = HOST_PAGE_SIZE;
		else
			local_sg_elem.size = size;
	}

	/* bytes copied */
	return bytes_copied;
}

/* Copy DSP memory to host memory.
 * Copies DSP memory to host in a single PAGE_SIZE or smaller block. Does not
 * waits/sleeps and can be used in IRQ context.
 */
int dma_copy_to_host_nowait(struct dma_copy *dc, struct dma_sg_config *host_sg,
	int32_t host_offset, void *local_ptr, int32_t size)
{
	struct dma_sg_config config;
	struct dma_sg_elem *host_sg_elem;
	struct dma_sg_elem local_sg_elem;
	int32_t err;
	int32_t offset = host_offset;

	if (size <= 0)
		return 0;

	/* find host element with host_offset */
	host_sg_elem = sg_get_elem_at(host_sg, &offset);
	if (host_sg_elem == NULL)
		return -EINVAL;

	/* set up DMA configuration */
	config.direction = DMA_DIR_LMEM_TO_HMEM;
	config.src_width = sizeof(uint32_t);
	config.dest_width = sizeof(uint32_t);
	config.cyclic = 0;
	list_init(&config.elem_list);

	/* configure local DMA elem */
	local_sg_elem.dest = host_sg_elem->dest + offset;
	local_sg_elem.src = (uint32_t)local_ptr;
	if (size >= HOST_PAGE_SIZE - offset)
		local_sg_elem.size = HOST_PAGE_SIZE - offset;
	else
		local_sg_elem.size = size;

	list_item_prepend(&local_sg_elem.list, &config.elem_list);

	/* start the DMA */
	err = dma_set_config(dc->dmac, dc->chan, &config);
	if (err < 0)
		return err;

	err = dma_start(dc->dmac, dc->chan);
	if (err < 0)
		return err;

	/* bytes copied */
	return local_sg_elem.size;
}

/* Copy host memory to DSP memory.
 * Copies host memory to host in PAGE_SIZE or smaller blocks and waits/sleeps
 * between blocks. Cant be used in IRQ context.
 */
int dma_copy_from_host(struct dma_copy *dc, struct dma_sg_config *host_sg,
	int32_t host_offset, void *local_ptr, int32_t size)
{
	struct dma_sg_config config;
	struct dma_sg_elem *host_sg_elem;
	struct dma_sg_elem local_sg_elem;
	int32_t err;
	int32_t offset = host_offset;
	int32_t bytes_copied = 0;

	if (size <= 0)
		return 0;

	/* find host element with host_offset */
	host_sg_elem = sg_get_elem_at(host_sg, &offset);
	if (host_sg_elem == NULL)
		return -EINVAL;

	/* set up DMA configuration */
	config.direction = DMA_DIR_HMEM_TO_LMEM;
	config.src_width = sizeof(uint32_t);
	config.dest_width = sizeof(uint32_t);
	config.cyclic = 0;
	list_init(&config.elem_list);

	/* configure local DMA elem */
	local_sg_elem.dest = (uint32_t)local_ptr;
	local_sg_elem.src = host_sg_elem->src + offset;
	if (size >= HOST_PAGE_SIZE - offset)
		local_sg_elem.size = HOST_PAGE_SIZE - offset;
	else
		local_sg_elem.size = size;
	list_item_prepend(&local_sg_elem.list, &config.elem_list);

	/* transfer max PAGE size at a time to SG buffer */
	while (size > 0) {

		/* start the DMA */
		wait_init(&dc->complete);
		err = dma_set_config(dc->dmac, dc->chan, &config);
		if (err < 0)
			return err;

		err = dma_start(dc->dmac, dc->chan);
		if (err < 0)
			return err;

		/* wait for DMA to complete */
		err = wait_for_completion_timeout(&dc->complete);
		if (err < 0) {
			trace_dma_error("ex2");
			return -EIO;
		}

		/* update offset and bytes remaining */
		size -= local_sg_elem.size;
		host_offset += local_sg_elem.size;

		/* next dest host address is in next host elem */
		host_sg_elem = list_next_item(host_sg_elem, list);
		local_sg_elem.src = host_sg_elem->src;

		/* local address is continuous */
		local_sg_elem.dest = (uint32_t)local_ptr + local_sg_elem.size;

		bytes_copied += local_sg_elem.size;

		/* do we have less than 1 PAGE to copy ? */
		if (size >= HOST_PAGE_SIZE)
			local_sg_elem.size = HOST_PAGE_SIZE;
		else
			local_sg_elem.size = size;
	}

	/* bytes copied */
	return bytes_copied;
}

/* Copy host memory to DSP memory.
 * Copies host memory to DSP in a single PAGE_SIZE or smaller block. Does not
 * waits/sleeps and can be used in IRQ context.
 */
int dma_copy_from_host_nowait(struct dma_copy *dc, struct dma_sg_config *host_sg,
	int32_t host_offset, void *local_ptr, int32_t size)
{
	struct dma_sg_config config;
	struct dma_sg_elem *host_sg_elem;
	struct dma_sg_elem local_sg_elem;
	int32_t err;
	int32_t offset = host_offset;

	if (size <= 0)
		return 0;

	/* find host element with host_offset */
	host_sg_elem = sg_get_elem_at(host_sg, &offset);
	if (host_sg_elem == NULL)
		return -EINVAL;

	/* set up DMA configuration */
	config.direction = DMA_DIR_HMEM_TO_LMEM;
	config.src_width = sizeof(uint32_t);
	config.dest_width = sizeof(uint32_t);
	config.cyclic = 0;
	list_init(&config.elem_list);

	/* configure local DMA elem */
	local_sg_elem.dest = (uint32_t)local_ptr;
	local_sg_elem.src = host_sg_elem->src + offset;
	if (size >= HOST_PAGE_SIZE - offset)
		local_sg_elem.size = HOST_PAGE_SIZE - offset;
	else
		local_sg_elem.size = size;
	list_item_prepend(&local_sg_elem.list, &config.elem_list);

	/* start the DMA */
	err = dma_set_config(dc->dmac, dc->chan, &config);
	if (err < 0)
		return err;

	err = dma_start(dc->dmac, dc->chan);
	if (err < 0)
		return err;

	/* bytes copied */
	return local_sg_elem.size;
}

int dma_copy_new(struct dma_copy *dc, int dmac)
{
	dc->dmac = dma_get(dmac);
	if (dc->dmac == NULL) {
		trace_dma_error("ec0");
		return -ENODEV;
	}

	/* get DMA channel from DMAC0 */
	dc->chan = dma_channel_get(dc->dmac, 0);
	if (dc->chan < 0) {
		trace_dma_error("ec1");
		return dc->chan;
	}

	dc->complete.timeout = 100;	/* wait 100 usecs for DMA to finish */
	dma_set_cb(dc->dmac, dc->chan, DMA_IRQ_TYPE_LLIST, dma_complete,
		&dc->complete);
	return 0;
}

