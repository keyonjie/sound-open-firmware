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

#include <reef/dma.h>
#include <reef/dw-dma.h>
#include <reef/hda-dma.h>
#include <platform/memory.h>
#include <platform/interrupt.h>
#include <platform/dma.h>
#include <stdint.h>
#include <string.h>

static struct dw_drv_plat_data dmac0 = {
	.chan[0] = {
		.class	= 1,
		.weight = 0,
	},
	.chan[1] = {
		.class	= 1,
		.weight = 0,
	},
	.chan[2] = {
		.class	= 1,
		.weight = 0,
	},
	.chan[3] = {
		.class	= 1,
		.weight = 0,
	},
	.chan[4] = {
		.class	= 1,
		.weight = 0,
	},
	.chan[5] = {
		.class	= 1,
		.weight = 0,
	},
	.chan[6] = {
		.class	= 1,
		.weight = 0,
	},
	.chan[7] = {
		.class	= 1,
		.weight = 0,
	},
};

static struct dw_drv_plat_data dmac1 = {
	.chan[0] = {
		.class	= 2,
		.weight = 0,
	},
	.chan[1] = {
		.class	= 2,
		.weight = 0,
	},
	.chan[2] = {
		.class	= 2,
		.weight = 0,
	},
	.chan[3] = {
		.class	= 2,
		.weight = 0,
	},
	.chan[4] = {
		.class	= 2,
		.weight = 0,
	},
	.chan[5] = {
		.class	= 2,
		.weight = 0,
	},
	.chan[6] = {
		.class	= 2,
		.weight = 0,
	},
	.chan[7] = {
		.class	= 2,
		.weight = 0,
	},
};

static struct dma dma[] = {
{	/* Low Power GP DMAC 0 */
	.plat_data = {
		.id		= DMA_GP_LP_DMAC0,
		.base		= LP_GP_DMA_BASE(0),
		.channels	= 8,
		.irq		= IRQ_EXT_LP_GPDMA0_LVL5(0, 0),
		.drv_plat_data	= &dmac0,
	},
	.ops		= &dw_dma_ops,
},
{	/* Low Power GP DMAC 1 */
	.plat_data = {
		.id		= DMA_GP_LP_DMAC1,
		.base		= LP_GP_DMA_BASE(1),
		.channels	= 8,
		.irq		= IRQ_EXT_LP_GPDMA1_LVL5(0, 0),
		.drv_plat_data	= &dmac1,
	},
	.ops		= &dw_dma_ops,
},
{	/* Host In DMAC */
	.plat_data = {
		.id		= DMA_HOST_IN_DMAC,
		.base		= GTW_HOST_IN_STREAM_BASE(0),
		.channels	= 7,
		.irq		= IRQ_EXT_HOST_DMA_IN_LVL3(0, 0),
		.chan_size	= GTW_HOST_IN_STREAM_SIZE,
	},
	.ops		= &hda_host_dma_ops,
},
{	/* Host out DMAC */
	.plat_data = {
		.id		= DMA_HOST_OUT_DMAC,
		.base		= GTW_HOST_OUT_STREAM_BASE(0),
		.channels	= 6,
		.irq		= IRQ_EXT_HOST_DMA_OUT_LVL3(0, 0),
		.chan_size	= GTW_HOST_OUT_STREAM_SIZE,
	},
	.ops		= &hda_host_dma_ops,
},
{	/* Link In DMAC */
	.plat_data = {
		.id		= DMA_LINK_IN_DMAC,
		.base		= GTW_LINK_IN_STREAM_BASE(0),
		.channels	= 8,
		.irq		= IRQ_EXT_LINK_DMA_IN_LVL4(0, 0),
		.chan_size	= GTW_LINK_IN_STREAM_SIZE,
	},
	.ops		= &hda_link_dma_ops,
},
{	/* Link out DMAC */
	.plat_data = {
		.id		= DMA_LINK_OUT_DMAC,
		.base		= GTW_LINK_OUT_STREAM_BASE(0),
		.channels	= 8,
		.irq		= IRQ_EXT_LINK_DMA_OUT_LVL4(0, 0),
		.chan_size	= GTW_LINK_OUT_STREAM_SIZE,
	},
	.ops		= &hda_link_dma_ops,
},};

struct dma *dma_get(int dmac_id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dma); i++) {
		if (dma[i].plat_data.id == dmac_id)
			return &dma[i];
	}

	return NULL;
}
