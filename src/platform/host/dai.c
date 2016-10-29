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

#include <reef/reef.h>
#include <reef/dai.h>
#include <reef/host-vssp.h>
#include <reef/stream.h>
#include <platform/dma.h>
#include <stdint.h>
#include <string.h>

static struct dai ssp[] = {
{
	.type = COMP_TYPE_DAI_SSP,
	.index = 0,
	.plat_data = {
		.fifo[STREAM_DIRECTION_PLAYBACK] = {
			.handshake	= DMA_DEV_PCM,
		},
		.fifo[STREAM_DIRECTION_CAPTURE] = {
			.handshake	= DMA_DEV_PCM,
		}
	},
	.ops		= &host_vssp_ops,
},
{
	.type = COMP_TYPE_DAI_SSP,
	.index = 1,
	.plat_data = {
		.fifo[STREAM_DIRECTION_PLAYBACK] = {
			.handshake	= DMA_DEV_PCM,
		},
		.fifo[STREAM_DIRECTION_CAPTURE] = {
			.handshake	= DMA_DEV_PCM,
		}
	},
	.ops		= &host_vssp_ops,
},
{
	.type = COMP_TYPE_DAI_SSP,
	.index = 2,
	.plat_data = {
		.fifo[STREAM_DIRECTION_PLAYBACK] = {
			.handshake	= DMA_DEV_WAV,
		},
		.fifo[STREAM_DIRECTION_CAPTURE] = {
			.handshake	= DMA_DEV_WAV,
		}
	},
	.ops		= &host_vssp_ops,
},};

struct dai *dai_get(uint32_t type, uint32_t index)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ssp); i++) {
		if (ssp[i].type == type && ssp[i].index == index)
			return &ssp[i];
	}

	return NULL;
}
