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

#ifndef __PLATFORM_PLATFORM_H__
#define __PLATFORM_PLATFORM_H__

#include <platform/shim.h>
#include <platform/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

/* default static pipeline SSP port - not used for dynamic pipes */
#define PLATFORM_SSP_PORT	2

/* IPC Interrupt */
#define PLATFORM_IPC_INTERUPT	IRQ_NUM_EXT_IA

/* Host page size */
#define HOST_PAGE_SIZE		4096

/* pipeline IRQ */
#define PLATFORM_PIPELINE_IRQ	IRQ_NUM_SOFTWARE4

/* DMA treats PHY addresses as host address unless within DSP region */
#define PLATFORM_HOST_DMA_MASK	0xFF000000

/* Platform stream capabilities */
#define PLATFORM_MAX_CHANNELS	4
#define PLATFORM_MAX_STREAMS	5

/* Platform Host DMA buffer config - these should align with DMA engine */
#define PLAT_HOST_PERSIZE	256	/* must be multiple of DMA burst size */
#define PLAT_HOST_PERIODS	2	/* give enough latency for DMA refill */

/* Platform Dev DMA buffer config - these should align with DMA engine */
#define PLAT_DEV_PERSIZE	256	/* must be multiple of DMA+DEV burst size */
#define PLAT_DEV_PERIODS	2	/* give enough latency for DMA refill */

/* DMA channel drain timeout in microseconds */
#define PLATFORM_DMA_TIMEOUT	1333

/* IPC page data copy timeout */
#define PLATFORM_IPC_DMA_TIMEOUT 2000

/* WorkQ window size in microseconds */
#define PLATFORM_WORKQ_WINDOW	2000

/* Platform defined panic code */
#define platform_panic(__x) \
	fprintf(stdout, "panic 0x%x\n", __x); \
	exit(0);

/* Platform defined trace code */
#define platform_trace_point(__x) \
	fprintf(stdout, "trace boot 0x%x\n", __x);
/*
 * APIs declared here are defined for every platform and IPC mechanism.
 */

int platform_boot_complete(uint32_t boot_message);

int platform_init(void);

int platform_ssp_set_mn(uint32_t ssp_port, uint32_t source, uint32_t rate,
	uint32_t bclk_fs);

void platform_ssp_disable_mn(uint32_t ssp_port);

int platform_mem_init(int argc, char *argv[]);

	/* allocate platform MMIO */
int platform_mmio_init(int argc, char *argv[]);

#endif
