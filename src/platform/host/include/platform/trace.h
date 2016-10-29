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

#ifndef __INCLUDE_PLATFORM_TRACE__
#define __INCLUDE_PLATFORM_TRACE__

#include <reef/timer.h>
#include <stdint.h>
#include <stdio.h>

static inline void platform_trace_event(uint32_t event)
{
	uint32_t class = event & 0xff000000;
	char *trace;

	if (class == TRACE_CLASS_IRQ)
		trace = "irq";
	else if (class == TRACE_CLASS_IPC)
		trace = "ipc";
	else if (class == TRACE_CLASS_PIPE)
		trace = "pipe";
	else if (class == TRACE_CLASS_HOST)
		trace = "host";
	else if (class == TRACE_CLASS_DAI)
		trace = "dai";
	else if (class == TRACE_CLASS_DMA)
		trace = "dma";
	else if (class == TRACE_CLASS_SSP)
		trace = "ssp";
	else if (class == TRACE_CLASS_COMP)
		trace = "comp";
	else if (class == TRACE_CLASS_WAIT)
		trace = "wfi";
	else {
		fprintf(stdout, "trace 0x%x val 0x%x\n", platform_timer_get(0), event);
		return;
	}

	/* write timestamp and event to trace buffer */
	fprintf(stdout, "trace 0x%x %s %c%c%c\n", platform_timer_get(0),
		trace, (event >> 16) & 0xff, (event >> 8) & 0xff,
		(event >> 0) & 0xff);
}

#endif
