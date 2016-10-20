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

#include <reef/init.h>
#include <reef/dma.h>
#include <reef/ipc.h>
#include <stdint.h>
#include <reef/audio/component.h>

unsigned long _buffer_heap;
unsigned long _system_heap;
unsigned long _module_heap;
unsigned long _stack_sentry;

int ipc_process_msg_queue(void)
{
	return 0;
}

int ipc_stream_send_notification(struct comp_dev *cdev,
	struct comp_position *cp)
{
	return 0;
}

int platform_ipc_init(struct ipc *ipc)
{
	return 0;
}

int dma_copy_to_host(struct dma_sg_config *host_sg, int32_t host_offset,
	void *local_ptr, int32_t size)
{
	return 0;
}

int dma_copy_from_host(struct dma_sg_config *host_sg, int32_t host_offset,
	void *local_ptr, int32_t size)
{
	return 0;
}

void arch_wait_for_interrupt(uint32_t irq)
{

}

/* do any architecture init here */
int arch_init(void)
{
	return 0;
}

