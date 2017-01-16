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
 *
 * Host test harness
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <baytrail/include/platform/memory.h>
#include <baytrail/include/platform/platform.h>
#include <reef/host-ipc.h>

void *_buffer_heap;
void *_system_heap;
void *_module_heap;
void *_stack_sentry;
void *_mailbox;

static uint32_t *shim;

int platform_mem_init(int argc, char *argv[])
{
	// TODO: have option to alloc memory during runtime for valgrind.

	/* allocate our system memory - TODO can we pull sizes from platform */
	_system_heap = calloc(DRAM0_SIZE, 1);
	if (_system_heap == NULL) {
		fprintf(stderr, "error: cant alloc 0x%x bytes for heap\n", DRAM0_SIZE);
		return -ENOMEM;
	}
	_module_heap = _system_heap + SYSTEM_MEM;
	_buffer_heap = _module_heap + HEAP_MOD_SIZE;
	_stack_sentry = _buffer_heap + HEAP_BUF_SIZE;

	_mailbox = calloc(MAILBOX_SIZE, 1);
	if (_mailbox == NULL) {
		fprintf(stderr, "error: cant alloc 0x%x bytes for mailbox\n",
			MAILBOX_SIZE);
		return -ENOMEM;
	}

	shim = calloc(SHIM_SIZE, 1);
	if (_mailbox == NULL) {
		fprintf(stderr, "error: cant alloc 0x%x bytes for shim\n",
				SHIM_SIZE);
		return -ENOMEM;
	}
	return 0;
}

	/* allocate platform MMIO */
int platform_mmio_init(int argc, char *argv[])
{
	int ret;

	ret = host_ipc_register_shm("mailbox", 0, MAILBOX_SIZE, &_mailbox);
	if (ret < 0)
	    return ret;

	ret = host_ipc_register_shm("shim", 0, SHIM_SIZE, &_mailbox);
	if (ret < 0)
	    return ret;

	ret = host_ipc_register_shm("pci", 0, MAILBOX_SIZE, &_mailbox);
	if (ret < 0)
	    return ret;

	ret = host_ipc_register_shm("dram", 0, DRAM0_SIZE, &_system_heap);
	if (ret < 0)
	    return ret;

	return 0;
}
