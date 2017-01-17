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

#ifndef __INCLUDE_PLATFORM_HOST_DEBUG__
#define __INCLUDE_PLATFORM_HOST_DEBUG__

#include <reef/mailbox.h>
#include <platform/platform.h>
#include <stdint.h>
#include <stdlib.h>

/* dump file and line to start of mailbox or shared memory */
#define dbg() \
	do { \
		printf("%s %d\n", __func__, __LINE__); \
	} while (0);

/* dump file and line to offset in mailbox or shared memory */
#define dbg_at(__x) \
	do { \
		printf("%s %d offset 0x%x\n", __func__, __LINE__, __x); \
	} while (0);

/* dump value to start of mailbox or shared memory */
#define dbg_val(__v) \
	do { \
		printf("%s %d val 0x%x\n", __func__, __LINE__, __v); \
	} while (0);

/* dump value to offset in mailbox or shared memory */
#define dbg_val_at(__v, __x) \
	do { \
		printf("%s %d val 0x%x offset 0x%x\n", __func__, __LINE__, __v, __x); \
	} while (0);

/* dump data area at addr and size count to start of mailbox or shared memory */
#define dump(addr, count) \
	do { \
		volatile int __c = count; \
		while (__c--) \
			printf("%s %d addr 0x%x\n", __func__, __LINE__, (long)*addr); \
	} while (0);

/* dump stack as part of panic */
/* TODO: rework to make this generic - the stack top can be moved to arch code */
#define panic_dump_stack(_p) \
	do { \
		dbg_val(0xdead0000 | _p) \
		platform_panic(_p); \
	} while (0);

#endif
