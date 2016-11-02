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

#include <platform/memory.h>
#include <platform/mailbox.h>
#include <platform/shim.h>
#include <platform/dma.h>
#include <platform/clk.h>
#include <platform/timer.h>
#include <platform/pmc.h>
#include <uapi/intel-ipc.h>
#include <reef/mailbox.h>
#include <reef/dai.h>
#include <reef/dma.h>
#include <reef/reef.h>
#include <reef/work.h>
#include <reef/clock.h>
#include <reef/ipc.h>
#include <reef/trace.h>
#include <reef/audio/component.h>
#include <config.h>
#include <string.h>

#if 0
static const struct sst_intel_ipc_fw_ready ready = {
	/* for host, we need exchange the naming of inxxx and outxxx */
	.inbox_offset = MAILBOX_HOST_OFFSET + MAILBOX_OUTBOX_OFFSET,
	.outbox_offset = MAILBOX_HOST_OFFSET + MAILBOX_INBOX_OFFSET,
	.inbox_size = MAILBOX_OUTBOX_SIZE,
	.outbox_size = MAILBOX_INBOX_SIZE,
	.fw_info_size = sizeof(struct fw_info),
	{
		.info = {
			.name = "REEF",
			.date = __DATE__,
			.time = __TIME__,
		},
	},
};
#endif

static struct work_queue_timesource platform_generic_queue = {
	.timer	 = {
		.id = TIMER3,	/* external timer */
		.irq = IRQ_NUM_EXT_TIMER,
	},
	.clk		= CLK_SSP,
	.notifier	= NOTIFIER_ID_SSP_FREQ,
	.timer_set	= platform_timer_set,
	.timer_clear	= platform_timer_clear,
	.timer_get	= platform_timer_get,
};

int platform_boot_complete(uint32_t boot_message)
{
	fprintf(stdout, "boot complete\n");
	return 0;
}


/* clear mask in PISR, bits are W1C in docs but some bits need preserved ?? */
void platform_interrupt_clear(uint32_t irq, uint32_t mask)
{
}

uint32_t platform_interrupt_get_enabled(void)
{
	return 0;
}

void platform_interrupt_mask(uint32_t irq, uint32_t mask)
{

}

void platform_interrupt_unmask(uint32_t irq, uint32_t mask)
{

}

static struct timer platform_ext_timer = {
	.id = TIMER3,
	.irq = IRQ_NUM_EXT_TIMER,
};

int platform_init(void)
{
	struct dma *dmac0, *dmac1;
	struct dai *ssp0, *ssp1, *ssp2;

	trace_point(TRACE_BOOT_PLATFORM_MBOX);

	trace_point(TRACE_BOOT_PLATFORM_SHIM);

	/* init work queues and clocks */
	trace_point(TRACE_BOOT_PLATFORM_TIMER);
	platform_timer_start(&platform_ext_timer);

	trace_point(TRACE_BOOT_PLATFORM_CLOCK);
	init_platform_clocks();

	trace_point(TRACE_BOOT_SYS_WORK);
	init_system_workq(&platform_generic_queue);

	/* Set CPU to default frequency for booting */
	trace_point(TRACE_BOOT_SYS_CPU_FREQ);

	/* initialise the host IPC mechanisms */
	trace_point(TRACE_BOOT_PLATFORM_IPC);
	ipc_init();

	/* init DMACs */
	trace_point(TRACE_BOOT_PLATFORM_DMA);
	dmac0 = dma_get(DMA_ID_DMAC0);
	if (dmac0 == NULL)
		return -ENODEV;
	dma_probe(dmac0);

	dmac1 = dma_get(DMA_ID_DMAC1);
	if (dmac1 == NULL)
		return -ENODEV;
	dma_probe(dmac1);

	/* init SSP ports */
	trace_point(TRACE_BOOT_PLATFORM_SSP);
	ssp0 = dai_get(COMP_TYPE_DAI_SSP, 0);
	if (ssp0 == NULL)
		return -ENODEV;
	dai_probe(ssp0);

	ssp1 = dai_get(COMP_TYPE_DAI_SSP, 1);
	if (ssp1 == NULL)
		return -ENODEV;
	dai_probe(ssp1);

	ssp2 = dai_get(COMP_TYPE_DAI_SSP, 2);
	if (ssp2 == NULL)
		return -ENODEV;
	dai_probe(ssp2);

	return 0;
}
