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

#include <reef/clock.h>
#include <reef/reef.h>
#include <reef/list.h>
#include <reef/alloc.h>
#include <reef/notifier.h>
#include <reef/lock.h>
#include <platform/clk.h>
#include <platform/shim.h>
#include <platform/timer.h>
#include <platform/pmc.h>
#include <stdint.h>

#define NUM_CLOCKS	2

struct clk_data {
	uint32_t freq;
	uint32_t ticks_per_usec;
	spinlock_t lock;
};

struct clk_pdata {
	struct clk_data clk[NUM_CLOCKS];
};

struct freq_table {
	uint32_t freq;
	uint32_t ticks_per_usec;
	uint32_t enc;
};

static struct clk_pdata *clk_pdata;

/* increasing frequency order */
static const struct freq_table cpu_freq[] = {
	{19200000, 19, 0x0},
	{19200000, 19, 0x1},
	{38400000, 38, 0x2},
	{50000000, 50, 0x3},	/* default */
	{100000000, 100, 0x4},
	{200000000, 200, 0x5},
	{267000000, 267, 0x6},
	{343000000, 343, 0x7},
};

#define CPU_DEFAULT_IDX		3

static inline uint32_t get_freq(const struct freq_table *table, int size,
	unsigned int hz)
{
	uint32_t i;

	/* find lowest available frequency that is >= requested hz */
	for (i = 0; i < size; i++) {
		if (hz <= table[i].freq)
			return i;
	}

	/* not found, so return max frequency */
	return size - 1;
}

void clock_enable(int clock)
{
	switch (clock) {
	case CLK_CPU:
		break;
	case CLK_SSP:
	default:
		break;
	}
}

void clock_disable(int clock)
{
	switch (clock) {
	case CLK_CPU:
		break;
	case CLK_SSP:
	default:
		break;
	}
}

uint32_t clock_set_freq(int clock, uint32_t hz)
{
	struct clock_notify_data notify_data;
	uint32_t idx, flags;

	notify_data.old_freq = clk_pdata->clk[clock].freq;
	notify_data.old_ticks_per_usec = clk_pdata->clk[clock].ticks_per_usec;

	/* atomic context for chaning clocks */
	spin_lock_irq(&clk_pdata->clk[clock].lock, flags);

	switch (clock) {
	case CLK_CPU:

		/* get nearest frequency that is >= requested Hz */
		idx = get_freq(cpu_freq, ARRAY_SIZE(cpu_freq), hz);
		notify_data.freq = cpu_freq[idx].freq;

		/* tell anyone interested we are about to change CPU freq */
		notifier_event(NOTIFIER_ID_CPU_FREQ, CLOCK_NOTIFY_PRE,
			&notify_data);

		/* tell anyone interested we have now changed CPU freq */
		notifier_event(NOTIFIER_ID_CPU_FREQ, CLOCK_NOTIFY_POST,
			&notify_data);
		break;
	default:
		break;
	}

	spin_unlock_irq(&clk_pdata->clk[clock].lock, flags);
	return clk_pdata->clk[clock].freq;
}

uint32_t clock_get_freq(int clock)
{
	return clk_pdata->clk[clock].freq;
}

uint32_t clock_us_to_ticks(int clock, uint32_t us)
{
	return clk_pdata->clk[clock].ticks_per_usec * us;
}

uint32_t clock_time_elapsed(int clock, uint32_t previous, uint32_t *current)
{
	uint32_t _current;

	// TODO: change timer APIs to clk APIs ??
	switch (clock) {
	case CLK_CPU:
		_current = arch_timer_get_system(NULL);
		break;
	default:
		return 0;
	}

	*current = _current;
	if (_current >= previous)
		return (_current - previous) /
			clk_pdata->clk[clock].ticks_per_usec;
	else
		return (_current + (MAX_INT - previous)) /
			clk_pdata->clk[clock].ticks_per_usec;
}

void init_platform_clocks(void)
{
	clk_pdata = rmalloc(RZONE_DEV, RMOD_SYS, sizeof(*clk_pdata));

	spinlock_init(&clk_pdata->clk[0].lock);
	spinlock_init(&clk_pdata->clk[1].lock);

	/* set defaults */
	clk_pdata->clk[CLK_CPU].freq = cpu_freq[CPU_DEFAULT_IDX].freq;
	clk_pdata->clk[CLK_CPU].ticks_per_usec =
			cpu_freq[CPU_DEFAULT_IDX].ticks_per_usec;
}
