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

#include <errno.h>
#include <reef/dai.h>
#include <reef/stream.h>
#include <reef/ssp.h>
#include <reef/alloc.h>
#include <reef/interrupt.h>
#include <reef/lock.h>
#include <reef/work.h>
#include <reef/trace.h>

/* SSP port status */
#define SSP_STATE_INIT			0
#define SSP_STATE_RUNNING		1
#define SSP_STATE_IDLE			2
#define SSP_STATE_DRAINING		3
#define SSP_STATE_PAUSING		4
#define SSP_STATE_PAUSED		5

/* tracing */
#define trace_ssp(__e)	trace_event(TRACE_CLASS_SSP, __e)
#define trace_host_vssp_error(__e)	trace_error(TRACE_CLASS_SSP, __e)
#define tracev_ssp(__e)	tracev_event(TRACE_CLASS_SSP, __e)

/* SSP private data */
struct host_vssp_pdata {
	spinlock_t lock;
	uint32_t state[2];		/* SSP_STATE_ for each direction */
};

/* Digital Audio interface formatting */
static inline int host_vssp_set_config(struct dai *dai, struct dai_config *dai_config)
{
	return 0;
}

/* Digital Audio interface formatting */
static inline int host_vssp_set_loopback_mode(struct dai *dai, uint32_t lbm)
{
	struct host_vssp_pdata *ssp = dai_get_drvdata(dai);

	trace_ssp("SLb");
	spin_lock(&ssp->lock);


	spin_unlock(&ssp->lock);

	return 0;
}

/* start the SSP for either playback or capture */
static void host_vssp_start(struct dai *dai, int direction)
{
	struct host_vssp_pdata *ssp = dai_get_drvdata(dai);

	spin_lock(&ssp->lock);

	/* enable port */
	ssp->state[direction] = SSP_STATE_RUNNING;

	trace_ssp("SEn");

	/* enable DMA */

	spin_unlock(&ssp->lock);
}

/* stop the SSP port stream DMA and disable SSP port if no users */
static void host_vssp_stop(struct dai *dai, int direction)
{
	struct host_vssp_pdata *ssp = dai_get_drvdata(dai);

	spin_lock(&ssp->lock);

	trace_ssp("SDc");

	/* disable DMA */

	ssp->state[direction] = SSP_STATE_IDLE;

	spin_unlock(&ssp->lock);
}

static void host_vssp_pause(struct dai *dai, int direction)
{
	struct host_vssp_pdata *ssp = dai_get_drvdata(dai);

	spin_lock(&ssp->lock);

	trace_ssp("SDp");

	ssp->state[direction] = SSP_STATE_PAUSED;

	spin_unlock(&ssp->lock);
}

static int host_vssp_trigger(struct dai *dai, int cmd, int direction)
{
	trace_ssp("STr");

	switch (cmd) {
	case DAI_TRIGGER_START:
		host_vssp_start(dai, direction);
		break;
	case DAI_TRIGGER_PAUSE_RELEASE:
		host_vssp_start(dai, direction);
		break;
	case DAI_TRIGGER_PAUSE_PUSH:
		host_vssp_pause(dai, direction);
		break;
	case DAI_TRIGGER_STOP:
		host_vssp_stop(dai, direction);
		break;
	case DAI_TRIGGER_RESUME:
		host_vssp_start(dai, direction);
		break;
	case DAI_TRIGGER_SUSPEND:
		host_vssp_stop(dai, direction);
		break;
	default:
		break;
	}

	return 0;
}

static int host_vssp_probe(struct dai *dai)
{
	struct host_vssp_pdata *ssp;

	/* allocate private data */
	ssp = rzalloc(RZONE_DEV, RMOD_SYS, sizeof(*ssp));
	dai_set_drvdata(dai, ssp);

	spinlock_init(&ssp->lock);

	return 0;
}

const struct dai_ops host_vssp_ops = {
	.trigger		= host_vssp_trigger,
	.set_config		= host_vssp_set_config,
	.probe			= host_vssp_probe,
	.set_loopback_mode	= host_vssp_set_loopback_mode,
};
