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
 */

#ifndef __ARCH_INTERRUPT_H
#define __ARCH_INTERRUPT_H

#include <reef/interrupt-map.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#define NUM_IRQ	32

struct irq_handler {
	void *data;
	void (*cb)(void *arg);
};

struct irq_context {
	int fd[2];		/* pipe fd used to wake WFI */

	/* interrupt state */
	uint32_t mask;
	uint32_t status;

	uint32_t global_disable;
	pthread_mutex_t mutex;

	struct irq_handler handler[NUM_IRQ];
};

static struct irq_context ic;

static inline void status_clear(uint32_t irq)
{
	ic.status &= ~(0x1 << irq);
}

static inline void status_set(uint32_t irq)
{
	ic.status |= (0x1 << irq);
}

static inline void mask_clear(uint32_t irq)
{
	ic.mask &= ~(0x1 << irq);
}

static inline void mask_set(uint32_t irq)
{
	ic.mask |= (0x1 << irq);
}

static inline int is_active(uint32_t irq)
{
	uint32_t bit = 0x1 << irq;

	if ((ic.status & bit) && !(ic.mask & bit))
		return 1;
	return 0;
}

static inline int arch_interrupt_register(int irq,
	void(*handler)(void *arg), void *arg)
{
	irq = REEF_IRQ_NUMBER(irq);

	status_clear(irq);
	ic.handler[irq].cb = handler;
	ic.handler[irq].data = arg;
	return 0;
}

static inline void arch_interrupt_unregister(int irq)
{
	irq = REEF_IRQ_NUMBER(irq);
	ic.handler[irq].cb = NULL;
	ic.handler[irq].data = NULL;
}

/* returns previous mask */
static inline uint32_t arch_interrupt_enable_mask(uint32_t mask)
{
	uint32_t old_mask = ic.mask;

	ic.mask |= mask;
	return old_mask;
}

/* returns previous mask */
static inline uint32_t arch_interrupt_disable_mask(uint32_t mask)
{
	uint32_t old_mask = ic.mask;

	ic.mask &= ~mask;
	return old_mask;
}

/* runs in same thread context as caller */
static inline void arch_interrupt_set(int irq)
{
	irq = REEF_IRQ_NUMBER(irq);

	status_set(irq);

	pthread_mutex_lock(&ic.mutex);

	/* return if masked */
	if (!is_active(irq) || ic.global_disable)
		goto out;

	/* run handler */
	if (ic.handler[irq].cb)
		ic.handler[irq].cb(ic.handler[irq].data);

	/* write IRQ to pipe to wake WFI */
	write(ic.fd[0], &irq, sizeof(irq));

out:
	pthread_mutex_unlock(&ic.mutex);
}

static inline void arch_interrupt_clear(int irq)
{
	irq = REEF_IRQ_NUMBER(irq);
	status_clear(irq);
}

static inline uint32_t arch_interrupt_get_enabled(void)
{
	return ~ic.mask;
}

static inline uint32_t arch_interrupt_get_status(void)
{
	return ic.status;
}

static inline uint32_t arch_interrupt_global_disable(void)
{
	uint32_t flags = 0;

	ic.global_disable = 1;
	return flags;
}

static inline void arch_interrupt_global_enable(uint32_t flags)
{
	ic.global_disable = 0;
}

int arch_interrupt_init(void);

#endif
