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

#ifndef IO_BRIDGE_H
#define IO_BRIDGE_H

#include <stdint.h>

#define QEMU_IO_DEBUG           0

/* IO type */
#define QEMU_IO_TYPE_QEMU       0
#define QEMU_IO_TYPE_REG        1
#define QEMU_IO_TYPE_IRQ        2
#define QEMU_IO_TYPE_GDB        3
#define QEMU_IO_TYPE_PM         4
#define QEMU_IO_TYPE_DMA        5
#define QEMU_IO_TYPE_MEM        6

/* Global Message Reply */
#define QEMU_IO_MSG_REPLY       0

/* Register Messages */
#define QEMU_IO_MSG_REG32W      32
#define QEMU_IO_MSG_REG64W      33
#define QEMU_IO_MSG_REG32R      34
#define QEMU_IO_MSG_REG64R      35

/* IRQ Messages */
#define QEMU_IO_MSG_IRQ         64

/* DMA Messages */
#define QEMU_IO_DMA_REQ_NEW     96
#define QEMU_IO_DMA_REQ_READY   97
#define QEMU_IO_DMA_REQ_COMPLETE    98

/* DMA Direction - relative to msg sender */
#define QEMU_IO_DMA_DIR_READ    256
#define QEMU_IO_DMA_DIR_WRITE   257

/* GDB Messages */
#define QEMU_IO_GDB_STALL       128
#define QEMU_IO_GDB_CONT        129
#define QEMU_IO_GDB_STALL_RPLY  130 /* stall after reply */

/* PM Messages */
#define QEMU_IO_PM_S0           192
#define QEMU_IO_PM_S1           193
#define QEMU_IO_PM_S2           194
#define QEMU_IO_PM_S3           195
#define QEMU_IO_PM_D0           196
#define QEMU_IO_PM_D1           197
#define QEMU_IO_PM_D2           198
#define QEMU_IO_PM_D3           199

/* Common message header */
struct host_ipc_msg {
    uint16_t type;
    uint16_t msg;
    uint32_t size;
    uint32_t id;
};

/* Generic message reply */
struct host_ipc_msg_reply {
    struct host_ipc_msg hdr;
    uint32_t reply;
};

/* Register messages */
struct host_ipc_msg_reg32 {
    struct host_ipc_msg hdr;
    uint32_t reg;
    uint32_t val;
};

struct host_ipc_msg_reg64 {
    struct host_ipc_msg hdr;
    uint64_t reg;
    uint64_t val;
};

/* IRQ Messages */
struct host_ipc_msg_irq {
    struct host_ipc_msg hdr;
    uint32_t irq;
};

/* PM Messages */
struct host_ipc_msg_pm_state {
    struct host_ipc_msg hdr;
};

/* DMA Messages - same message used as reply */
struct host_ipc_msg_dma32 {
    struct host_ipc_msg hdr;
    uint32_t direction;	/*  QEMU_IO_DMA_DIR_ */
    uint32_t reply;		/* 0 or errno */
    uint32_t src;
    uint32_t dest;
    uint32_t size;
    uint32_t dmac_id;
    uint32_t chan_id;
    uint64_t host_data;
    uint64_t client_data;
};

struct host_ipc_msg_dma64 {
    struct host_ipc_msg hdr;
    uint32_t direction;	/*  QEMU_IO_DMA_DIR_ */
    uint32_t reply;		/* 0 or errno */
    uint64_t src;
    uint64_t dest;
    uint64_t size;
    uint32_t dmac_id;
    uint32_t chan_id;
    uint64_t host_data;
    uint64_t client_data;
};

/* API calls for parent and child */
int host_ipc_register_parent(const char *name,
    int (*cb)(void *, struct host_ipc_msg *msg), void *data);
int host_ipc_register_child(const char *name,
    int (*cb)(void *, struct host_ipc_msg *msg), void *data);

int host_ipc_send_msg(struct host_ipc_msg *msg);
int host_ipc_send_msg_reply(struct host_ipc_msg *msg);

int host_ipc_register_shm(const char *name, int region, size_t size,
    void **addr);
int host_ipc_sync(int region, unsigned int offset, size_t length);

void host_ipc_free(void);
void host_ipc_free_shm(int region);

#endif
