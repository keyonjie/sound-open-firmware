/*
 * Copyright (c) 2017, Intel Corporation
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
 *         Rander Wang <rander.wang@intel.com>
 */

#ifndef __PLATFORM_MEMORY_H__
#define __PLATFORM_MEMORY_H__

#include <config.h>
#include <arch/memory.h>

/* physical DSP addresses */

/* shim */
#define SHIM_BASE		0x00071F00
#define SHIM_SIZE		0x00000100

/* Digital Mic Shim Registers */
#define DMIC_SHIM_BASE	0x00071E80
#define DMICLCTL_OFFSET 0x04
#define DMICLCTL	(DMIC_SHIM_BASE + DMICLCTL_OFFSET)

/* cmd IO to audio codecs */
#define CMD_BASE		0x00001100
#define CMD_SIZE		0x00000010

/* resource allocation */
#define RES_BASE		0x00001110
#define RES_SIZE		0x00000010

/* IPC to the host */
#define IPC_HOST_BASE		0x00071E00
#define IPC_HOST_SIZE		0x00000020

/* intra DSP  IPC */
#define IPC_DSP_SIZE		0x00000080
#define IPC_DSP_BASE(x)		(0x00001200 + x * IPC_DSP_SIZE)

/* SRAM window for HOST */
#define HOST_WIN_SIZE		0x00000008
#define HOST_WIN_BASE(x)	(0x00071A00 + x * HOST_WIN_SIZE)

/* IRQ controller */
#define IRQ_BASE		0x00078800
#define IRQ_SIZE		0x00000200

/* time stamping */
#define TIME_BASE		0x00071800
#define TIME_SIZE		0x00000200

/* M/N dividers */
#define MN_BASE			0x00078C00
#define MN_SIZE			0x00000200

/* low power DMA position */
#define LP_GP_DMA_LINK_SIZE	0x00000010
#define LP_GP_DMA_LINK_BASE(x) (0x00001C00 + x * LP_GP_DMA_LINK_SIZE)

/* high performance DMA position */
#define HP_GP_DMA_LINK_SIZE	0x00000010
#define HP_GP_DMA_LINK_BASE(x)	(0x00001D00 + x * HP_GP_DMA_LINK_SIZE)

/* link DMAC stream */
#define GTW_LINK_OUT_STREAM_SIZE	0x00000020
#define GTW_LINK_OUT_STREAM_BASE(x) \
				(0x00072400 + x * GTW_LINK_OUT_STREAM_SIZE)

#define GTW_LINK_IN_STREAM_SIZE	0x00000020
#define GTW_LINK_IN_STREAM_BASE(x) \
				(0x00072600 + x * GTW_LINK_IN_STREAM_SIZE)

/* host DMAC stream */
#define GTW_HOST_OUT_STREAM_SIZE	0x00000040
#define GTW_HOST_OUT_STREAM_BASE(x) \
				(0x00072800 + x * GTW_HOST_OUT_STREAM_SIZE)

#define GTW_HOST_IN_STREAM_SIZE		0x00000040
#define GTW_HOST_IN_STREAM_BASE(x) \
				(0x00072C00 + x * GTW_HOST_IN_STREAM_SIZE)

/* code loader */
#define GTW_CODE_LDR_SIZE	0x00000040
#define GTW_CODE_LDR_BASE	0x00002BC0

/* L2 TLBs */
#define L2_HP_SRAM_TLB_SIZE	0x00001000
#define L2_HP_SRAM_TLB_BASE	0x00003000

/* DMICs */
#define DMIC_BASE		0x00010000
#define DMIC_SIZE		0x00008000

/* SSP */
#define SSP_BASE(x)		(0x00077000 + x * SSP_SIZE)
#define SSP_SIZE		0x0000200

/* low power DMACs */
#define LP_GP_DMA_SIZE		0x00001000
#define LP_GP_DMA_BASE(x)	(0x0007C000 + x * LP_GP_DMA_SIZE)

/* high performance DMACs */
#define HP_GP_DMA_SIZE		0x00001000
#define HP_GP_DMA_BASE(x)	(0x0000E000 + x * HP_GP_DMA_SIZE)

/* ROM */
#define ROM_BASE		0xBEFE0000
#define ROM_SIZE		0x00002000

#define LOG_ENTRY_ELF_BASE	0x20000000
#define LOG_ENTRY_ELF_SIZE	0x2000000

/*
 * The HP SRAM Region on Icelake is organised like this :-
 * +--------------------------------------------------------------------------+
 * | Offset              | Region         |  Size                             |
 * +---------------------+----------------+-----------------------------------+
 * | SRAM_SW_REG_BASE    | SW Registers W0|  SRAM_SW_REG_SIZE                 |
 * +---------------------+----------------+-----------------------------------+
 * | SRAM_OUTBOX_BASE    | Outbox W0      |  SRAM_MAILBOX_SIZE                |
 * +---------------------+----------------+-----------------------------------+
 * | SRAM_INBOX_BASE     | Inbox  W1      |  SRAM_INBOX_SIZE                  |
 * +---------------------+----------------+-----------------------------------+
 * | SRAM_DEBUG_BASE     | Debug data  W2 |  SRAM_DEBUG_SIZE                  |
 * +---------------------+----------------+-----------------------------------+
 * | SRAM_EXCEPT_BASE    | Debug data  W2 |  SRAM_EXCEPT_SIZE                 |
 * +---------------------+----------------+-----------------------------------+
 * | SRAM_STREAM_BASE    | Stream data W2 |  SRAM_STREAM_SIZE                 |
 * +---------------------+----------------+-----------------------------------+
 * | SRAM_TRACE_BASE     | Trace Buffer W3|  SRAM_TRACE_SIZE                  |
 * +---------------------+----------------+-----------------------------------+
 * | HP_SRAM_BASE        | DMA            |  HEAP_HP_BUFFER_SIZE              |
 * +---------------------+----------------+-----------------------------------+
 * | HEAP_SYSTEM_BASE    | System Heap    |  HEAP_SYSTEM_SIZE                 |
 * +---------------------+----------------+-----------------------------------+
 * | HEAP_RUNTIME_BASE   | Runtime Heap   |  HEAP_RUNTIME_SIZE                |
 * +---------------------+----------------+-----------------------------------+
 * | HEAP_BUFFER_BASE    | Module Buffers |  HEAP_BUFFER_SIZE                 |
 * +---------------------+----------------+-----------------------------------+
 * | SOF_STACK_END      | Stack          |  SOF_STACK_SIZE                  |
 * +---------------------+----------------+-----------------------------------+
 * | SOF_STACK_BASE     |                |                                   |
 * +---------------------+----------------+-----------------------------------+
 */

/* HP SRAM */
#define SRAM_ALIAS_OFFSET	0x20000000
#define HP_SRAM_BASE		0xBE000000
#define HP_SRAM_SIZE		0x002F0000
#define HP_SRAM_MASK		0xFF000000

/* HP SRAM Base */
#define HP_SRAM_VECBASE_RESET	(HP_SRAM_BASE + 0x40000)

/* Heap section sizes for system runtime heap for master core */
#define HEAP_SYS_RT_0_COUNT64		64
#define HEAP_SYS_RT_0_COUNT512		16
#define HEAP_SYS_RT_0_COUNT1024		4

/* Heap section sizes for system runtime heap for slave core */
#define HEAP_SYS_RT_X_COUNT64		32
#define HEAP_SYS_RT_X_COUNT512		8
#define HEAP_SYS_RT_X_COUNT1024		4

/* Heap section sizes for module pool */
#define HEAP_RT_COUNT64			128
#define HEAP_RT_COUNT128		64
#define HEAP_RT_COUNT256		128
#define HEAP_RT_COUNT512		64
#define HEAP_RT_COUNT1024		8

#define L2_VECTOR_SIZE		0x1000

/* HP SRAM windows */
/* window 0 */
#define SRAM_SW_REG_BASE	(HP_SRAM_BASE + 0x4000)
#define SRAM_SW_REG_SIZE	0x1000

/* SRAM window 0 FW "registers" */
#define SRAM_REG_ROM_STATUS	0x0
#define SRAM_REG_FW_STATUS	0x4
#define SRAM_REG_FW_TRACEP	0x8
#define SRAM_REG_FW_END		0xc

#define SRAM_OUTBOX_BASE	(SRAM_SW_REG_BASE + SRAM_SW_REG_SIZE)
#define SRAM_OUTBOX_SIZE	0x1000

/* window 1 */
#define SRAM_INBOX_BASE		(SRAM_OUTBOX_BASE + SRAM_OUTBOX_SIZE)
#define SRAM_INBOX_SIZE		0x2000

/* window 2 */
#define SRAM_DEBUG_BASE		(SRAM_INBOX_BASE + SRAM_INBOX_SIZE)
#define SRAM_DEBUG_SIZE		0x800

#define SRAM_EXCEPT_BASE	(SRAM_DEBUG_BASE + SRAM_DEBUG_SIZE)
#define SRAM_EXCEPT_SIZE	0x800

#define SRAM_STREAM_BASE	(SRAM_EXCEPT_BASE + SRAM_EXCEPT_SIZE)
#define SRAM_STREAM_SIZE	0x1000

/* window 3 */
#define SRAM_TRACE_BASE		(SRAM_STREAM_BASE + SRAM_STREAM_SIZE)
#define SRAM_TRACE_SIZE		0x2000

#define HP_SRAM_WIN0_BASE	SRAM_SW_REG_BASE
#define HP_SRAM_WIN0_SIZE	(SRAM_SW_REG_SIZE + SRAM_OUTBOX_SIZE)
#define HP_SRAM_WIN1_BASE	SRAM_INBOX_BASE
#define HP_SRAM_WIN1_SIZE	SRAM_INBOX_SIZE
#define HP_SRAM_WIN2_BASE	SRAM_DEBUG_BASE
#define HP_SRAM_WIN2_SIZE	(SRAM_DEBUG_SIZE + SRAM_EXCEPT_SIZE + \
				SRAM_STREAM_SIZE)
#define HP_SRAM_WIN3_BASE	SRAM_TRACE_BASE
#define HP_SRAM_WIN3_SIZE	SRAM_TRACE_SIZE

#define HEAP_HP_BUFFER_BASE		(SRAM_TRACE_BASE + SRAM_TRACE_SIZE)
#define HEAP_HP_BUFFER_SIZE		0x20000
#define HEAP_HP_BUFFER_BLOCK_SIZE	0x180
#define HEAP_HP_BUFFER_COUNT \
			(HEAP_HP_BUFFER_SIZE / HEAP_HP_BUFFER_BLOCK_SIZE)

/* text and data share the same HP L2 SRAM on Icelake */
#define SOF_TEXT_START		0xBE040400
#define SOF_TEXT_BASE		(SOF_TEXT_START)
#define SOF_TEXT_SIZE		(0x1d000 - 0x400)

/* initialized data */
#define SOF_DATA_START		(SOF_TEXT_BASE + SOF_TEXT_SIZE)
#if defined CONFIG_DMIC
#define SOF_DATA_SIZE		0x1b000
#else
#define SOF_DATA_SIZE		0x19000
#endif

/* bss data */
#define SOF_BSS_DATA_START	(SOF_TEXT_BASE + SOF_TEXT_SIZE + SOF_DATA_SIZE)
#define SOF_BSS_DATA_SIZE	0x10900

/* Heap configuration */
#define HEAP_SYSTEM_0_BASE		(SOF_TEXT_BASE + SOF_TEXT_SIZE + \
					SOF_DATA_SIZE + SOF_BSS_DATA_SIZE)

#define HEAP_SYSTEM_0_SIZE		0x8000

#define HEAP_SYSTEM_1_BASE	(HEAP_SYSTEM_0_BASE + HEAP_SYSTEM_0_SIZE)
#define HEAP_SYSTEM_1_SIZE	0x5000

#define HEAP_SYSTEM_2_BASE	(HEAP_SYSTEM_1_BASE + HEAP_SYSTEM_1_SIZE)
#define HEAP_SYSTEM_2_SIZE	0x5000

#define HEAP_SYSTEM_3_BASE	(HEAP_SYSTEM_2_BASE + HEAP_SYSTEM_2_SIZE)
#define HEAP_SYSTEM_3_SIZE	0x5000

#define HEAP_SYSTEM_T_SIZE	(HEAP_SYSTEM_0_SIZE + HEAP_SYSTEM_1_SIZE + \
				 HEAP_SYSTEM_2_SIZE + HEAP_SYSTEM_3_SIZE)

#define HEAP_SYS_RUNTIME_0_BASE	(HEAP_SYSTEM_3_BASE + HEAP_SYSTEM_3_SIZE)
#define HEAP_SYS_RUNTIME_0_SIZE \
	(HEAP_SYS_RT_0_COUNT64 * 64 + HEAP_SYS_RT_0_COUNT512 * 512 + \
	HEAP_SYS_RT_0_COUNT1024 * 1024)

#define HEAP_SYS_RUNTIME_1_BASE	(HEAP_SYS_RUNTIME_0_BASE + \
				HEAP_SYS_RUNTIME_0_SIZE)
#define HEAP_SYS_RUNTIME_1_SIZE \
	(HEAP_SYS_RT_X_COUNT64 * 64 + HEAP_SYS_RT_X_COUNT512 * 512 + \
	HEAP_SYS_RT_X_COUNT1024 * 1024)

#define HEAP_SYS_RUNTIME_2_BASE	(HEAP_SYS_RUNTIME_1_BASE + \
				HEAP_SYS_RUNTIME_1_SIZE)
#define HEAP_SYS_RUNTIME_2_SIZE \
	(HEAP_SYS_RT_X_COUNT64 * 64 + HEAP_SYS_RT_X_COUNT512 * 512 + \
	HEAP_SYS_RT_X_COUNT1024 * 1024)

#define HEAP_SYS_RUNTIME_3_BASE	(HEAP_SYS_RUNTIME_2_BASE + \
				HEAP_SYS_RUNTIME_2_SIZE)
#define HEAP_SYS_RUNTIME_3_SIZE \
	(HEAP_SYS_RT_X_COUNT64 * 64 + HEAP_SYS_RT_X_COUNT512 * 512 + \
	HEAP_SYS_RT_X_COUNT1024 * 1024)

#define HEAP_SYS_RUNTIME_T_SIZE \
	(HEAP_SYS_RUNTIME_0_SIZE + HEAP_SYS_RUNTIME_1_SIZE + \
	HEAP_SYS_RUNTIME_2_SIZE + HEAP_SYS_RUNTIME_3_SIZE)

#define HEAP_RUNTIME_BASE	(HEAP_SYS_RUNTIME_3_BASE + \
				HEAP_SYS_RUNTIME_3_SIZE)
#define HEAP_RUNTIME_SIZE \
	(HEAP_RT_COUNT64 * 64 + HEAP_RT_COUNT128 * 128 + \
	HEAP_RT_COUNT256 * 256 + HEAP_RT_COUNT512 * 512 + \
	HEAP_RT_COUNT1024 * 1024)

/* Stack configuration */
#define SOF_STACK_SIZE		ARCH_STACK_SIZE
#define SOF_STACK_TOTAL_SIZE	ARCH_STACK_TOTAL_SIZE
#define SOF_STACK_BASE		(HP_SRAM_BASE + HP_SRAM_SIZE)
#define SOF_STACK_END		(SOF_STACK_BASE - SOF_STACK_TOTAL_SIZE)

#define HEAP_BUFFER_BASE	(HEAP_RUNTIME_BASE + HEAP_RUNTIME_SIZE)
#define HEAP_BUFFER_SIZE	\
	(SOF_STACK_END - HEAP_BUFFER_BASE)
#define HEAP_BUFFER_BLOCK_SIZE		0x180
#define HEAP_BUFFER_COUNT	(HEAP_BUFFER_SIZE / HEAP_BUFFER_BLOCK_SIZE)

/*
 * The LP SRAM Heap and Stack on Icelake are organised like this :-
 *
 * +--------------------------------------------------------------------------+
 * | Offset              | Region         |  Size                             |
 * +---------------------+----------------+-----------------------------------+
 * | LP_SRAM_BASE        | RO Data        |  SOF_LP_DATA_SIZE                |
 * |                     | Data           |                                   |
 * |                     | BSS            |                                   |
 * +---------------------+----------------+-----------------------------------+
 * | HEAP_LP_SYSTEM_BASE | System Heap    |  HEAP_LP_SYSTEM_SIZE                 |
 * +---------------------+----------------+-----------------------------------+
 * | HEAP_LP_RUNTIME_BASE| Runtime Heap   |  HEAP_LP_RUNTIME_SIZE                |
 * +---------------------+----------------+-----------------------------------+
 * | HEAP_LP_BUFFER_BASE | Module Buffers |  HEAP_LP_BUFFER_SIZE                 |
 * +---------------------+----------------+-----------------------------------+
 * | SOF_LP_STACK_END   | Stack          |  SOF_LP_STACK_SIZE                  |
 * +---------------------+----------------+-----------------------------------+
 * | SOF_STACK_BASE     |                |                                   |
 * +---------------------+----------------+-----------------------------------+
 */

/* LP SRAM */
#define LP_SRAM_BASE		0xBE800000
#define LP_SRAM_SIZE		0x00020000

/* Heap section sizes for module pool */
#define HEAP_RT_LP_COUNT8			0
#define HEAP_RT_LP_COUNT16			256
#define HEAP_RT_LP_COUNT32			128
#define HEAP_RT_LP_COUNT64			64
#define HEAP_RT_LP_COUNT128			64
#define HEAP_RT_LP_COUNT256			96
#define HEAP_RT_LP_COUNT512			8
#define HEAP_RT_LP_COUNT1024			4

/* Heap configuration */
#define SOF_LP_DATA_SIZE			0x4000

#define HEAP_LP_SYSTEM_BASE		(LP_SRAM_BASE + SOF_LP_DATA_SIZE)
#define HEAP_LP_SYSTEM_SIZE		0x1000

#define HEAP_LP_RUNTIME_BASE		(HEAP_LP_SYSTEM_BASE + HEAP_LP_SYSTEM_SIZE)
#define HEAP_LP_RUNTIME_SIZE \
	(HEAP_RT_LP_COUNT8 * 8 + HEAP_RT_LP_COUNT16 * 16 + \
	HEAP_RT_LP_COUNT32 * 32 + HEAP_RT_LP_COUNT64 * 64 + \
	HEAP_RT_LP_COUNT128 * 128 + HEAP_RT_LP_COUNT256 * 256 + \
	HEAP_RT_LP_COUNT512 * 512 + HEAP_RT_LP_COUNT1024 * 1024)

#define HEAP_LP_BUFFER_BASE		(HEAP_LP_RUNTIME_BASE + HEAP_LP_RUNTIME_SIZE)
#define HEAP_LP_BUFFER_SIZE	\
    (LP_SRAM_SIZE - HEAP_LP_RUNTIME_SIZE - SOF_LP_STACK_SIZE - HEAP_LP_SYSTEM_SIZE)

#define HEAP_LP_BUFFER_BLOCK_SIZE		0x180
#define HEAP_LP_BUFFER_COUNT	(HEAP_LP_BUFFER_SIZE / HEAP_LP_BUFFER_BLOCK_SIZE)

#define PLATFORM_HEAP_SYSTEM		4 /* one per core */
#define PLATFORM_HEAP_SYSTEM_RUNTIME	4 /* one per core */
#define PLATFORM_HEAP_RUNTIME		1
#define PLATFORM_HEAP_BUFFER		3

/* Stack configuration */
#define SOF_LP_STACK_SIZE			0x1000
#define SOF_LP_STACK_BASE			(LP_SRAM_BASE + LP_SRAM_SIZE)
#define SOF_LP_STACK_END			(SOF_LP_STACK_BASE - SOF_LP_STACK_SIZE)


/* Vector and literal sizes - do not use core-isa.h */
#define SOF_MEM_VECBASE			HP_SRAM_VECBASE_RESET
#define SOF_MEM_VECT_LIT_SIZE		0x8
#define SOF_MEM_VECT_TEXT_SIZE		0x38
#define SOF_MEM_VECT_SIZE		(SOF_MEM_VECT_TEXT_SIZE + SOF_MEM_VECT_LIT_SIZE)

#define SOF_MEM_ERROR_TEXT_SIZE	0x180
#define SOF_MEM_ERROR_LIT_SIZE		0x8

#define SOF_MEM_RESET_TEXT_SIZE	0x268
#define SOF_MEM_RESET_LIT_SIZE		0x8
#define SOF_MEM_VECBASE_LIT_SIZE	0x178

#define SOF_MEM_RO_SIZE			0x8

/* VM ROM sizes */
#define ROM_RESET_TEXT_SIZE	0x400
#define ROM_RESET_LIT_SIZE	0x200

/* boot loader in IMR */
#define IMR_BOOT_LDR_MANIFEST_BASE	0xB0032000
#define IMR_BOOT_LDR_MANIFEST_SIZE	0x6000

#define IMR_BOOT_LDR_TEXT_ENTRY_BASE	0xB0038000
#define IMR_BOOT_LDR_TEXT_ENTRY_SIZE	0x86
#define IMR_BOOT_LDR_LIT_BASE		(IMR_BOOT_LDR_TEXT_ENTRY_BASE + IMR_BOOT_LDR_TEXT_ENTRY_SIZE)
#define IMR_BOOT_LDR_LIT_SIZE		0x22
#define IMR_BOOT_LDR_TEXT_BASE		(IMR_BOOT_LDR_LIT_BASE + IMR_BOOT_LDR_LIT_SIZE)
#define IMR_BOOT_LDR_TEXT_SIZE		0x1C00
#define IMR_BOOT_LDR_DATA_BASE		0xB0039000
#define IMR_BOOT_LDR_DATA_SIZE		0x1000
#define IMR_BOOT_LDR_BSS_BASE		0xB0100000
#define IMR_BOOT_LDR_BSS_SIZE		0x10000

#define uncache_to_cache(address) \
	((__typeof__((address)))((uint32_t)((address)) + SRAM_ALIAS_OFFSET))
#define cache_to_uncache(address) \
	((__typeof__((address)))((uint32_t)((address)) - SRAM_ALIAS_OFFSET))
#define is_uncached(address) \
	(((uint32_t)(address) & HP_SRAM_MASK) != HP_SRAM_BASE)

#endif
