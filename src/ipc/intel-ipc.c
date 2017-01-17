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
 *
 * IPC (InterProcessor Communication) provides a method of two way
 * communication between the host processor and the DSP. The IPC used here
 * utilises a shared mailbox and door bell between the host and DSP.
 *
 * The IPC here is currently based on the upstream BDW driver, but it's
 * missing some of the newer IPC call used for pipeline management used by
 * the SKL/BXT drivers.
 */

#include <reef/debug.h>
#include <reef/timer.h>
#include <reef/interrupt.h>
#include <reef/ipc.h>
#include <reef/mailbox.h>
#include <reef/reef.h>
#include <reef/stream.h>
#include <reef/dai.h>
#include <reef/dma.h>
#include <reef/alloc.h>
#include <reef/wait.h>
#include <reef/trace.h>
#include <reef/ssp.h>
#include <platform/interrupt.h>
#include <platform/mailbox.h>
#include <platform/shim.h>
#include <platform/dma.h>
#include <reef/audio/component.h>
#include <reef/audio/pipeline.h>
#include <uapi/intel-ipc.h>
#include <reef/intel-ipc.h>
#include <config.h>
#include <version.h>

/* convert DSP mailbox address to host offset */
#define to_host_offset(_s) \
	(uint32_t)((((long)&_s) - MAILBOX_BASE + MAILBOX_HOST_OFFSET) & 0xffffffff)

struct ipc *_ipc;

/* hard coded stream offset TODO: make this into programmable map */
static struct sst_intel_ipc_stream_data *_stream_dataP =
	(struct sst_intel_ipc_stream_data *)(MAILBOX_BASE + MAILBOX_STREAM_OFFSET);
static struct sst_intel_ipc_stream_data *_stream_dataC =
	(struct sst_intel_ipc_stream_data *)(MAILBOX_BASE + MAILBOX_STREAM_OFFSET +
		sizeof(struct sst_intel_ipc_stream_data));
static struct sst_intel_ipc_stream_data *_stream_dataR =
	(struct sst_intel_ipc_stream_data *)(MAILBOX_BASE + MAILBOX_STREAM_OFFSET +
		2 * sizeof(struct sst_intel_ipc_stream_data));
static struct sst_intel_ipc_stream_data *_stream_dataM =
	(struct sst_intel_ipc_stream_data *)(MAILBOX_BASE + MAILBOX_STREAM_OFFSET +
		3 * sizeof(struct sst_intel_ipc_stream_data));

/*
 * BDW IPC dialect.
 *
 * This IPC ABI only supports a fixed static pipeline using the upstream BDW
 * driver. The stream and mixer IDs are hard coded (must match the static
 * pipeline definition).
 */

/* static IDs. TODO: to be replaced with dynamic IDs */
struct stream_ids {
	uint32_t host_id;
	uint32_t volume_id;
	uint32_t mixer_id;
	uint32_t dai_id;
};

static const struct stream_ids
	stream_comp[IPC_INTEL_STREAM_TYPE_MAX_STREAM_TYPE] = {
	{2, 3, 4, 6},	/* render stream */
	{0, 1, 4, 6},	/* system stream */
	{9, 8, 4, 7},	/* capture stream */
};

static inline uint32_t msg_get_global_type(uint32_t msg)
{
	return (msg & IPC_INTEL_GLB_TYPE_MASK) >> IPC_INTEL_GLB_TYPE_SHIFT;
}

static inline uint32_t msg_get_stream_type(uint32_t msg)
{
	return (msg & IPC_INTEL_STR_TYPE_MASK) >> IPC_INTEL_STR_TYPE_SHIFT;
}

static inline uint32_t msg_get_stage_type(uint32_t msg)
{
	return (msg & IPC_INTEL_STG_TYPE_MASK) >> IPC_INTEL_STG_TYPE_SHIFT;
}

/* TODO: add build count */
static const struct ipc_intel_ipc_fw_version fw_version = {
	.build = 0,
	.minor = REEF_MINOR,
	.major = REEF_MAJOR,
	.type = 0,
	.fw_build_hash = REEF_BUILD,
	.fw_log_providers_hash = 0,
};

static uint32_t ipc_fw_version(uint32_t header)
{
	trace_ipc("Ver");

	mailbox_outbox_write(0, &fw_version, sizeof(fw_version));

	return IPC_INTEL_GLB_REPLY_SUCCESS;
}

static uint32_t ipc_fw_caps(uint32_t header)
{
	trace_ipc("Cap");

	return IPC_INTEL_GLB_REPLY_SUCCESS;
}

static void dma_complete(void *data, uint32_t type, struct dma_sg_elem *next)
{
	struct intel_ipc_data *iipc = (struct intel_ipc_data *)data;

	if (type == DMA_IRQ_TYPE_LLIST)
		wait_completed(&iipc->complete);
}

/*
 * Copy the audio buffer page tables from the host to the DSP max of 4K.
 */
static int get_page_descriptors(struct intel_ipc_data *iipc,
	struct ipc_intel_ipc_stream_alloc_req *req)
{
	struct ipc_intel_ipc_stream_ring *ring = &req->ringinfo;
	struct dma_sg_config config;
	struct dma_sg_elem elem;
	struct dma *dma;
	int chan, ret = 0;

	/* get DMA channel from DMAC0 */
	chan = dma_channel_get(iipc->dmac0);
	if (chan < 0) {
		trace_ipc_error("ePC");
		return chan;
	}
	dma = iipc->dmac0;

	/* set up DMA configuration */
	config.direction = DMA_DIR_HMEM_TO_LMEM;
	config.src_width = sizeof(uint32_t);
	config.dest_width = sizeof(uint32_t);
	config.cyclic = 0;
	list_init(&config.elem_list);

	/* set up DMA desciptor */
	elem.dest = (void*)iipc->page_table;
	/* avoid gcc error for different sizes */
	elem.src = NULL + ring->ring_pt_address;

	/* source buffer size is always PAGE_SIZE bytes */
	/* 20 bits for each page, round up to 32 */
	elem.size = (ring->num_pages * 5 * 16 + 31) / 32;
	list_item_prepend(&elem.list, &config.elem_list);

	ret = dma_set_config(dma, chan, &config);
	if (ret < 0) {
		trace_ipc_error("ePs");
		goto out;
	}

	/* set up callback */
	dma_set_cb(dma, chan, DMA_IRQ_TYPE_LLIST, dma_complete, iipc);

	wait_init(&iipc->complete);

	/* start the copy of page table to DSP */
	dma_start(dma, chan);

	/* wait for DMA to complete */
	iipc->complete.timeout = PLATFORM_IPC_DMA_TIMEOUT;
	ret = wait_for_completion_timeout(&iipc->complete);

	/* compressed page tables now in buffer at _ipc->page_table */
out:
	dma_channel_put(dma, chan);
	return ret;
}

/*
 * Parse the host page tables and create the audio DMA SG configuration
 * for host audio DMA buffer. This involves creating a dma_sg_elem for each
 * page table entry and adding each elem to a list in struct dma_sg_config.
 */
static int parse_page_descriptors(struct intel_ipc_data *iipc,
	struct ipc_intel_ipc_stream_alloc_req *req,
	struct comp_dev *host, uint8_t direction)
{
	struct ipc_intel_ipc_stream_ring *ring = &req->ringinfo;
	struct dma_sg_elem elem;
	int i, err;
	uint32_t idx, phy_addr;

	elem.size = HOST_PAGE_SIZE;

	for (i = 0; i < ring->num_pages; i++) {

		idx = (((i << 2) + i)) >> 1;
		phy_addr = iipc->page_table[idx] | (iipc->page_table[idx + 1] << 8)
				| (iipc->page_table[idx + 2] << 16);

		if (i & 0x1)
			phy_addr <<= 8;
		else
			phy_addr <<= 12;
		phy_addr &= 0xfffff000;

		/* use NULL here to avoid compiler warning casting different sizes */
		if (direction == STREAM_DIRECTION_PLAYBACK)
			elem.src = NULL + phy_addr;
		else
			elem.dest = NULL + phy_addr;

		err = pipeline_host_buffer(pipeline_static, host, &elem,
				req->ringinfo.ring_size);
		if (err < 0) {
			trace_ipc_error("ePb");
			return err;
		}
	}

	return 0;
}

/* allocate a new stream */
static uint32_t ipc_stream_alloc(uint32_t header)
{
	struct intel_ipc_data *iipc = ipc_get_drvdata(_ipc);
	struct ipc_intel_ipc_stream_alloc_req req;
	struct ipc_intel_ipc_stream_alloc_reply reply;
	struct stream_params *params;
	struct ipc_pcm_dev *pcm_dev;
	struct ipc_dai_dev *dai_dev;
	struct ipc_comp_dev *mixer_dev;
	struct ipc_comp_dev *volume_dev;
	struct sst_intel_ipc_stream_data *_stream_data;
	int err, i;
	uint8_t direction;

	trace_ipc("SAl");

	/* read alloc stream IPC from the inbox */
	mailbox_inbox_read(&req, 0, sizeof(req));

	/* format always PCM, now check source type */
	/* host a & dai ID values are from hard coded static pipeline */ 
	switch (req.stream_type) {
	case IPC_INTEL_STREAM_TYPE_SYSTEM:
		direction = STREAM_DIRECTION_PLAYBACK;
		_stream_data = _stream_dataP;
		break;
	case IPC_INTEL_STREAM_TYPE_CAPTURE:
		direction = STREAM_DIRECTION_CAPTURE;
		_stream_data = _stream_dataC;
		break;
	case IPC_INTEL_STREAM_TYPE_RENDER:
		direction = STREAM_DIRECTION_PLAYBACK;
		_stream_data = _stream_dataR;
		break;
	default:
		trace_ipc_error("eAt");
		return IPC_INTEL_GLB_REPLY_ERROR_INVALID_PARAM;
	};

	/* get the pcm_dev */
	pcm_dev = ipc_get_pcm_comp(stream_comp[req.stream_type].host_id);
	if (pcm_dev == NULL) {
		trace_ipc_error("eAC");
		return IPC_INTEL_GLB_REPLY_ERROR_INVALID_PARAM;
	}

	/* get the dai_dev */
	dai_dev = ipc_get_dai_comp(stream_comp[req.stream_type].dai_id);
	if (dai_dev == NULL) {
		trace_ipc_error("eAD");
		return IPC_INTEL_GLB_REPLY_ERROR_INVALID_PARAM; 
	}

	/* get the mixer_dev */
	mixer_dev = ipc_get_comp(stream_comp[req.stream_type].mixer_id);
	if (mixer_dev == NULL) {
		trace_ipc_error("eAM");
		return IPC_INTEL_GLB_REPLY_ERROR_INVALID_PARAM; 
	}

	/* get the volume_dev */
	volume_dev = ipc_get_comp(stream_comp[req.stream_type].volume_id);
	if (volume_dev == NULL) {
		trace_ipc_error("eAV");
		return IPC_INTEL_GLB_REPLY_ERROR_INVALID_PARAM;
	}

	/* check the state - if we are still allocated then reset and free */
	if (pcm_dev->state == IPC_HOST_ALLOC) {
		/* reset the pipeline */
		err = pipeline_reset(pcm_dev->dev.p, pcm_dev->dev.cd);
		if (err < 0) {
			trace_ipc_error("err");
			goto error;
		}

		pcm_dev->state = IPC_HOST_RESET;
	}

	params = &pcm_dev->params;

	/* read in format to create params */
	params->channels = req.format.ch_num;
	params->pcm.rate = req.format.frequency;
	params->direction = direction;

	/* work out bitdepth and framesize */
	switch (req.format.bitdepth) {
	case 16:
		params->pcm.format = STREAM_FORMAT_S16_LE;
		params->frame_size = 2 * params->channels;
		break;
	default:
		trace_ipc_error("eAb");
		goto error;
	}

	params->period_frames = req.format.period_frames;

	/* use DMA to read in compressed page table ringbuffer from host */
	err = get_page_descriptors(iipc, &req);
	if (err < 0) {
		trace_ipc_error("eAp");
		goto error;
	}

	/* Parse host tables */
	err = parse_page_descriptors(iipc, &req, pcm_dev->dev.cd, direction);
	if (err < 0) {
		trace_ipc_error("eAP");
		goto error;
	}

	/* configure pipeline audio params */
	err = pipeline_params(pipeline_static, pcm_dev->dev.cd, params);	
	if (err < 0) {
		trace_ipc_error("eAa");
		goto error;
	}

	/* pass the IPC presentation posn pointer to the DAI */
	err = comp_cmd(dai_dev->dev.cd, COMP_CMD_IPC_MMAP_PPOS,
		&_stream_data->presentation_posn);
	if (err < 0) {
		trace_ipc_error("eAr");
		goto error;
	}

	/* pass the IPC read posn pointer to the host */
	err = comp_cmd(pcm_dev->dev.cd, COMP_CMD_IPC_MMAP_RPOS,
		&_stream_data->read_posn);
	if (err < 0) {
		trace_ipc_error("eAR");
		goto error;
	}

	/* pass the volume readback posn to the host */
	err = comp_cmd(volume_dev->cd, COMP_CMD_IPC_MMAP_VOL(0),
		&_stream_data->vol[0].vol);
	if (err < 0) {
		trace_ipc_error("eAv");
		goto error;
	}
	err = comp_cmd(volume_dev->cd, COMP_CMD_IPC_MMAP_VOL(1),
		&_stream_data->vol[1].vol);
	if (err < 0) {
		trace_ipc_error("eAv");
		goto error;
	}

	/* at this point pipeline is ready for command so send stream reply */
	reply.stream_hw_id = stream_comp[req.stream_type].host_id;
	reply.mixer_hw_id = stream_comp[req.stream_type].mixer_id;

	/* set read pos and presentation pos address */
	reply.read_position_register_address =
		to_host_offset(_stream_data->read_posn);
	reply.presentation_position_register_address =
		to_host_offset(_stream_data->presentation_posn);

	/* set volume address */
	for (i = 0; i < IPC_INTEL_NO_CHANNELS; i++) {
		reply.peak_meter_register_address[i] =
			to_host_offset(_stream_data->vol[i].peak);
		reply.volume_register_address[i] =
			to_host_offset(_stream_data->vol[i].vol);
	}

	/* write reply to mailbox */
	mailbox_outbox_write(0, &reply, sizeof(reply));

	/* update stream state */
	pcm_dev->state = IPC_HOST_ALLOC;
	return IPC_INTEL_GLB_REPLY_SUCCESS;

error:
	pipeline_reset(pipeline_static, pcm_dev->dev.cd);
	return IPC_INTEL_GLB_REPLY_ERROR_INVALID_PARAM;
}

/* free stream resources */
static uint32_t ipc_stream_free(uint32_t header)
{
	struct ipc_intel_ipc_stream_free_req free_req;
	struct ipc_pcm_dev *pcm_dev;

	trace_ipc("SFr");

	/* read alloc stream IPC from the inbox */
	mailbox_inbox_read(&free_req, 0, sizeof(free_req));

	/* TODO: stream ID appears duplicated by mailbox */
	//stream_id = header & IPC_INTEL_STR_ID_MASK;
	//stream_id >>= IPC_INTEL_STR_ID_SHIFT;

	/* get the pcm_dev */
	pcm_dev = ipc_get_pcm_comp(free_req.stream_id);
	if (pcm_dev == NULL)
		goto error; 

	/* check stream state */
	if (pcm_dev->state == IPC_HOST_RUNNING ||
		pcm_dev->state == IPC_HOST_PAUSED)
		goto error;

	return IPC_INTEL_GLB_REPLY_SUCCESS;

error:
	return IPC_INTEL_GLB_REPLY_ERROR_INVALID_PARAM;
}

static uint32_t ipc_stream_info(uint32_t header)
{
	struct ipc_intel_ipc_stream_info_reply info;
	struct ipc_comp_dev *comp_dev;
	int err, i;

	trace_ipc("SIn");

	/* TODO: get data from topology */ 
	info.mixer_hw_id = stream_comp[IPC_INTEL_STREAM_TYPE_SYSTEM].mixer_id;

	/* get the mixer_dev */
	comp_dev = ipc_get_comp(info.mixer_hw_id);
	if (comp_dev == NULL)
		goto error; 

	/* pass the volume readback posn to the host */
	err = comp_cmd(comp_dev->cd, COMP_CMD_IPC_MMAP_VOL(0),
		&_stream_dataM->vol[0].vol);
	if (err < 0) {
		trace_ipc_error("eIv");
		goto error;
	}
	err = comp_cmd(comp_dev->cd, COMP_CMD_IPC_MMAP_VOL(1),
		&_stream_dataM->vol[1].vol);
	if (err < 0) {
		trace_ipc_error("eIv");
		goto error;
	}

	/* TODO: this is duplicating standard stream alloc mixer */
	for (i = 0; i < IPC_INTEL_NO_CHANNELS; i++) {
		info.peak_meter_register_address[i] =
			to_host_offset(_stream_dataM->vol[i].peak);
		info.volume_register_address[i] =
			to_host_offset(_stream_dataM->vol[i].vol);
	}

	mailbox_outbox_write(0, &info, sizeof(info));
	return IPC_INTEL_GLB_REPLY_SUCCESS;

/* TODO: define error code for failure here */
error:
	return IPC_INTEL_GLB_REPLY_SUCCESS;
}

static uint32_t ipc_dump(uint32_t header)
{
	trace_ipc("Dum");

	return IPC_INTEL_GLB_REPLY_SUCCESS;
}

static uint32_t ipc_device_get_formats(uint32_t header)
{
	trace_ipc("DgF");

	return IPC_INTEL_GLB_REPLY_SUCCESS;
}

static uint32_t ipc_device_set_formats(uint32_t header)
{
	struct ipc_intel_ipc_device_config_req config_req;
	struct ipc_dai_dev *dai_dev;

	trace_ipc("DsF");

	/* read alloc stream IPC from the inbox */
	mailbox_inbox_read(&config_req, 0, sizeof(config_req));

	/* get SSP port TODO: align ports*/
	switch (config_req.ssp_interface) {
	case IPC_INTEL_DEVICE_SSP_0:

		break;
	case IPC_INTEL_DEVICE_SSP_1:

		break;
	case IPC_INTEL_DEVICE_SSP_2:

		break;
	default:
		/* TODO: */
		goto error;
	};

	/* TODO: playback/capture DAI dev get the pcm_dev */
	dai_dev = ipc_get_dai_comp(stream_comp[IPC_INTEL_STREAM_TYPE_SYSTEM].dai_id);
	if (dai_dev == NULL) {
		trace_ipc_error("eDg");
		goto error;
	}

	/* setup the DAI HW config - TODO hard coded due to IPC limitations */
	dai_dev->dai_config.mclk = config_req.clock_frequency;
	dai_dev->dai_config.format = DAI_FMT_I2S | DAI_FMT_CONT |
		DAI_FMT_NB_NF | DAI_FMT_CBS_CFS;
	dai_dev->dai_config.sample_size = 24;
	dai_dev->dai_config.bclk = 2400000;
	dai_dev->dai_config.mclk = 19200000;
	dai_dev->dai_config.clk_src = SSP_CLK_EXT;

	comp_dai_config(dai_dev->dev.cd, &dai_dev->dai_config);

error:
	return IPC_INTEL_GLB_REPLY_SUCCESS;
}

static uint32_t ipc_device_set_loopback(uint32_t header, uint32_t lbm)
{
	struct ipc_dai_dev *dai_dev;

	trace_ipc("DsL");

	/* TODO: playback/capture DAI dev get the pcm_dev */
	dai_dev = ipc_get_dai_comp(stream_comp[IPC_INTEL_STREAM_TYPE_SYSTEM].dai_id);
	if (dai_dev == NULL) {
		trace_ipc_error("eDg");
		goto error;
	}

	comp_dai_loopback(dai_dev->dev.cd, lbm);

error:
	return IPC_INTEL_GLB_REPLY_SUCCESS;
}

static uint32_t ipc_context_save(uint32_t header)
{
	struct intel_ipc_data *iipc = ipc_get_drvdata(_ipc);
	struct ipc_intel_ipc_dx_reply reply;

	trace_ipc("PMs");

	/* TODO: check we are inactive - all streams are suspended */

	/* TODO: mask ALL platform interrupts except DMA */

	/* TODO now save the context - create SG buffer config using */
	//mm_pm_context_save(struct dma_sg_config *sg);

	/* mask all DSP interrupts */
	arch_interrupt_disable_mask(0xffff);
	/* TODO: mask ALL platform interrupts inc DMA */

	/* TODO: clear any outstanding platform IRQs - TODO refine */

	/* TODO: stop ALL timers */
	platform_timer_stop(0);

	/* TODO: disable SSP and DMA HW */

	/* TODO: save the context */
	reply.entries_no = 0;

	mailbox_outbox_write(0, &reply, sizeof(reply));

	iipc->pm_prepare_D3 = 1;

	return IPC_INTEL_GLB_REPLY_SUCCESS;
}

static uint32_t ipc_context_restore(uint32_t header)
{
	trace_ipc("PMr");

	return IPC_INTEL_GLB_REPLY_SUCCESS;
}

static uint32_t ipc_stage_set_volume(uint32_t header)
{
	struct ipc_comp_dev *stream_dev;
	struct ipc_intel_ipc_volume_req req;
	struct comp_volume cv;
	uint32_t stream_id;
	int err;

	trace_ipc("VoS");

	/* read volume from the inbox */
	mailbox_inbox_read(&req, 0, sizeof(req));

	if (req.channel > 1 && req.channel != CHANNELS_ALL)
		goto error;

	/* TODO: add other channels */ 
	memset(&cv, 0, sizeof(cv));
	cv.update_bits = req.channel == CHANNELS_ALL ?
		CHANNELS_ALL : 0x1 << req.channel;
	cv.volume = req.target_volume;

	/* TODO: use mixer comp IDs to get mixer */
	/* the driver uses stream ID to also identify certain mixers */
	stream_id = header & IPC_INTEL_STR_ID_MASK;
	stream_id >>= IPC_INTEL_STR_ID_SHIFT;

	/* get the pcm_dev TODO: command for pipeline or mixer comp */
	stream_dev = ipc_get_comp(stream_id);
	if (stream_dev == NULL)
		goto error; 

	/* volume component is the one next to host(stream) or mixer(master) */
	err = comp_cmd(stream_dev->cd, COMP_CMD_VOLUME, &cv);
	if (err < 0)
		goto error;

	return IPC_INTEL_GLB_REPLY_SUCCESS;

error:
	return IPC_INTEL_GLB_REPLY_ERROR_INVALID_PARAM;
}

static uint32_t ipc_stage_get_volume(uint32_t header)
{
	struct ipc_comp_dev *stream_dev;
	struct comp_volume cv;
	uint32_t stream_id;
	int err;

	trace_ipc("VoG");

	/* the driver uses stream ID to also identify certain mixers */
	stream_id = header & IPC_INTEL_STR_ID_MASK;
	stream_id >>= IPC_INTEL_STR_ID_SHIFT;

	/* get the pcm_dev */
	stream_dev = ipc_get_comp(stream_id);
	if (stream_dev == NULL)
		goto error; 
	
	/* TODO: complete call with private volume data */
	err = comp_cmd(stream_dev->cd, COMP_CMD_VOLUME, &cv);
	if (err < 0)
		goto error;

	return IPC_INTEL_GLB_REPLY_SUCCESS;

error:
	return IPC_INTEL_GLB_REPLY_ERROR_INVALID_PARAM;
}

static uint32_t ipc_stage_write_pos(uint32_t header)
{
	trace_ipc("PoW");

	return IPC_INTEL_GLB_REPLY_SUCCESS;
}

static uint32_t ipc_stage_message(uint32_t header)
{
	uint32_t stg = msg_get_stage_type(header);

	switch (stg) {
	case IPC_INTEL_STG_GET_VOLUME:
		return ipc_stage_get_volume(header);
	case IPC_INTEL_STG_SET_VOLUME:
		return ipc_stage_set_volume(header);
	case IPC_INTEL_STG_SET_WRITE_POSITION:
		return ipc_stage_write_pos(header);
	case IPC_INTEL_STG_MUTE_LOOPBACK:
	case IPC_INTEL_STG_SET_FX_ENABLE:
	case IPC_INTEL_STG_SET_FX_DISABLE:
	case IPC_INTEL_STG_SET_FX_GET_PARAM:
	case IPC_INTEL_STG_SET_FX_SET_PARAM:
	case IPC_INTEL_STG_SET_FX_GET_INFO:
	default:
		return IPC_INTEL_GLB_REPLY_UNKNOWN_MESSAGE_TYPE;
	}
}

static uint32_t ipc_stream_reset(uint32_t header)
{
	struct ipc_pcm_dev *pcm_dev;
	uint32_t stream_id;
	int err;

	trace_ipc("SRt");

	stream_id = header & IPC_INTEL_STR_ID_MASK;
	stream_id >>= IPC_INTEL_STR_ID_SHIFT;

	/* get the pcm_dev */
	pcm_dev = ipc_get_pcm_comp(stream_id);
	if (pcm_dev == NULL) {
		trace_ipc_error("erg");
		goto error; 
	}

	/* send stop TODO: this should be done in trigger */
	err = pipeline_cmd(pcm_dev->dev.p, pcm_dev->dev.cd,
		COMP_CMD_STOP, NULL);
	if (err < 0) {
		trace_ipc_error("erc");
		goto error;
	}

	/* reset the pipeline */
	err = pipeline_reset(pcm_dev->dev.p, pcm_dev->dev.cd);
	if (err < 0) {
		trace_ipc_error("err");
		goto error;
	}

	pcm_dev->state = IPC_HOST_RESET;
	return IPC_INTEL_GLB_REPLY_SUCCESS;
error:
	return IPC_INTEL_GLB_REPLY_ERROR_INVALID_PARAM;
}

static uint32_t ipc_stream_pause(uint32_t header)
{
	struct ipc_pcm_dev *pcm_dev;
	uint32_t stream_id;
	int err;

	trace_ipc("SPa");

	stream_id = header & IPC_INTEL_STR_ID_MASK;
	stream_id >>= IPC_INTEL_STR_ID_SHIFT;

	/* get the pcm_dev */
	pcm_dev = ipc_get_pcm_comp(stream_id);
	if (pcm_dev == NULL) {
		trace_ipc_error("ePg");
		goto error;
	}

	/* IPC design require we have to pause an alloced stream ??  */
	if (pcm_dev->state == IPC_HOST_ALLOC)
		return IPC_INTEL_GLB_REPLY_SUCCESS;

	err = pipeline_cmd(pcm_dev->dev.p, pcm_dev->dev.cd,
		COMP_CMD_PAUSE, NULL);
	if (err < 0) {
		trace_ipc_error("ePc");
		goto error;
	}

	pcm_dev->state = IPC_HOST_PAUSED;

error:
	return IPC_INTEL_GLB_REPLY_SUCCESS;
}

static uint32_t ipc_stream_resume(uint32_t header)
{
	struct ipc_pcm_dev *pcm_dev;
	uint32_t stream_id, cmd = COMP_CMD_RELEASE;
	int err;

	trace_ipc("SRe");

	stream_id = header & IPC_INTEL_STR_ID_MASK;
	stream_id >>= IPC_INTEL_STR_ID_SHIFT;

	/* get the pcm_dev */
	pcm_dev = ipc_get_pcm_comp(stream_id);
	if (pcm_dev == NULL) {
		trace_ipc_error("eRg");
		goto error;
	}

	/* only prepare at the allocation statge */
	if (pcm_dev->state == IPC_HOST_ALLOC) {
		/* initialise the pipeline, preparing pcm data */
		err = pipeline_prepare(pipeline_static, pcm_dev->dev.cd);
		if (err < 0) {
			trace_ipc_error("eRp");
			goto error;
		}
		cmd = COMP_CMD_START;
	}

	/* initialise the pipeline */
	err = pipeline_cmd(pcm_dev->dev.p, pcm_dev->dev.cd,
			cmd, NULL);
	if (err < 0) {
		trace_ipc_error("eRc");
		goto error;
	}

	pcm_dev->state = IPC_HOST_RUNNING;

error:
	return IPC_INTEL_GLB_REPLY_SUCCESS;
}

static uint32_t ipc_stream_notification(uint32_t header)
{
	trace_ipc("SNo");

	return IPC_INTEL_GLB_REPLY_SUCCESS;
}

static uint32_t ipc_stream_message(uint32_t header)
{
	uint32_t str = msg_get_stream_type(header);

	switch (str) {
	case IPC_INTEL_STR_RESET:
		return ipc_stream_reset(header);
	case IPC_INTEL_STR_PAUSE:
		return ipc_stream_pause(header);
	case IPC_INTEL_STR_RESUME:
		return ipc_stream_resume(header);
	case IPC_INTEL_STR_STAGE_MESSAGE:
		return ipc_stage_message(header);
	case IPC_INTEL_STR_NOTIFICATION:
		return ipc_stream_notification(header);
	default:
		return IPC_INTEL_GLB_REPLY_UNKNOWN_MESSAGE_TYPE;
	}
}

uint32_t ipc_cmd(void)
{
	uint32_t type, header;

	header = _ipc->host_msg;
	type = msg_get_global_type(header);

	switch (type) {
	case IPC_INTEL_GLB_GET_FW_VERSION:
		return ipc_fw_version(header);
	case IPC_INTEL_GLB_ALLOCATE_STREAM:
		return ipc_stream_alloc(header);
	case IPC_INTEL_GLB_FREE_STREAM:
		return ipc_stream_free(header);
	case IPC_INTEL_GLB_GET_FW_CAPABILITIES:
		return ipc_fw_caps(header);
	case IPC_INTEL_GLB_REQUEST_DUMP:
		return ipc_dump(header);
	case IPC_INTEL_GLB_GET_DEVICE_FORMATS:
		return ipc_device_get_formats(header);
	case IPC_INTEL_GLB_SET_DEVICE_FORMATS:
		return ipc_device_set_formats(header);
	case IPC_INTEL_GLB_ENABLE_LOOPBACK:
		return ipc_device_set_loopback(header, 1);
	case IPC_INTEL_GLB_DISABLE_LOOPBACK:
		return ipc_device_set_loopback(header, 0);
	case IPC_INTEL_GLB_ENTER_DX_STATE:
		return ipc_context_save(header);
	case IPC_INTEL_GLB_GET_MIXER_STREAM_INFO:
		return ipc_stream_info(header);
	case IPC_INTEL_GLB_RESTORE_CONTEXT:
		return ipc_context_restore(header);
	case IPC_INTEL_GLB_STREAM_MESSAGE:
		return ipc_stream_message(header);
	default:
		return IPC_INTEL_GLB_REPLY_UNKNOWN_MESSAGE_TYPE;
	}
}

/* locks held by caller */
static inline struct ipc_msg *msg_get_empty(struct ipc *ipc)
{
	struct ipc_msg *msg = NULL;

	if (!list_is_empty(&ipc->empty_list)) {
		msg = list_first_item(&ipc->empty_list, struct ipc_msg, list);
		list_item_del(&msg->list);
	}

	return msg;
}

/* Send stream command */
int ipc_stream_send_notification(struct comp_dev *cdev,
	struct comp_position *cp)
{
	struct ipc_intel_ipc_stream_get_position msg;
	uint32_t header;

	/* TODO: position not used in driver, it only uses the pos registers */
	msg.position = 100;
	msg.fw_cycle_count = 0;

	header = IPC_INTEL_GLB_TYPE(IPC_INTEL_GLB_STREAM_MESSAGE) |
		IPC_INTEL_STR_TYPE(IPC_INTEL_STR_NOTIFICATION) |
		IPC_INTEL_STG_TYPE(IPC_POSITION_CHANGED) |
		IPC_INTEL_STR_ID(cdev->id);

	return ipc_queue_host_message(_ipc, header, &msg, sizeof(msg),
		NULL, 0, NULL, NULL);
}

int ipc_queue_host_message(struct ipc *ipc, uint32_t header,
	void *tx_data, size_t tx_bytes, void *rx_data,
	size_t rx_bytes, void (*cb)(void*, void*), void *cb_data)
{
	struct ipc_msg *msg;
	uint32_t flags;
	int ret = 0;

	spin_lock_irq(&ipc->lock, flags);

	/* get a free message */
	msg = msg_get_empty(ipc);
	if (msg == NULL) {
		trace_ipc_error("eQb");
		ret = -EBUSY;
		goto out;
	}

	/* prepare the message */
	msg->header = header;
	msg->tx_size = tx_bytes;
	msg->rx_size = rx_bytes;
	msg->cb_data = cb_data;
	msg->cb = cb;

	/* copy mailbox data to message */
	if (tx_bytes > 0 && tx_bytes < MSG_MAX_SIZE)
		rmemcpy(msg->tx_data, tx_data, tx_bytes);

	/* now queue the message */
	ipc->dsp_pending = 1;
	list_item_append(&msg->list, &ipc->msg_list);

out:
	spin_unlock_irq(&ipc->lock, flags);
	return ret;
}

/* process current message */
int ipc_process_msg_queue(void)
{
	if (_ipc->host_pending)
		ipc_platform_do_cmd(_ipc);
	if (_ipc->dsp_pending)
		ipc_platform_send_msg(_ipc);
	return 0;
}
