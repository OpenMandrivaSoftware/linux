// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author: Chris Zhong <zyw@rock-chips.com>
 *
 * Cadence MHDP Audio driver
 *
 */
#include <linux/clk.h>
#include <linux/reset.h>
#include <drm/bridge/cdns-mhdp.h>
#include <sound/hdmi-codec.h>
#include <drm/drm_of.h>
#include <drm/drm_vblank.h>
#include <drm/drm_print.h>

#include "cdns-mhdp-common.h"

#define CDNS_DP_SPDIF_CLK		200000000

int cdns_mhdp_audio_stop(struct cdns_mhdp_device *mhdp, struct audio_info *audio)
{
	int ret;

	ret = cdns_mhdp_reg_write(mhdp, AUDIO_PACK_CONTROL, 0);
	if (ret) {
		DRM_DEV_ERROR(mhdp->dev, "audio stop failed: %d\n", ret);
		return ret;
	}

	cdns_mhdp_bus_write(0, mhdp, SPDIF_CTRL_ADDR);

	/* clearn the audio config and reset */
	cdns_mhdp_bus_write(0, mhdp, AUDIO_SRC_CNTL);
	cdns_mhdp_bus_write(0, mhdp, AUDIO_SRC_CNFG);
	cdns_mhdp_bus_write(AUDIO_SW_RST, mhdp, AUDIO_SRC_CNTL);
	cdns_mhdp_bus_write(0, mhdp, AUDIO_SRC_CNTL);

	/* reset smpl2pckt component  */
	cdns_mhdp_bus_write(0, mhdp, SMPL2PKT_CNTL);
	cdns_mhdp_bus_write(AUDIO_SW_RST, mhdp, SMPL2PKT_CNTL);
	cdns_mhdp_bus_write(0, mhdp, SMPL2PKT_CNTL);

	/* reset FIFO */
	cdns_mhdp_bus_write(AUDIO_SW_RST, mhdp, FIFO_CNTL);
	cdns_mhdp_bus_write(0, mhdp, FIFO_CNTL);

	if (audio->format == AFMT_SPDIF_INT)
		clk_disable_unprepare(mhdp->spdif_clk);

	return 0;
}
EXPORT_SYMBOL(cdns_mhdp_audio_stop);

int cdns_mhdp_audio_mute(struct cdns_mhdp_device *mhdp, bool enable)
{
	int ret = true;

	ret = cdns_mhdp_reg_write_bit(mhdp, DP_VB_ID, 4, 1, enable);
	if (ret)
		DRM_DEV_ERROR(mhdp->dev, "audio mute failed: %d\n", ret);

	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_audio_mute);

static void cdns_mhdp_audio_config_i2s(struct cdns_mhdp_device *mhdp,
				       struct audio_info *audio)
{
	int sub_pckt_num = 1, i2s_port_en_val = 0xf, i;
	u32 val;

	if (audio->channels == 2) {
		if (mhdp->dp.num_lanes == 1)
			sub_pckt_num = 2;
		else
			sub_pckt_num = 4;

		i2s_port_en_val = 1;
	} else if (audio->channels == 4) {
		i2s_port_en_val = 3;
	}

	cdns_mhdp_bus_write(0x0, mhdp, SPDIF_CTRL_ADDR);

	cdns_mhdp_bus_write(SYNC_WR_TO_CH_ZERO, mhdp, FIFO_CNTL);

	val = MAX_NUM_CH(audio->channels);
	val |= NUM_OF_I2S_PORTS(audio->channels);
	val |= AUDIO_TYPE_LPCM;
	val |= CFG_SUB_PCKT_NUM(sub_pckt_num);
	cdns_mhdp_bus_write(val, mhdp, SMPL2PKT_CNFG);

	if (audio->sample_width == 16)
		val = 0;
	else if (audio->sample_width == 24)
		val = 1 << 9;
	else
		val = 2 << 9;

	val |= AUDIO_CH_NUM(audio->channels);
	val |= I2S_DEC_PORT_EN(i2s_port_en_val);
	val |= TRANS_SMPL_WIDTH_32;
	cdns_mhdp_bus_write(val, mhdp, AUDIO_SRC_CNFG);

	for (i = 0; i < (audio->channels + 1) / 2; i++) {
		if (audio->sample_width == 16)
			val = (0x02 << 8) | (0x02 << 20);
		else if (audio->sample_width == 24)
			val = (0x0b << 8) | (0x0b << 20);

		val |= ((2 * i) << 4) | ((2 * i + 1) << 16);
		cdns_mhdp_bus_write(val, mhdp, STTS_BIT_CH(i));
	}

	switch (audio->sample_rate) {
	case 32000:
		val = SAMPLING_FREQ(3) |
		      ORIGINAL_SAMP_FREQ(0xc);
		break;
	case 44100:
		val = SAMPLING_FREQ(0) |
		      ORIGINAL_SAMP_FREQ(0xf);
		break;
	case 48000:
		val = SAMPLING_FREQ(2) |
		      ORIGINAL_SAMP_FREQ(0xd);
		break;
	case 88200:
		val = SAMPLING_FREQ(8) |
		      ORIGINAL_SAMP_FREQ(0x7);
		break;
	case 96000:
		val = SAMPLING_FREQ(0xa) |
		      ORIGINAL_SAMP_FREQ(5);
		break;
	case 176400:
		val = SAMPLING_FREQ(0xc) |
		      ORIGINAL_SAMP_FREQ(3);
		break;
	case 192000:
		val = SAMPLING_FREQ(0xe) |
		      ORIGINAL_SAMP_FREQ(1);
		break;
	}
	val |= 4;
	cdns_mhdp_bus_write(val, mhdp, COM_CH_STTS_BITS);

	cdns_mhdp_bus_write(SMPL2PKT_EN, mhdp, SMPL2PKT_CNTL);
	cdns_mhdp_bus_write(I2S_DEC_START, mhdp, AUDIO_SRC_CNTL);
}

static void cdns_mhdp_audio_config_spdif(struct cdns_mhdp_device *mhdp)
{
	u32 val;

	cdns_mhdp_bus_write(SYNC_WR_TO_CH_ZERO, mhdp, FIFO_CNTL);

	val = MAX_NUM_CH(2) | AUDIO_TYPE_LPCM | CFG_SUB_PCKT_NUM(4);
	cdns_mhdp_bus_write(val, mhdp, SMPL2PKT_CNFG);
	cdns_mhdp_bus_write(SMPL2PKT_EN, mhdp, SMPL2PKT_CNTL);

	val = SPDIF_ENABLE | SPDIF_AVG_SEL | SPDIF_JITTER_BYPASS;
	cdns_mhdp_bus_write(val, mhdp, SPDIF_CTRL_ADDR);

	clk_prepare_enable(mhdp->spdif_clk);
	clk_set_rate(mhdp->spdif_clk, CDNS_DP_SPDIF_CLK);
}

int cdns_mhdp_audio_config(struct cdns_mhdp_device *mhdp,
							struct audio_info *audio)
{
	int ret;

	/* reset the spdif clk before config */
	if (audio->format == AFMT_SPDIF_INT) {
		reset_control_assert(mhdp->spdif_rst);
		reset_control_deassert(mhdp->spdif_rst);
	}

	ret = cdns_mhdp_reg_write(mhdp, CM_LANE_CTRL, LANE_REF_CYC);
	if (ret)
		goto err_audio_config;

	ret = cdns_mhdp_reg_write(mhdp, CM_CTRL, 0);
	if (ret)
		goto err_audio_config;

	if (audio->format == AFMT_I2S)
		cdns_mhdp_audio_config_i2s(mhdp, audio);
	else if (audio->format == AFMT_SPDIF_INT)
		cdns_mhdp_audio_config_spdif(mhdp);

	ret = cdns_mhdp_reg_write(mhdp, AUDIO_PACK_CONTROL, AUDIO_PACK_EN);

err_audio_config:
	if (ret)
		DRM_DEV_ERROR(mhdp->dev, "audio config failed: %d\n", ret);
	return ret;
}
EXPORT_SYMBOL(cdns_mhdp_audio_config);

static int audio_hw_params(struct device *dev,  void *data,
				  struct hdmi_codec_daifmt *daifmt,
				  struct hdmi_codec_params *params)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);
	struct audio_info audio = {
		.sample_width = params->sample_width,
		.sample_rate = params->sample_rate,
		.channels = params->channels,
		.connector_type = mhdp->connector.base.connector_type,
	};
	int ret;

	switch (daifmt->fmt) {
	case HDMI_I2S:
		audio.format = AFMT_I2S;
		break;
	case HDMI_SPDIF:
		audio.format = AFMT_SPDIF_EXT;
		break;
	default:
		DRM_DEV_ERROR(dev, "Invalid format %d\n", daifmt->fmt);
		ret = -EINVAL;
		goto out;
	}

	ret = cdns_mhdp_audio_config(mhdp, &audio);
	if (!ret)
		mhdp->audio_info = audio;

out:
	return ret;
}

static void audio_shutdown(struct device *dev, void *data)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);
	int ret;

	ret = cdns_mhdp_audio_stop(mhdp, &mhdp->audio_info);
	if (!ret)
		mhdp->audio_info.format = AFMT_UNUSED;
}

static int audio_digital_mute(struct device *dev, void *data,
				     bool enable)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);
	int ret;

	ret = cdns_mhdp_audio_mute(mhdp, enable);

	return ret;
}

static int audio_get_eld(struct device *dev, void *data,
				u8 *buf, size_t len)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);

	memcpy(buf, mhdp->connector.base.eld,
	       min(sizeof(mhdp->connector.base.eld), len));

	return 0;
}

static const struct hdmi_codec_ops audio_codec_ops = {
	.hw_params = audio_hw_params,
	.audio_shutdown = audio_shutdown,
	.digital_mute = audio_digital_mute,
	.get_eld = audio_get_eld,
};

int cdns_mhdp_register_audio_driver(struct device *dev)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);
	struct hdmi_codec_pdata codec_data = {
		.i2s = 1,
		.spdif = 1,
		.ops = &audio_codec_ops,
		.max_i2s_channels = 8,
	};

	mhdp->audio_pdev = platform_device_register_data(
			      dev, HDMI_CODEC_DRV_NAME, 1,
			      &codec_data, sizeof(codec_data));

	return PTR_ERR_OR_ZERO(mhdp->audio_pdev);
}

void cdns_mhdp_unregister_audio_driver(struct device *dev)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);

	platform_device_unregister(mhdp->audio_pdev);
}
