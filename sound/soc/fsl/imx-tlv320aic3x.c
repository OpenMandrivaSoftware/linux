/*
 * imx-tlv320aic3x.c  --  SoC audio for Prosoft PKM-01 device
 *
 * Copyright 2014 Denis Grigoryev, PROSOFT Co. <grigoryev@spb.prosoft.ru>
 *
 * Based on sound/soc/imx/eukrea-tlv320.c  which is
 * Copyright 2010 Eric Bénard, Eukréa Electromatique <eric@eukrea.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <asm/mach-types.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>


#include "../codecs/tlv320aic3x.h"
#include "imx-ssi.h"
#include "imx-audmux.h"

#define CODEC_CLOCK 24000000

struct imx_tlv320aic3x_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	int amp_en_gpio;
	struct clk *clk;
};

static int imx_tlv320_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret) {
		pr_err("%s: failed set cpu dai format\n", __func__);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret) {
		pr_err("%s: failed set codec dai format\n", __func__);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
				     CODEC_CLOCK, SND_SOC_CLOCK_IN);
	if (ret) {
		pr_err("%s: failed setting codec sysclk\n", __func__);
		return ret;
	}
	snd_soc_dai_set_tdm_slot(cpu_dai, 0xffffffc, 0xffffffc, 2, 0);

	ret = snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0,
				     SND_SOC_CLOCK_IN);
	if (ret) {
		pr_err("can't set CPU system clock IMX_SSP_SYS_CLK\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops imx_tlv320_snd_ops = {
	.hw_params	= imx_tlv320_hw_params,
};

static const char *aic3x_mic_bias[] = { "Off", "2V", "2.5V", "AVDD" };
static const struct soc_enum aic3x_mic_bias_enum[] = {
	SOC_ENUM_SINGLE(MICBIAS_CTRL, 6, 4, aic3x_mic_bias),
};

static const struct snd_kcontrol_new aic3x_snd_controls[] = {
	SOC_ENUM("Mic Bias", aic3x_mic_bias_enum),
};

static const struct snd_soc_dapm_route pkm01_amp_intercon[] = {
	{ "Speaker", NULL, "LLOUT" },
};

static int pkm01_amp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *ctl, int event)
{
	struct imx_tlv320aic3x_data *data = snd_soc_card_get_drvdata(w->codec->card);

	if (gpio_is_valid(data->amp_en_gpio))
		gpio_set_value(data->amp_en_gpio, SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static const struct snd_soc_dapm_widget pkm01_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", pkm01_amp_event),
};

static int imx_tlv320_link_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;

	snd_soc_add_codec_controls(codec, aic3x_snd_controls, ARRAY_SIZE(aic3x_snd_controls));

	snd_soc_update_bits(codec, LINE2R_2_RADC_CTRL, 0x4, 0x4);
	snd_soc_update_bits(codec, LINE2L_2_LADC_CTRL, 0x4, 0x4);

	/*
	 * Weakly driven output common-mode voltage is generated from bandgap reference.
	 * Driver Ramp-up Step Timing Control = 2 ms.
	 * Output Driver Power-On Delay = 10 ms.
	 */
	snd_soc_write(codec, HPOUT_POP_REDUCTION, 0x4a);

	snd_soc_dapm_new_controls(&codec->dapm, pkm01_dapm_widgets,
				  ARRAY_SIZE(pkm01_dapm_widgets));

	snd_soc_dapm_add_routes(&codec->dapm, pkm01_amp_intercon,
			ARRAY_SIZE(pkm01_amp_intercon));

	snd_soc_dapm_sync(&codec->dapm);

	return 0;
}

static int imx_tlv320aic3x_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *cpu_np = NULL, *codec_np = NULL;
	struct imx_tlv320aic3x_data *data;
	struct i2c_client *codec_dev;
	struct platform_device *cpu_pdev;
	int int_port, ext_port;
	unsigned long clk_rate;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		return -EINVAL;
	}

	if (!strstr(cpu_np->name, "ssi"))
		goto audmux_bypass;

	ret = of_property_read_u32(np, "mux-int-port", &int_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-int-port missing or invalid\n");
		return ret;
	}

	ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-ext-port missing or invalid\n");
		return ret;
	}

	ret = of_property_read_u32(np, "audio-mclk-rate", (int *)&clk_rate);
	if (ret)
		dev_warn(&pdev->dev, "audio-mclk-rate is not set\n");

	/*
	 * The port numbering in the hardware manual starts at 1, while
	 * the audmux API expects it starts at 0.
	 */
	int_port--;
	ext_port--;
	ret = imx_audmux_v2_configure_port(int_port,
			IMX_AUDMUX_V2_PTCR_SYN |
			IMX_AUDMUX_V2_PTCR_TFSEL(ext_port) |
			IMX_AUDMUX_V2_PTCR_TCSEL(ext_port) |
			IMX_AUDMUX_V2_PTCR_TFSDIR |
			IMX_AUDMUX_V2_PTCR_TCLKDIR,
			IMX_AUDMUX_V2_PDCR_RXDSEL(ext_port));
	if (ret) {
		dev_err(&pdev->dev, "audmux internal port setup failed\n");
		return ret;
	}
	imx_audmux_v2_configure_port(ext_port,
			IMX_AUDMUX_V2_PTCR_SYN,
			IMX_AUDMUX_V2_PDCR_RXDSEL(int_port));
	if (ret) {
		dev_err(&pdev->dev, "audmux external port setup failed\n");
		return ret;
	}

audmux_bypass:
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EINVAL;
		goto fail;
	}
	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	data->amp_en_gpio = of_get_named_gpio(np, "amp-en-gpios", 0);

	data->dai.cpu_dai_name = dev_name(&cpu_pdev->dev);
	data->dai.codec_of_node = codec_np;
	data->dai.platform_of_node = cpu_np;
	data->dai.ops = &imx_tlv320_snd_ops;
	data->dai.init = imx_tlv320_link_init;
	data->dai.name = "tlv320aic3x";
	data->dai.stream_name = "tlv320aic3x";
	data->dai.codec_dai_name = "tlv320aic3x-hifi";
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBM_CFM;

	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;

	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		dev_warn(&pdev->dev, "snd_soc_of_parse_audio_routing failed\n");

	data->card.dai_link = &data->dai;
	data->card.num_links = 1;
	//data->card.dapm_widgets = imx_wm8962_dapm_widgets;
	//data->card.num_dapm_widgets = ARRAY_SIZE(imx_wm8962_dapm_widgets);

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	if (gpio_is_valid(data->amp_en_gpio)) {
		ret = devm_gpio_request(&pdev->dev, data->amp_en_gpio, "speaker-en");
		if (ret)
			dev_warn(&pdev->dev, "request gpio failed (%d)\n", ret);
		else
			gpio_direction_output(data->amp_en_gpio, 0);
	}

	ret = snd_soc_register_card(&data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

        data->clk = devm_clk_get(&pdev->dev, "audio-mclk");
	if (IS_ERR(data->clk)) {
		dev_warn(&pdev->dev, "get audio-mclk failed (%li)\n", PTR_ERR(data->clk));
		return ret;
	}

	ret = clk_prepare_enable(data->clk);
	if (ret)
		dev_warn(&pdev->dev, "clk_prepare_enable: %i\n", ret);

	return 0;
fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static int imx_tlv320aic3x_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id imx_tlv320_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-tlv320aic3x", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_tlv320_dt_ids);

static struct platform_driver imx_tlv320aic3x_driver = {
	.driver = {
		.name = "imx_tlv320aic3x",
		.owner = THIS_MODULE,
		.of_match_table = imx_tlv320_dt_ids,
	},
	.probe = imx_tlv320aic3x_probe,
	.remove = imx_tlv320aic3x_remove,
};

module_platform_driver(imx_tlv320aic3x_driver);

MODULE_AUTHOR("Denis Grigoryev <grigoryev@spb.prosoft.ru>");
MODULE_DESCRIPTION("CPUIMX ALSA SoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx_tlv320aic3x");
