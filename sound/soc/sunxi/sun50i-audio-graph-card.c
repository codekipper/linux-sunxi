// SPDX-License-Identifier: GPL-2.0-only
//
// sun50i_audio_graph_card.c - Audio Graph based Machine Driver
//
// Copyright (c) 2020-2021 NVIDIA CORPORATION.  All rights reserved.

#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <sound/graph_card.h>
#include <sound/pcm_params.h>

#define MAX_PLLA_OUT0_DIV 128

#define simple_to_sun50i_priv(simple) \
		container_of(simple, struct sun50i_audio_priv, simple)

enum srate_type {
	/*
	 * Sample rates multiple of 8000 Hz and below are supported:
	 * ( 8000, 16000, 32000, 48000, 96000, 192000 Hz )
	 */
	x8_RATE,

	/*
	 * Sample rates multiple of 11025 Hz and below are supported:
	 * ( 11025, 22050, 44100, 88200, 176400 Hz )
	 */
	x11_RATE,

	NUM_RATE_TYPE,
};

struct sun50i_audio_priv {
	struct simple_util_priv simple;
	struct clk *clk_plla_out0;
	struct clk *clk_plla;
};

/* sun50i audio chip data */
struct sun50i_audio_cdata {
	unsigned int plla_rates[NUM_RATE_TYPE];
	unsigned int plla_out0_rates[NUM_RATE_TYPE];
};

/* Setup PLL clock as per the given sample rate */
static int sun50i_audio_graph_update_pll(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct simple_util_priv *simple = snd_soc_card_get_drvdata(rtd->card);
	struct sun50i_audio_priv *priv = simple_to_sun50i_priv(simple);
	struct device *dev = rtd->card->dev;
	const struct sun50i_audio_cdata *data = of_device_get_match_data(dev);
	unsigned int plla_rate, plla_out0_rate, bclk;
	unsigned int srate = params_rate(params);
	int err;

	printk("COOPS %s:%d\n", __func__, __LINE__);
	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
	case 176400:
		plla_out0_rate = data->plla_out0_rates[x11_RATE];
		plla_rate = data->plla_rates[x11_RATE];
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 96000:
	case 192000:
		plla_out0_rate = data->plla_out0_rates[x8_RATE];
		plla_rate = data->plla_rates[x8_RATE];
		break;
	default:
		dev_err(rtd->card->dev, "Unsupported sample rate %u\n",
			srate);
		return -EINVAL;
	}

	/*
	 * Below is the clock relation:
	 *
	 *	PLLA
	 *	  |
	 *	  |--> PLLA_OUT0
	 *		  |
	 *		  |---> I2S modules
	 *		  |
	 *		  |---> DMIC modules
	 *		  |
	 *		  |---> DSPK modules
	 *
	 *
	 * Default PLLA_OUT0 rate might be too high when I/O is running
	 * at minimum PCM configurations. This may result in incorrect
	 * clock rates and glitchy audio. The maximum divider is 128
	 * and any thing higher than that won't work. Thus reduce PLLA_OUT0
	 * to work for lower configurations.
	 *
	 * This problem is seen for I2S only, as DMIC and DSPK minimum
	 * clock requirements are under allowed divider limits.
	 */
	bclk = srate * params_channels(params) * params_width(params);
	if (div_u64(plla_out0_rate, bclk) > MAX_PLLA_OUT0_DIV)
		plla_out0_rate >>= 1;

	dev_dbg(rtd->card->dev,
		"Update clock rates: PLLA(= %u Hz) and PLLA_OUT0(= %u Hz)\n",
		plla_rate, plla_out0_rate);

	/* Set PLLA rate */
	err = clk_set_rate(priv->clk_plla, plla_rate);
	if (err) {
		dev_err(rtd->card->dev,
			"Can't set plla rate for %u, err: %d\n",
			plla_rate, err);
		return err;
	}

	/* Set PLLA_OUT0 rate */
	err = clk_set_rate(priv->clk_plla_out0, plla_out0_rate);
	if (err) {
		dev_err(rtd->card->dev,
			"Can't set plla_out0 rate %u, err: %d\n",
			plla_out0_rate, err);
		return err;
	}

	return err;
}

static int sun50i_audio_graph_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int err;

	printk("COOPS %s:%d\n", __func__, __LINE__);
	/*
	 * This gets called for each DAI link (FE or BE) when DPCM is used.
	 * We may not want to update PLLA rate for each call. So PLLA update
	 * must be restricted to external I/O links (I2S, DMIC or DSPK) since
	 * they actually depend on it. I/O modules update their clocks in
	 * hw_param() of their respective component driver and PLLA rate
	 * update here helps them to derive appropriate rates.
	 *
	 * TODO: When more HW accelerators get added (like sample rate
	 * converter, volume gain controller etc., which don't really
	 * depend on PLLA) we need a better way to filter here.
	 */
	if (cpu_dai->driver->ops && rtd->dai_link->no_pcm) {
		err = sun50i_audio_graph_update_pll(substream, params);
		if (err)
			return err;
	}

	return simple_util_hw_params(substream, params);
}

static const struct snd_soc_ops sun50i_audio_graph_ops = {
	.startup	= simple_util_startup,
	.shutdown	= simple_util_shutdown,
	.hw_params	= sun50i_audio_graph_hw_params,
};

static int sun50i_audio_graph_card_probe(struct snd_soc_card *card)
{
	struct simple_util_priv *simple = snd_soc_card_get_drvdata(card);
	struct sun50i_audio_priv *priv = simple_to_sun50i_priv(simple);

	printk("COOPS %s:%d\n", __func__, __LINE__);
	priv->clk_plla = devm_clk_get(card->dev, "pll_a");
	if (IS_ERR(priv->clk_plla)) {
		dev_err(card->dev, "Can't retrieve clk pll_a\n");
		return PTR_ERR(priv->clk_plla);
	}

	priv->clk_plla_out0 = devm_clk_get(card->dev, "plla_out0");
	if (IS_ERR(priv->clk_plla_out0)) {
		dev_err(card->dev, "Can't retrieve clk plla_out0\n");
		return PTR_ERR(priv->clk_plla_out0);
	}

	return graph_util_card_probe(card);
}

static int sun50i_audio_graph_probe(struct platform_device *pdev)
{
	struct sun50i_audio_priv *priv;
	struct device *dev = &pdev->dev;
	struct snd_soc_card *card;

	printk("COOPS %s:%d\n", __func__, __LINE__);
	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	card = simple_priv_to_card(&priv->simple);
	card->driver_name = "sun50i-ape";

	card->probe = sun50i_audio_graph_card_probe;

	/* audio_graph_parse_of() depends on below */
	card->component_chaining = 1;
	priv->simple.ops = &sun50i_audio_graph_ops;
	priv->simple.force_dpcm = 1;

	printk("COOPS %s:%d\n", __func__, __LINE__);
	return audio_graph_parse_of(&priv->simple, dev);
}

static const struct sun50i_audio_cdata sun50i_data = {
	/* PLLA */
	.plla_rates[x8_RATE] = 368640000,
	.plla_rates[x11_RATE] = 338688000,
	/* PLLA_OUT0 */
	.plla_out0_rates[x8_RATE] = 49152000,
	.plla_out0_rates[x11_RATE] = 45158400,
};

static const struct of_device_id graph_of_sun50i_match[] = {
	{ .compatible = "allwinner,sun50i-audio-graph-card",
	  .data = &sun50i_data },
	{},
};
MODULE_DEVICE_TABLE(of, graph_of_sun50i_match);

static struct platform_driver sun50i_audio_graph_card = {
	.driver = {
		.name = "sun50i-audio-graph-card",
		.pm = &snd_soc_pm_ops,
		.of_match_table = graph_of_sun50i_match,
	},
	.probe = sun50i_audio_graph_probe,
	.remove_new = simple_util_remove,
};
module_platform_driver(sun50i_audio_graph_card);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ASoC Allwinner SUNXI Audio Graph Sound Card");
MODULE_AUTHOR("Sameer Pujar <spujar@nvidia.com>");
