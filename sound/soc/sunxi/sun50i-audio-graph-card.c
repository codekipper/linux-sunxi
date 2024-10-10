// SPDX-License-Identifier: GPL-2.0-only
//
// sun50i_audio_graph_card.c - Audio Graph based Machine Driver
//
// Copyright (c) 2020-2021 NVIDIA CORPORATION.  All rights reserved.

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/graph_card.h>
#include <sound/pcm_params.h>

#define simple_to_sun50i_priv(simple) \
		container_of(simple, struct sun50i_audio_priv, simple)

struct sun50i_audio_priv {
	struct simple_util_priv simple;
};

/* sun50i audio chip data */
struct sun50i_audio_cdata {
};

static int sun50i_audio_graph_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	printk("COOPS %s:%d\n", __func__, __LINE__);

	return simple_util_hw_params(substream, params);
}

static const struct snd_soc_ops sun50i_audio_graph_ops = {
	.startup	= simple_util_startup,
	.shutdown	= simple_util_shutdown,
	.hw_params	= sun50i_audio_graph_hw_params,
};

static int sun50i_audio_graph_card_probe(struct snd_soc_card *card)
{
	printk("COOPS %s:%d\n", __func__, __LINE__);

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

	return audio_graph_parse_of(&priv->simple, dev);
}

static const struct sun50i_audio_cdata sun50i_data = {
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
