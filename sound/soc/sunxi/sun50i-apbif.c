// SPDX-License-Identifier: GPL-2.0-only
//
// sun50i-apbif.c - Allwinner APBIF driver
//
// Copyright (c) 2020 NVIDIA CORPORATION.  All rights reserved.

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

/* SUN50I Audio Hub APBIF registers list */
#define SUN50I_AHUB_APBIF_TX_CTL(n)		(0x10 + ((n) * 0x30))
#define SUN50I_AHUB_APBIF_TX_IRQ_CTL(n)		(0x14 + ((n) * 0x30))
#define SUN50I_AHUB_APBIF_TX_IRQ_STA(n)		(0x18 + ((n) * 0x30))
#define SUN50I_AHUB_APBIF_TXFIFO_CTL(n)		(0x20 + ((n) * 0x30))
#define SUN50I_AHUB_APBIF_TXFIFO_STA(n)		(0x24 + ((n) * 0x30))
#define SUN50I_AHUB_APBIF_TXFIFO(n)		(0x30 + ((n) * 0x30))
#define SUN50I_AHUB_APBIF_TXFIFO_CNT(n)		(0x34 + ((n) * 0x30))
#define SUN50I_AHUB_APBIF_RX_CTL(n)		(0x100 + ((n) * 0x30))
#define SUN50I_AHUB_APBIF_RX_IRQ_CTL(n)		(0x104 + ((n) * 0x30))
#define SUN50I_AHUB_APBIF_RX_IRQ_STA(n)		(0x108 + ((n) * 0x30))
#define SUN50I_AHUB_APBIF_RXFIFO_CTL(n)		(0x110 + ((n) * 0x30))
#define SUN50I_AHUB_APBIF_RXFIFO_STA(n)		(0x114 + ((n) * 0x30))
#define SUN50I_AHUB_APBIF_RXFIFO_CONT(n)	(0x118 + ((n) * 0x30))
#define SUN50I_AHUB_APBIF_RXFIFO(n)		(0x120 + ((n) * 0x30))
#define SUN50I_AHUB_APBIF_RXFIFO_CNT(n)		(0x124 + ((n) * 0x30))

/* SUN50I_AHUB_APBIF_TX_CTL */
#define APBIF_TX_WS		16
#define APBIF_TX_CHAN_NUM	8
#define	APBIF_TX_START		4

/* SUN50I_AHUB_APBIF_TX_IRQ_CTL */
#define APBIF_TX_DRQ		3
#define APBIF_TX_OVEN		1
#define APBIF_TX_EMEN		0

/* SUN50I_AHUB_APBIF_TX_IRQ_STA */
#define APBIF_TX_OV_PEND	1
#define APBIF_TX_EM_PEND	0

/* SUN50I_AHUB_APBIF_TXFIFO_CTL */
#define APBIF_TX_FTX		12
#define APBIF_TX_LEVEL		4
#define APBIF_TX_TXIM		0

/* SUN50I_AHUB_APBIF_TXFIFO_STA */
#define APBIF_TX_EMPTY		8
#define APBIF_TX_EMCNT		0

/* SUN50I_AHUB_APBIF_RX_CTL */
#define APBIF_RX_WS		16
#define APBIF_RX_CHAN_NUM	8
#define	APBIF_RX_START		4

/* SUN50I_AHUB_APBIF_RX_IRQ_CTL */
#define APBIF_RX_DRQ		3
#define APBIF_RX_UVEN		2
#define APBIF_RX_AVEN		0

/* SUN50I_AHUB_APBIF_RX_IRQ_STA */
#define APBIF_RX_UV_PEND	1
#define APBIF_RX_AV_PEND	0

/* SUN50I_AHUB_APBIF_RXFIFO_CTL */
#define APBIF_RX_FRX		12
#define APBIF_RX_LEVEL		4
#define APBIF_RX_RXOM		0

/* SUN50I_AHUB_APBIF_RXFIFO_STA */
#define APBIF_RX_AVAIL		8
#define APBIF_RX_AVCNT		0

/* SUN50I_AHUB_APBIF_RXFIFO_CONT */
#define APBIF_RX_APBIF_TXDIF0		31
#define APBIF_RX_APBIF_TXDIF1		30
#define APBIF_RX_APBIF_TXDIF2		29
#define APBIF_RX_I2S0_TXDIF		27
#define APBIF_RX_I2S1_TXDIF		26
#define APBIF_RX_I2S2_TXDIF		25
#define APBIF_RX_I2S3_TXDIF		23
#define APBIF_RX_DAM0_TXDIF		19
#define APBIF_RX_DAM1_TXDIF		15

struct sun50i_apbif_quirks {
	const struct snd_soc_component_driver *cmpnt;
	const struct regmap_config *regmap_conf;
	struct snd_soc_dai_driver *dais;
	unsigned int tx_base;
	unsigned int rx_base;
	unsigned int num_ch;
};

struct sun50i_apbif {
	struct regmap				*regmap;
	struct snd_dmaengine_dai_dma_data 	*playback_dma_data;
	struct snd_dmaengine_dai_dma_data 	*capture_dma_data;
	const struct sun50i_apbif_quirks	*variant;
};
static const struct reg_default sun50i_apbif_reg_defaults[] = {
};

static bool sun50i_apbif_wr_reg(struct device *dev, unsigned int reg)
{
	return true;
}

static bool sun50i_apbif_rd_reg(struct device *dev, unsigned int reg)
{
	return true;
}

static bool sun50i_apbif_volatile_reg(struct device *dev, unsigned int reg)
{
	return true;
}

static const struct regmap_config sun50i_apbif_regmap_config = {
	.reg_bits		= 32,
	.reg_stride		= 4,
	.val_bits		= 32,
	.max_register		= SUN50I_AHUB_APBIF_RXFIFO_CNT(2),
	.writeable_reg		= sun50i_apbif_wr_reg,
	.readable_reg		= sun50i_apbif_rd_reg,
	.volatile_reg		= sun50i_apbif_volatile_reg,
	.reg_defaults		= sun50i_apbif_reg_defaults,
	.cache_type		= REGCACHE_FLAT,
};

static int __maybe_unused sun50i_apbif_runtime_suspend(struct device *dev)
{
	struct sun50i_apbif *apbif = dev_get_drvdata(dev);

	regcache_cache_only(apbif->regmap, true);
	regcache_mark_dirty(apbif->regmap);

	return 0;
}

static int __maybe_unused sun50i_apbif_runtime_resume(struct device *dev)
{
	struct sun50i_apbif *apbif = dev_get_drvdata(dev);

	regcache_cache_only(apbif->regmap, false);
	regcache_sync(apbif->regmap);

	return 0;
}

static int sun50i_apbif_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct sun50i_apbif *apbif = snd_soc_dai_get_drvdata(dai);

	switch (params_format(params)) {
	case	SNDRV_PCM_FORMAT_S16_LE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			/* special handle for HDMI rawdata mode */
			//if (sunxi_hdmi->hdmi_format > 1) {
			if (0) {
				regmap_update_bits(apbif->regmap,
					SUN50I_AHUB_APBIF_TX_CTL(dai->id),
					(7<<APBIF_TX_WS), (7<<APBIF_TX_WS));
				regmap_update_bits(apbif->regmap,
					SUN50I_AHUB_APBIF_TXFIFO_CTL(dai->id),
					(1<<APBIF_TX_TXIM), (0<<APBIF_TX_TXIM));
			} else {
				regmap_update_bits(apbif->regmap,
					SUN50I_AHUB_APBIF_TX_CTL(dai->id),
					(7<<APBIF_TX_WS), (3<<APBIF_TX_WS));
				regmap_update_bits(apbif->regmap,
					SUN50I_AHUB_APBIF_TXFIFO_CTL(dai->id),
					(1<<APBIF_TX_TXIM), (1<<APBIF_TX_TXIM));
			}
		} else {
			regmap_update_bits(apbif->regmap,
					SUN50I_AHUB_APBIF_RX_CTL(dai->id),
					(7<<APBIF_RX_WS), (3<<APBIF_RX_WS));
			regmap_update_bits(apbif->regmap,
				SUN50I_AHUB_APBIF_RXFIFO_CTL(dai->id),
				(3<<APBIF_RX_RXOM), (1<<APBIF_RX_RXOM));
		}
		break;
	case	SNDRV_PCM_FORMAT_S24_LE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			regmap_update_bits(apbif->regmap,
					SUN50I_AHUB_APBIF_TX_CTL(dai->id),
					(7<<APBIF_TX_WS), (5<<APBIF_TX_WS));
			regmap_update_bits(apbif->regmap,
				SUN50I_AHUB_APBIF_TXFIFO_CTL(dai->id),
				(1<<APBIF_TX_TXIM), (1<<APBIF_TX_TXIM));
		} else {
			regmap_update_bits(apbif->regmap,
					SUN50I_AHUB_APBIF_RX_CTL(dai->id),
					(7<<APBIF_RX_WS), (5<<APBIF_RX_WS));
			regmap_update_bits(apbif->regmap,
				SUN50I_AHUB_APBIF_RXFIFO_CTL(dai->id),
				(3<<APBIF_RX_RXOM), (1<<APBIF_RX_RXOM));
		}
		break;
	case	SNDRV_PCM_FORMAT_S32_LE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			regmap_update_bits(apbif->regmap,
					SUN50I_AHUB_APBIF_TX_CTL(dai->id),
					(7<<APBIF_TX_WS), (7<<APBIF_TX_WS));
			regmap_update_bits(apbif->regmap,
				SUN50I_AHUB_APBIF_TXFIFO_CTL(dai->id),
				(1<<APBIF_TX_TXIM), (1<<APBIF_TX_TXIM));
		} else {
			regmap_update_bits(apbif->regmap,
					SUN50I_AHUB_APBIF_RX_CTL(dai->id),
					(7<<APBIF_RX_WS), (7<<APBIF_RX_WS));
			regmap_update_bits(apbif->regmap,
				SUN50I_AHUB_APBIF_RXFIFO_CTL(dai->id),
				(3<<APBIF_RX_RXOM), (1<<APBIF_RX_RXOM));
		}
		break;
	default:
		dev_info(dai->dev, "unsupported format");
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		regmap_update_bits(apbif->regmap,
			SUN50I_AHUB_APBIF_TX_CTL(dai->id),
			(0xf<<APBIF_TX_CHAN_NUM),
			((params_channels(params)-1)<<APBIF_TX_CHAN_NUM));
	else
		regmap_update_bits(apbif->regmap,
			SUN50I_AHUB_APBIF_RX_CTL(dai->id),
			(0xf<<APBIF_RX_CHAN_NUM),
			((params_channels(params)-1)<<APBIF_RX_CHAN_NUM));

	return 0;
}

static int sun50i_apbif_start(struct snd_soc_dai *dai, int direction)
{
	struct sun50i_apbif *apbif = snd_soc_dai_get_drvdata(dai);

	switch (direction) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		regmap_update_bits(apbif->regmap,
			SUN50I_AHUB_APBIF_TXFIFO_CTL(dai->id),
			(1<<APBIF_TX_FTX), (1<<APBIF_TX_FTX));
		break;
	case SNDRV_PCM_STREAM_CAPTURE:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sun50i_apbif_stop(struct snd_soc_dai *dai, int direction)
{
	struct sun50i_apbif *apbif = snd_soc_dai_get_drvdata(dai);

	switch (direction) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		regmap_update_bits(apbif->regmap,
			SUN50I_AHUB_APBIF_TXFIFO_CTL(dai->id),
			(1<<APBIF_TX_FTX), (0<<APBIF_TX_FTX));
		break;
	case SNDRV_PCM_STREAM_CAPTURE:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sun50i_apbif_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	int err;

	err = snd_dmaengine_pcm_trigger(substream, cmd);
	if (err)
		return err;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		return sun50i_apbif_start(dai, substream->stream);
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		return sun50i_apbif_stop(dai, substream->stream);
	default:
		return -EINVAL;
	}
}

static const struct snd_soc_dai_ops sun50i_apbif_dai_ops = {
	.hw_params	= sun50i_apbif_hw_params,
	.trigger	= sun50i_apbif_trigger,
};

static int sun50i_apbif_dai_probe(struct snd_soc_dai *dai)
{
	struct sun50i_apbif *apbif = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai,
				  &apbif->playback_dma_data[dai->id],
				  &apbif->capture_dma_data[dai->id]);

	return 0;
}

#define DAI(dai_name)					\
	{							\
		.name = dai_name,				\
		.probe = sun50i_apbif_dai_probe,		\
		.playback = {					\
			.stream_name = dai_name " Playback",	\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_192000,	\
			.formats = SNDRV_PCM_FMTBIT_S8 |	\
				SNDRV_PCM_FMTBIT_S16_LE |	\
				SNDRV_PCM_FMTBIT_S32_LE,	\
		},						\
		.capture = {					\
			.stream_name = dai_name " Capture",	\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_192000,	\
			.formats = SNDRV_PCM_FMTBIT_S8 |	\
				SNDRV_PCM_FMTBIT_S16_LE |	\
				SNDRV_PCM_FMTBIT_S32_LE,	\
		},						\
		.ops = &sun50i_apbif_dai_ops,			\
	}

static struct snd_soc_dai_driver sun50i_apbif_cmpnt_dais[] = {
	DAI("APBIF0"),
	DAI("APBIF1"),
	DAI("APBIF2"),
};

static struct snd_kcontrol_new sun50i_apbif_controls[] = {
};

static const struct snd_soc_component_driver sun50i_apbif_cmpnt = {
	.controls		= sun50i_apbif_controls,
	.num_controls		= ARRAY_SIZE(sun50i_apbif_controls),
#if 0
	.pcm_construct		= tegra_pcm_construct,
	.open			= tegra_pcm_open,
	.close			= tegra_pcm_close,
	.hw_params		= tegra_pcm_hw_params,
	.pointer		= tegra_pcm_pointer,
#endif
};

static const struct sun50i_apbif_quirks sun50i_h6_apbif_quirks = {
	.num_ch		= 3,
	.cmpnt		= &sun50i_apbif_cmpnt,
	.dais		= sun50i_apbif_cmpnt_dais,
	.regmap_conf	= &sun50i_apbif_regmap_config,
	.tx_base	= 0, /* fix up */
	.rx_base	= 0, /* fix up */
};

static const struct sun50i_apbif_quirks sun50i_h616_apbif_quirks = {
	/* Diffs so far */
	/* Default for reg 0x114 different */
	.num_ch		= 3,
	.cmpnt		= &sun50i_apbif_cmpnt,
	.dais		= sun50i_apbif_cmpnt_dais,
	.regmap_conf	= &sun50i_apbif_regmap_config,
	.tx_base	= 0, /* fix up */
	.rx_base	= 0, /* fix up */
};

static const struct of_device_id sun50i_apbif_of_match[] = {
	{
		.compatible = "allwinner,sun50i-h6-apbif",
		.data = &sun50i_h6_apbif_quirks,
	},
	{
		.compatible = "allwinner,sun50i-h616-apbif",
		.data = &sun50i_h616_apbif_quirks,
	},
	{},
};
MODULE_DEVICE_TABLE(of, sun50i_apbif_of_match);

static int sun50i_apbif_probe(struct platform_device *pdev)
{
	struct sun50i_apbif *apbif;
	void __iomem *regs;
	struct resource *res;
	int err, i;

	apbif = devm_kzalloc(&pdev->dev, sizeof(*apbif), GFP_KERNEL);
	if (!apbif)
		return -ENOMEM;

	apbif->variant = of_device_get_match_data(&pdev->dev);

	dev_set_drvdata(&pdev->dev, apbif);

	apbif->capture_dma_data =
		devm_kcalloc(&pdev->dev,
			     apbif->variant->num_ch,
			     sizeof(struct snd_dmaengine_dai_dma_data),
			     GFP_KERNEL);
	if (apbif->capture_dma_data == NULL)
		return -ENOMEM;

	apbif->playback_dma_data =
		devm_kcalloc(&pdev->dev,
			     apbif->variant->num_ch,
			     sizeof(struct snd_dmaengine_dai_dma_data),
			     GFP_KERNEL);
	if (apbif->playback_dma_data == NULL)
		return -ENOMEM;

	regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	apbif->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					       apbif->variant->regmap_conf);
	if (IS_ERR(apbif->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		return PTR_ERR(apbif->regmap);
	}

	regcache_cache_only(apbif->regmap, true);

	for (i = 0; i < apbif->variant->num_ch; i++) {
		apbif->playback_dma_data[i].addr = res->start/* + FIFO OFFSET */;

		apbif->capture_dma_data[i].addr = res->start/* + FIFO OFFSET */;

		apbif->playback_dma_data[i].addr_width = 32;

		if (of_property_read_string_index(pdev->dev.of_node,
				"dma-names", (i * 2) + 1,
				&apbif->playback_dma_data[i].chan_name) < 0) {
			dev_err(&pdev->dev,
				"missing property allwinner,dma-names\n");

			return -ENODEV;
		}

		apbif->capture_dma_data[i].addr_width = 32;

		if (of_property_read_string_index(pdev->dev.of_node,
				"dma-names",
				(i * 2),
				&apbif->capture_dma_data[i].chan_name) < 0) {
			dev_err(&pdev->dev,
				"missing property allwinner,dma-names\n");

			return -ENODEV;
		}
	}

	err = devm_snd_soc_register_component(&pdev->dev,
					      apbif->variant->cmpnt,
					      apbif->variant->dais,
					      apbif->variant->num_ch);
	if (err) {
		dev_err(&pdev->dev,
			"can't register APBIF component, err: %d\n", err);
		return err;
	}

	pm_runtime_enable(&pdev->dev);

	return 0;
}

static int sun50i_apbif_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static const struct dev_pm_ops sun50i_apbif_pm_ops = {
	SET_RUNTIME_PM_OPS(sun50i_apbif_runtime_suspend,
			   sun50i_apbif_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static struct platform_driver sun50i_apbif_driver = {
	.probe = sun50i_apbif_probe,
	.remove = sun50i_apbif_remove,
	.driver = {
		.name = "sun50i-apbif",
		.of_match_table = sun50i_apbif_of_match,
		.pm = &sun50i_apbif_pm_ops,
	},
};
module_platform_driver(sun50i_apbif_driver);

MODULE_AUTHOR("Marcus Cooper <codekipper@gmail.com>");
MODULE_DESCRIPTION("Allwinner SUN50i ASoC APBIF driver");
MODULE_LICENSE("GPL v2");
