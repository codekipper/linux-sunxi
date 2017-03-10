/*
 * ALSA SoC HDMI Audio Layer
 *
 * Copyright 2017 Marcus Cooper <codekipper@gmail.com>
 *
 * Based on the Allwinner SDK driver, released under the GPL.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/regmap.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define	SUN4I_HDMI_CTL		(0x00)
	#define SUN4I_HDMI_CTL_GEN			BIT(31)
	#define SUN4I_HDMI_CTL_RESET			BIT(30)

#define SUN4I_HDMI_DMA_FIFO_CTL	(0x04)
	#define SUN4I_HDMI_DMA_FIFO_CTL_1_NORMAL_DMA	BIT(31)
	#define SUN4I_HDMI_DMA_FIFO_CTL_DMA_REQ		GENMASK(25, 24)
	#define SUN4I_HDMI_DMA_FIFO_CTL_HALF_EMPTY	(0 << 24)
	#define SUN4I_HDMI_DMA_FIFO_CTL_QUARTER_EMPTY	(1 << 24)
	#define SUN4I_HDMI_DMA_FIFO_CTL_EIGHTH_EMPTY	(2 << 24)
	#define SUN4I_HDMI_DMA_FIFO_CTL_1_SAMPLE	BIT(19)
	#define SUN4I_HDMI_DMA_FIFO_CTL_MSB		BIT(18)
	#define SUN4I_HDMI_DMA_FIFO_CTL_FMTRVD		GENMASK(17, 16)
	#define SUN4I_HDMI_DMA_FIFO_CTL_FMT16BIT	(0 << 16)
	#define SUN4I_HDMI_DMA_FIFO_CTL_FMT20BIT	(1 << 16)
	#define SUN4I_HDMI_DMA_FIFO_CTL_FMT24BIT	(2 << 16)
	#define SUN4I_HDMI_DMA_FIFO_CTL_FTX		BIT(15)
	#define SUN4I_HDMI_DMA_FIFO_CTL_ASS		BIT(0)

#define SUN4I_HDMI_AUDIO_FMT	(0x08)
	#define SUN4I_HDMI_AUDIO_FMT_FROM_ASG		BIT(31)
	#define SUN4I_HDMI_AUDIO_FMT_SEL		GENMASK(26, 24)
	#define SUN4I_HDMI_AUDIO_FMT_LPCM		(0 << 24)
	#define SUN4I_HDMI_AUDIO_FMT_COMPRESSED		(1 << 24)
	#define SUN4I_HDMI_AUDIO_FMT_HDR		(2 << 24)
	#define SUN4I_HDMI_AUDIO_FMT_1BIT		(3 << 24)
	#define SUN4I_HDMI_AUDIO_FMT_DSD_FMT		BIT(4)
	#define SUN4I_HDMI_AUDIO_FMT_AUD_LAYOUT		BIT(3)
	#define SUN4I_HDMI_AUDIO_FMT_SRC_CH_CFG		GENMASK(2, 0)
	#define SUN4I_HDMI_AUDIO_FMT_CH_NUMBER(v)	((v-1) << 0)

#define SUN4I_HDMI_AUD_PCM_CTL	(0x0C)

#define SUN4I_HDMI_AUD_CTS	(0x10)
	#define SUN4I_HDMI_AUD_CTS_GEN			GENMASK(19, 0)

#define SUN4I_HDMI_AUD_N	(0x14)
	#define SUN4I_HDMI_AUD_CTS_GEN			GENMASK(19, 0)

#define SUN4I_HDMI_AUD_CH_STAT0	(0x18)
	#define SUN4I_HDMI_AUD_CH_STAT0_CHNL_BIT1	BIT(30)
	#define SUN4I_HDMI_AUD_CH_STAT0_CLK_ACCUR	BIT(28)
	#define SUN4I_HDMI_AUD_CH_STAT0_CLK_ACCUR	BIT(28)
	#define SUN4I_HDMI_AUD_CH_STAT0_SAMFREQ(v)	((v) << 24)
	#define SUN4I_HDMI_AUD_CH_STAT0_SAMFREQ_MASK	GENMASK(27, 24)
	#define SUN4I_HDMI_AUD_CH_STAT0_CHNUM_MASK	GENMASK(23, 20)
	#define SUN4I_HDMI_AUD_CH_STAT0_SRCNUM_MASK	GENMASK(19, 16)
	#define SUN4I_HDMI_AUD_CH_STAT0_CATCODE_MASK	GENMASK(15, 8)
	#define SUN4I_HDMI_AUD_CH_STAT0_MODE_MASK	GENMASK(7, 6)
	#define SUN4I_HDMI_AUD_CH_STAT0_EMPHASIS_MASK	GENMASK(5, 3)
	#define SUN4I_HDMI_AUD_CH_STAT0_CP		BIT(2)
	#define SUN4I_HDMI_AUD_CH_STAT0_AUDIO		BIT(1)
	#define SUN4I_HDMI_AUD_CH_STAT0_PRO		BIT(0)

#define SUN4I_HDMI_AUD_CH_STAT1	(0x1C)
	#define SUN4I_HDMI_AUD_CH_STAT1_CGMSA(v)		((v) << 8)
	#define SUN4I_HDMI_AUD_CH_STAT1_ORISAMFREQ(v)	((v) << 4)
	#define SUN4I_HDMI_AUD_CH_STAT1_ORISAMFREQ_MASK	GENMASK(7, 4)
	#define SUN4I_HDMI_AUD_CH_STAT1_SAMWORDLEN(v)	((v) << 1)
	#define SUN4I_HDMI_AUD_CH_STAT1_SAMWORDLEN_MASK	GENMASK(3, 1)
	#define SUN4I_HDMI_AUD_CH_STAT1_MAXWORDLEN		BIT(0)

#define SUN8I_HDMI_TXFIFO	(0x360)

/* Defines for Sampling Frequency */
#define SUN4I_HDMI_SAMFREQ_44_1KHZ		0x0
#define SUN4I_HDMI_SAMFREQ_NOT_INDICATED	0x1
#define SUN4I_HDMI_SAMFREQ_48KHZ		0x2
#define SUN4I_HDMI_SAMFREQ_32KHZ		0x3
#define SUN4I_HDMI_SAMFREQ_22_05KHZ		0x4
#define SUN4I_HDMI_SAMFREQ_24KHZ		0x6
#define SUN4I_HDMI_SAMFREQ_88_2KHZ		0x8
#define SUN4I_HDMI_SAMFREQ_76_8KHZ		0x9
#define SUN4I_HDMI_SAMFREQ_96KHZ		0xa
#define SUN4I_HDMI_SAMFREQ_176_4KHZ		0xc
#define SUN4I_HDMI_SAMFREQ_192KHZ		0xe

struct sun4i_hdmi_dev {
	struct platform_device *pdev;
	struct clk *hdmi_clk;
	struct clk *apb_clk;
	struct snd_soc_dai_driver cpu_dai_drv;
	struct regmap *regmap;
	struct snd_dmaengine_dai_dma_data dma_params_tx;
};

static void sun4i_hdmi_configure(struct sun4i_hdmi_dev *host)
{
	/* soft reset HDMI */
	regmap_write(host->regmap, SUN4I_HDMI_CTL, SUN4I_HDMI_CTL_RESET);

	/* flush TX FIFO */
	regmap_update_bits(host->regmap, SUN4I_HDMI_DMA_FIFO_CTL,
			   SUN4I_HDMI_DMA_FIFO_CTL_FTX,
			   SUN4I_HDMI_DMA_FIFO_CTL_FTX);
}

static void sun4i_snd_txctrl_on(struct snd_pcm_substream *substream,
				struct sun4i_hdmi_dev *host)
{
	/* Global enable */
	regmap_update_bits(host->regmap, SUN4I_HDMI_CTL,
			   SUN4I_HDMI_CTL_GEN, SUN4I_HDMI_CTL_GEN);
}

static void sun4i_snd_txctrl_off(struct snd_pcm_substream *substream,
				 struct sun4i_hdmi_dev *host)
{
	/* Global disable */
	regmap_update_bits(host->regmap, SUN4I_HDMI_CTL,
			   SUN4I_HDMI_CTL_GEN, 0);
}

static int sun4i_hdmi_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *cpu_dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct sun4i_hdmi_dev *host = snd_soc_dai_get_drvdata(rtd->cpu_dai);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -EINVAL;

	sun4i_hdmi_configure(host);

	return 0;
}

static int sun4i_hdmi_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *cpu_dai)
{
	int ret = 0;
	int fmt;
	unsigned long rate = params_rate(params);
	u32 mclk_div = 0;
	unsigned int mclk = 0;
	u32 reg_val;
	struct sun4i_hdmi_dev *host = snd_soc_dai_get_drvdata(cpu_dai);
	struct platform_device *pdev = host->pdev;
	int sample_freq, original_sample_freq;

	/* Add the PCM and raw data select interface */
	switch (params_channels(params)) {
	case 1: /* PCM mode */
	case 2:
		fmt = SUN4I_HDMI_AUDIO_FMT_LPCM;
		break;
	default:
		return -EINVAL;
	}

	switch (params_width(params)) {
	case 16://SNDRV_PCM_FORMAT_S16_LE:
		fmt |= SUN4I_HDMI_DMA_FIFO_CTL_FMT16BIT;
		break;
	case 20://SNDRV_PCM_FORMAT_S20_3LE:
		fmt |= SUN4I_HDMI_DMA_FIFO_CTL_FMT20BIT;
		break;
	case 24://SNDRV_PCM_FORMAT_S24_LE:
		fmt |= SUN4I_HDMI_DMA_FIFO_CTL_FMT24BIT;
		break;
	default:
		return -EINVAL;
	}

	switch (rate) {
	case 22050:
	case 24000:
		mclk_div = 8;
		break;
	case 32000:
		mclk_div = 6;
		break;
	case 44100:
	case 48000:
		mclk_div = 4;
		break;
	case 88200:
	case 96000:
		mclk_div = 2;
		break;
	case 176400:
	case 192000:
		mclk_div = 1;
		break;
	default:
		return -EINVAL;
	}
#if 0
	reg_val = 0;
	reg_val |= SUN4I_HDMI_TXCFG_ASS;
	reg_val |= fmt; /* set non audio and bit depth */
//	reg_val |= SUN4I_HDMI_TXCFG_CHSTMODE;
	reg_val |= SUN4I_HDMI_TXCFG_TXRATIO(mclk_div - 1);
	regmap_write(host->regmap, SUN4I_HDMI_TXCFG, reg_val);

	/* Test to see if this fixes playback issue */
	if (mclk == 24576000) {
		switch (mclk_div) {
		/* 24KHZ */
		case 8:
			sample_freq = SUN4I_HDMI_SAMFREQ_24KHZ;
			original_sample_freq
				= ORIGINAL(SUN4I_HDMI_SAMFREQ_24KHZ);
			break;

		/* 32KHZ */
		case 6:
			sample_freq = SUN4I_HDMI_SAMFREQ_32KHZ;
			original_sample_freq
				= ORIGINAL(SUN4I_HDMI_SAMFREQ_32KHZ);
			break;

		/* 48KHZ */
		case 4:
			sample_freq = SUN4I_HDMI_SAMFREQ_48KHZ;
			original_sample_freq
				= ORIGINAL(SUN4I_HDMI_SAMFREQ_48KHZ);
			break;

		/* 96KHZ */
		case 2:
			sample_freq = SUN4I_HDMI_SAMFREQ_96KHZ;
			original_sample_freq
				= ORIGINAL(SUN4I_HDMI_SAMFREQ_96KHZ);
			break;

		/* 192KHZ */
		case 1:
			sample_freq = SUN4I_HDMI_SAMFREQ_192KHZ;
			original_sample_freq
				= ORIGINAL(SUN4I_HDMI_SAMFREQ_192KHZ);
			break;

		default:
			sample_freq = SUN4I_HDMI_SAMFREQ_NOT_INDICATED;
			original_sample_freq = 0;
			break;
		}
	} else {
		/* 22.5792MHz */
		switch (mclk_div) {
		/* 22.05KHZ */
		case 8:
			sample_freq = SUN4I_HDMI_SAMFREQ_22_05KHZ;
			original_sample_freq
				= ORIGINAL(SUN4I_HDMI_SAMFREQ_22_05KHZ);
			break;

		/* 44.1KHZ */
		case 4:
			sample_freq = SUN4I_HDMI_SAMFREQ_44_1KHZ;
			original_sample_freq
				= ORIGINAL(SUN4I_HDMI_SAMFREQ_44_1KHZ);
			break;

		/* 88.2KHZ */
		case 2:
			sample_freq = SUN4I_HDMI_SAMFREQ_88_2KHZ;
			original_sample_freq
				= ORIGINAL(SUN4I_HDMI_SAMFREQ_88_2KHZ);
			break;

		/* 176.4KHZ */
		case 1:
			sample_freq = SUN4I_HDMI_SAMFREQ_176_4KHZ;
			original_sample_freq
				= ORIGINAL(SUN4I_HDMI_SAMFREQ_176_4KHZ);
			break;

		default:
			sample_freq = SUN4I_HDMI_SAMFREQ_NOT_INDICATED;
			original_sample_freq = 0;
			break;
		}
	}

	regmap_update_bits(host->regmap, SUN4I_HDMI_TXCHSTA0,
			SUN4I_HDMI_TXCHSTA0_SAMFREQ_MASK,
			SUN4I_HDMI_TXCHSTA0_SAMFREQ(sample_freq));

	regmap_update_bits(host->regmap, SUN4I_HDMI_TXCHSTA1,
			SUN4I_HDMI_TXCHSTA1_ORISAMFREQ_MASK,
			SUN4I_HDMI_TXCHSTA1_ORISAMFREQ(original_sample_freq));

	/* Set the channel number in status INVESTIGATION */
	regmap_update_bits(host->regmap, SUN4I_HDMI_TXCHSTA0,
			   SUN4I_HDMI_TXCHSTA0_CHNUM_MASK,
			   SUN4I_HDMI_TXCHSTA0_CHNUM(params_channels(params)));
	regmap_update_bits(host->regmap, SUN4I_HDMI_TXCHSTA1,
			   SUN4I_HDMI_TXCHSTA1_SAMWORDLEN_MASK,
			   SUN4I_HDMI_TXCHSTA1_SAMWORDLEN(1));
#endif
	return 0;
}

static int sun4i_hdmi_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	int ret = 0;
	struct sun4i_hdmi_dev *host = snd_soc_dai_get_drvdata(dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			sun4i_snd_txctrl_on(substream, host);
		else
			ret = -EINVAL;
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			sun4i_snd_txctrl_off(substream, host);
		else
			ret = -EINVAL;
		break;

	default:
		ret = -EINVAL;
		break;
	}
	{
	/* COOPS DEBUGGING FOR NOW */
	struct platform_device *pdev = host->pdev;
	u32 reg_val = 0;

	regmap_read(host->regmap, SUN4I_HDMI_CTL, &reg_val);
	dev_err(&pdev->dev,
			"SUN4I_HDMI_CTL 0x%x\n", reg_val);
	regmap_read(host->regmap, SUN4I_HDMI_DMA_FIFO_CTL, &reg_val);
	dev_err(&pdev->dev,
			"SUN4I_HDMI_DMA_FIFO_CTL 0x%x\n", reg_val);
	regmap_read(host->regmap, SUN4I_HDMI_AUDIO_FMT, &reg_val);
	dev_err(&pdev->dev,
			"SUN4I_HDMI_AUDIO_FMT 0x%x\n", reg_val);
	regmap_read(host->regmap, SUN4I_HDMI_AUD_PCM_CTL, &reg_val);
	dev_err(&pdev->dev,
			"SUN4I_HDMI_AUD_PCM_CTL 0x%x\n", reg_val);
	regmap_read(host->regmap, SUN4I_HDMI_AUD_CTS, &reg_val);
	dev_err(&pdev->dev,
			"SUN4I_HDMI_AUD_CTS 0x%x\n", reg_val);
	regmap_read(host->regmap, SUN4I_HDMI_AUD_N, &reg_val);
	dev_err(&pdev->dev,
			"SUN4I_HDMI_AUD_N 0x%x\n", reg_val);
	regmap_read(host->regmap, SUN4I_HDMI_AUD_CH_STAT0, &reg_val);
	dev_err(&pdev->dev,
			"SUN4I_HDMI_AUD_CH_STAT0 0x%x\n", reg_val);
	regmap_read(host->regmap, SUN4I_HDMI_AUD_CH_STAT1, &reg_val);
	dev_err(&pdev->dev,
			"SUN4I_HDMI_AUD_CH_STAT1 0x%x\n", reg_val);
	}

	return ret;
}

static int sun4i_hdmi_soc_dai_probe(struct snd_soc_dai *dai)
{
	struct sun4i_hdmi_dev *host = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai, &host->dma_params_tx, NULL);
	return 0;
}

static const struct snd_soc_dai_ops sun4i_hdmi_dai_ops = {
	.startup	= sun4i_hdmi_startup,
	.trigger	= sun4i_hdmi_trigger,
	.hw_params	= sun4i_hdmi_hw_params,
};

static const struct regmap_config sun4i_hdmi_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = SUN4I_HDMI_AUD_CH_STAT1,
};

#define SUN4I_RATES	SNDRV_PCM_RATE_8000_192000

#define SUN4I_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
			SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_driver sun4i_hdmi_dai = {
	.playback = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SUN4I_RATES,
		.formats = SUN4I_FORMATS,
	},
	.probe = sun4i_hdmi_soc_dai_probe,
	.ops = &sun4i_hdmi_dai_ops,
	.name = "HDMI",
};

static const struct of_device_id sun4i_hdmi_of_match[] = {
	{
		.compatible = "allwinner,sun4i-a10-hdmi",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sun4i_hdmi_of_match);

static const struct snd_soc_component_driver sun4i_hdmi_component = {
	.name		= "sun4i-hdmi",
};

static int sun4i_hdmi_runtime_suspend(struct device *dev)
{
	struct sun4i_hdmi_dev *host  = dev_get_drvdata(dev);

	return 0;
}

static int sun4i_hdmi_runtime_resume(struct device *dev)
{
	struct sun4i_hdmi_dev *host  = dev_get_drvdata(dev);

	return 0;
}

static int sun4i_hdmi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct sun4i_hdmi_dev *host;
	struct resource *res;
	int ret;
	void __iomem *base;

	dev_dbg(&pdev->dev, "Entered %s\n", __func__);

	host = devm_kzalloc(&pdev->dev, sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;

	host->pdev = pdev;

	/* Initialize this copy of the CPU DAI driver structure */
	memcpy(&host->cpu_dai_drv, &sun4i_hdmi_dai, sizeof(sun4i_hdmi_dai));
	host->cpu_dai_drv.name = dev_name(&pdev->dev);

	/* Get the addresses */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	host->regmap = devm_regmap_init_mmio(&pdev->dev, base,
						&sun4i_hdmi_regmap_config);

	/* Clocks */
	host->apb_clk = devm_clk_get(&pdev->dev, "apb");
	if (IS_ERR(host->apb_clk)) {
		dev_err(&pdev->dev, "failed to get a apb clock.\n");
		return PTR_ERR(host->apb_clk);
	}

	host->hdmi_clk = devm_clk_get(&pdev->dev, "hdmi");
	if (IS_ERR(host->hdmi_clk)) {
		dev_err(&pdev->dev, "failed to get a hdmi clock.\n");
		return PTR_ERR(host->hdmi_clk);
	}

	host->dma_params_tx.addr = res->start + SUN8I_HDMI_TXFIFO;
	host->dma_params_tx.maxburst = 8;
	host->dma_params_tx.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;

	platform_set_drvdata(pdev, host);

	ret = devm_snd_soc_register_component(&pdev->dev,
				&sun4i_hdmi_component, &sun4i_hdmi_dai, 1);
	if (ret)
		return ret;

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = sun4i_hdmi_runtime_resume(&pdev->dev);
		if (ret)
			goto err_unregister;
	}

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
	if (ret)
		goto err_suspend;
	return 0;
err_suspend:
	if (!pm_runtime_status_suspended(&pdev->dev))
		sun4i_hdmi_runtime_suspend(&pdev->dev);
err_unregister:
	pm_runtime_disable(&pdev->dev);
	return ret;
}

static int sun4i_hdmi_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		sun4i_hdmi_runtime_suspend(&pdev->dev);

	return 0;
}

static const struct dev_pm_ops sun4i_hdmi_pm = {
	SET_RUNTIME_PM_OPS(sun4i_hdmi_runtime_suspend,
			   sun4i_hdmi_runtime_resume, NULL)
};

static struct platform_driver sun4i_hdmi_driver = {
	.driver		= {
		.name	= "sun4i-",
		.of_match_table = of_match_ptr(sun4i_hdmi_of_match),
		.pm	= &sun4i_hdmi_pm,
	},
	.probe		= sun4i_hdmi_probe,
	.remove		= sun4i_hdmi_remove,
};

module_platform_driver(sun4i_hdmi_driver);

MODULE_AUTHOR("Marcus Cooper <codekipper@gmail.com>");
MODULE_DESCRIPTION("Allwinner sun4i HDMI SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sun4i-hdmi");
