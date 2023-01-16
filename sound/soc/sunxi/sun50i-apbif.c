// SPDX-License-Identifier: GPL-2.0-only
//
// sun50i-apbif.c - Allwinner APBIF driver
//
// Copyright (c) 2020 NVIDIA CORPORATION.  All rights reserved.

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
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
	unsigned int num_ch;
};

struct sun50i_apbif {
	struct regmap				*regmap;
	struct snd_dmaengine_dai_dma_data 	*playback_dma_data;
	struct snd_dmaengine_dai_dma_data 	*capture_dma_data;
	const struct sun50i_apbif_quirks	*variant;
};

#define APBIF_CHANNEL_REG_STRIDE	0x30
#define APBIF_LAST_REG		0x190
#define APBIF_CHANNEL_COUNT	3
#define APBIF_RX_BASE		0x100
#define APBIF_TX_BASE		0x10
#define APBIF_CTL		(0x00)
#define APBIF_IRQ_CTL		(0x04)
#define APBIF_IRQ_STA		(0x08)
#define APBIF_FIFO_CTL		(0x10)
#define APBIF_FIFO_STA		(0x14)

#define CH_REG(offset, reg, id)						       \
	((offset) + (reg) + (APBIF_CHANNEL_REG_STRIDE * (id)))

#define CH_TX_REG(reg, id) CH_REG(APBIF_TX_BASE, reg, id)

#define CH_RX_REG(reg, id) CH_REG(APBIF_RX_BASE, reg, id)

#define REG_DEFAULTS(id, tx_base, rx_base)		     \
	{ CH_REG(tx_base, APBIF_CTL, id), 0x00000100 },	     \
	{ CH_REG(tx_base, APBIF_IRQ_STA, id), 0x00000001 },  \
	{ CH_REG(tx_base, APBIF_FIFO_CTL, id), 0x00000200 }, \
	{ CH_REG(tx_base, APBIF_FIFO_STA, id), 0x00000140 }, \
	{ CH_REG(rx_base, APBIF_CTL, id), 0x00000100 },	     \
	{ CH_REG(rx_base, APBIF_IRQ_STA, id), 0x00000001 },  \
	{ CH_REG(rx_base, APBIF_FIFO_CTL, id), 0x00000400 }, \
	{ CH_REG(rx_base, APBIF_FIFO_STA, id), 0x00000100 }

#define APBIF_REG_DEFAULTS(id)  \
	REG_DEFAULTS((id) - 1, 	\
		APBIF_TX_BASE,  \
		APBIF_RX_BASE)

static const struct reg_default sun50i_apbif_reg_defaults[] = {
	APBIF_REG_DEFAULTS(1),
	APBIF_REG_DEFAULTS(2),
	APBIF_REG_DEFAULTS(3),
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

static int sun50i_apbif_dai_probe(struct snd_soc_dai *dai)
{
	struct sun50i_apbif *apbif = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai,
				  &apbif->playback_dma_data[dai->id],
				  &apbif->capture_dma_data[dai->id]);

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
			regmap_update_bits(apbif->regmap,
					   CH_TX_REG(APBIF_CTL, dai->id),
					   (7 << APBIF_TX_WS), (3 << APBIF_TX_WS));
			regmap_update_bits(apbif->regmap,
					   CH_TX_REG(APBIF_FIFO_CTL, dai->id),
					   (1 << APBIF_TX_TXIM), (1 << APBIF_TX_TXIM));
		} else {
			regmap_update_bits(apbif->regmap,
					   CH_RX_REG(APBIF_CTL, dai->id),
					   (7 << APBIF_RX_WS), (3 << APBIF_RX_WS));
			regmap_update_bits(apbif->regmap,
					   CH_RX_REG(APBIF_FIFO_CTL, dai->id),
					   (3 << APBIF_RX_RXOM), (1 << APBIF_RX_RXOM));
		}
		break;
	case	SNDRV_PCM_FORMAT_S24_LE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			regmap_update_bits(apbif->regmap,
					   CH_TX_REG(APBIF_CTL, dai->id),
					   (7 << APBIF_TX_WS), (5 << APBIF_TX_WS));
			regmap_update_bits(apbif->regmap,
					   CH_TX_REG(APBIF_FIFO_CTL, dai->id),
					   (1 << APBIF_TX_TXIM), (1 << APBIF_TX_TXIM));
		} else {
			regmap_update_bits(apbif->regmap,
					   CH_RX_REG(APBIF_CTL, dai->id),
					   (7 << APBIF_RX_WS), (5 << APBIF_RX_WS));
			regmap_update_bits(apbif->regmap,
					   CH_RX_REG(APBIF_FIFO_CTL, dai->id),
					   (3 << APBIF_RX_RXOM), (1 << APBIF_RX_RXOM));
		}
		break;
	case	SNDRV_PCM_FORMAT_S32_LE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			regmap_update_bits(apbif->regmap,
					   CH_TX_REG(APBIF_CTL, dai->id),
					   (7 << APBIF_TX_WS), (7 << APBIF_TX_WS));
			regmap_update_bits(apbif->regmap,
					   CH_TX_REG(APBIF_FIFO_CTL, dai->id),
					   (1 << APBIF_TX_TXIM), (1 << APBIF_TX_TXIM));
		} else {
			regmap_update_bits(apbif->regmap,
					   CH_RX_REG(APBIF_CTL, dai->id),
					   (7 << APBIF_RX_WS), (7 << APBIF_RX_WS));
			regmap_update_bits(apbif->regmap,
					   CH_RX_REG(APBIF_FIFO_CTL, dai->id),
					   (3 << APBIF_RX_RXOM), (1 << APBIF_RX_RXOM));
		}
		break;
	default:
		dev_info(dai->dev, "unsupported format");
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		regmap_update_bits(apbif->regmap,
				   CH_TX_REG(APBIF_CTL, dai->id),
				   (0xf<<APBIF_TX_CHAN_NUM),
				   ((params_channels(params) - 1) << APBIF_TX_CHAN_NUM));
	else
		regmap_update_bits(apbif->regmap,
				   CH_TX_REG(APBIF_CTL, dai->id),
				   (0xf << APBIF_RX_CHAN_NUM),
				   ((params_channels(params) - 1) << APBIF_RX_CHAN_NUM));

	return 0;
}

static int sun50i_apbif_start(struct snd_soc_dai *dai, int direction)
{
	struct sun50i_apbif *apbif = snd_soc_dai_get_drvdata(dai);

	switch (direction) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		regmap_update_bits(apbif->regmap,
				   CH_TX_REG(APBIF_FIFO_CTL, dai->id),
				   (1 << APBIF_TX_FTX), (1 << APBIF_TX_FTX));
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
				   CH_TX_REG(APBIF_FIFO_CTL, dai->id),
				   (1 << APBIF_TX_FTX), (0 << APBIF_TX_FTX));
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
	.probe		= sun50i_apbif_dai_probe,
	.hw_params	= sun50i_apbif_hw_params,
	.trigger	= sun50i_apbif_trigger,
};

#define DAI(dai_name)					\
	{							\
		.name = dai_name,				\
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

static const struct snd_pcm_hardware sun50i_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.period_bytes_min	= 1024,
	.period_bytes_max	= PAGE_SIZE,
	.periods_min		= 2,
	.periods_max		= 8,
	.buffer_bytes_max	= PAGE_SIZE * 8,
	.fifo_size		= 4,
};

static int sun50i_apbif_pcm_construct(struct snd_soc_component *component,
				      struct snd_soc_pcm_runtime *rtd)
{
	struct device *dev = component->dev;
	struct snd_pcm *pcm = rtd->pcm;
	int ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret < 0)
		return ret;

	return snd_pcm_set_fixed_buffer_all(pcm,
					    SNDRV_DMA_TYPE_DEV_WC,
					    dev,
					    sun50i_pcm_hardware.buffer_bytes_max);
}

static int sun50i_apbif_pcm_open(struct snd_soc_component *component,
				 struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_dmaengine_dai_dma_data *dmap;
	struct dma_chan *chan;
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	int ret;

	if (rtd->dai_link->no_pcm)
		return 0;

	dmap = snd_soc_dai_get_dma_data(cpu_dai, substream);

	/* Set HW params now that initialization is complete */
	snd_soc_set_runtime_hwparams(substream, &sun50i_pcm_hardware);

	/* Ensure period size is multiple of 8 */
	ret = snd_pcm_hw_constraint_step(substream->runtime, 0,
					 SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 0x8);
	if (ret) {
		dev_err(rtd->dev, "failed to set constraint %d\n", ret);
		return ret;
	}

	chan = dma_request_slave_channel(cpu_dai->dev, dmap->chan_name);
	if (!chan) {
		dev_err(cpu_dai->dev,
			"dmaengine request slave channel failed! (%s)\n",
			dmap->chan_name);
		return -ENODEV;
	}

	ret = snd_dmaengine_pcm_open(substream, chan);
	if (ret) {
		dev_err(rtd->dev,
			"dmaengine pcm open failed with err %d (%s)\n", ret,
			dmap->chan_name);

		dma_release_channel(chan);

		return ret;
	}

	/* Set wait time to 500ms by default */
	substream->wait_time = 500;

	return 0;
}

static int sun50i_apbif_pcm_close(struct snd_soc_component *component,
				  struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	if (rtd->dai_link->no_pcm)
		return 0;

	snd_dmaengine_pcm_close_release_chan(substream);

	return 0;
}

static int sun50i_apbif_pcm_hw_params(struct snd_soc_component *component,
				      struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_dmaengine_dai_dma_data *dmap;
	struct dma_slave_config slave_config;
	struct dma_chan *chan;
	int ret;

	if (rtd->dai_link->no_pcm)
		return 0;

	dmap = snd_soc_dai_get_dma_data(snd_soc_rtd_to_cpu(rtd, 0), substream);
	if (!dmap)
		return 0;

	chan = snd_dmaengine_pcm_get_chan(substream);

	ret = snd_hwparams_to_dma_slave_config(substream, params,
					       &slave_config);
	if (ret) {
		dev_err(rtd->dev, "hw params config failed with err %d\n", ret);
		return ret;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		slave_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		slave_config.dst_addr = dmap->addr;
		slave_config.dst_maxburst = 8;
	} else {
		slave_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		slave_config.src_addr = dmap->addr;
		slave_config.src_maxburst = 8;
	}

	ret = dmaengine_slave_config(chan, &slave_config);
	if (ret < 0) {
		dev_err(rtd->dev, "dma slave config failed with err %d\n", ret);
		return ret;
	}

	return 0;
}

static snd_pcm_uframes_t sun50i_apbif_pcm_pointer(struct snd_soc_component *component,
						  struct snd_pcm_substream *substream)
{
	return snd_dmaengine_pcm_pointer(substream);
}

static const struct snd_soc_component_driver sun50i_apbif_cmpnt = {
	.controls		= sun50i_apbif_controls,
	.num_controls		= ARRAY_SIZE(sun50i_apbif_controls),
	.pcm_construct		= sun50i_apbif_pcm_construct,
	.open			= sun50i_apbif_pcm_open,
	.close			= sun50i_apbif_pcm_close,
	.hw_params		= sun50i_apbif_pcm_hw_params,
	.pointer		= sun50i_apbif_pcm_pointer,
};

static const struct sun50i_apbif_quirks sun50i_h6_apbif_quirks = {
	.num_ch		= 3,
	.cmpnt		= &sun50i_apbif_cmpnt,
	.dais		= sun50i_apbif_cmpnt_dais,
	.regmap_conf	= &sun50i_apbif_regmap_config,
};

static const struct sun50i_apbif_quirks sun50i_h616_apbif_quirks = {
	/* Diffs so far */
	/* Default for reg 0x114 different */
	.num_ch		= 3,
	.cmpnt		= &sun50i_apbif_cmpnt,
	.dais		= sun50i_apbif_cmpnt_dais,
	.regmap_conf	= &sun50i_apbif_regmap_config,
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

static void sun50i_apbif_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
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
