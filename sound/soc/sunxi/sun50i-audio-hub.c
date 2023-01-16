// SPDX-License-Identifier: GPL-2.0
//
//	SUN50i Audio HUB (AHUB) ALSA SoC machine driver
//
//

#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

#define SUN50I_AUDIO_HUB_MAX_REG 0x580 /* I don't give a DAM! */

#define SUN50I_AUDIO_HUB_CTL_REG	0x00
#define SUN50I_AUDIO_HUB_CTL_HDMI_SRC_SEL	BIT(2)

#define SUN50I_AUDIO_HUB_VER_REG	0x04

#define SUN50I_AUDIO_HUB_RST_REG	0x08
#define SUN50I_AUDIO_HUB_RST_APBIF_TXDIF0	BIT(31)
#define SUN50I_AUDIO_HUB_RST_APBIF_TXDIF1	BIT(30)
#define SUN50I_AUDIO_HUB_RST_APBIF_TXDIF2	BIT(29)
#define SUN50I_AUDIO_HUB_RST_APBIF_RXDIF0	BIT(27)
#define SUN50I_AUDIO_HUB_RST_APBIF_RXDIF1	BIT(26)
#define SUN50I_AUDIO_HUB_RST_APBIF_RXDIF2	BIT(25)
#define SUN50I_AUDIO_HUB_RST_I2S0		BIT(23)
#define SUN50I_AUDIO_HUB_RST_I2S1		BIT(22)
#define SUN50I_AUDIO_HUB_RST_I2S2		BIT(21)
#define SUN50I_AUDIO_HUB_RST_I2S3		BIT(20)
#define SUN50I_AUDIO_HUB_RST_DAM0		BIT(15)
#define SUN50I_AUDIO_HUB_RST_DAM1		BIT(14)

#define SUN50I_AUDIO_HUB_GAT_REG	0x0c
#define SUN50I_AUDIO_HUB_GAT_APBIF_TXDIF0	BIT(31)
#define SUN50I_AUDIO_HUB_GAT_APBIF_TXDIF1	BIT(30)
#define SUN50I_AUDIO_HUB_GAT_APBIF_TXDIF2	BIT(29)
#define SUN50I_AUDIO_HUB_GAT_APBIF_RXDIF0	BIT(27)
#define SUN50I_AUDIO_HUB_GAT_APBIF_RXDIF1	BIT(26)
#define SUN50I_AUDIO_HUB_GAT_APBIF_RXDIF2	BIT(25)
#define SUN50I_AUDIO_HUB_GAT_I2S0		BIT(23)
#define SUN50I_AUDIO_HUB_GAT_I2S1		BIT(22)
#define SUN50I_AUDIO_HUB_GAT_I2S2		BIT(21)
#define SUN50I_AUDIO_HUB_GAT_I2S3		BIT(20)
#define SUN50I_AUDIO_HUB_GAT_DAM0		BIT(15)
#define SUN50I_AUDIO_HUB_GAT_DAM1		BIT(14)

/**
 * struct sun50i_audio_hub_quirks - Differences between SoC variants.
 * @has_audio_pll_clks: SoC needs to set up audio pll clks.
 */
struct sun50i_audio_hub_quirks {
	bool					has_audio_pll_clks;
	const struct regmap_config		*sun50i_audio_hub_regmap;
	const struct snd_soc_component_driver	*components;
	struct snd_soc_dai_driver		*dais;
	int					*num_dais;
};

struct sun50i_audio_hub {
	struct clk	*bus_clk;
	struct clk	*mod_clk;
	struct clk      *audio_hub_clk;
	struct regmap	*regmap;
	struct reset_control *rst;

	const struct sun50i_audio_hub_quirks	*variant;
};

/* FE */
SND_SOC_DAILINK_DEFS(playback1,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(playback2,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL2")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(playback3,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL3")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture1,
		     DAILINK_COMP_ARRAY(COMP_CPU("UL1")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture2,
		     DAILINK_COMP_ARRAY(COMP_CPU("UL2")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture3,
		     DAILINK_COMP_ARRAY(COMP_CPU("UL3")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* BE */
#if 0
SND_SOC_DAILINK_DEFS(primary_codec,
		     DAILINK_COMP_ARRAY(COMP_CPU("DAM0")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(secondary_codec,
		     DAILINK_COMP_ARRAY(COMP_CPU("DAM1")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));
#endif
SND_SOC_DAILINK_DEFS(i2s0,
		     DAILINK_COMP_ARRAY(COMP_CPU("I2S0")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(i2s1,
		     DAILINK_COMP_ARRAY(COMP_CPU("I2S1")),
		     DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "i2s-hifi")),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(i2s2,
		     DAILINK_COMP_ARRAY(COMP_CPU("I2S2")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(i2s3,
		     DAILINK_COMP_ARRAY(COMP_CPU("I2S3")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

#define DAI(sname)							\
	{								\
		.name = "AHUB-" #sname,					\
		.playback = {						\
			.stream_name = #sname " AHUB-Playback",		\
			.channels_min = 1,				\
			.channels_max = 16,				\
			.rates = SNDRV_PCM_RATE_8000_192000,		\
			.formats = SNDRV_PCM_FMTBIT_S8 |		\
				SNDRV_PCM_FMTBIT_S16_LE |		\
				SNDRV_PCM_FMTBIT_S24_LE |		\
				SNDRV_PCM_FMTBIT_S32_LE,		\
		},							\
		.capture = {						\
			.stream_name = #sname " AHUB-Capture",		\
			.channels_min = 1,				\
			.channels_max = 16,				\
			.rates = SNDRV_PCM_RATE_8000_192000,		\
			.formats = SNDRV_PCM_FMTBIT_S8 |		\
				SNDRV_PCM_FMTBIT_S16_LE |		\
				SNDRV_PCM_FMTBIT_S24_LE |		\
				SNDRV_PCM_FMTBIT_S32_LE,		\
		},							\
	}

static struct snd_soc_dai_driver sun50i_audio_hub_dais[] = {
	DAI(APDIF1),
	DAI(APDIF2),
	DAI(APDIF3),
	/* AHUB <-> I2S <-> Codec */
	DAI(I2S0),
	DAI(I2S1),
	DAI(I2S2),
	DAI(I2S3),
};

static const char * const sun50i_ahub_mux_texts[] = {
	"None",
	"APDIF1",
	"APDIF2",
	"APDIF3",
	"I2S0",
	"I2S1",
	"I2S2",
	"I2S3",
};

#define MUX_VALUE(npart, nbit) (1 + (nbit) + (npart) * 32)
static const unsigned int sun50i_ahub_mux_values[] = {
	0,
	/* APDIF */
	MUX_VALUE(0, 0),
	MUX_VALUE(0, 1),
	MUX_VALUE(0, 2),
	/* I2S */
	MUX_VALUE(0, 16),
	MUX_VALUE(0, 17),
	MUX_VALUE(0, 18),
	MUX_VALUE(0, 19),
};

static int sun50i_ahub_get_value_enum(struct snd_kcontrol *kctl,
				     struct snd_ctl_elem_value *uctl)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_kcontrol_component(kctl);
	struct sun50i_audio_hub *ahub = snd_soc_component_get_drvdata(cmpnt);
	struct soc_enum *e = (struct soc_enum *)kctl->private_value;

	return 0;
}

static int sun50i_ahub_put_value_enum(struct snd_kcontrol *kctl,
				     struct snd_ctl_elem_value *uctl)
{
	struct snd_soc_component *cmpnt = snd_soc_dapm_kcontrol_component(kctl);
	struct sun50i_audio_hub *ahub = snd_soc_component_get_drvdata(cmpnt);
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kctl);
	struct soc_enum *e = (struct soc_enum *)kctl->private_value;

	return 0;
}

#define SOC_VALUE_ENUM_WIDE(xreg, shift, xmax, xtexts, xvalues)		\
	{								\
		.reg = xreg,						\
		.shift_l = shift,					\
		.shift_r = shift,					\
		.items = xmax,						\
		.texts = xtexts,					\
		.values = xvalues,					\
	}

#define SOC_VALUE_ENUM_WIDE_DECL(name, xreg, shift, xtexts, xvalues)	\
	static struct soc_enum name =					\
		SOC_VALUE_ENUM_WIDE(xreg, shift, ARRAY_SIZE(xtexts),	\
		xtexts, xvalues)

#define MUX_ENUM_CTRL_DECL(ename)					\
	SOC_VALUE_ENUM_WIDE_DECL(ename##_enum, SUN50I_AUDIO_HUB_GAT_REG, 0,		\
				 sun50i_ahub_mux_texts,		\
				 sun50i_ahub_mux_values);		\
	static const struct snd_kcontrol_new ename##_control =		\
		SOC_DAPM_ENUM_EXT("Route", ename##_enum,		\
				  sun50i_ahub_get_value_enum,		\
				  sun50i_ahub_put_value_enum)

MUX_ENUM_CTRL_DECL(sun50i_apdif1_tx);
MUX_ENUM_CTRL_DECL(sun50i_apdif2_tx);
MUX_ENUM_CTRL_DECL(sun50i_apdif3_tx);
MUX_ENUM_CTRL_DECL(sun50i_i2s0_tx);
MUX_ENUM_CTRL_DECL(sun50i_i2s1_tx);
MUX_ENUM_CTRL_DECL(sun50i_i2s2_tx);
MUX_ENUM_CTRL_DECL(sun50i_i2s3_tx);

#define WIDGETS(sname, ename)						     \
	SND_SOC_DAPM_AIF_IN(sname " AHUB-RX", NULL, 0, SND_SOC_NOPM, 0, 0),  \
	SND_SOC_DAPM_AIF_OUT(sname " AHUB-TX", NULL, 0, SND_SOC_NOPM, 0, 0), \
	SND_SOC_DAPM_MUX(sname " Mux", SND_SOC_NOPM, 0, 0,		     \
			 &ename##_control)

static const struct snd_soc_dapm_widget sun50i_audio_hub_widgets[] = {
	WIDGETS("APDIF1", sun50i_apdif1_tx),
	WIDGETS("APDIF2", sun50i_apdif2_tx),
	WIDGETS("APDIF3", sun50i_apdif3_tx),
	WIDGETS("I2S0", sun50i_i2s0_tx),
	WIDGETS("I2S1", sun50i_i2s1_tx),
	WIDGETS("I2S2", sun50i_i2s2_tx),
	WIDGETS("I2S3", sun50i_i2s3_tx),
};

/* Connect FEs with AHUB */
#define SUN50I_FE_ROUTES(name) \
	{ name " AHUB-Playback",	NULL,	name " Playback" },	\
	{ name " AHUB-RX",		NULL,	name " AHUB-Playback"}, \
	{ name " AHUB-Capture",		NULL,	name " AHUB-TX" },      \
	{ name " Capture",		NULL,	name " AHUB-Capture" },

#define SUN50I_MUX_ROUTES(name)					\
	{ name " AHUB-TX",	 NULL,		name " Mux" },		\
	{ name " Mux",		"APDIF1",	"APDIF1 AHUB-RX" },	\
	{ name " Mux",		"APDIF2",	"APDIF2 AHUB-RX" },	\
	{ name " Mux",		"APDIF3",	"APDIF3 AHUB-RX" },	\
	{ name " Mux",		"I2S0",		"I2S0 AHUB-RX" },	\
	{ name " Mux",		"I2S1",		"I2S1 AHUB-RX" },	\
	{ name " Mux",		"I2S2",		"I2S2 AHUB-RX" },	\
	{ name " Mux",		"I2S3",		"I2S3 AHUB-RX" },

static const struct snd_soc_dapm_route sun50i_audio_hub_routes[] = {
	SUN50I_FE_ROUTES("APDIF1")
	SUN50I_FE_ROUTES("APDIF2")
	SUN50I_FE_ROUTES("APDIF3")
	SUN50I_MUX_ROUTES("APDIF1")
	SUN50I_MUX_ROUTES("APDIF2")
	SUN50I_MUX_ROUTES("APDIF3")
	SUN50I_MUX_ROUTES("I2S0")
	SUN50I_MUX_ROUTES("I2S1")
	SUN50I_MUX_ROUTES("I2S2")
	SUN50I_MUX_ROUTES("I2S3")
};

static const struct snd_soc_component_driver sun50i_audio_hub_component = {
	.dapm_widgets = sun50i_audio_hub_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sun50i_audio_hub_widgets),
	.dapm_routes = sun50i_audio_hub_routes,
	.num_dapm_routes = ARRAY_SIZE(sun50i_audio_hub_routes),
};

static const struct regmap_config sun50i_audio_hub_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= SUN50I_AUDIO_HUB_MAX_REG,
	.cache_type	= REGCACHE_FLAT,
};

struct sun50i_audio_hub_quirks sun50i_h6_audio_hub = {
	.has_audio_pll_clks = false,
	.sun50i_audio_hub_regmap = &sun50i_audio_hub_regmap_config,
	.components = &sun50i_audio_hub_component,
	.dais = sun50i_audio_hub_dais,
	.num_dais = ARRAY_SIZE(sun50i_audio_hub_dais),
};

struct sun50i_audio_hub_quirks sun50i_h616_audio_hub = {
	.has_audio_pll_clks = true,
	.sun50i_audio_hub_regmap = &sun50i_audio_hub_regmap_config,
	.components = &sun50i_audio_hub_component,
	.dais = sun50i_audio_hub_dais,
	.num_dais = ARRAY_SIZE(sun50i_audio_hub_dais),
};

static int sun50i_audio_hub_dev_probe(struct platform_device *pdev)
{
	struct sun50i_audio_hub *audio_hub;
	void __iomem *regs;
	int ret;

	audio_hub = devm_kzalloc(&pdev->dev, sizeof(*audio_hub), GFP_KERNEL);
	if (!audio_hub)
		return -ENOMEM;

	audio_hub->variant = of_device_get_match_data(&pdev->dev);
	if (!audio_hub->variant) {
		dev_err(&pdev->dev, "Failed to determine the quirks to use\n");
		return -ENODEV;
	}

	audio_hub->bus_clk = devm_clk_get(&pdev->dev, "apb");
	if (IS_ERR(audio_hub->bus_clk)) {
		dev_err(&pdev->dev, "Can't get our bus clock\n");
		return PTR_ERR(audio_hub->bus_clk);
	}

	audio_hub->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					    audio_hub->variant->sun50i_audio_hub_regmap);
	if (IS_ERR(audio_hub->regmap)) {
		dev_err(&pdev->dev, "Regmap initialisation failed\n");
		return PTR_ERR(audio_hub->regmap);
	}

	if (audio_hub->variant->has_audio_pll_clks) {
		audio_hub->mod_clk = devm_clk_get(&pdev->dev, "mod");
		if (IS_ERR(audio_hub->mod_clk)) {
			dev_err(&pdev->dev, "Can't get our mod clocks\n");
			return PTR_ERR(audio_hub->mod_clk);
		}
	}

	audio_hub->audio_hub_clk = devm_clk_get(&pdev->dev, "audio_hub");
	if (IS_ERR(audio_hub->audio_hub_clk)) {
		dev_err(&pdev->dev, "Can't get our audio_hub clock\n");
		return PTR_ERR(audio_hub->audio_hub_clk);
	}
#if 0
        ret = devm_snd_soc_register_component(&pdev->dev,
					      audio_hub->variant->components,
					      audio_hub->variant->dais,
					      audio_hub->variant->num_dais);
	if (ret) {
		dev_err(&pdev->dev, "can't register AUDIO HUB component, err: %d\n", ret);
		return ret;
	}
#endif
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sun50i_audio_hub_dt_match[] = {
	{
		.compatible = "allwinner,sun50i-h6-audio-hub",
		.data = &sun50i_h6_audio_hub,
	},
	{
		.compatible = "allwinner,sun50i-h616-audio-hub",
		.data = &sun50i_h616_audio_hub,
	},
	{}
};
#endif

static const struct dev_pm_ops sun50i_audio_hub_pm_ops = {
	.poweroff = snd_soc_poweroff,
	.restore = snd_soc_resume,
};

static struct platform_driver sun50i_audio_hub_driver = {
	.driver = {
		.name = "sun50i-audio-hub",
#ifdef CONFIG_OF
		.of_match_table = sun50i_audio_hub_dt_match,
#endif
		.pm = &sun50i_audio_hub_pm_ops,
	},
	.probe = sun50i_audio_hub_dev_probe,
};

module_platform_driver(sun50i_audio_hub_driver);

/* Module information */
MODULE_DESCRIPTION("SUN50i Audio HUB ALSA SoC machine driver");
MODULE_AUTHOR("Marcus Cooper <codekipper@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("sun50i_audio_hub soc card");
