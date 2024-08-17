// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) Jernej Skrabec <jernej.skrabec@gmail.com>
 */

#include <uapi/linux/media-bus-format.h>

#include "sun50i_fmt.h"

static bool sun50i_fmt_is_10bit(u32 format)
{
	switch (format) {
	case MEDIA_BUS_FMT_RGB101010_1X30:
	case MEDIA_BUS_FMT_YUV10_1X30:
	case MEDIA_BUS_FMT_UYYVYY10_0_5X30:
	case MEDIA_BUS_FMT_UYVY10_1X20:
		return true;
	default:
		return false;
	}
}

static u32 sun50i_fmt_get_colorspace(u32 format)
{
	switch (format) {
	case MEDIA_BUS_FMT_UYYVYY8_0_5X24:
	case MEDIA_BUS_FMT_UYYVYY10_0_5X30:
		return SUN50I_FMT_CS_YUV420;
	case MEDIA_BUS_FMT_UYVY8_1X16:
	case MEDIA_BUS_FMT_UYVY10_1X20:
		return SUN50I_FMT_CS_YUV422;
	default:
		return SUN50I_FMT_CS_YUV444RGB;
	}
}

static void sun50i_fmt_de3_limits(u32 *limits, u32 colorspace, bool bit10)
{
	if (colorspace != SUN50I_FMT_CS_YUV444RGB) {
		limits[0] = SUN50I_FMT_LIMIT(64, 940);
		limits[1] = SUN50I_FMT_LIMIT(64, 960);
		limits[2] = SUN50I_FMT_LIMIT(64, 960);
	} else if (bit10) {
		limits[0] = SUN50I_FMT_LIMIT(0, 1023);
		limits[1] = SUN50I_FMT_LIMIT(0, 1023);
		limits[2] = SUN50I_FMT_LIMIT(0, 1023);
	} else {
		limits[0] = SUN50I_FMT_LIMIT(0, 1021);
		limits[1] = SUN50I_FMT_LIMIT(0, 1021);
		limits[2] = SUN50I_FMT_LIMIT(0, 1021);
	}
}

void sun50i_fmt_setup(struct sun8i_mixer *mixer, u16 width,
		      u16 height, u32 format)
{
	u32 colorspace, limit[3], base;
	struct regmap *regs;
	bool bit10;

	colorspace = sun50i_fmt_get_colorspace(format);
	bit10 = sun50i_fmt_is_10bit(format);
	base = SUN50I_FMT_DE3;
	regs = sun8i_blender_regmap(mixer);

	sun50i_fmt_de3_limits(limit, colorspace, bit10);

	regmap_write(regs, SUN50I_FMT_CTRL(base), 0);

	regmap_write(regs, SUN50I_FMT_SIZE(base),
		     SUN8I_MIXER_SIZE(width, height));
	regmap_write(regs, SUN50I_FMT_SWAP(base), 0);
	regmap_write(regs, SUN50I_FMT_DEPTH(base), bit10);
	regmap_write(regs, SUN50I_FMT_FORMAT(base), colorspace);
	regmap_write(regs, SUN50I_FMT_COEF(base), 0);

	regmap_write(regs, SUN50I_FMT_LMT_Y(base), limit[0]);
	regmap_write(regs, SUN50I_FMT_LMT_C0(base), limit[1]);
	regmap_write(regs, SUN50I_FMT_LMT_C1(base), limit[2]);

	regmap_write(regs, SUN50I_FMT_CTRL(base), 1);
}
