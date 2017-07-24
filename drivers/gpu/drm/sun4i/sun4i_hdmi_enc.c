/*
 * Copyright (C) 2016 Maxime Ripard
 *
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/iopoll.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <sound/hdmi-codec.h>

#include "sun4i_backend.h"
#include "sun4i_crtc.h"
#include "sun4i_drv.h"
#include "sun4i_hdmi.h"

static inline struct sun4i_hdmi *
drm_encoder_to_sun4i_hdmi(struct drm_encoder *encoder)
{
	return container_of(encoder, struct sun4i_hdmi,
			    encoder);
}

static inline struct sun4i_hdmi *
drm_connector_to_sun4i_hdmi(struct drm_connector *connector)
{
	return container_of(connector, struct sun4i_hdmi,
			    connector);
}

static int sun4i_hdmi_setup_avi_infoframes(struct sun4i_hdmi *hdmi,
					   struct drm_display_mode *mode)
{
	struct hdmi_avi_infoframe frame;
	u8 buffer[17];
	int i, ret;

	ret = drm_hdmi_avi_infoframe_from_display_mode(&frame, mode, false);
	if (ret < 0) {
		DRM_ERROR("Failed to get infoframes from mode\n");
		return ret;
	}

	ret = hdmi_avi_infoframe_pack(&frame, buffer, sizeof(buffer));
	if (ret < 0) {
		DRM_ERROR("Failed to pack infoframes\n");
		return ret;
	}

	for (i = 0; i < sizeof(buffer); i++)
		writeb(buffer[i], hdmi->base + SUN4I_HDMI_AVI_INFOFRAME_REG(i));

	return 0;
}

static int sun4i_hdmi_atomic_check(struct drm_encoder *encoder,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *conn_state)
{
	struct drm_display_mode *mode = &crtc_state->mode;

	if (mode->flags & DRM_MODE_FLAG_DBLCLK)
		return -EINVAL;

	return 0;
}

static void sun4i_hdmi_disable(struct drm_encoder *encoder)
{
	struct sun4i_hdmi *hdmi = drm_encoder_to_sun4i_hdmi(encoder);
	u32 val;

	DRM_DEBUG_DRIVER("Disabling the HDMI Output\n");

	val = readl(hdmi->base + SUN4I_HDMI_VID_CTRL_REG);
	val &= ~SUN4I_HDMI_VID_CTRL_ENABLE;
	writel(val, hdmi->base + SUN4I_HDMI_VID_CTRL_REG);
}

static void sun4i_hdmi_enable(struct drm_encoder *encoder)
{
	struct drm_display_mode *mode = &encoder->crtc->state->adjusted_mode;
	struct sun4i_hdmi *hdmi = drm_encoder_to_sun4i_hdmi(encoder);
	u32 val = 0;

	DRM_DEBUG_DRIVER("Enabling the HDMI Output\n");

	sun4i_hdmi_setup_avi_infoframes(hdmi, mode);
	val |= SUN4I_HDMI_PKT_CTRL_TYPE(0, SUN4I_HDMI_PKT_AVI);
	val |= SUN4I_HDMI_PKT_CTRL_TYPE(1, SUN4I_HDMI_PKT_END);
	writel(val, hdmi->base + SUN4I_HDMI_PKT_CTRL_REG(0));

	val = SUN4I_HDMI_VID_CTRL_ENABLE;
	if (hdmi->hdmi_monitor)
		val |= SUN4I_HDMI_VID_CTRL_HDMI_MODE;

	writel(val, hdmi->base + SUN4I_HDMI_VID_CTRL_REG);
}

static void sun4i_hdmi_mode_set(struct drm_encoder *encoder,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct sun4i_hdmi *hdmi = drm_encoder_to_sun4i_hdmi(encoder);
	unsigned int x, y;
	u32 val;

	clk_set_rate(hdmi->mod_clk, mode->crtc_clock * 1000);
	clk_set_rate(hdmi->tmds_clk, mode->crtc_clock * 1000);

	/* Set input sync enable */
	writel(SUN4I_HDMI_UNKNOWN_INPUT_SYNC,
	       hdmi->base + SUN4I_HDMI_UNKNOWN_REG);

	/*
	 * Setup output pad (?) controls
	 *
	 * This is done here instead of at probe/bind time because
	 * the controller seems to toggle some of the bits on its own.
	 *
	 * We can't just initialize the register there, we need to
	 * protect the clock bits that have already been read out and
	 * cached by the clock framework.
	 */
	val = readl(hdmi->base + SUN4I_HDMI_PAD_CTRL1_REG);
	val &= SUN4I_HDMI_PAD_CTRL1_HALVE_CLK;
	val |= hdmi->variant->pad_ctrl1_init_val;
	writel(val, hdmi->base + SUN4I_HDMI_PAD_CTRL1_REG);
	val = readl(hdmi->base + SUN4I_HDMI_PAD_CTRL1_REG);

	/* Setup timing registers */
	writel(SUN4I_HDMI_VID_TIMING_X(mode->hdisplay) |
	       SUN4I_HDMI_VID_TIMING_Y(mode->vdisplay),
	       hdmi->base + SUN4I_HDMI_VID_TIMING_ACT_REG);

	x = mode->htotal - mode->hsync_start;
	y = mode->vtotal - mode->vsync_start;
	writel(SUN4I_HDMI_VID_TIMING_X(x) | SUN4I_HDMI_VID_TIMING_Y(y),
	       hdmi->base + SUN4I_HDMI_VID_TIMING_BP_REG);

	x = mode->hsync_start - mode->hdisplay;
	y = mode->vsync_start - mode->vdisplay;
	writel(SUN4I_HDMI_VID_TIMING_X(x) | SUN4I_HDMI_VID_TIMING_Y(y),
	       hdmi->base + SUN4I_HDMI_VID_TIMING_FP_REG);

	x = mode->hsync_end - mode->hsync_start;
	y = mode->vsync_end - mode->vsync_start;
	writel(SUN4I_HDMI_VID_TIMING_X(x) | SUN4I_HDMI_VID_TIMING_Y(y),
	       hdmi->base + SUN4I_HDMI_VID_TIMING_SPW_REG);

	val = SUN4I_HDMI_VID_TIMING_POL_TX_CLK;
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		val |= SUN4I_HDMI_VID_TIMING_POL_HSYNC;

	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		val |= SUN4I_HDMI_VID_TIMING_POL_VSYNC;

	writel(val, hdmi->base + SUN4I_HDMI_VID_TIMING_POL_REG);
}

static enum drm_mode_status sun4i_hdmi_mode_valid(struct drm_encoder *encoder,
					const struct drm_display_mode *mode)
{
	struct sun4i_hdmi *hdmi = drm_encoder_to_sun4i_hdmi(encoder);
	unsigned long rate = mode->clock * 1000;
	unsigned long diff = rate / 200; /* +-0.5% allowed by HDMI spec */
	long rounded_rate;

	/* 165 MHz is the typical max pixelclock frequency for HDMI <= 1.2 */
	if (rate > 165000000)
		return MODE_CLOCK_HIGH;
	rounded_rate = clk_round_rate(hdmi->tmds_clk, rate);
	if (rounded_rate > 0 &&
	    max_t(unsigned long, rounded_rate, rate) -
	    min_t(unsigned long, rounded_rate, rate) < diff)
		return MODE_OK;
	return MODE_NOCLOCK;
}

static const struct drm_encoder_helper_funcs sun4i_hdmi_helper_funcs = {
	.atomic_check	= sun4i_hdmi_atomic_check,
	.disable	= sun4i_hdmi_disable,
	.enable		= sun4i_hdmi_enable,
	.mode_set	= sun4i_hdmi_mode_set,
	.mode_valid	= sun4i_hdmi_mode_valid,
};

static const struct drm_encoder_funcs sun4i_hdmi_funcs = {
	.destroy	= drm_encoder_cleanup,
};

static int sun4i_hdmi_get_modes(struct drm_connector *connector)
{
	struct sun4i_hdmi *hdmi = drm_connector_to_sun4i_hdmi(connector);
	struct edid *edid;
	int ret;

	edid = drm_get_edid(connector, hdmi->i2c);
	if (!edid)
		return 0;

	hdmi->hdmi_monitor = drm_detect_hdmi_monitor(edid);
	DRM_DEBUG_DRIVER("Monitor is %s monitor\n",
			 hdmi->hdmi_monitor ? "an HDMI" : "a DVI");

	drm_mode_connector_update_edid_property(connector, edid);
	cec_s_phys_addr_from_edid(hdmi->cec_adap, edid);
	ret = drm_add_edid_modes(connector, edid);
	kfree(edid);

	return ret;
}

static const struct drm_connector_helper_funcs sun4i_hdmi_connector_helper_funcs = {
	.get_modes	= sun4i_hdmi_get_modes,
};

static enum drm_connector_status
sun4i_hdmi_connector_detect(struct drm_connector *connector, bool force)
{
	struct sun4i_hdmi *hdmi = drm_connector_to_sun4i_hdmi(connector);
	unsigned long reg;

	if (readl_poll_timeout(hdmi->base + SUN4I_HDMI_HPD_REG, reg,
			       reg & SUN4I_HDMI_HPD_HIGH,
			       0, 500000)) {
		cec_phys_addr_invalidate(hdmi->cec_adap);
		return connector_status_disconnected;
	}

	return connector_status_connected;
}

static const struct drm_connector_funcs sun4i_hdmi_connector_funcs = {
	.detect			= sun4i_hdmi_connector_detect,
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.destroy		= drm_connector_cleanup,
	.reset			= drm_atomic_helper_connector_reset,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
};

#ifdef CONFIG_DRM_SUN4I_HDMI_CEC
static bool sun4i_hdmi_cec_pin_read(struct cec_adapter *adap)
{
	struct sun4i_hdmi *hdmi = cec_get_drvdata(adap);

	return readl(hdmi->base + SUN4I_HDMI_CEC) & SUN4I_HDMI_CEC_RX;
}

static void sun4i_hdmi_cec_pin_low(struct cec_adapter *adap)
{
	struct sun4i_hdmi *hdmi = cec_get_drvdata(adap);

	/* Start driving the CEC pin low */
	writel(SUN4I_HDMI_CEC_ENABLE, hdmi->base + SUN4I_HDMI_CEC);
}

static void sun4i_hdmi_cec_pin_high(struct cec_adapter *adap)
{
	struct sun4i_hdmi *hdmi = cec_get_drvdata(adap);

	/*
	 * Stop driving the CEC pin, the pull up will take over
	 * unless another CEC device is driving the pin low.
	 */
	writel(0, hdmi->base + SUN4I_HDMI_CEC);
}

static const struct cec_pin_ops sun4i_hdmi_cec_pin_ops = {
	.read = sun4i_hdmi_cec_pin_read,
	.low = sun4i_hdmi_cec_pin_low,
	.high = sun4i_hdmi_cec_pin_high,
};
#endif

#define SUN4I_HDMI_PAD_CTRL1_MASK	(GENMASK(24, 7) | GENMASK(5, 0))
#define SUN4I_HDMI_PLL_CTRL_MASK	(GENMASK(31, 8) | GENMASK(3, 0))

/* Only difference from sun5i is AMP is 4 instead of 6 */
static const struct sun4i_hdmi_variant sun4i_variant = {
	.pad_ctrl0_init_val	= SUN4I_HDMI_PAD_CTRL0_TXEN |
				  SUN4I_HDMI_PAD_CTRL0_CKEN |
				  SUN4I_HDMI_PAD_CTRL0_PWENG |
				  SUN4I_HDMI_PAD_CTRL0_PWEND |
				  SUN4I_HDMI_PAD_CTRL0_PWENC |
				  SUN4I_HDMI_PAD_CTRL0_LDODEN |
				  SUN4I_HDMI_PAD_CTRL0_LDOCEN |
				  SUN4I_HDMI_PAD_CTRL0_BIASEN,
	.pad_ctrl1_init_val	= SUN4I_HDMI_PAD_CTRL1_REG_AMP(4) |
				  SUN4I_HDMI_PAD_CTRL1_REG_EMP(2) |
				  SUN4I_HDMI_PAD_CTRL1_REG_DENCK |
				  SUN4I_HDMI_PAD_CTRL1_REG_DEN |
				  SUN4I_HDMI_PAD_CTRL1_EMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_EMP_OPT |
				  SUN4I_HDMI_PAD_CTRL1_AMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_AMP_OPT,
	.pll_ctrl_init_val	= SUN4I_HDMI_PLL_CTRL_VCO_S(8) |
				  SUN4I_HDMI_PLL_CTRL_CS(7) |
				  SUN4I_HDMI_PLL_CTRL_CP_S(15) |
				  SUN4I_HDMI_PLL_CTRL_S(7) |
				  SUN4I_HDMI_PLL_CTRL_VCO_GAIN(4) |
				  SUN4I_HDMI_PLL_CTRL_SDIV2 |
				  SUN4I_HDMI_PLL_CTRL_LDO2_EN |
				  SUN4I_HDMI_PLL_CTRL_LDO1_EN |
				  SUN4I_HDMI_PLL_CTRL_HV_IS_33 |
				  SUN4I_HDMI_PLL_CTRL_BWS |
				  SUN4I_HDMI_PLL_CTRL_PLL_EN,

	.ddc_clk_reg		= REG_FIELD(SUN4I_HDMI_DDC_CLK_REG, 0, 6),
	.ddc_clk_pre_divider	= 2,
	.ddc_clk_m_offset	= 1,

	.field_ddc_en		= REG_FIELD(SUN4I_HDMI_DDC_CTRL_REG, 31, 31),
	.field_ddc_start	= REG_FIELD(SUN4I_HDMI_DDC_CTRL_REG, 30, 30),
	.field_ddc_reset	= REG_FIELD(SUN4I_HDMI_DDC_CTRL_REG, 0, 0),
	.field_ddc_addr_reg	= REG_FIELD(SUN4I_HDMI_DDC_ADDR_REG, 0, 31),
	.field_ddc_slave_addr	= REG_FIELD(SUN4I_HDMI_DDC_ADDR_REG, 0, 6),
	.field_ddc_int_status	= REG_FIELD(SUN4I_HDMI_DDC_INT_STATUS_REG, 0, 8),
	.field_ddc_fifo_clear	= REG_FIELD(SUN4I_HDMI_DDC_FIFO_CTRL_REG, 31, 31),
	.field_ddc_fifo_rx_thres = REG_FIELD(SUN4I_HDMI_DDC_FIFO_CTRL_REG, 4, 7),
	.field_ddc_fifo_tx_thres = REG_FIELD(SUN4I_HDMI_DDC_FIFO_CTRL_REG, 0, 3),
	.field_ddc_byte_count	= REG_FIELD(SUN4I_HDMI_DDC_BYTE_COUNT_REG, 0, 9),
	.field_ddc_cmd		= REG_FIELD(SUN4I_HDMI_DDC_CMD_REG, 0, 2),
	.field_ddc_sda_en	= REG_FIELD(SUN4I_HDMI_DDC_LINE_CTRL_REG, 9, 9),
	.field_ddc_sck_en	= REG_FIELD(SUN4I_HDMI_DDC_LINE_CTRL_REG, 8, 8),

	.ddc_fifo_reg		= SUN4I_HDMI_DDC_FIFO_DATA_REG,
	.ddc_fifo_has_dir	= true,
};

static const struct sun4i_hdmi_variant sun5i_variant = {
	.pad_ctrl0_init_val	= SUN4I_HDMI_PAD_CTRL0_TXEN |
				  SUN4I_HDMI_PAD_CTRL0_CKEN |
				  SUN4I_HDMI_PAD_CTRL0_PWENG |
				  SUN4I_HDMI_PAD_CTRL0_PWEND |
				  SUN4I_HDMI_PAD_CTRL0_PWENC |
				  SUN4I_HDMI_PAD_CTRL0_LDODEN |
				  SUN4I_HDMI_PAD_CTRL0_LDOCEN |
				  SUN4I_HDMI_PAD_CTRL0_BIASEN,
	.pad_ctrl1_init_val	= SUN4I_HDMI_PAD_CTRL1_REG_AMP(6) |
				  SUN4I_HDMI_PAD_CTRL1_REG_EMP(2) |
				  SUN4I_HDMI_PAD_CTRL1_REG_DENCK |
				  SUN4I_HDMI_PAD_CTRL1_REG_DEN |
				  SUN4I_HDMI_PAD_CTRL1_EMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_EMP_OPT |
				  SUN4I_HDMI_PAD_CTRL1_AMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_AMP_OPT,
	.pll_ctrl_init_val	= SUN4I_HDMI_PLL_CTRL_VCO_S(8) |
				  SUN4I_HDMI_PLL_CTRL_CS(7) |
				  SUN4I_HDMI_PLL_CTRL_CP_S(15) |
				  SUN4I_HDMI_PLL_CTRL_S(7) |
				  SUN4I_HDMI_PLL_CTRL_VCO_GAIN(4) |
				  SUN4I_HDMI_PLL_CTRL_SDIV2 |
				  SUN4I_HDMI_PLL_CTRL_LDO2_EN |
				  SUN4I_HDMI_PLL_CTRL_LDO1_EN |
				  SUN4I_HDMI_PLL_CTRL_HV_IS_33 |
				  SUN4I_HDMI_PLL_CTRL_BWS |
				  SUN4I_HDMI_PLL_CTRL_PLL_EN,

	.ddc_clk_reg		= REG_FIELD(SUN4I_HDMI_DDC_CLK_REG, 0, 6),
	.ddc_clk_pre_divider	= 2,
	.ddc_clk_m_offset	= 1,

	.field_ddc_en		= REG_FIELD(SUN4I_HDMI_DDC_CTRL_REG, 31, 31),
	.field_ddc_start	= REG_FIELD(SUN4I_HDMI_DDC_CTRL_REG, 30, 30),
	.field_ddc_reset	= REG_FIELD(SUN4I_HDMI_DDC_CTRL_REG, 0, 0),
	.field_ddc_addr_reg	= REG_FIELD(SUN4I_HDMI_DDC_ADDR_REG, 0, 31),
	.field_ddc_slave_addr	= REG_FIELD(SUN4I_HDMI_DDC_ADDR_REG, 0, 6),
	.field_ddc_int_status	= REG_FIELD(SUN4I_HDMI_DDC_INT_STATUS_REG, 0, 8),
	.field_ddc_fifo_clear	= REG_FIELD(SUN4I_HDMI_DDC_FIFO_CTRL_REG, 31, 31),
	.field_ddc_fifo_rx_thres = REG_FIELD(SUN4I_HDMI_DDC_FIFO_CTRL_REG, 4, 7),
	.field_ddc_fifo_tx_thres = REG_FIELD(SUN4I_HDMI_DDC_FIFO_CTRL_REG, 0, 3),
	.field_ddc_byte_count	= REG_FIELD(SUN4I_HDMI_DDC_BYTE_COUNT_REG, 0, 9),
	.field_ddc_cmd		= REG_FIELD(SUN4I_HDMI_DDC_CMD_REG, 0, 2),
	.field_ddc_sda_en	= REG_FIELD(SUN4I_HDMI_DDC_LINE_CTRL_REG, 9, 9),
	.field_ddc_sck_en	= REG_FIELD(SUN4I_HDMI_DDC_LINE_CTRL_REG, 8, 8),

	.ddc_fifo_reg		= SUN4I_HDMI_DDC_FIFO_DATA_REG,
	.ddc_fifo_has_dir	= true,
};

static const struct sun4i_hdmi_variant sun6i_variant = {
	.has_ddc_parent_clk	= true,
	.has_reset_control	= true,
	.pad_ctrl0_init_val	= 0xff |
				  SUN4I_HDMI_PAD_CTRL0_TXEN |
				  SUN4I_HDMI_PAD_CTRL0_CKEN |
				  SUN4I_HDMI_PAD_CTRL0_PWENG |
				  SUN4I_HDMI_PAD_CTRL0_PWEND |
				  SUN4I_HDMI_PAD_CTRL0_PWENC |
				  SUN4I_HDMI_PAD_CTRL0_LDODEN |
				  SUN4I_HDMI_PAD_CTRL0_LDOCEN,
	.pad_ctrl1_init_val	= SUN4I_HDMI_PAD_CTRL1_REG_AMP(6) |
				  SUN4I_HDMI_PAD_CTRL1_REG_EMP(4) |
				  SUN4I_HDMI_PAD_CTRL1_REG_DENCK |
				  SUN4I_HDMI_PAD_CTRL1_REG_DEN |
				  SUN4I_HDMI_PAD_CTRL1_EMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_EMP_OPT |
				  SUN4I_HDMI_PAD_CTRL1_PWSDT |
				  SUN4I_HDMI_PAD_CTRL1_PWSCK |
				  SUN4I_HDMI_PAD_CTRL1_AMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_AMP_OPT |
				  SUN4I_HDMI_PAD_CTRL1_UNKNOWN,
	.pll_ctrl_init_val	= SUN4I_HDMI_PLL_CTRL_VCO_S(8) |
				  SUN4I_HDMI_PLL_CTRL_CS(3) |
				  SUN4I_HDMI_PLL_CTRL_CP_S(10) |
				  SUN4I_HDMI_PLL_CTRL_S(4) |
				  SUN4I_HDMI_PLL_CTRL_VCO_GAIN(4) |
				  SUN4I_HDMI_PLL_CTRL_SDIV2 |
				  SUN4I_HDMI_PLL_CTRL_LDO2_EN |
				  SUN4I_HDMI_PLL_CTRL_LDO1_EN |
				  SUN4I_HDMI_PLL_CTRL_HV_IS_33 |
				  SUN4I_HDMI_PLL_CTRL_PLL_EN,

	.ddc_clk_reg		= REG_FIELD(SUN6I_HDMI_DDC_CLK_REG, 0, 6),
	.ddc_clk_pre_divider	= 1,
	.ddc_clk_m_offset	= 2,

	.tmds_clk_div_offset	= 1,

	.field_ddc_en		= REG_FIELD(SUN6I_HDMI_DDC_CTRL_REG, 0, 0),
	.field_ddc_start	= REG_FIELD(SUN6I_HDMI_DDC_CTRL_REG, 27, 27),
	.field_ddc_reset	= REG_FIELD(SUN6I_HDMI_DDC_CTRL_REG, 31, 31),
	.field_ddc_addr_reg	= REG_FIELD(SUN6I_HDMI_DDC_ADDR_REG, 1, 31),
	.field_ddc_slave_addr	= REG_FIELD(SUN6I_HDMI_DDC_ADDR_REG, 1, 7),
	.field_ddc_int_status	= REG_FIELD(SUN6I_HDMI_DDC_INT_STATUS_REG, 0, 8),
	.field_ddc_fifo_clear	= REG_FIELD(SUN6I_HDMI_DDC_FIFO_CTRL_REG, 18, 18),
	.field_ddc_fifo_rx_thres = REG_FIELD(SUN6I_HDMI_DDC_FIFO_CTRL_REG, 4, 7),
	.field_ddc_fifo_tx_thres = REG_FIELD(SUN6I_HDMI_DDC_FIFO_CTRL_REG, 0, 3),
	.field_ddc_byte_count	= REG_FIELD(SUN6I_HDMI_DDC_CMD_REG, 16, 25),
	.field_ddc_cmd		= REG_FIELD(SUN6I_HDMI_DDC_CMD_REG, 0, 2),
	.field_ddc_sda_en	= REG_FIELD(SUN6I_HDMI_DDC_CTRL_REG, 6, 6),
	.field_ddc_sck_en	= REG_FIELD(SUN6I_HDMI_DDC_CTRL_REG, 4, 4),

	.ddc_fifo_reg		= SUN6I_HDMI_DDC_FIFO_DATA_REG,
	.ddc_fifo_thres_incl	= true,
};

static const struct regmap_config sun4i_hdmi_regmap_config = {
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= 0x580,
};

/**
 * Prepare and configure the AUDIO infoframe
 *
 * AUDIO infoframe are transmitted once per frame and
 * contains information about HDMI transmission mode such as audio codec,
 * sample size, ...
 *
 * @hdmi: pointer on the hdmi internal structure
 *
 * Return negative value if error occurs
 */
static int hdmi_audio_infoframe_config(struct sun4i_hdmi *hdmi)
{
	struct hdmi_audio_params *audio = &hdmi->audio;
	u8 buffer[HDMI_INFOFRAME_SIZE(AUDIO)];

	DRM_DEBUG_DRIVER("enter %s, AIF %s\n", __func__,
			 audio->enabled ? "enable" : "disable");
	if (audio->enabled) {
		/* set audio parameters stored*/
		int ret = hdmi_audio_infoframe_pack(&audio->cea, buffer,
						sizeof(buffer));
		if (ret < 0) {
			DRM_ERROR("failed to pack audio infoframe: %d\n", ret);
			return ret;
		}
#if 0
		hdmi_infoframe_write_infopack(hdmi, buffer, ret);
	} else {
		/*disable audio info frame transmission */
		int val = hdmi_read(hdmi, HDMI_SW_DI_CFG);
		val &= ~HDMI_IFRAME_CFG_DI_N(HDMI_IFRAME_MASK,
					     HDMI_IFRAME_SLOT_AUDIO);
		hdmi_write(hdmi, val, HDMI_SW_DI_CFG);
#endif
	}
	return 0;
}

static void hdmi_audio_shutdown(struct device *dev, void *data)
{
	struct sun4i_hdmi *hdmi = dev_get_drvdata(dev);

	DRM_DEBUG_DRIVER("\n");

	/* disable audio */
	writel(0, hdmi->base + SUN4I_HDMI_AUD_CTL);
	hdmi->audio.enabled = false;
	hdmi_audio_infoframe_config(hdmi);
}

static int hdmi_audio_configure(struct sun4i_hdmi *hdmi)
{
	int audio_cfg = 0;
	struct hdmi_audio_params *params = &hdmi->audio;
	struct hdmi_audio_infoframe *info = &params->cea;
	u32 n;

	DRM_DEBUG_DRIVER("\n");

	if (!hdmi->audio.enabled)
		return 0;

	DRM_DEBUG_DRIVER("Audio rate = %d Hz\n",params->sample_rate);

	/*
	 * ACR_N 32000 44100 48000 88200 96000 176400 192000
	 *       4096  6272  6144  12544 12288  25088  24576
	 */

	switch (params->sample_rate) {
	case 32000:
		n = 4096;
		break;
	case 44100:
		n = 6272;
		audio_cfg = SUN4I_HDMI_AUD_CH_STAT0_SAMFREQ(0);
		break;
	case 48000:
		n = 6144;
		audio_cfg = SUN4I_HDMI_AUD_CH_STAT0_SAMFREQ(2);
		break;
	case 88200:
		n = 12544;
		audio_cfg = SUN4I_HDMI_AUD_CH_STAT0_SAMFREQ(8);
		break;
	case 96000:
		n = 12288;
		audio_cfg = SUN4I_HDMI_AUD_CH_STAT0_SAMFREQ(10);
		break;
	case 176400:
		n = 25088;
		audio_cfg = SUN4I_HDMI_AUD_CH_STAT0_SAMFREQ(12);
		break;
	case 192000:
		n = 24576;
		audio_cfg = SUN4I_HDMI_AUD_CH_STAT0_SAMFREQ(14);
		break;
	default:
		DRM_ERROR("un-support sample rate %d\n", params->sample_rate);
		return -EINVAL;
	}

	writel(audio_cfg, hdmi->base + SUN4I_HDMI_AUD_CH_STAT0);
	writel(0x0000000b, hdmi->base + SUN4I_HDMI_AUD_CH_STAT1);
#if 0
	/* cts needs to be calculated from legacy driver*/
	audio_info.CTS = ((video_timing[vic_tab].PCLK / 100) *
			  (audio_info.ACR_N / 128)) / (sample_rate / 100);
#endif
	audio_cfg = 165000;
	DRM_DEBUG_DRIVER("audio CTS calc:%d\n", audio_cfg);
	writel(audio_cfg, hdmi->base + SUN4I_HDMI_AUD_CTS);
	writel(n, hdmi->base + SUN4I_HDMI_AUD_N);

	/* update HDMI registers according to configuration */
	switch (info->channels) {
	case 2:
		audio_cfg = SUN4I_HDMI_AUDIO_FMT_CH_NUMBER(info->channels);
		break;
	default:
		DRM_ERROR("ERROR: Unsupported number of channels (%d)!\n",
			  info->channels);
		return -EINVAL;
	}

	writel(audio_cfg, hdmi->base + SUN4I_HDMI_AUDIO_FMT);
	return hdmi_audio_infoframe_config(hdmi);
}

static int hdmi_audio_hw_params(struct device *dev,
				void *data,
				struct hdmi_codec_daifmt *daifmt,
				struct hdmi_codec_params *params)
{
	struct sun4i_hdmi *hdmi = dev_get_drvdata(dev);
	int ret;

	DRM_DEBUG_DRIVER("\n");

	hdmi->audio.sample_width = params->sample_width;
	hdmi->audio.sample_rate = params->sample_rate;
	hdmi->audio.cea = params->cea;

	hdmi->audio.enabled = true;

	ret = hdmi_audio_configure(hdmi);
	if (ret < 0)
		return ret;

	return 0;
}

static int hdmi_audio_digital_mute(struct device *dev, void *data, bool enable)
{
	struct sun4i_hdmi *hdmi = dev_get_drvdata(dev);

	DRM_DEBUG_DRIVER("%s\n", enable ? "enable" : "disable");

	if (enable)
		writel(SUN4I_HDMI_ADMA_CTL_ASS, hdmi->base + SUN4I_HDMI_ADMA_CTL);
	else
		writel(0, hdmi->base + SUN4I_HDMI_ADMA_CTL);

	return 0;
}

static int hdmi_audio_get_eld(struct device *dev, void *data, uint8_t *buf, size_t len)
{
	struct sun4i_hdmi *hdmi = dev_get_drvdata(dev);
	struct drm_connector *connector = &hdmi->connector;

	DRM_DEBUG_DRIVER("\n");
	memcpy(buf, connector->eld, min(sizeof(connector->eld), len));

	return 0;
}

static const struct hdmi_codec_ops audio_codec_ops = {
	.hw_params = hdmi_audio_hw_params,
	.audio_shutdown = hdmi_audio_shutdown,
	.digital_mute = hdmi_audio_digital_mute,
	.get_eld = hdmi_audio_get_eld,
};

static inline struct sun4i_hdmi *dai_to_hdmi(struct snd_soc_dai *dai)
{
	struct snd_soc_card *card = snd_soc_dai_get_drvdata(dai);

	return snd_soc_card_get_drvdata(card);
}

static int sun4i_hdmi_audio_startup(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	struct sun4i_hdmi *hdmi = dai_to_hdmi(dai);
	int ret;

	if (hdmi->audio.substream && hdmi->audio.substream != substream)
		return -EINVAL;

	hdmi->audio.substream = substream;

	ret = snd_pcm_hw_constraint_eld(substream->runtime,
					hdmi->connector.eld);
	if (ret)
		return ret;

	return 0;
}

static int sun4i_hdmi_audio_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	return 0;
}

static void sun4i_hdmi_audio_reset(struct sun4i_hdmi *hdmi)
{
	struct device *dev = hdmi->dev;
	int ret;

	ret = 0; //sun4i_hdmi_stop_packet(encoder, HDMI_INFOFRAME_TYPE_AUDIO);
	if (ret)
		dev_err(dev, "Failed to stop audio infoframe: %d\n", ret);
}

static void sun4i_hdmi_audio_shutdown(struct snd_pcm_substream *substream,
				    struct snd_soc_dai *dai)
{
	struct sun4i_hdmi *hdmi = dai_to_hdmi(dai);

	if (substream != hdmi->audio.substream)
		return;

	sun4i_hdmi_audio_reset(hdmi);

	hdmi->audio.substream = NULL;
}

/* HDMI audio codec callbacks */
static int sun4i_hdmi_audio_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params,
				    struct snd_soc_dai *dai)
{
	struct sun4i_hdmi *hdmi = dai_to_hdmi(dai);
	struct device *dev = hdmi->dev;
	u32 audio_packet_config, channel_mask;
	u32 channel_map, i;

	if (substream != hdmi->audio.substream)
		return -EINVAL;

	dev_dbg(dev, "%s: %u Hz, %d bit, %d channels\n", __func__,
		params_rate(params), params_width(params),
		params_channels(params));

	hdmi->audio.channels = params_channels(params);
	hdmi->audio.sample_rate = params_rate(params);

	channel_map = 0;
	for (i = 0; i < 8; i++) {
		if (channel_mask & BIT(i))
			channel_map |= i << (3 * i);
	}

	return 0;
}

static int sun4i_hdmi_audio_trigger(struct snd_pcm_substream *substream, int cmd,
				  struct snd_soc_dai *dai)
{
	struct sun4i_hdmi *hdmi = dai_to_hdmi(dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		hdmi_audio_infoframe_config(hdmi);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		break;
	default:
		break;
	}

	return 0;
}

static const struct snd_kcontrol_new sun4i_hdmi_audio_controls[] = {
	{
		.access = SNDRV_CTL_ELEM_ACCESS_READ |
			  SNDRV_CTL_ELEM_ACCESS_VOLATILE,
		.iface = SNDRV_CTL_ELEM_IFACE_PCM,
		.name = "ELD",
		//.info = sun4i_hdmi_audio_eld_ctl_info,
		//.get = sun4i_hdmi_audio_eld_ctl_get,
	},
};

static const struct snd_soc_dapm_widget sun4i_hdmi_audio_widgets[] = {
	SND_SOC_DAPM_OUTPUT("TX"),
};

static const struct snd_soc_dapm_route sun4i_hdmi_audio_routes[] = {
	{ "TX", NULL, "Playback" },
};

static const struct snd_soc_codec_driver sun4i_hdmi_audio_codec_drv = {
	.component_driver = {
		.controls = sun4i_hdmi_audio_controls,
		.num_controls = ARRAY_SIZE(sun4i_hdmi_audio_controls),
		.dapm_widgets = sun4i_hdmi_audio_widgets,
		.num_dapm_widgets = ARRAY_SIZE(sun4i_hdmi_audio_widgets),
		.dapm_routes = sun4i_hdmi_audio_routes,
		.num_dapm_routes = ARRAY_SIZE(sun4i_hdmi_audio_routes),
	},
};

static const struct snd_soc_dai_ops sun4i_hdmi_audio_dai_ops = {
	.startup = sun4i_hdmi_audio_startup,
	.shutdown = sun4i_hdmi_audio_shutdown,
	.hw_params = sun4i_hdmi_audio_hw_params,
	.set_fmt = sun4i_hdmi_audio_set_fmt,
	.trigger = sun4i_hdmi_audio_trigger,
};

static struct snd_soc_dai_driver sun4i_hdmi_audio_codec_dai_drv = {
	.name = "sun4i-hdmi-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
			 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
			 SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE,
	},
};

static const struct snd_soc_component_driver sun4i_hdmi_audio_cpu_dai_comp = {
	.name = "sun4i-hdmi-cpu-dai-component",
};

static int sun4i_hdmi_audio_cpu_dai_probe(struct snd_soc_dai *dai)
{
	struct sun4i_hdmi *hdmi = dai_to_hdmi(dai);

	snd_soc_dai_init_dma_data(dai, &hdmi->audio.dma_data, NULL);

	return 0;
}

static struct snd_soc_dai_driver sun4i_hdmi_audio_cpu_dai_drv = {
	.name = "sun4i-hdmi-cpu-dai",
	.probe  = sun4i_hdmi_audio_cpu_dai_probe,
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
			 SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |
			 SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |
			 SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_IEC958_SUBFRAME_LE,
	},
	.ops = &sun4i_hdmi_audio_dai_ops,
};

static const struct snd_dmaengine_pcm_config pcm_conf = {
	.chan_names[SNDRV_PCM_STREAM_PLAYBACK] = "audio-rx",
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
};

static int sun4i_hdmi_register_audio_driver(struct device *dev,
					    struct sun4i_hdmi *hdmi)
{
	struct hdmi_codec_pdata codec_data = {
		.ops = &audio_codec_ops,
		.max_i2s_channels = 8,
		.i2s = 1,
	};

	DRM_DEBUG_DRIVER("\n");

	hdmi->audio.enabled = false;

#if 0
	hdmi->audio_pdev = platform_device_register_data(
		dev, HDMI_CODEC_DRV_NAME, PLATFORM_DEVID_AUTO,
		&codec_data, sizeof(codec_data));

	if (IS_ERR(hdmi->audio_pdev))
		return PTR_ERR(hdmi->audio_pdev);

	DRM_INFO("%s Driver bound %s\n", HDMI_CODEC_DRV_NAME, dev_name(dev));
#else
{
	int ret;
	struct snd_soc_dai_link *dai_link = &hdmi->audio.link;
	struct snd_soc_card *card = &hdmi->audio.card;

	ret = devm_snd_dmaengine_pcm_register(dev, &pcm_conf, 0);
	if (ret) {
		dev_err(dev, "Could not register PCM component: %d\n", ret);
		return ret;
	}

	ret = devm_snd_soc_register_component(dev, &sun4i_hdmi_audio_cpu_dai_comp,
					      &sun4i_hdmi_audio_cpu_dai_drv, 1);
	if (ret) {
		dev_err(dev, "Could not register CPU DAI: %d\n", ret);
		return ret;
	}

	/* register codec and codec dai */
	ret = snd_soc_register_codec(dev, &sun4i_hdmi_audio_codec_drv,
				     &sun4i_hdmi_audio_codec_dai_drv, 1);
	if (ret) {
		dev_err(dev, "Could not register codec: %d\n", ret);
		return ret;
	}

	dai_link->name = "MAI";
	dai_link->stream_name = "MAI PCM";
	dai_link->codec_dai_name = sun4i_hdmi_audio_codec_dai_drv.name;
	dai_link->cpu_dai_name = dev_name(dev);
	dai_link->codec_name = dev_name(dev);
	dai_link->platform_name = dev_name(dev);

	card->dai_link = dai_link;
	card->num_links = 1;
	card->name = "sun4i-hdmi";
	card->dev = dev;

	snd_soc_card_set_drvdata(card, hdmi);
	ret = devm_snd_soc_register_card(dev, card);
	if (ret) {
		dev_err(dev, "Could not register sound card: %d\n", ret);
		goto unregister_codec;
	}

	return 0;

unregister_codec:
	snd_soc_unregister_codec(dev);
}
#endif
	return 0;
}

static int sun4i_hdmi_bind(struct device *dev, struct device *master,
			   void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct drm_device *drm = data;
	struct sun4i_drv *drv = drm->dev_private;
	struct sun4i_hdmi *hdmi;
	struct resource *res;
	u32 reg;
	int ret;

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;
	dev_set_drvdata(dev, hdmi);
	hdmi->dev = dev;
	hdmi->drv = drv;

	hdmi->variant = of_device_get_match_data(dev);
	if (!hdmi->variant)
		return -EINVAL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hdmi->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(hdmi->base)) {
		dev_err(dev, "Couldn't map the HDMI encoder registers\n");
		return PTR_ERR(hdmi->base);
	}

	if (hdmi->variant->has_reset_control) {
		hdmi->reset = devm_reset_control_get(dev, NULL);
		if (IS_ERR(hdmi->reset)) {
			dev_err(dev, "Couldn't get the HDMI reset control\n");
			return PTR_ERR(hdmi->reset);
		}

		ret = reset_control_deassert(hdmi->reset);
		if (ret) {
			dev_err(dev, "Couldn't deassert HDMI reset\n");
			return ret;
		}
	}

	hdmi->bus_clk = devm_clk_get(dev, "ahb");
	if (IS_ERR(hdmi->bus_clk)) {
		dev_err(dev, "Couldn't get the HDMI bus clock\n");
		ret = PTR_ERR(hdmi->bus_clk);
		goto err_assert_reset;
	}
	clk_prepare_enable(hdmi->bus_clk);

	hdmi->mod_clk = devm_clk_get(dev, "mod");
	if (IS_ERR(hdmi->mod_clk)) {
		dev_err(dev, "Couldn't get the HDMI mod clock\n");
		ret = PTR_ERR(hdmi->mod_clk);
		goto err_disable_bus_clk;
	}
	clk_prepare_enable(hdmi->mod_clk);

	hdmi->pll0_clk = devm_clk_get(dev, "pll-0");
	if (IS_ERR(hdmi->pll0_clk)) {
		dev_err(dev, "Couldn't get the HDMI PLL 0 clock\n");
		ret = PTR_ERR(hdmi->pll0_clk);
		goto err_disable_mod_clk;
	}

	hdmi->pll1_clk = devm_clk_get(dev, "pll-1");
	if (IS_ERR(hdmi->pll1_clk)) {
		dev_err(dev, "Couldn't get the HDMI PLL 1 clock\n");
		ret = PTR_ERR(hdmi->pll1_clk);
		goto err_disable_mod_clk;
	}

	hdmi->regmap = devm_regmap_init_mmio(dev, hdmi->base,
					     &sun4i_hdmi_regmap_config);
	if (IS_ERR(hdmi->regmap)) {
		dev_err(dev, "Couldn't create HDMI encoder regmap\n");
		return PTR_ERR(hdmi->regmap);
	}

	ret = sun4i_tmds_create(hdmi);
	if (ret) {
		dev_err(dev, "Couldn't create the TMDS clock\n");
		goto err_disable_mod_clk;
	}

	if (hdmi->variant->has_ddc_parent_clk) {
		hdmi->ddc_parent_clk = devm_clk_get(dev, "ddc");
		if (IS_ERR(hdmi->ddc_parent_clk)) {
			dev_err(dev, "Couldn't get the HDMI DDC clock\n");
			return PTR_ERR(hdmi->ddc_parent_clk);
		}
	} else {
		hdmi->ddc_parent_clk = hdmi->tmds_clk;
	}

	writel(SUN4I_HDMI_CTRL_ENABLE, hdmi->base + SUN4I_HDMI_CTRL_REG);

	writel(hdmi->variant->pad_ctrl0_init_val,
	       hdmi->base + SUN4I_HDMI_PAD_CTRL0_REG);

	reg = readl(hdmi->base + SUN4I_HDMI_PLL_CTRL_REG);
	reg &= SUN4I_HDMI_PLL_CTRL_DIV_MASK;
	reg |= hdmi->variant->pll_ctrl_init_val;
	writel(reg, hdmi->base + SUN4I_HDMI_PLL_CTRL_REG);

	ret = sun4i_hdmi_i2c_create(dev, hdmi);
	if (ret) {
		dev_err(dev, "Couldn't create the HDMI I2C adapter\n");
		goto err_disable_mod_clk;
	}

	drm_encoder_helper_add(&hdmi->encoder,
			       &sun4i_hdmi_helper_funcs);
	ret = drm_encoder_init(drm,
			       &hdmi->encoder,
			       &sun4i_hdmi_funcs,
			       DRM_MODE_ENCODER_TMDS,
			       NULL);
	if (ret) {
		dev_err(dev, "Couldn't initialise the HDMI encoder\n");
		goto err_del_i2c_adapter;
	}

	hdmi->encoder.possible_crtcs = drm_of_find_possible_crtcs(drm,
								  dev->of_node);
	if (!hdmi->encoder.possible_crtcs) {
		ret = -EPROBE_DEFER;
		goto err_del_i2c_adapter;
	}

#ifdef CONFIG_DRM_SUN4I_HDMI_CEC
	hdmi->cec_adap = cec_pin_allocate_adapter(&sun4i_hdmi_cec_pin_ops,
		hdmi, "sun4i", CEC_CAP_TRANSMIT | CEC_CAP_LOG_ADDRS |
		CEC_CAP_PASSTHROUGH | CEC_CAP_RC);
	ret = PTR_ERR_OR_ZERO(hdmi->cec_adap);
	if (ret < 0)
		goto err_cleanup_connector;
	writel(readl(hdmi->base + SUN4I_HDMI_CEC) & ~SUN4I_HDMI_CEC_TX,
	       hdmi->base + SUN4I_HDMI_CEC);
#endif

	drm_connector_helper_add(&hdmi->connector,
				 &sun4i_hdmi_connector_helper_funcs);
	ret = drm_connector_init(drm, &hdmi->connector,
				 &sun4i_hdmi_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);
	if (ret) {
		dev_err(dev,
			"Couldn't initialise the HDMI connector\n");
		goto err_cleanup_connector;
	}

	/* There is no HPD interrupt, so we need to poll the controller */
	hdmi->connector.polled = DRM_CONNECTOR_POLL_CONNECT |
		DRM_CONNECTOR_POLL_DISCONNECT;

	ret = cec_register_adapter(hdmi->cec_adap, dev);
	if (ret < 0)
		goto err_cleanup_connector;
	drm_mode_connector_attach_encoder(&hdmi->connector, &hdmi->encoder);

	ret = sun4i_hdmi_register_audio_driver(dev, hdmi);
	if (ret) {
		DRM_ERROR("Failed to attach an audio codec\n");
		goto err_cleanup_connector;
	}

	/* Initialize audio infoframe */
	ret = hdmi_audio_infoframe_init(&hdmi->audio.cea);
	if (ret) {
		DRM_ERROR("Failed to init audio infoframe\n");
		goto err_cleanup_connector;
	}

	return 0;

err_cleanup_connector:
	cec_delete_adapter(hdmi->cec_adap);
	drm_encoder_cleanup(&hdmi->encoder);
err_del_i2c_adapter:
	i2c_del_adapter(hdmi->i2c);
err_disable_mod_clk:
	clk_disable_unprepare(hdmi->mod_clk);
err_disable_bus_clk:
	clk_disable_unprepare(hdmi->bus_clk);
err_assert_reset:
	reset_control_assert(hdmi->reset);
	return ret;
}

static void sun4i_hdmi_unbind(struct device *dev, struct device *master,
			    void *data)
{
	struct sun4i_hdmi *hdmi = dev_get_drvdata(dev);

	cec_unregister_adapter(hdmi->cec_adap);
	drm_connector_cleanup(&hdmi->connector);
	drm_encoder_cleanup(&hdmi->encoder);
	i2c_del_adapter(hdmi->i2c);
	clk_disable_unprepare(hdmi->mod_clk);
	clk_disable_unprepare(hdmi->bus_clk);
}

static const struct component_ops sun4i_hdmi_ops = {
	.bind	= sun4i_hdmi_bind,
	.unbind	= sun4i_hdmi_unbind,
};

static int sun4i_hdmi_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &sun4i_hdmi_ops);
}

static int sun4i_hdmi_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &sun4i_hdmi_ops);

	return 0;
}

static const struct of_device_id sun4i_hdmi_of_table[] = {
	{ .compatible = "allwinner,sun4i-a10-hdmi", .data = &sun4i_variant, },
	{ .compatible = "allwinner,sun5i-a10s-hdmi", .data = &sun5i_variant, },
	{ .compatible = "allwinner,sun6i-a31-hdmi", .data = &sun6i_variant, },
	{ }
};
MODULE_DEVICE_TABLE(of, sun4i_hdmi_of_table);

static struct platform_driver sun4i_hdmi_driver = {
	.probe		= sun4i_hdmi_probe,
	.remove		= sun4i_hdmi_remove,
	.driver		= {
		.name		= "sun4i-hdmi",
		.of_match_table	= sun4i_hdmi_of_table,
	},
};
module_platform_driver(sun4i_hdmi_driver);

MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_DESCRIPTION("Allwinner A10 HDMI Driver");
MODULE_LICENSE("GPL");
