/*
 * Hisilicon Terminal SoCs drm driver
 *
 * Copyright (c) 2014-2015 Hisilicon Limited.
 * Author: Xinwei Kong <kong.kongxinwei@hisilicon.com> for hisilicon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <video/display_timing.h>


#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_atomic_helper.h>

#include "hisi_drm_fb.h"
#include "hisi_drm_fbdev.h"
#include "hisi_drm_drv.h"
#include "hisi_drm_plane.h"
#include "hisi_ade_reg.h"
#include "hisi_ade.h"

#define FORCE_PIXEL_CLOCK_SAME_OR_HIGHER 0
#define PRIMARY_CH	(ADE_CH1)

#define to_ade_crtc(c)		container_of(c, struct hisi_crtc, base)
#define to_ade_plane(p)		container_of(p, struct hisi_plane, base)


struct ade_context {
	void __iomem  *base;
	void __iomem  *media_base;
	
	int irq;
	u32 ade_core_rate;
	u32 media_noc_rate;

	struct drm_device  *dev;

	struct clk *ade_core_clk;
	struct clk *media_noc_clk;
	struct clk *ade_pix_clk;

	struct hisi_crtc *crtc;
	struct hisi_plane *plane[ADE_CH_NUM];

	bool power_on;

	/* plane properties */
	struct drm_property *cap_scal_prop; /* 0: unsupport, 1: support */
	struct drm_property *cap_rot_prop;  /*  */
	struct drm_property *plane_zorder_prop;  /*  */
};

/* ade-format info: */
struct ade_format {
	u32 pixel_format;
	enum ADE_FORMAT ade_format;
};

static const struct ade_format ade_formats[] = {
	/* 16bpp RGB: */
	{ DRM_FORMAT_RGB565, ADE_RGB_565 },
	{ DRM_FORMAT_BGR565, ADE_BGR_565 },
	/* 24bpp RGB: */
	{ DRM_FORMAT_RGB888, ADE_RGB_888 },
	{ DRM_FORMAT_BGR888, ADE_BGR_888 },
	/* 32bpp [A]RGB: */
	{ DRM_FORMAT_XRGB8888, ADE_XRGB_8888 },
	{ DRM_FORMAT_XBGR8888, ADE_XBGR_8888 },
	{ DRM_FORMAT_RGBA8888, ADE_RGBA_8888 },
	{ DRM_FORMAT_BGRA8888, ADE_BGRA_8888 },
	{ DRM_FORMAT_ARGB8888, ADE_ARGB_8888 },
	{ DRM_FORMAT_ABGR8888, ADE_ABGR_8888 },
	/* packed YCbCr */
	{ DRM_FORMAT_YUYV, ADE_YUYV },
	{ DRM_FORMAT_YVYU, ADE_YVYU },
	{ DRM_FORMAT_UYVY, ADE_UYVY },
	{ DRM_FORMAT_VYUY, ADE_VYUY },
	/* 2 plane YCbCr */
	{ DRM_FORMAT_NV12, ADE_NV12 },
	{ DRM_FORMAT_NV21, ADE_NV21 },
	/* 3 plane YCbCr */
	{ DRM_FORMAT_YUV444, ADE_YUV444 },
};

static const uint32_t channel_formats1[] = {
	/* channel 1,2,3,4 */
	DRM_FORMAT_RGB565, DRM_FORMAT_BGR565, DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888, DRM_FORMAT_XRGB8888, DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBA8888, DRM_FORMAT_BGRA8888, DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888
};
static const uint32_t channel_formats2[] = {
	/* channel 5,6 */
	DRM_FORMAT_RGB565, DRM_FORMAT_BGR565, DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888, DRM_FORMAT_XRGB8888, DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBA8888, DRM_FORMAT_BGRA8888, DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888, DRM_FORMAT_YUYV, DRM_FORMAT_YVYU, DRM_FORMAT_UYVY,
	DRM_FORMAT_VYUY, DRM_FORMAT_NV12, DRM_FORMAT_NV21, DRM_FORMAT_YUV444
};
static const uint32_t channel_formats3[] = {
	/* disp channel 7 */
	DRM_FORMAT_RGB565, DRM_FORMAT_BGR565, DRM_FORMAT_RGB888,
	DRM_FORMAT_BGR888, DRM_FORMAT_XRGB8888, DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBA8888, DRM_FORMAT_BGRA8888, DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888, DRM_FORMAT_YUYV, DRM_FORMAT_YVYU, DRM_FORMAT_UYVY,
	DRM_FORMAT_VYUY, DRM_FORMAT_YUV444
};

static void ade_display_commit(struct hisi_crtc *crtc);

u32 ade_get_channel_formats(u32 ch, const u32 **formats)
{
	switch(ch)
	{
	case ADE_CH1:
	case ADE_CH2:
	case ADE_CH3:
	case ADE_CH4:
		*formats = channel_formats1;
		return ARRAY_SIZE(channel_formats1);
	case ADE_CH5:
	case ADE_CH6:
		*formats = channel_formats2;
		return ARRAY_SIZE(channel_formats2);
	case ADE_DISP:
		*formats = channel_formats3;
		return ARRAY_SIZE(channel_formats3);
	default:
		DRM_ERROR("no this channel %d\n", ch);
		*formats = NULL;
		return 0;
	}
}

/* convert from fourcc format to ade format */
static u32 ade_get_format(u32 pixel_format)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ade_formats); i++)
		if (ade_formats[i].pixel_format == pixel_format)
			return ade_formats[i].ade_format;

	/* not found, return rgb565 */
	DRM_ERROR("Not found pixel format!!fourcc_format= %d\n", pixel_format);
	return ADE_RGB_565;
}

static bool ade_is_need_csc(u32 fmt)
{
	switch (fmt) {
        case ADE_YUYV:
        case ADE_YVYU:
        case ADE_UYVY:
        case ADE_VYUY:
        case ADE_YUV444:
        case ADE_NV12:
        case ADE_NV21:
            return true;
        default:
            return false;
	}
}

static void ade_init(struct ade_context *ctx)
{
	void __iomem *base = ctx->base;

	/* enable clk gate */
	set_TOP_CTL_clk_gate_en(base, 1);
	/* clear overlay */
	writel(0, base + ADE_OVLY1_TRANS_CFG);
	writel(0, base + ADE_OVLY_CTL);
	/* for video set to 1, means that ade registers
	 * became effective at frame end */
	set_TOP_CTL_frm_end_start(base, 1);
}

static void ade_ldi_set_mode(struct ade_context *ctx, struct drm_display_mode *mode)
{
	void __iomem *base = ctx->base;
	u32 hfp, hbp, hsw, vfp, vbp, vsw;
	u32 plr_flags;

	plr_flags = (mode->flags & DRM_MODE_FLAG_NVSYNC)
			? HISI_LDI_FLAG_NVSYNC : 0;
	plr_flags |= (mode->flags & DRM_MODE_FLAG_NHSYNC)
			? HISI_LDI_FLAG_NHSYNC : 0;
	hfp = mode->hsync_start - mode->hdisplay;
	hbp = mode->htotal - mode->hsync_end;
	hsw = mode->hsync_end - mode->hsync_start;
	vfp = mode->vsync_start - mode->vdisplay;
	vbp = mode->vtotal - mode->vsync_end;
	vsw = mode->vsync_end - mode->vsync_start;
	if (vsw > 15) {
		pr_err("%s: vsw exceeded 15\n", __func__);
		vsw = 15;
	}

	writel((hbp << 20) | (hfp << 0), base + LDI_HRZ_CTRL0);
	/* p3-73 6220V100 pdf:
	 *  "The configured value is the actual width - 1"
	 */
	writel(hsw - 1, base + LDI_HRZ_CTRL1);
	writel((vbp << 20) | (vfp << 0), base + LDI_VRT_CTRL0);
	/* p3-74 6220V100 pdf:
	 *  "The configured value is the actual width - 1"
	 */
	writel(vsw - 1, base + LDI_VRT_CTRL1);

	/* p3-75 6220V100 pdf:
	 *  "The configured value is the actual width - 1"
	 */
	writel(((mode->vdisplay - 1) << 20) | ((mode->hdisplay - 1) << 0),
	       base + LDI_DSP_SIZE);
	writel(plr_flags, base + LDI_PLR_CTRL);

	/*
	 * other parameters setting
	 */
	writel(BIT(0), base + LDI_WORK_MODE);
	writel((0x3c << 6) | (ADE_OUT_RGB_888 << 3) | BIT(2) | BIT(0),
	       base + LDI_CTRL);
	set_reg(base + LDI_DE_SPACE_LOW, 0x1, 1, 1);
}

static int ade_power_up(struct ade_context *ctx)
{
	void __iomem *media_base = ctx->media_base;
	int ret;

	ret = clk_set_rate(ctx->ade_core_clk, ctx->ade_core_rate);
	if (ret) {
		DRM_ERROR("clk_set_rate ade_core_rate error\n");
		return ret;
	}
	ret = clk_set_rate(ctx->media_noc_clk, ctx->media_noc_rate);
	if (ret) {
		DRM_ERROR("media_noc_clk media_noc_rate error\n");
		return ret;
	}
	ret = clk_prepare_enable(ctx->media_noc_clk);
	if (ret) {
		DRM_ERROR("fail to clk_prepare_enable media_noc_clk\n");
		return ret;
	}

	writel(0x20, media_base + SC_MEDIA_RSTDIS);

	ret = clk_prepare_enable(ctx->ade_core_clk);
	if (ret) {
		DRM_ERROR("fail to clk_prepare_enable ade_core_clk\n");
		return ret;
	}

	ade_init(ctx);
	ctx->power_on = true;
	return 0;
}

static void ade_power_down(struct ade_context *ctx)
{
	void __iomem *base = ctx->base;
	void __iomem *media_base = ctx->media_base;

	set_LDI_CTRL_ldi_en(base, ADE_DISABLE);
	/* dsi pixel off */
	set_reg(base + LDI_HDMI_DSI_GT, 0x1, 1, 0);

	clk_disable_unprepare(ctx->ade_core_clk);
	writel(0x20, media_base + SC_MEDIA_RSTEN);
	clk_disable_unprepare(ctx->media_noc_clk);
	ctx->power_on = false;
}

void  hisi_crtc_enable(struct drm_crtc *c)
{
	struct hisi_crtc *crtc = to_ade_crtc(c);
	struct ade_context *ctx = crtc->ctx;
	int ret;

	DRM_DEBUG_DRIVER("enter.\n");
	if (crtc->enable)
		return;

	if (!ctx->power_on) {
		ret = ade_power_up(ctx);
		if (ret) {
			DRM_ERROR("failed to initialize ade clk\n");
			return ;
		}
	}

	drm_crtc_vblank_on(c);
	ade_display_commit(crtc);

	crtc->enable = true;
	DRM_DEBUG_DRIVER("exit success.\n");
}

void hisi_crtc_disable(struct drm_crtc *c)
{
	struct hisi_crtc *crtc = to_ade_crtc(c);
	struct ade_context *ctx = crtc->ctx;

	DRM_DEBUG_DRIVER("enter.\n");
	if (!crtc->enable)
		return;

	drm_crtc_vblank_off(c);
	ade_power_down(ctx);

	crtc->enable = false;
	crtc->ch_mask = 0;
	crtc->use_mask = 0;
	DRM_DEBUG_DRIVER("exit success.\n");
}

bool hisi_drm_crtc_mode_fixup(struct drm_crtc *c,
				      const struct drm_display_mode *mode,
				      struct drm_display_mode *adj_mode)
{
	struct hisi_crtc *crtc = to_ade_crtc(c);
	struct ade_context *ctx = crtc->ctx;
	u32 clock_kHz = mode->clock;
	int ret;

	DRM_DEBUG_DRIVER("enter.\n");

	if (!ctx->power_on)
		if (ade_power_up(ctx))
			DRM_ERROR("%s: failed to power up ade\n", __func__);

	do {
		ret = clk_set_rate(ctx->ade_pix_clk, clock_kHz * 1000);
		if (ret) {
			DRM_ERROR("set ade_pixel_clk_rate fail\n");
			return false;
		}
		adj_mode->clock = clk_get_rate(ctx->ade_pix_clk) / 1000;
#if FORCE_PIXEL_CLOCK_SAME_OR_HIGHER
		if (adj_mode->clock >= clock_kHz)
#endif
		/* This avoids a bad 720p DSI clock with 1.2GHz DPI PLL */
		if (adj_mode->clock != 72000)
			break;

		clock_kHz += 10;
	} while (1);

	pr_info("%s: pixel clock: req %dkHz -> actual: %dkHz\n",
		__func__, mode->clock, adj_mode->clock);

	DRM_DEBUG_DRIVER("mode_fixup  exit successfully.\n");
	return true;
}

void hisi_drm_crtc_mode_set_nofb(struct drm_crtc *c)
{
	struct hisi_crtc *crtc = to_ade_crtc(c);
	struct ade_context *ctx = crtc->ctx;

	DRM_DEBUG_DRIVER("enter.\n");
	crtc->dmode = &c->state->mode;
	ade_ldi_set_mode(ctx, &c->state->mode);
	DRM_DEBUG_DRIVER("exit success.\n");
}

void hisi_crtc_atomic_begin(struct drm_crtc *c)
{
	struct hisi_crtc *crtc = to_ade_crtc(c);
	struct ade_context *ctx = crtc->ctx;

	DRM_DEBUG_DRIVER("enter.\n");
	if (!ctx->power_on)
		(void) ade_power_up(ctx);
	DRM_DEBUG_DRIVER("exit success.\n");
}

static u32 ade_get_display_source(struct hisi_crtc *crtc)
{
	DRM_DEBUG_DRIVER("ch_mask=0x%X\n", crtc->ch_mask);

	if (crtc->ch_mask)
		return TOP_DISP_SRC_OVLY2;
	else
		return TOP_DISP_SRC_NONE;
}

/*
 * set modules' reset mode: by software or hardware
 * set modules' reload enable/disable
 * */
static void ade_set_reset_and_reload(struct hisi_crtc *crtc)
{
	struct ade_context *ctx = crtc->ctx;
	void __iomem *base = ctx->base;
	u32 mask0 = (u32)crtc->use_mask;
	u32 mask1 = (u32)(crtc->use_mask >> 32);

	DRM_DEBUG_DRIVER("mask=0x%llX, mask0=0x%X, mask1=0x%X\n",
			crtc->use_mask, mask0, mask1);

	writel(mask0, base + ADE_SOFT_RST_SEL0);
	writel(mask1, base + ADE_SOFT_RST_SEL1);
	writel(~mask0, base + ADE_RELOAD_DIS0);
	writel(~mask1, base + ADE_RELOAD_DIS1);
}

/*
 * commit to ldi to display
 */
static void ade_display_commit(struct hisi_crtc *crtc)
{
	struct ade_context *ctx = crtc->ctx;
	void __iomem *base = ctx->base;
	u32 out_w = crtc->dmode->hdisplay;
	u32 out_h = crtc->dmode->vdisplay;
	u32 disp_src;

	/* TODO: set rotator after overlay */

	/* TODO: set scale after overlay */

	/* display source setting */
	disp_src = ade_get_display_source(crtc);
	if (disp_src == TOP_DISP_SRC_NONE)
		return;
	writel(disp_src, base + ADE_DISP_SRC_CFG);

	/* ctran6 setting */
	writel(1, base + ADE_CTRAN_DIS(ADE_CTRAN6));
	writel(out_w * out_h - 1, base + ADE_CTRAN_IMAGE_SIZE(ADE_CTRAN6));
	crtc->use_mask |= BIT(ADE_CTRAN_BIT_OFST + ADE_CTRAN6);

	/* set reset mode:soft or hw, and reload modules */
	ade_set_reset_and_reload(crtc);

	DRM_INFO("ADE GO : src=%d\n", disp_src);
	/* enable ade */
	wmb();
	writel(ADE_ENABLE, base + ADE_EN);
	wmb();
	set_LDI_CTRL_ldi_en(base, ADE_ENABLE);
	/* dsi pixel on */
	set_reg(base + LDI_HDMI_DSI_GT, 0x0, 1, 0);
}

void hisi_crtc_atomic_flush(struct drm_crtc *c)

{
	struct hisi_crtc *crtc = to_ade_crtc(c);
	//struct drm_crtc_state *state;

	DRM_DEBUG_DRIVER("enter.\n");
	/* commit to  display: LDI input setting */
	if (crtc->enable) {
		ade_display_commit(crtc);
	}
	DRM_DEBUG_DRIVER("exit success.\n");
}

void hisi_drm_crtc_mode_prepare(struct drm_crtc *c)
{
	struct hisi_crtc *crtc = to_ade_crtc(c);
	struct ade_context *ctx = crtc->ctx;

	DRM_DEBUG_DRIVER("enter.\n");
	if (!ctx->power_on)
		(void) ade_power_up(ctx);
	DRM_DEBUG_DRIVER("exit success.\n");
}

void hisi_drm_crtc_destroy(struct drm_crtc *c)
{
	struct hisi_crtc *crtc = to_ade_crtc(c);

	drm_crtc_cleanup(c);
	devm_kfree(c->dev->dev, crtc);
}

static int hisi_drm_ade_dts_parse(struct platform_device *pdev,
				    struct ade_context *ctx)
{
	struct resource	    *res;
	struct device   *dev;
	struct device_node  *np;
	int ret;

	dev = &pdev->dev;
	np  = dev->of_node;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ade_base");
	ctx->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ctx->base)) {
		DRM_ERROR("failed to remap ade io base\n");
		return  PTR_ERR(ctx->base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "media_base");
	ctx->media_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ctx->media_base)) {
		DRM_ERROR("failed to remap media io base\n");
		return PTR_ERR(ctx->media_base);
	}

	ctx->irq = platform_get_irq(pdev, 0);
	if (!ctx->irq) {
		DRM_ERROR("failed to parse the irq\n");
		return -ENODEV;
	}

	ctx->ade_core_clk = devm_clk_get(&pdev->dev, "clk_ade_core");
	if (ctx->ade_core_clk == NULL) {
		DRM_ERROR("failed to parse the ADE_CORE\n");
		return -ENODEV;
	}
	ctx->media_noc_clk = devm_clk_get(&pdev->dev,
					"aclk_codec_jpeg_src");
	if (ctx->media_noc_clk == NULL) {
		DRM_ERROR("failed to parse the CODEC_JPEG\n");
	    return -ENODEV;
	}
	ctx->ade_pix_clk = devm_clk_get(&pdev->dev, "clk_ade_pix");
	if (ctx->ade_pix_clk == NULL) {
		DRM_ERROR("failed to parse the ADE_PIX_SRC\n");
	    return -ENODEV;
	}

	ret = of_property_read_u32(np, "ade_core_clk_rate",
				    &ctx->ade_core_rate);
	if (ret) {
		DRM_ERROR("failed to parse the ade_core_clk_rate\n");
	    return -ENODEV;
	}
	ret = of_property_read_u32(np, "media_noc_clk_rate",
				    &ctx->media_noc_rate);
	if (ret) {
		DRM_ERROR("failed to parse the media_noc_clk_rate\n");
		return -ENODEV;
	}

	return 0;
}

int hisi_drm_enable_vblank(struct drm_device *dev, int c)
{
	struct hisi_drm_private *private = dev->dev_private;
	struct hisi_crtc *crtc = to_ade_crtc(private->crtc);
	struct ade_context *ctx = crtc->ctx;
	void __iomem *base = ctx->base;
	u32 intr_en;

	DRM_INFO("enable_vblank enter.\n");
	if (!ctx->power_on)
		(void) ade_power_up(ctx);

	intr_en = readl(base + LDI_INT_EN);
	intr_en |= LDI_ISR_FRAME_END_INT;
	writel(intr_en, base + LDI_INT_EN);
	return 0;
}

void hisi_drm_disable_vblank(struct drm_device *dev, int c)
{
	struct hisi_drm_private *private = dev->dev_private;
	struct hisi_crtc *crtc = to_ade_crtc(private->crtc);
	struct ade_context *ctx = crtc->ctx;
	void __iomem *base = ctx->base;
	u32 intr_en;

	DRM_INFO("disable_vblank enter.\n");
	if (!ctx->power_on) {
		DRM_ERROR("power is down! vblank disable fail\n");
		return ;
	}
	intr_en = readl(base + LDI_INT_EN);
	intr_en &= ~LDI_ISR_FRAME_END_INT;
	writel(intr_en, base + LDI_INT_EN);
}

irqreturn_t hisi_drm_irq_handler(int irq, void *arg)
{
	struct drm_device *dev = (struct drm_device *) arg;
	struct hisi_drm_private *private = dev->dev_private;
	struct hisi_crtc *crtc = to_ade_crtc(private->crtc);
	struct ade_context *ctx = crtc->ctx;
	void __iomem *base = ctx->base;
	u32 status;

	status = readl(base + LDI_MSK_INT);
	/* DRM_INFO("LDI IRQ: status=0x%X\n",status); */

	/* vblank irq */
	if (status & LDI_ISR_FRAME_END_INT) {
		writel(LDI_ISR_FRAME_END_INT, base + LDI_INT_CLR);
		drm_handle_vblank(dev, drm_crtc_index(&crtc->base));
	}

	return IRQ_HANDLED;
}


static void ade_rdma_set(struct hisi_crtc *crtc, struct drm_framebuffer *fb, u32 ch,
			 u32 x, u32 y, u32 in_w, u32 in_h, u32 fmt)
{
	u32 reg_ctrl, reg_addr, reg_size, reg_stride, reg_space, reg_en;
	struct drm_gem_cma_object *obj = hisi_drm_fb_get_gem_obj(fb, 0);
	struct ade_context *ctx = crtc->ctx;
	void __iomem *base = ctx->base;
	u32 stride;
	u32 addr;

	stride = fb->pitches[0];
	addr = (u32)obj->paddr + y * stride + x * (fb->bits_per_pixel >> 3);

	DRM_DEBUG_DRIVER("rdma%d: (x=%d, y=%d, %dx%d), stride=%d, paddr=0x%x,"
			 "addr=0x%x, fb:%dx%d, pixel_format=%d(%s)\n",
			 ch+1, x, y, in_w, in_h, stride, (u32)obj->paddr,
			 addr, fb->width, fb->height,
			 fmt, drm_get_format_name(fb->pixel_format));

	/* get reg offset */
	switch(ch) {
	case ADE_DISP:
		reg_ctrl = RD_CH_DISP_CTRL;
		reg_addr = RD_CH_DISP_ADDR;
		reg_size = RD_CH_DISP_SIZE;
		reg_stride = RD_CH_DISP_STRIDE;
		reg_space = RD_CH_DISP_SPACE;
		reg_en = RD_CH_DISP_EN;
		break;
	default:
		reg_ctrl = RD_CH_CTRL(ch);
		reg_addr = RD_CH_ADDR(ch);
		reg_size = RD_CH_SIZE(ch);
		reg_stride = RD_CH_STRIDE(ch);
		reg_space = RD_CH_SPACE(ch);
		reg_en = RD_CH_EN(ch);
		break;
	}

	DRM_DEBUG_DRIVER("channel%d: reg_ctrl=0x%X, reg_addr=0x%X, reg_size=0x%X,"
			"reg_stride=0x%X, reg_space=0x%X, reg_en=0x%X\n",
			ch+1, reg_ctrl, reg_addr, reg_size,
			reg_stride, reg_space, reg_en);

	/*
	 * TODO: set rotation
	 * */
	writel((fmt << 16) & 0x1f0000, base + reg_ctrl);
	writel(addr, base + reg_addr);
	writel((in_h << 16) | stride, base + reg_size);
	writel(stride, base + reg_stride);
	writel(in_h * stride, base + reg_space);
	writel(1, base + reg_en);
	
	crtc->use_mask |= BIT(ADE_CH_RDMA_BIT_OFST + ch);
}

static void ade_rdma_disable(struct hisi_crtc *crtc, u32 ch)
{
	struct ade_context *ctx = crtc->ctx;
	void __iomem *base = ctx->base;
	u32 reg_en;

	/* get reg offset */
	switch(ch) {
	case ADE_DISP:
		reg_en = RD_CH_DISP_EN;
		break;
	default:
		reg_en = RD_CH_EN(ch);
		break;
	}

	writel(0, base + reg_en);
	crtc->use_mask &= ~BIT(ADE_CH_RDMA_BIT_OFST + ch);
}

static void ade_clip_set(struct hisi_crtc *crtc, u32 ch, u32 in_w, u32 in_h,
			 u32 *out_w, u32 *out_h)
{
	struct ade_context *ctx = crtc->ctx;
	void __iomem *base = ctx->base;
	bool need_clip = false;
	u32 o_w, o_h;
	u32 disable_val;

	if (ch == ADE_DISP)
		return;

	if (!need_clip) {/* bypass */
		disable_val = 1;
		o_w = in_w;
		o_h = in_h;
	} else {/* TODO: clip setting
		disable_val = 0;
		o_w = ;
		o_h = ;
		writel(1, base + ADE_CLIP_SIZE1(ch));
		writel(1, base + ADE_CLIP_SIZE2(ch)); */
	}

	DRM_DEBUG_DRIVER("clip%d: reg_disable=0x%X, reg_size0=0x%X\n",
			ch+1, ADE_CLIP_DISABLE(ch), ADE_CLIP_SIZE0(ch));

	writel(disable_val, base + ADE_CLIP_DISABLE(ch));
	writel((in_w - 1) << 16 | (in_h - 1), base + ADE_CLIP_SIZE0(ch));

	*out_w = o_w;
	*out_h = o_h;

	crtc->use_mask |= BIT(ADE_CLIP_BIT_OFST + ch);
}

static void ade_clip_disable(struct hisi_crtc *crtc, u32 ch)
{
	struct ade_context *ctx = crtc->ctx;
	void __iomem *base = ctx->base;

	if (ch == ADE_DISP)
		return;

	writel(1, base + ADE_CLIP_DISABLE(ch));
	//writel(1, base + ADE_CLIP_CFG_OK(ch));
	crtc->use_mask &= ~BIT(ADE_CLIP_BIT_OFST + ch);
}

static void ade_scale_set(struct hisi_crtc *crtc, u32 ch, u32 in_w, u32 in_h,
			  u32 *out_w, u32 *out_h)
{
	struct ade_context *ctx = crtc->ctx;
	void __iomem *base = ctx->base;
	bool need_scale = false;
	u32 o_w, o_h;
	u32 ctrl_val;
	u8 x;

	switch (ch) {
	case ADE_CH5:
		x = ADE_SCL3;
		break;
	case ADE_CH6:
		x = ADE_SCL1;
		break;
	default: /* channel 1,2,3,4,disp has no scale capability */
		return;
	}

	if (!need_scale) {/* bypass */
		ctrl_val = 0x400;
		o_w = in_w;
		o_h = in_h;
	} else {/* TODO: scale setting
		ctrl_val = 0x400;
		o_w = ;
		o_h = ; */
	}

	DRM_DEBUG_DRIVER("channel%d, scl%d: reg_ctrl=0x%X, reg_ires=0x%X, reg_ores=0x%X, reg_start=0x%X\n",
			 ch+1, x+1, ADE_SCL_CTRL(x), ADE_SCL_IRES(x), ADE_SCL_ORES(x),
			 ADE_SCL_START(x));

	writel(ctrl_val, base + ADE_SCL_CTRL(x));
	writel((in_h - 1) << 16 | (in_w - 1), base + ADE_SCL_IRES(x));
	writel((o_h - 1) << 16 | (o_w - 1), base + ADE_SCL_ORES(x));
	writel(1, base + ADE_SCL_START(x));

	*out_w = o_w;
	*out_h = o_h;

	crtc->use_mask |= BIT(ADE_SCL_BIT_OFST + x);
}

static void ade_scale_disable(struct hisi_crtc *crtc, u32 ch)
{
	struct ade_context *ctx = crtc->ctx;
	void __iomem *base = ctx->base;
	u8 x;

	switch (ch) {
	case ADE_CH5:
		x = ADE_SCL3;
		break;
	case ADE_CH6:
		x = ADE_SCL1;
		break;
	default: /* channel 1,2,3,4,disp has no scale capability */
		return;
	}

	writel(0, base + ADE_SCL_START(x));
	crtc->use_mask &= ~BIT(ADE_SCL_BIT_OFST + x);
}

/*
 * corlor space converting
 * */
static void ade_ctran_set(struct hisi_crtc *crtc, u32 ch, u32 in_w, u32 in_h,
			  u32 fmt)
{
	struct ade_context *ctx = crtc->ctx;
	void __iomem *base = ctx->base;
	bool need_ctran = ade_is_need_csc(fmt);
	u8 x;

	switch (ch) {
	case ADE_DISP:
		x = ADE_CTRAN5;
		break;
	case ADE_CH5:
		x = ADE_CTRAN1;
		break;
	case ADE_CH6:
		x = ADE_CTRAN2;
		break;
	default: /* channel 1,2,3,4 has no csc capability */
		return;
	}

	DRM_DEBUG_DRIVER("channel%d, ctran%d: reg_dis=0x%X, reg_size=0x%X\n",
			ch+1, x+1, ADE_CTRAN_DIS(x),
			ADE_CTRAN_IMAGE_SIZE(x));

	if (!need_ctran) {/* bypass */
		writel(1, base + ADE_CTRAN_DIS(x));
		writel(in_w * in_h - 1, base + ADE_CTRAN_IMAGE_SIZE(x));
	} else {/* TODO */
	}

	crtc->use_mask |= BIT(ADE_CTRAN_BIT_OFST + x);
}

static void ade_ctran_disable(struct hisi_crtc *crtc, u32 ch)
{
	struct ade_context *ctx = crtc->ctx;
	void __iomem *base = ctx->base;
	u8 x;

	switch (ch) {
	case ADE_DISP:
		x = ADE_CTRAN5;
		break;
	case ADE_CH5:
		x = ADE_CTRAN1;
		break;
	case ADE_CH6:
		x = ADE_CTRAN2;
		break;
	default: /* channel 1,2,3,4 has no csc capability */
		return;
	}
	

	writel(1, base + ADE_CTRAN_DIS(x));
	//writel(1, base + ADE_CTRAN_CFG_OK(x));
	crtc->use_mask &= ~BIT(ADE_CTRAN_BIT_OFST + x);
}

static void ade_overlay_set(struct hisi_crtc *crtc, u8 ch, u8 zorder,
			    u32 x0, u32 y0, u32 in_w, u32 in_h)
{
	struct ade_context *ctx = crtc->ctx;
	void __iomem *base = ctx->base;
	u8 ovly_ch = zorder;
	u8 x = ADE_OVLY2;
	u32 x1 = x0 + in_w - 1;
	u32 y1 = y0 + in_h - 1;
	u32 val;
	u8 alp_sel = ADE_ALP_MUL_COEFF_3;
	u8 under_alp_sel = ADE_ALP_MUL_COEFF_2;
	u8 alp_mode = ADE_ALP_PIXEL;

	DRM_DEBUG_DRIVER("channel%d--> ovly_ch=%d of ovly%d: reg_CH%d_XY0=0x%X, reg_CH%d_XY1=0x%X, reg_CH%d_ctl=0x%X\n",
			ch+1, ovly_ch+1, x+1,
			ovly_ch+1, ADE_OVLY_CH_XY0(ovly_ch),
			ovly_ch+1, ADE_OVLY_CH_XY1(ovly_ch),
			ovly_ch+1, ADE_OVLY_CH_CTL(ovly_ch));
	DRM_DEBUG_DRIVER("reg_ovly%d_size=0x%X, reg_ovly%d_color=0x%X, reg_ovly%d_ctl=0x%X\n",
			x+1, ADE_OVLY_OUTPUT_SIZE(x),
			x+1, ADE_OVLY_BASE_COLOR(x),
			x+1, ADE_OVLYX_CTL(x));

	/* overlay routing setting */
	writel(x0 << 16 | y0, base + ADE_OVLY_CH_XY0(ovly_ch));
	writel(x1 << 16 | y1, base + ADE_OVLY_CH_XY1(ovly_ch));
	val = (ch + 1) << ADE_OVLY_CH_SEL_OFST | BIT(ADE_OVLY_CH_EN_OFST) |
		alp_sel << ADE_OVLY_CH_ALP_SEL_OFST |
		under_alp_sel << ADE_OVLY_CH_UNDER_ALP_SEL_OFST |
		alp_mode << ADE_OVLY_CH_ALP_MODE_OFST;
	DRM_DEBUG_DRIVER("ch%d_ctl=0x%X\n", ovly_ch+1, val);
	writel(val, base + ADE_OVLY_CH_CTL(ovly_ch));
	val= (x + 1) << (ovly_ch * 4) | readl(base + ADE_OVLY_CTL);
	DRM_DEBUG_DRIVER("ovly_ctl=0x%X\n", val);
	writel(val, base + ADE_OVLY_CTL);

	if (ch == ADE_DISP)
		writel(1, base + ADE_CTRAN5_TRANS_CFG);

	/* when primary is enable, indicate that it's ready to output. */
	if (ch == PRIMARY_CH) {
		val = (in_w - 1 ) << 16 | (in_h - 1);
		writel(val, base + ADE_OVLY_OUTPUT_SIZE(x));
		writel(1, base + ADE_OVLYX_CTL(x));
		crtc->use_mask |= BIT(ADE_OVLY_BIT_OFST + x);
	}
}

static void ade_overlay_disable(struct hisi_crtc *crtc, u32 ch)
{
	struct ade_context *ctx = crtc->ctx;
	void __iomem *base = ctx->base;
	//u8 x = ADE_OVLY2;
	u32 val;

	val = ~BIT(6) & readl(base + ADE_OVLY_CH_CTL(ch));
	DRM_DEBUG_DRIVER("ch%d_ctl=0x%X\n", ch+1, val);
	writel(val, base + ADE_OVLY_CH_CTL(ch));
	val = ~(0x3 << (ch * 4) & readl(base + ADE_OVLY_CTL));
	DRM_DEBUG_DRIVER("ovly_ctl=0x%X\n", val);
	writel(val, base + ADE_OVLY_CTL);

	if (ch == ADE_DISP)
		writel(0, base + ADE_CTRAN5_TRANS_CFG);

	/* primary plane
	if (ch == PRIMARY_CH) {
		writel(0, base + ADE_OVLYX_CTL(x));
		crtc->use_mask |= BIT(ADE_OVLY_BIT_OFST + x);
	} */
}

static void ade_update_channel(struct hisi_plane *plane, struct hisi_crtc *crtc,
			  struct drm_framebuffer *fb, int crtc_x, int crtc_y,
			  unsigned int crtc_w, unsigned int crtc_h,
			  uint32_t src_x, uint32_t src_y,
			  uint32_t src_w, uint32_t src_h)
{
	u8 ch = plane->ch;
	u8 zorder = plane->zorder;
	u32 fmt = ade_get_format(fb->pixel_format);
	u32 in_w;
	u32 in_h;
	u32 out_w;
	u32 out_h;


	DRM_DEBUG_DRIVER("update channel%d, src:(%d, %d)-%dx%d, crtc:(%d, %d)-%dx%d\n",
			ch+1, src_x, src_y, src_w, src_h,
			crtc_x, crtc_y, crtc_w, crtc_h);

	/* 1) DMA setting */
	in_w = src_w;
	in_h = src_h;
	ade_rdma_set(crtc, fb, ch, src_x, src_y, in_w, in_h, fmt);

	/* 2) clip setting */
	out_w = crtc_w;
	out_h = crtc_h;
	ade_clip_set(crtc, ch, in_w, in_h, &out_w, &out_h);

	/* 3) scale setting */
	in_w = out_w;
	in_h = out_h;
	out_w = crtc_w;
	out_h = crtc_h;
	ade_scale_set(crtc, ch, in_w, in_h, &out_w, &out_h);

	/* 4) ctran/csc setting */
	in_w = out_w;
	in_h = out_h;
	ade_ctran_set(crtc, ch, in_w, in_h, fmt);

	/* 5) overlay routing setting */
	ade_overlay_set(crtc, ch, zorder, crtc_x, crtc_y, in_w, in_h);

	crtc->ch_mask |= BIT(ch);
	DRM_DEBUG_DRIVER("exit success.\n");
}

static void ade_disable_channel(struct hisi_plane *plane, struct hisi_crtc *crtc)
{
	u32 ch = plane->ch;

	DRM_DEBUG_DRIVER("disable channel%d\n", ch);

	/*
	 * when primary is disable, power is down
	 * so no need to disable this channel.
	 */
	if (ch == PRIMARY_CH)
		return;

	/* disable read DMA */
	ade_rdma_disable(crtc, ch);

	/* disable clip */
	ade_clip_disable(crtc, ch);

	/* disable scale */
	ade_scale_disable(crtc, ch);

	/* disable ctran */
	ade_ctran_disable(crtc, ch);

	/* disable overlay routing */
	ade_overlay_disable(crtc, ch);

	crtc->ch_mask &= ~BIT(ch);
	DRM_DEBUG_DRIVER("exit success.\n");
}

void hisi_plane_destroy(struct drm_plane *p)
{
	struct hisi_plane *plane = to_ade_plane(p);

	drm_plane_cleanup(p);
	devm_kfree(p->dev->dev, plane);
}

int hisi_plane_prepare_fb(struct drm_plane *p,
				struct drm_framebuffer *fb,
				const struct drm_plane_state *new_state)
{
	DRM_DEBUG_DRIVER("enter.\n");
	DRM_DEBUG_DRIVER("exit success.\n");
	return 0;
}

void hisi_plane_cleanup_fb(struct drm_plane *p,
				struct drm_framebuffer *fb,
				const struct drm_plane_state *old_state)
{
	DRM_DEBUG_DRIVER("enter.\n");
	DRM_DEBUG_DRIVER("exit success.\n");
}

int hisi_plane_atomic_check(struct drm_plane *p,
				     struct drm_plane_state *state)
{
	DRM_DEBUG_DRIVER("enter.\n");
	DRM_DEBUG_DRIVER("exit success.\n");
	return 0;
}

void hisi_plane_atomic_update(struct drm_plane *p,
				       struct drm_plane_state *old_state)
{
	struct hisi_plane *plane = to_ade_plane(p);
	struct drm_plane_state	*state	= p->state;
	struct hisi_crtc *crtc;

	DRM_DEBUG_DRIVER("enter.\n");
	if (!state->crtc)
		return;

	crtc = to_ade_crtc(state->crtc);
	ade_update_channel(plane, crtc, state->fb,
		      state->crtc_x, state->crtc_y,
		      state->crtc_w, state->crtc_h,
		      state->src_x >> 16, state->src_y >> 16,
		      state->src_w >> 16, state->src_h >> 16);

	DRM_DEBUG_DRIVER("exit success.\n");
}

void hisi_plane_atomic_disable(struct drm_plane *p,
			       struct drm_plane_state *old_state)
{
	struct hisi_plane *plane = to_ade_plane(p);
	struct hisi_crtc *crtc;

	DRM_DEBUG_DRIVER("enter.\n");
	if (!old_state->crtc)
		return;

	crtc = to_ade_crtc(old_state->crtc);
	ade_disable_channel(plane, crtc);

	DRM_DEBUG_DRIVER("exit success.\n");
}

#define ADE_CHANNEL_SCALE_UNSUPPORT 0
#define ADE_CHANNEL_SCALE_SUPPORT 1

static const struct drm_prop_enum_list ade_ch_scale_enum_list[] = {
	{ ADE_CHANNEL_SCALE_UNSUPPORT, "unsupport" },
	{ ADE_CHANNEL_SCALE_SUPPORT, "support" },
};

int ade_install_plane_properties(struct drm_device *dev,
					struct hisi_plane *plane)
{
	struct drm_mode_object *obj = &plane->base.base;
	struct ade_context *ctx = plane->ctx;
	struct drm_property *prop;
	u8 ch = plane->ch;

	/* create properties */
	if (!ctx->cap_scal_prop) {
		prop = drm_property_create_enum(dev, DRM_MODE_PROP_IMMUTABLE,
						"scale capability",
						ade_ch_scale_enum_list,
						ARRAY_SIZE(ade_ch_scale_enum_list));
		if (!prop)
			return 0;

		ctx->cap_scal_prop = prop;
	}

	if (!ctx->cap_rot_prop) {
		prop = drm_mode_create_rotation_property(dev,
							 BIT(DRM_ROTATE_0) |
							 BIT(DRM_ROTATE_90) |
							 BIT(DRM_ROTATE_180) |
							 BIT(DRM_ROTATE_270) |
							 BIT(DRM_REFLECT_X) |
							 BIT(DRM_REFLECT_Y));
		if (!prop)
			return 0;

		ctx->cap_rot_prop = prop;
	}

	if (!ctx->plane_zorder_prop) {
		prop = drm_property_create_range(dev, 0, "zpos", 0,
						ADE_CH_NUM - 1);
		if (!prop)
			return 0;

		ctx->plane_zorder_prop = prop;
	}

	/* attach rotation capability properties */
	switch(ch) {
	case ADE_CH5:
	case ADE_CH6:
		drm_object_attach_property(obj, ctx->cap_rot_prop,
					BIT(DRM_ROTATE_0) |
					BIT(DRM_ROTATE_90) |
					BIT(DRM_ROTATE_180) |
					BIT(DRM_ROTATE_270) |
					BIT(DRM_REFLECT_X) |
					BIT(DRM_REFLECT_Y));
		break;
	default:
		drm_object_attach_property(obj, ctx->cap_rot_prop, DRM_ROTATE_0);
		break;
	}

	/* attach scale capability properties */
	switch(ch) {
	case ADE_CH5:
	case ADE_CH6:
		drm_object_attach_property(obj, ctx->cap_scal_prop,
				ADE_CHANNEL_SCALE_SUPPORT);
		break;
	default:
		drm_object_attach_property(obj, ctx->cap_scal_prop,
				ADE_CHANNEL_SCALE_UNSUPPORT);
		break;
	}

	/* attach zorder properties */
	drm_object_attach_property(obj, ctx->plane_zorder_prop, plane->zorder);

	return 0;

}

static int hisi_ade_bind(struct device *dev, struct device *master,
							void *data)
{
	struct hisi_plane *plane;
	struct hisi_crtc *crtc;
	enum drm_plane_type type;
	struct ade_context *ctx = dev_get_drvdata(dev);
	const u32 *fmts;
	u32 fmts_cnt;
	int ret = 0;
	int i;
	
	ctx->dev = data;
	ctx->dev->dev = dev;
	
	/* plane init */
	for (i=0; i<ADE_CH_NUM; i++) {
		type = i == PRIMARY_CH ? DRM_PLANE_TYPE_PRIMARY :
			DRM_PLANE_TYPE_OVERLAY;
		fmts_cnt = ade_get_channel_formats(i, &fmts);
		plane = hisi_drm_plane_init(ctx->dev, ctx, i, fmts, fmts_cnt, type);
		if (IS_ERR(plane))
			return PTR_ERR(plane);

		ctx->plane[i] = plane;
	}

	crtc = hisi_drm_crtc_create(ctx->dev, ctx, &ctx->plane[PRIMARY_CH]->base);
	if (IS_ERR(crtc)) {
		DRM_ERROR("failed to crtc creat\n");
		return PTR_ERR(crtc);
	}

	/* ldi irq install */
	ret = drm_irq_install(ctx->dev, ctx->irq);
	if (ret) {
		DRM_ERROR("failed to install IRQ handler\n");
		return ret;
	}
	
	return ret;

}

static void hisi_ade_unbind(struct device *dev, struct device *master,
	void *data)
{
	/* do nothing */
}

static const struct component_ops hisi_ade_ops = {
	.bind	= hisi_ade_bind,
	.unbind	= hisi_ade_unbind,
};

static int hisi_ade_probe(struct platform_device *pdev)
{
	struct ade_context *ctx;
	int ret;

	DRM_DEBUG_DRIVER("enter.\n");

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		DRM_ERROR("failed to alloc the space\n");
		return -ENOMEM;
	}

	ret = hisi_drm_ade_dts_parse(pdev, ctx);
	if (ret) {
		DRM_ERROR("failed to dts parse\n");
		return ret;
	}
	
	platform_set_drvdata(pdev, ctx);
	
	return component_add(&pdev->dev, &hisi_ade_ops);
	
	DRM_DEBUG_DRIVER("drm_ade exit successfully.\n");
}

static int hisi_ade_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &hisi_ade_ops);
	return 0;
}

static struct of_device_id hisi_ade_of_match[] = {
	{ .compatible = "hisilicon,hi6220-ade" },
	{ }
};
MODULE_DEVICE_TABLE(of, hisi_ade_of_match);

static struct platform_driver hisi_ade_driver = {
	.probe = hisi_ade_probe,
	.remove = hisi_ade_remove,
	.driver = {
		   .name = "hisi-ade",
		   .owner = THIS_MODULE,
		   .of_match_table = hisi_ade_of_match,
	},
};

module_platform_driver(hisi_ade_driver);

MODULE_AUTHOR("Xinwei Kong <kong.kongxinwei@hisilicon.com>");
MODULE_AUTHOR("Xinliang Liu <z.liuxinliang@huawei.com>");
MODULE_DESCRIPTION("hisilicon SoC DRM driver");
MODULE_LICENSE("GPL v2");
