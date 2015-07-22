/*
 * Hisilicon Terminal SoCs drm driver
 *
 * Copyright (c) 2014-2015 Hisilicon Limited.
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <video/display_timing.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_atomic_helper.h>

#include "hisi_drm_fb.h"
#include "hisi_ade_reg.h"
#include "hisi_ldi_reg.h"
#include "hisi_drm_fbdev.h"
#include "hisi_drm_drv.h"
#include "hisi_drm_ade.h"

#define FORCE_PIXEL_CLOCK_SAME_OR_HIGHER 0

#define SC_MEDIA_RSTDIS		(0x530)
#define SC_MEDIA_RSTEN		(0x52C)

#define to_ade_crtc(c)		container_of(c, struct hisi_ade_crtc, base)
#define to_ade_plane(p)		container_of(p, struct hisi_ade_plane, base)

enum {
	LDI_TEST = 0,
	LDI_WORK
};

enum {
	LDI_ISR_FRAME_END_INT           = 0x2,
	LDI_ISR_UNDER_FLOW_INT          = 0x4
};

enum {
	ADE_ISR1_RES_SWITCH_CMPL         = 0x80000000
};

enum {
	LDI_DISP_MODE_NOT_3D_FBF = 0,
	LDI_DISP_MODE_3D_FBF
};

enum {
	ADE_RGB = 0,
	ADE_BGR
};
enum {
	ADE_DISABLE = 0,
	ADE_ENABLE
};

enum {
	ADE_OUT_RGB_565 = 0,
	ADE_OUT_RGB_666,
	ADE_OUT_RGB_888
};

/*
 * ADE read as big-endian, so revert the
 * rgb order described in the SoC datasheet
 * */
enum ADE_FORMAT {
	ADE_BGR_565,
	ADE_RGB_565,
	ADE_XBGR_8888,
	ADE_XRGB_8888,
	ADE_ABGR_8888,
	ADE_ARGB_8888,
	ADE_BGRA_8888,
	ADE_RGBA_8888,
	ADE_BGR_888,
	ADE_RGB_888,
	ADE_YUYV = 16,
	ADE_YVYU,
	ADE_UYVY,
	ADE_VYUY,
	ADE_YUV444,
	ADE_NV12,
	ADE_NV21,
	ADE_FORMAT_NOT_SUPPORT = 800
};

enum {
	TOP_DISP_CH_SRC_RDMA = 2
};

enum {
	ADE_ISR_DMA_ERROR               = 0x2000000
};

enum ade_channel {
	ADE_CH1 = 0,
	ADE_CH2,
	ADE_CH3,
	ADE_CH4,
	ADE_CH5,
	ADE_CH6,
	ADE_DISP,

	ADE_CH_NUM
};

struct hisi_ade_plane {
	struct drm_plane base;
	void *ctx;
	u32 ch;
};

struct hisi_ade_crtc {
	struct drm_crtc base;
	void *ctx;
	struct drm_display_mode *dmode;
	bool enable;
};

struct ade_context {
	void __iomem  *ade_base;
	void __iomem  *media_base;

	u32 ade_core_rate;
	u32 media_noc_rate;

	struct drm_device  *dev;

	struct clk *ade_core_clk;
	struct clk *media_noc_clk;
	struct clk *ade_pix_clk;

	struct hisi_ade_crtc *ade_crtc;
	struct hisi_ade_plane *plane[ADE_CH_NUM];

	bool power_on;
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
u32 hisi_get_ade_format(u32 pixel_format)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ade_formats); i++)
		if (ade_formats[i].pixel_format == pixel_format)
			return ade_formats[i].ade_format;

	/* not found, return rgb565 */
	DRM_ERROR("Not found pixel format!!fourcc_format= %d\n", pixel_format);
	return ADE_RGB_565;
}

static void ade_init(struct ade_context *ctx)
{
	void __iomem *ade_base = ctx->ade_base;

	/* enable clk gate */
	set_TOP_CTL_clk_gate_en(ade_base, 1);
	/* for video set to 1, means that ade registers
	 * became effective at frame end */
	set_TOP_CTL_frm_end_start(ade_base, 1);
}

static void ldi_init(struct ade_context *ctx, struct drm_display_mode *mode)
{
	void __iomem *ade_base = ctx->ade_base;
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

	writel((hbp << 20) | (hfp << 0), ade_base + LDI_HRZ_CTRL0_REG);
	/* p3-73 6220V100 pdf:
	 *  "The configured value is the actual width - 1"
	 */
	writel(hsw - 1, ade_base + LDI_HRZ_CTRL1_REG);
	writel((vbp << 20) | (vfp << 0), ade_base + LDI_VRT_CTRL0_REG);
	/* p3-74 6220V100 pdf:
	 *  "The configured value is the actual width - 1"
	 */
	writel(vsw - 1, ade_base + LDI_VRT_CTRL1_REG);

	/* p3-75 6220V100 pdf:
	 *  "The configured value is the actual width - 1"
	 */
	writel(((mode->vdisplay - 1) << 20) | ((mode->hdisplay - 1) << 0),
	       ade_base + LDI_DSP_SIZE_REG);
	writel(plr_flags, ade_base + LDI_PLR_CTRL_REG);

	/*
	 * other parameters setting
	 */
	writel(BIT(0), ade_base + LDI_WORK_MODE_REG);
	writel((0x3c << 6) | (ADE_OUT_RGB_888 << 3) | BIT(2) | BIT(0),
	       ade_base + LDI_CTRL_REG);

	set_reg(ade_base + LDI_DE_SPACE_LOW_REG, 0x1, 1, 1);
	/* dsi pixel on */
	set_reg(ade_base + LDI_HDMI_DSI_GT_REG, 0x0, 1, 0);
	wmb();
	set_LDI_CTRL_ldi_en(ade_base, ADE_ENABLE);
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
	ctx->power_on = true;
	return 0;
}

static void ade_power_down(struct ade_context *ctx)
{
	void __iomem *ade_base = ctx->ade_base;
	void __iomem *media_base = ctx->media_base;

	set_LDI_CTRL_ldi_en(ade_base, ADE_DISABLE);
	/* dsi pixel off */
	set_reg(ade_base + LDI_HDMI_DSI_GT_REG, 0x1, 1, 0);

	clk_disable_unprepare(ctx->ade_core_clk);
	writel(0x20, media_base + SC_MEDIA_RSTEN);
	clk_disable_unprepare(ctx->media_noc_clk);
	ctx->power_on = false;
}

static void hisi_update_disp_channel(struct ade_context *ctx,
				struct drm_framebuffer *fb, int x, int y)
{
	struct drm_gem_cma_object *obj = hisi_drm_fb_get_gem_obj(fb, 0);
	struct hisi_drm_fb *hisi_fb = to_hisi_drm_fb(fb);
	void __iomem *ade_base = ctx->ade_base;
	u32 stride;
	u32 display_addr;
	u32 offset;
	u32 fb_hight;
	u32 format;

	stride = fb->pitches[0];
	offset = y * fb->pitches[0] + x * (fb->bits_per_pixel >> 3);
	display_addr = (u32)obj->paddr + offset;
	fb_hight = hisi_fb->is_fbdev_fb ? fb->height / HISI_NUM_FRAMEBUFFERS
			: fb->height;
	format = hisi_get_ade_format(fb->pixel_format);

	DRM_DEBUG_DRIVER("enter stride=%d,paddr=0x%x,display_addr=0x%x,%dx%d\n",
			stride, (u32)obj->paddr, display_addr,
			fb->width, fb_hight);
	DRM_INFO("update sanout: pixel_format=%d(%s)\n",
			format, drm_get_format_name(fb->pixel_format));

	/* TOP setting */
	writel(0, ade_base + ADE_WDMA2_SRC_CFG_REG);
	writel(0, ade_base + ADE_SCL3_MUX_CFG_REG);
	writel(0, ade_base + ADE_SCL1_MUX_CFG_REG);
	writel(0, ade_base + ADE_ROT_SRC_CFG_REG);
	writel(0, ade_base + ADE_SCL2_SRC_CFG_REG);
	writel(0, ade_base + ADE_SEC_OVLY_SRC_CFG_REG);
	writel(0, ade_base + ADE_WDMA3_SRC_CFG_REG);
	writel(0, ade_base + ADE_OVLY1_TRANS_CFG_REG);
	writel(0, ade_base + ADE_CTRAN5_TRANS_CFG_REG);
	writel(0, ade_base + ADE_OVLY_CTL_REG);
	writel(0, ade_base + ADE_SOFT_RST_SEL0_REG);
	writel(0, ade_base + ADE_SOFT_RST_SEL1_REG);
	set_TOP_SOFT_RST_SEL0_disp_rdma(ade_base, 1);
	set_TOP_SOFT_RST_SEL0_ctran5(ade_base, 1);
	set_TOP_SOFT_RST_SEL0_ctran6(ade_base, 1);
	writel(0, ade_base + ADE_RELOAD_DIS0_REG);
	writel(0, ade_base + ADE_RELOAD_DIS1_REG);
	writel(TOP_DISP_CH_SRC_RDMA, ade_base + ADE_DISP_SRC_CFG_REG);

	/* DISP DMA setting */
	writel((format << 16) & 0x1f0000, ade_base + RD_CH_DISP_CTRL_REG);
	writel(display_addr, ade_base + RD_CH_DISP_ADDR_REG);
	writel((fb_hight << 16) | stride, ade_base + RD_CH_DISP_SIZE_REG);
	writel(stride, ade_base + RD_CH_DISP_STRIDE_REG);
	writel(fb_hight * stride, ade_base + RD_CH_DISP_SPACE_REG);
	writel(1, ade_base + RD_CH_DISP_EN_REG);

	/* ctran5 setting */
	writel(1, ade_base + ADE_CTRAN5_DIS_REG);
	writel(fb->width * fb_hight - 1,
		ade_base + ADE_CTRAN5_IMAGE_SIZE_REG);

	/* ctran6 setting */
	writel(1, ade_base + ADE_CTRAN6_DIS_REG);
	writel(fb->width * fb_hight - 1,
		ade_base + ADE_CTRAN6_IMAGE_SIZE_REG);

	DRM_INFO("ADE GO.\n");
	/* enable ade */
	wmb();
	writel(ADE_ENABLE, ade_base + ADE_EN_REG);
}

static void  hisi_ade_crtc_enable(struct drm_crtc *crtc)
{
	struct hisi_ade_crtc *ade_crtc = to_ade_crtc(crtc);
	struct ade_context *ctx = ade_crtc->ctx;
	int ret;

	DRM_DEBUG_DRIVER("enter.\n");
	if (ade_crtc->enable)
		return;

	if (!ctx->power_on) {
		ret = ade_power_up(ctx);
		if (ret) {
			DRM_ERROR("failed to initialize ade clk\n");
			return ;
		}
	}

	ade_init(ctx);
	ldi_init(ctx, ade_crtc->dmode);
	drm_crtc_vblank_on(crtc);

	ade_crtc->enable = true;
	DRM_DEBUG_DRIVER("exit success.\n");
}

static void hisi_ade_crtc_disable(struct drm_crtc *crtc)
{
	struct hisi_ade_crtc *ade_crtc = to_ade_crtc(crtc);
	struct ade_context *ctx = ade_crtc->ctx;

	DRM_DEBUG_DRIVER("enter.\n");
	if (!ade_crtc->enable)
		return;

	drm_crtc_vblank_off(crtc);
	ade_power_down(ctx);

	ade_crtc->enable = false;
	DRM_DEBUG_DRIVER("exit success.\n");
}

static bool hisi_drm_crtc_mode_fixup(struct drm_crtc *crtc,
				      const struct drm_display_mode *mode,
				      struct drm_display_mode *adj_mode)
{
	struct hisi_ade_crtc *ade_crtc = to_ade_crtc(crtc);
	struct ade_context *ctx = ade_crtc->ctx;
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

static void hisi_drm_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct hisi_ade_crtc *ade_crtc = to_ade_crtc(crtc);

	DRM_DEBUG_DRIVER("enter.\n");
	ade_crtc->dmode = &crtc->state->mode;
	DRM_DEBUG_DRIVER("exit success.\n");
}

static void hisi_crtc_atomic_begin(struct drm_crtc *crtc)
{
	DRM_DEBUG_DRIVER("enter.\n");
	DRM_DEBUG_DRIVER("exit success.\n");
}

static void hisi_crtc_atomic_flush(struct drm_crtc *crtc)
{
/*	struct hisi_ade_crtc *ade_crtc = to_ade_crtc(crtc);
	struct ade_context *ctx = ade_crtc->ctx;
	void __iomem *ade_base = ctx->ade_base; */

	DRM_DEBUG_DRIVER("enter.\n");
	DRM_DEBUG_DRIVER("exit success.\n");
}

static void hisi_drm_crtc_mode_prepare(struct drm_crtc *crtc)
{
	struct hisi_ade_crtc *ade_crtc = to_ade_crtc(crtc);
	struct ade_context *ctx = ade_crtc->ctx;

	DRM_DEBUG_DRIVER("enter.\n");
	if (!ctx->power_on)
		(void) ade_power_up(ctx);
	DRM_DEBUG_DRIVER("exit success.\n");
}

static const struct drm_crtc_helper_funcs crtc_helper_funcs = {
	.enable		= hisi_ade_crtc_enable,
	.disable	= hisi_ade_crtc_disable,
	.prepare	= hisi_drm_crtc_mode_prepare,
	.mode_fixup	= hisi_drm_crtc_mode_fixup,
	.mode_set_nofb	= hisi_drm_crtc_mode_set_nofb,
	.atomic_begin	= hisi_crtc_atomic_begin,
	.atomic_flush	= hisi_crtc_atomic_flush,
};

static void hisi_drm_crtc_destroy(struct drm_crtc *crtc)
{
	struct hisi_ade_crtc *ade_crtc = to_ade_crtc(crtc);

	drm_crtc_cleanup(crtc);
	devm_kfree(crtc->dev->dev, ade_crtc);
}

static const struct drm_crtc_funcs crtc_funcs = {
	.destroy	= hisi_drm_crtc_destroy,
	.set_config	= drm_atomic_helper_set_config,
	.page_flip	= drm_atomic_helper_page_flip,
	.reset		= drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
};

static struct hisi_ade_crtc *hisi_drm_crtc_create(struct drm_device  *dev,
						  struct ade_context *ctx,
						  struct drm_plane *plane)
{
	struct hisi_ade_crtc *crtc;
	struct hisi_drm_private *private = dev->dev_private;
	int ret;

	crtc = devm_kzalloc(dev->dev, sizeof(*crtc), GFP_KERNEL);
	if (!crtc)
		return ERR_PTR(-ENOMEM);

	ret = drm_crtc_init_with_planes(dev, &crtc->base, plane, 
					NULL, &crtc_funcs);
	if (ret)
		return ERR_PTR(ret);

	drm_crtc_helper_add(&crtc->base, &crtc_helper_funcs);

	private->crtc = &crtc->base;
	crtc->ctx = ctx;
	return crtc;
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
	ctx->ade_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ctx->ade_base)) {
		DRM_ERROR("failed to remap io region0\n");
		ret = PTR_ERR(ctx->ade_base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "media_base");
	ctx->media_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ctx->media_base)) {
		DRM_ERROR("failed to remap io region1\n");
		ret = PTR_ERR(ctx->media_base);
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

	return ret;
}

int hisi_drm_enable_vblank(struct drm_device *dev, int crtc)
{
	struct hisi_drm_private *private = dev->dev_private;
	struct hisi_ade_crtc *ade_crtc = to_ade_crtc(private->crtc);
	struct ade_context *ctx = ade_crtc->ctx;
	void __iomem *ade_base = ctx->ade_base;
	u32 intr_en;

	DRM_INFO("enable_vblank enter.\n");
	if (!ctx->power_on)
		(void) ade_power_up(ctx);

	intr_en = readl(ade_base + LDI_INT_EN_REG);
	intr_en |= LDI_ISR_FRAME_END_INT;
	writel(intr_en, ade_base + LDI_INT_EN_REG);
	return 0;
}

void hisi_drm_disable_vblank(struct drm_device *dev, int crtc)
{
	struct hisi_drm_private *private = dev->dev_private;
	struct hisi_ade_crtc *ade_crtc = to_ade_crtc(private->crtc);
	struct ade_context *ctx = ade_crtc->ctx;
	void __iomem *ade_base = ctx->ade_base;
	u32 intr_en;

	DRM_INFO("disable_vblank enter.\n");
	if (!ctx->power_on) {
		DRM_ERROR("power is down! vblank disable fail\n");
		return ;
	}
	intr_en = readl(ade_base + LDI_INT_EN_REG);
	intr_en &= ~LDI_ISR_FRAME_END_INT;
	writel(intr_en, ade_base + LDI_INT_EN_REG);
}

irqreturn_t hisi_drm_irq_handler(int irq, void *arg)
{
	struct drm_device *dev = (struct drm_device *) arg;
	struct hisi_drm_private *private = dev->dev_private;
	struct hisi_ade_crtc *ade_crtc = to_ade_crtc(private->crtc);
	struct ade_context *ctx = ade_crtc->ctx;
	void __iomem *ade_base = ctx->ade_base;
	u32 status;

	status = readl(ade_base + LDI_MSK_INT_REG);
	/* DRM_INFO("LDI IRQ: status=0x%X\n",status); */

	/* vblank irq */
	if (status & LDI_ISR_FRAME_END_INT) {
		writel(LDI_ISR_FRAME_END_INT, ade_base + LDI_INT_CLR_REG);
		drm_handle_vblank(dev, drm_crtc_index(&ade_crtc->base));
	}

	return IRQ_HANDLED;
}



void hisi_update_channel(struct drm_plane *plane, struct drm_crtc *crtc,
			  struct drm_framebuffer *fb, int crtc_x, int crtc_y,
			  unsigned int crtc_w, unsigned int crtc_h,
			  uint32_t src_x, uint32_t src_y,
			  uint32_t src_w, uint32_t src_h)
{
	struct hisi_ade_plane *ade_plane = to_ade_plane(plane);
	struct ade_context *ctx = ade_plane->ctx;

	DRM_DEBUG_DRIVER("enter, channel=%d\n", ade_plane->ch);
	switch (ade_plane->ch) {
	case ADE_DISP:
		hisi_update_disp_channel(ctx, fb, 0, 0);
		break;
	default:
		break;
	}

	DRM_DEBUG_DRIVER("exit success.\n");
}

static void hisi_plane_destroy(struct drm_plane *plane)
{
	struct hisi_ade_plane *ade_plane = to_ade_plane(plane);

	drm_plane_cleanup(plane);
	devm_kfree(plane->dev->dev, ade_plane);
}

static struct drm_plane_funcs hisi_plane_funcs = {
	.update_plane	= drm_atomic_helper_update_plane,
	.disable_plane	= drm_atomic_helper_disable_plane,
	.set_property = drm_atomic_helper_plane_set_property,
	.destroy	= hisi_plane_destroy,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

int hisi_plane_prepare_fb(struct drm_plane *plane,
				struct drm_framebuffer *fb,
				const struct drm_plane_state *new_state)
{
	DRM_DEBUG_DRIVER("enter.\n");
	DRM_DEBUG_DRIVER("exit success.\n");
	return 0;
}

void hisi_plane_cleanup_fb(struct drm_plane *plane,
				struct drm_framebuffer *fb,
				const struct drm_plane_state *old_state)
{
	DRM_DEBUG_DRIVER("enter.\n");
	DRM_DEBUG_DRIVER("exit success.\n");
}

static int hisi_plane_atomic_check(struct drm_plane *plane,
				     struct drm_plane_state *state)
{
	DRM_DEBUG_DRIVER("enter.\n");
	DRM_DEBUG_DRIVER("exit success.\n");
	return 0;
}

static void hisi_plane_atomic_update(struct drm_plane *plane,
				       struct drm_plane_state *old_state)
{
	struct drm_plane_state	*state	= plane->state;

	DRM_DEBUG_DRIVER("enter.\n");
	if (!state->crtc)
		return;

	hisi_update_channel(plane, state->crtc, state->fb,
		      state->crtc_x, state->crtc_y,
		      state->crtc_w, state->crtc_h,
		      state->src_x >> 16, state->src_y >> 16,
		      state->src_w >> 16, state->src_h >> 16);

	DRM_DEBUG_DRIVER("exit success.\n");
}

void hisi_plane_atomic_disable(struct drm_plane *plane,
			       struct drm_plane_state *old_state)
{
	DRM_DEBUG_DRIVER("enter.\n");
	DRM_DEBUG_DRIVER("exit success.\n");
}

static const struct drm_plane_helper_funcs hisi_plane_helper_funcs = {
	.prepare_fb = hisi_plane_prepare_fb,
	.cleanup_fb = hisi_plane_cleanup_fb,
	.atomic_check = hisi_plane_atomic_check,
	.atomic_update = hisi_plane_atomic_update,
	.atomic_disable = hisi_plane_atomic_disable,
};

struct hisi_ade_plane *hisi_drm_plane_init(struct drm_device *dev,
					   void *ctx,
					   u32 ch,
					   const u32 *formats,
					   u32 formats_cnt,
					   enum drm_plane_type type)
{
	struct hisi_ade_plane *plane;
	int ret;

	plane = devm_kzalloc(dev->dev, sizeof(*plane), GFP_KERNEL);
	if (!plane)
		return ERR_PTR(-ENOMEM);
	plane->ctx = ctx;
	plane->ch = ch;

	DRM_DEBUG_DRIVER("plane init: ch=%d,type=%d, formats_count=%d\n",
			ch, type, formats_cnt);
	ret = drm_universal_plane_init(dev, &plane->base, 1,
					&hisi_plane_funcs,
					formats,
					formats_cnt,
					type);
	if (ret) {
		DRM_ERROR("fail to init plane\n");
		return ERR_PTR(ret);
	}

	drm_plane_helper_add(&plane->base, &hisi_plane_helper_funcs);

	/* TODO: set property */

	return plane;
}

static int hisi_ade_probe(struct platform_device *pdev)
{
	struct ade_context *ctx;
	struct hisi_ade_plane *plane;
	struct hisi_ade_crtc *crtc;
	enum drm_plane_type type;
	struct drm_device  *dev;
	const u32 *fmts;
	u32 fmts_cnt;
	int ret;
	int i;

	DRM_DEBUG_DRIVER("enter.\n");
	dev = dev_get_platdata(&pdev->dev);
	if (!dev) {
		DRM_ERROR("no platform data\n");
		return -EINVAL;
	}


	ctx = devm_kzalloc(dev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		DRM_ERROR("failed to alloc the space\n");
		return -ENOMEM;
	}

	ret = hisi_drm_ade_dts_parse(pdev, ctx);
	if (ret) {
		DRM_ERROR("failed to dts parse\n");
		return ret;
	}

	/* plane init */
	for (i=0; i<ADE_CH_NUM; i++) {
		type = i == ADE_DISP ? DRM_PLANE_TYPE_PRIMARY :
			DRM_PLANE_TYPE_OVERLAY;
		fmts_cnt = ade_get_channel_formats(i, &fmts);
		plane = hisi_drm_plane_init(dev, ctx, i, fmts, fmts_cnt, type);
		if (IS_ERR(plane))
			return PTR_ERR(plane);

		ctx->plane[i] = plane;
	}

	crtc = hisi_drm_crtc_create(dev, ctx,
				    &ctx->plane[ADE_DISP]->base);
	if (IS_ERR(crtc)) {
		DRM_ERROR("failed to crtc creat\n");
		return PTR_ERR(crtc);
	}

	/* ldi irq install */
	ret = drm_irq_install(dev, platform_get_irq(pdev, 0));
	if (ret) {
		DRM_ERROR("failed to install IRQ handler\n");
		return ret;
	}

	ctx->dev = dev;
	ctx->ade_crtc = crtc;

	DRM_DEBUG_DRIVER("drm_ade exit successfully.\n");

	return 0;
}

static int hisi_ade_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id hisi_ade_of_match[] = {
	{ .compatible = "hisilicon,hi6220-ade" },
	{ }
};
MODULE_DEVICE_TABLE(of, hisi_ade_of_match);

static struct platform_driver ade_driver = {
	.remove = hisi_ade_remove,
	.driver = {
		   .name = "hisi-ade",
		   .owner = THIS_MODULE,
		   .of_match_table = hisi_ade_of_match,
	},
};

module_platform_driver_probe(ade_driver, hisi_ade_probe);
