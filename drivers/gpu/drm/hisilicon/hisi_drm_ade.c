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

#include <linux/hisi_ion.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>

#include "hisi_drm_fb.h"
#include "hisi_ade_reg.h"
#include "hisi_ldi_reg.h"
#include "hisi_drm_ade.h"
#include "hisi_ade_cmdqueue.h"

#define SC_MEDIA_RSTDIS		(0x530)
#define SC_MEDIA_RSTEN		(0x52C)
#define ADE_CMD_BUFF_SIZE_MAX   (ALIGN((SZ_1M), 8<<10))
//#define ADE_CMD_BUFF_SIZE_MAX   100000
#define IOMMU_PAGE_SIZE		(4<<10)
#define ADE_ONLINE_PE		0xf000004
#define ADE_INVAL_PATTERN_NUM   (0xffff) 

enum {
	LDI_TEST = 0,
	LDI_WORK
};

enum {
	LDI_ISR_FRAME_END_INT           = 0x2,
	LDI_ISR_UNDER_FLOW_INT          = 0x4
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

enum ADE_FORMAT {
	ADE_RGB_565 = 0,
	ADE_BGR_565,
	ADE_XRGB_8888,
	ADE_XBGR_8888,
	ADE_ARGB_8888,
	ADE_ABGR_8888,
	ADE_RGBA_8888,
	ADE_BGRA_8888,
	ADE_RGB_888,
	ADE_BGR_888 = 9,
	ADE_YUYV_I = 16,
	ADE_YVYU_I,
	ADE_UYVY_I,
	ADE_VYUY_I,
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


struct hisi_drm_ade_crtc {

	int dpms;
	int ade_irq;
	int res_switch_cmpl;
	u8 __iomem  *ade_base;
	u8 __iomem  *media_base;
	u32 ade_core_rate;
	u32 media_noc_rate;
	u32 x , y;
	u32 frame_count;

	struct drm_device  *drm_dev;
	struct drm_crtc    crtc;
	struct drm_display_mode *dmode;

	struct clk *ade_core_clk;
	struct clk *media_noc_clk;
	struct clk *ade_pix_clk;

	struct semaphore sem;
	struct cmdfile_buffer   *cf;
	struct ion_client *ion_client;
	wait_queue_head_t wait_res_cmpl;
};

struct cmdfile_buffer{                                                                                             
	void    *vaddr;
	u32     paddr;
	u32     cmd_len;
	size_t     buff_size;
	struct ion_handle *cmd_ion_handle;
};


static int hisi_drm_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
					struct drm_framebuffer *old_fb);
static void ldi_init(struct hisi_drm_ade_crtc *crtc_ade);

static void ade_init(struct hisi_drm_ade_crtc *crtc_ade)
{
	u8 __iomem *ade_base = crtc_ade->ade_base;
	u32    cpu0_mask;
	u32    cpu1_mask;

	cpu0_mask = ADE_ISR_DMA_ERROR;
	cpu1_mask = ADE_ISR1_RES_SWITCH_CMPL;

	writel(cpu0_mask, ade_base + INTR_MASK_CPU0_REG);
	writel(cpu1_mask, ade_base + INTR_MASK_CPU1_REG);
	set_TOP_CTL_frm_end_start(ade_base, 2);

	/* disable wdma2 and wdma3 frame discard */
	writel(0x0, ade_base + ADE_FRM_DISGARD_CTRL_REG);

	writel(0, ade_base + ADE_SOFT_RST_SEL0_REG);
	writel(0, ade_base + ADE_SOFT_RST_SEL1_REG);
	writel(0, ade_base + ADE_RELOAD_DIS0_REG);
	writel(0, ade_base + ADE_RELOAD_DIS1_REG);

	/* enable clk gate */
	set_TOP_CTL_clk_gate_en(ade_base, 1);

	/* init ovly ctrl, if not,when the first
	* frame is hybrid, will happen
	*/
	writel(0, ade_base + ADE_OVLY_CTL_REG);

	/* TODO:init scl coeff */
}

static int ade_power_up(struct hisi_drm_ade_crtc *crtc_ade)
{
	u8 __iomem *media_base = crtc_ade->media_base;
	int ret;

	ret = clk_set_rate(crtc_ade->ade_core_clk, crtc_ade->ade_core_rate);
	if (ret) {
		DRM_ERROR("clk_set_rate ade_core_rate error\n");
		return ret;
	}
	ret = clk_set_rate(crtc_ade->media_noc_clk, crtc_ade->media_noc_rate);
	if (ret) {
		DRM_ERROR("media_noc_clk media_noc_rate error\n");
		return ret;
	}
	ret = clk_prepare_enable(crtc_ade->media_noc_clk);
	if (ret) {
		DRM_ERROR("fail to clk_prepare_enable media_noc_clk\n");
		return ret;
	}

	writel(0x20, media_base + SC_MEDIA_RSTDIS);

	ret = clk_prepare_enable(crtc_ade->ade_core_clk);
	if (ret) {
		DRM_ERROR("fail to clk_prepare_enable ade_core_clk\n");
		return ret;
	}
	return 0;
}

static int ade_power_down(struct hisi_drm_ade_crtc *crtc_ade)
{
	u8 __iomem *media_base = crtc_ade->media_base;

	clk_disable_unprepare(crtc_ade->ade_core_clk);
	writel(0x20, media_base + SC_MEDIA_RSTEN);
	clk_disable_unprepare(crtc_ade->media_noc_clk);
	return 0;
}

static int hisi_drm_crtc_ade_enable(struct hisi_drm_ade_crtc *crtc_ade)
{
	int ret;

	ret = ade_power_up(crtc_ade);
	if (ret) {
		DRM_ERROR("failed to initialize ade clk\n");
		return ret;
	}

	ade_init(crtc_ade);
	ldi_init(crtc_ade);
	if (crtc_ade->crtc.primary->fb)
		hisi_drm_crtc_mode_set_base(&crtc_ade->crtc, 0, 0, NULL);
	return 0;
}

static int hisi_drm_crtc_ade_disable(struct hisi_drm_ade_crtc *crtc_ade)
{
	int ret;
	u8 __iomem *ade_base = crtc_ade->ade_base;

	set_LDI_CTRL_ldi_en(ade_base, ADE_DISABLE);
	/* dsi pixel off */
	set_reg(ade_base + LDI_HDMI_DSI_GT_REG, 0x1, 1, 0);
	ret = ade_power_down(crtc_ade);
	if (ret) {
		DRM_ERROR("failed to initialize ade clk\n");
		return ret;
	}

	return 0;
}

static void ldi_init(struct hisi_drm_ade_crtc *crtc_ade)
{
	int ret;
	u32 hfront_porch, hback_porch, hsync_len;
	u32 vfront_porch, vback_porch, vsync_len;
	u32 plr_flags;
	u32 ldi_mask;
	struct drm_display_mode *mode = crtc_ade->dmode;
	u8 __iomem *ade_base = crtc_ade->ade_base;

	/*
	 * Timing setting
	 */
	plr_flags = (mode->flags & DRM_MODE_FLAG_NVSYNC)
			? HISI_LDI_FLAG_NVSYNC : 0;
	plr_flags |= (mode->flags & DRM_MODE_FLAG_NHSYNC)
			? HISI_LDI_FLAG_NHSYNC : 0;
	hfront_porch = mode->hsync_start - mode->hdisplay;
	hback_porch = mode->htotal - mode->hsync_end;
	hsync_len = mode->hsync_end - mode->hsync_start;
	vfront_porch = mode->vsync_start - mode->vdisplay;
	vback_porch  = mode->vtotal - mode->vsync_end;
	vsync_len  = mode->vsync_end - mode->vsync_start;
	if (vsync_len > 15)
		vsync_len = 15;

	set_LDI_HRZ_CTRL0(ade_base, hfront_porch, hback_porch);
	set_LDI_HRZ_CTRL1_hsw(ade_base, hsync_len);
	set_LDI_VRT_CTRL0(ade_base, vfront_porch, vback_porch);
	set_LDI_VRT_CTRL1_vsw(ade_base, vsync_len);
	writel(plr_flags, ade_base + LDI_PLR_CTRL_REG);
	set_LDI_DSP_SIZE_size(ade_base, mode->hdisplay, mode->vdisplay);
	ret = clk_set_rate(crtc_ade->ade_pix_clk, mode->clock * 1000);
	if (ret) {
		DRM_ERROR("set ade_pixel_clk_rate fail\n");
		return;
	}

	/*
	 * other parameters setting
	 */
	set_LDI_WORK_MODE_work_mode(ade_base, LDI_WORK);
	set_LDI_WORK_MODE_colorbar_en(ade_base, ADE_DISABLE);
	ldi_mask = LDI_ISR_FRAME_END_INT | LDI_ISR_UNDER_FLOW_INT;
	writel(ldi_mask, ade_base + LDI_INT_EN_REG);

	set_LDI_CTRL_bgr(ade_base, ADE_RGB);
	set_LDI_CTRL_bpp(ade_base, ADE_OUT_RGB_888);
	set_LDI_CTRL_disp_mode(ade_base, LDI_DISP_MODE_NOT_3D_FBF);
	set_LDI_CTRL_corlorbar_width(ade_base, 0x3C);
	writel(0xFFFFFFFF, ade_base + LDI_INT_CLR_REG);
	set_reg(ade_base + LDI_DE_SPACE_LOW_REG, 0x1, 1, 1);
	/* dsi pixel on */
	set_reg(ade_base + LDI_HDMI_DSI_GT_REG, 0x0, 1, 0);
}

/* -----------------------------------------------------------------------------
 * CRTC
 */

#define to_hisi_crtc(c)  container_of(c, struct hisi_drm_ade_crtc, crtc)

static void hisi_drm_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct hisi_drm_ade_crtc *crtc_ade = to_hisi_crtc(crtc);

	DRM_DEBUG_DRIVER("crtc_dpms  enter successfully.\n");
	if (crtc_ade->dpms == mode)
		return;

	if (mode == DRM_MODE_DPMS_ON)
		hisi_drm_crtc_ade_enable(crtc_ade);
	else
		hisi_drm_crtc_ade_disable(crtc_ade);

	crtc_ade->dpms = mode;
	DRM_DEBUG_DRIVER("crtc_dpms exit successfully.\n");
}

static bool hisi_drm_crtc_mode_fixup(struct drm_crtc *crtc,
				      const struct drm_display_mode *mode,
				      struct drm_display_mode *adjusted_mode)
{
	DRM_DEBUG_DRIVER("mode_fixup  enter successfully.\n");
	DRM_DEBUG_DRIVER("mode_fixup  exit successfully.\n");
	return true;
}

static void hisi_drm_crtc_mode_prepare(struct drm_crtc *crtc)
{
	DRM_DEBUG_DRIVER(" enter successfully.\n");
	DRM_DEBUG_DRIVER(" exit successfully.\n");
}

static int hisi_drm_crtc_mode_set(struct drm_crtc *crtc,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode,
				   int x, int y,
				   struct drm_framebuffer *old_fb)
{
	struct hisi_drm_ade_crtc *crtc_ade = to_hisi_crtc(crtc);

	DRM_DEBUG_DRIVER("mode_set  enter successfully.\n");
	crtc_ade->dmode = mode;
	DRM_DEBUG_DRIVER("mode_set  exit successfully.\n");
	return 0;
}

static void hisi_drm_crtc_mode_commit(struct drm_crtc *crtc)
{

	DRM_DEBUG_DRIVER("mode_commit enter successfully.\n");
	DRM_DEBUG_DRIVER("mode_commit exit successfully.\n");
}

static void ade_overlay_commit_set_online_cmdq(u8* ade_base, struct cmdfile_buffer  *cf_buff, u32 pattern_num)
{
	DRM_DEBUG_DRIVER("ade_overlay_commit_set_online_cmdq enter succ, paddr=0x%x \n", cf_buff->paddr);

	/* cfg reg to ADE TOP REG */
	set_CMDQ_RDMA1_PE(ade_base, ADE_ONLINE_PE);
	set_CMDQ_RDMA1_CTRL(ade_base, 0);
	set_CMDQ_RDMA1_ADDR(ade_base, cf_buff->paddr);
	set_CMDQ_RDMA1_LEN(ade_base, cf_buff->cmd_len / ADE_CMD_ALIGN_BYTE_LEN);
	
	DRM_DEBUG_DRIVER("ade_overlay_commit_set_online_cmdq exit succ, paddr=0x%x \n", cf_buff->paddr);
}
static int hisi_drm_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
					struct drm_framebuffer *old_fb)
{
	struct hisi_drm_ade_crtc *crtc_ade = to_hisi_crtc(crtc);
	struct drm_framebuffer *fb = crtc->primary->fb;
	struct drm_gem_cma_object *obj = hisi_drm_fb_get_gem_obj(fb, 0);
	struct hisi_drm_fb *hisi_fb = to_hisi_drm_fb(fb);

	struct cmdfile_buffer *cf_buff = crtc_ade->cf;
	
	u8 __iomem *ade_base;
	u32 stride;
	u32 display_addr;
	u32 offset;
	u32 fb_hight;

	ade_base = crtc_ade->ade_base;
	stride = fb->pitches[0];
	offset = y * fb->pitches[0] + x * (fb->bits_per_pixel >> 3);
	display_addr = (u32)obj->paddr + offset;
	fb_hight = hisi_fb->is_fbdev_fb ? fb->height / HISI_NUM_FRAMEBUFFERS
			: fb->height;

	DRM_DEBUG_DRIVER("enter stride=%d,paddr=0x%x,display_addr=0x%x,%dx%d\n",
			stride, (u32)obj->paddr, display_addr,
			fb->width, fb_hight);
down(&crtc_ade->sem);
	
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
	
	set_TOP_DISP_SRC_CFG(ade_base, TOP_DISP_CH_SRC_RDMA);
	set_TOP_SOFT_RST_SEL0(ade_base, 0);
	set_TOP_RELOAD_DIS0(ade_base, 0);
	set_TOP_SOFT_RST_SEL1(ade_base, 0);
	set_TOP_RELOAD_DIS1(ade_base, 0); 

	/* set offline axi_mux */
	writel(0x763f, ade_base + ADE_DMA_AXI_MUX_REG);

	set_TOP_SOFT_RST_SEL0_cmdq1_rdma(ade_base, 1);
	set_TOP_RELOAD_DIS0_cmdq1_rdma(ade_base, 0); 

	/********************************DISP DMA**************************/

	/* set disp channel into AXI0 */
	set_TOP_DMA_AXI_MUX(ade_base, OVERLAY_PIPE_ADE_DISP, OVERLAY_PIPE_TYPE_ONLINE);

	ade_cmdq_wr_rdma_pe_cmd(RD_CH_DISP_PE_REG, OVERLAY_PIPE_TYPE_ONLINE, ADE_ROT_NOP);
	/* DISP DMA setting */
	if (16 == fb->bits_per_pixel)
		ade_cmdq_wr_cmd(RD_CH_DISP_CTRL_REG, (ADE_RGB_565 << 16) & 0x1f0000);
	else if (32 == fb->bits_per_pixel)
		ade_cmdq_wr_cmd(RD_CH_DISP_CTRL_REG, (ADE_ABGR_8888 << 16) & 0x1f0000);
	ade_cmdq_wr_cmd(RD_CH_DISP_ADDR_REG, display_addr);

	ade_cmdq_wr_cmd(RD_CH_DISP_SIZE_REG, (fb_hight << 16) | stride);
	ade_cmdq_wr_cmd(RD_CH_DISP_STRIDE_REG, stride);
	ade_cmdq_wr_cmd(RD_CH_DISP_SPACE_REG, fb_hight * stride);

	ade_cmdq_wr_cmd2buff(cf_buff->vaddr, &(cf_buff->cmd_len));
	ade_cmdq_wr_cmd(RD_CH_DISP_EN_REG, 1);
	ade_cmdq_wr_cmd2buff(cf_buff->vaddr, &(cf_buff->cmd_len));

	/************************ ctran5 *************/ 
	ade_cmdq_wr_cmd(ADE_CTRAN5_DIS_REG, ADE_ENABLE);
	ade_cmdq_wr_cmd2buff(cf_buff->vaddr, &(cf_buff->cmd_len));
	ade_cmdq_wr_cmd(ADE_CTRAN5_IMAGE_SIZE_REG, fb->width * fb_hight - 1);
	ade_cmdq_wr_cmd(ADE_CTRAN5_CFG_OK_REG, 1);
	ade_cmdq_wr_cmd2buff(cf_buff->vaddr, &(cf_buff->cmd_len));

	/**************************CTRAN6***************************/
	ade_cmdq_wr_cmd(ADE_CTRAN6_DIS_REG, 1);                                                                   
	ade_cmdq_wr_cmd2buff(cf_buff->vaddr, &(cf_buff->cmd_len));
	ade_cmdq_wr_cmd(ADE_CTRAN6_IMAGE_SIZE_REG, fb->width * fb_hight - 1);
	ade_cmdq_wr_cmd(ADE_CTRAN6_CFG_OK_REG, 1);
	ade_cmdq_wr_cmd2buff(cf_buff->vaddr, &(cf_buff->cmd_len));

	ade_cmdq_wr_eof_cmd(cf_buff->vaddr, &(cf_buff->cmd_len));

	mb();
	
	/* set ONLINE cmdQueue RDMA, 0xffff is invalid value */
	ade_overlay_commit_set_online_cmdq(ade_base, cf_buff, ADE_INVAL_PATTERN_NUM);

	set_TOP_INTR_MASK_CPU1(ade_base, ADE_ISR1_RES_SWITCH_CMPL);

	crtc_ade->res_switch_cmpl = 0;
	writel(ADE_ENABLE, ade_base + ADE_EN_REG);
	printk("kkkkkkkkkkkkkXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");

	if (wait_event_interruptible_timeout(crtc_ade->wait_res_cmpl,
					crtc_ade->res_switch_cmpl, HZ) <= 0) {
	printk("kkkkkkkkkkkkk000000000display:wait_event_interruptible_timeout res_switch_cmpl timeout!\n");
	
		DRM_DEBUG_DRIVER("display:wait_event_interruptible_timeout res_switch_cmpl timeout!\n");
	}

	set_TOP_CTL_frm_end_start(ade_base, 1);
	set_LDI_CTRL_ldi_en(ade_base, ADE_ENABLE);

up(&crtc_ade->sem);
	
	DRM_DEBUG_DRIVER("mode_set_base exit successfully.\n");
	return 0;
}

static const struct drm_crtc_helper_funcs crtc_helper_funcs = {
	.dpms = hisi_drm_crtc_dpms,
	.mode_fixup = hisi_drm_crtc_mode_fixup,
	.prepare = hisi_drm_crtc_mode_prepare,
	.commit = hisi_drm_crtc_mode_commit,
	.mode_set = hisi_drm_crtc_mode_set,
	.mode_set_base = hisi_drm_crtc_mode_set_base,
};

static int hisi_drm_crtc_page_flip(struct drm_crtc *crtc,
				    struct drm_framebuffer *fb,
				    struct drm_pending_vblank_event *event,
				    uint32_t page_flip_flags)
{
	struct drm_framebuffer *old_fb;
	int ret;

	DRM_DEBUG_DRIVER("page_flip enter successfully.\n");

	old_fb = crtc->primary->fb;
	crtc->primary->fb = fb;

	ret = hisi_drm_crtc_mode_set_base(crtc, crtc->x, crtc->y, old_fb);
	if (ret) {
		DRM_ERROR("failed to hisi drm crtc mode set base\n");
		return ret;
	}

	DRM_DEBUG_DRIVER("page_flip exit successfully.\n");

	return 0;
}

static const struct drm_crtc_funcs crtc_funcs = {
	.destroy = drm_crtc_cleanup,
	.set_config = drm_crtc_helper_set_config,
	.page_flip = hisi_drm_crtc_page_flip,
};

static int hisi_drm_crtc_create(struct hisi_drm_ade_crtc *crtc_ade)
{
	struct drm_crtc *crtc = &crtc_ade->crtc;
	int ret;

	crtc_ade->dpms = DRM_MODE_DPMS_OFF;
	ret = drm_crtc_init(crtc_ade->drm_dev, crtc, &crtc_funcs);
	if (ret < 0)
		return ret;

	drm_crtc_helper_add(crtc, &crtc_helper_funcs);

	return 0;
}

static int hisi_drm_ade_cmdfile_buff_init(struct platform_device *pdev,
					struct hisi_drm_ade_crtc *crtc_ade)
{
	struct cmdfile_buffer *cf_buff;
	struct ion_client *ade_ion_client;
	struct ion_device *ion_dev = NULL;
	
	u32     heap_mask = 0;
	u32     heap_flag = 0;

	cf_buff = devm_kzalloc(&pdev->dev, sizeof(*cf_buff), GFP_KERNEL);
	cf_buff->buff_size = ADE_CMD_BUFF_SIZE_MAX;
	cf_buff->cmd_len   = 0;
	
	ion_dev = get_ion_device();
	ade_ion_client = ion_client_create(ion_dev, "fb_ion");
	if (ade_ion_client == NULL) {
		DRM_ERROR("create ion client create fail \n");
		return -EINVAL;
	}
	
	/* alloc un-cache memory for cmdfile,
	 * if heap_flag = ION_FLAG_CACHED | ION_FLAG_CACHED_NEEDS_SYNC;
	* memory will be cache.
	*/

	heap_mask = ION_HEAP(ION_OVERLAY_HEAP_ID);
	heap_flag = 0;

	/* get cmd buffer handle,  */
	cf_buff->cmd_ion_handle = ion_alloc(ade_ion_client,
		cf_buff->buff_size, IOMMU_PAGE_SIZE, heap_mask, heap_flag);
	if (IS_ERR(cf_buff->cmd_ion_handle)) {
		cf_buff->cmd_ion_handle = NULL;
		DRM_ERROR("ion_alloc alloc handle error \n");
		return -EINVAL;
	}

	/* get cmd buffer phy addr, io addr for IP */
	if (ion_phys(ade_ion_client, cf_buff->cmd_ion_handle,
		(ion_phys_addr_t *)(&cf_buff->paddr), &cf_buff->buff_size) < 0) {
		DRM_ERROR("ion_alloc alloc handle error \n");
		cf_buff->paddr = 0xdeadbeaf;
		return -EINVAL;
	}

	/* get cmf buffer virturl addr */
	cf_buff->vaddr = ion_map_kernel(ade_ion_client, cf_buff->cmd_ion_handle);
	if (IS_ERR(cf_buff->vaddr)) {
		DRM_ERROR("ion_alloc alloc handle error \n");
		return -EINVAL;
	}
	
	crtc_ade->cf = cf_buff;
	crtc_ade->ion_client = ade_ion_client;
	
	return 0;
}

static irqreturn_t hisi_ade_irq_thread(int irq, void *dev)
{
	struct hisi_drm_ade_crtc *crtc_ade = dev;
	u8 __iomem *ade_base;
	u32 temp = 0;
	
	ade_base = crtc_ade->ade_base;

	temp = readl(ade_base + INTR_MASK_STATE_CPU1_REG);
	
	printk("ade_isr: intr_mask_state_1 = 0x%x \n",temp);
	
	if ((temp & ADE_ISR1_RES_SWITCH_CMPL) == ADE_ISR1_RES_SWITCH_CMPL) {
		writel(ADE_ISR1_RES_SWITCH_CMPL, ade_base + INTR_CLEAR_CPU1_REG);
		crtc_ade->res_switch_cmpl = 1;
	printk("kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk\n");
		wake_up_interruptible_all(&crtc_ade->wait_res_cmpl);
	}
	
	return IRQ_HANDLED;
}

static int hisi_drm_ade_dts_parse(struct platform_device *pdev,
				    struct hisi_drm_ade_crtc *crtc_ade)
{
	struct resource	    *res;
	struct device	    *dev;
	struct device_node  *np;
	int ret;

	dev = &pdev->dev;
	np  = dev->of_node;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ade_base");
	crtc_ade->ade_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(crtc_ade->ade_base)) {
		DRM_ERROR("failed to remap io region0\n");
		ret = PTR_ERR(crtc_ade->ade_base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "media_base");
	crtc_ade->media_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(crtc_ade->media_base)) {
		DRM_ERROR("failed to remap io region1\n");
		ret = PTR_ERR(crtc_ade->media_base);
	}

	crtc_ade->ade_irq = platform_get_irq(pdev, 0);
	if (crtc_ade->ade_irq < 0)
		return crtc_ade->ade_irq;
	
	ret = devm_request_threaded_irq(&pdev->dev, crtc_ade->ade_irq,
			NULL, hisi_ade_irq_thread,
			IRQF_ONESHOT,
			"hisi_ade", crtc_ade);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request alarm irq: %d\n", ret);
		return ret;
	}
	
	crtc_ade->ade_core_clk = devm_clk_get(&pdev->dev, "clk_ade_core");
	if (crtc_ade->ade_core_clk == NULL) {
		DRM_ERROR("failed to parse the ADE_CORE\n");
	    return -ENODEV;
	}
	crtc_ade->media_noc_clk = devm_clk_get(&pdev->dev,
					"aclk_codec_jpeg_src");
	if (crtc_ade->media_noc_clk == NULL) {
		DRM_ERROR("failed to parse the CODEC_JPEG\n");
	    return -ENODEV;
	}
	crtc_ade->ade_pix_clk = devm_clk_get(&pdev->dev, "clk_ade_pix");
	if (crtc_ade->ade_pix_clk == NULL) {
		DRM_ERROR("failed to parse the ADE_PIX_SRC\n");
	    return -ENODEV;
	}

	ret = of_property_read_u32(np, "ade_core_clk_rate",
				    &crtc_ade->ade_core_rate);
	if (ret) {
		DRM_ERROR("failed to parse the ade_core_clk_rate\n");
	    return -ENODEV;
	}
	ret = of_property_read_u32(np, "media_noc_clk_rate",
				    &crtc_ade->media_noc_rate);
	if (ret) {
		DRM_ERROR("failed to parse the media_noc_clk_rate\n");
	    return -ENODEV;
	}

	return ret;
}

static int hisi_ade_probe(struct platform_device *pdev)
{
	struct hisi_drm_ade_crtc *crtc_ade;
	int ret;

	/* debug setting */
	DRM_DEBUG_DRIVER("drm_ade enter.\n");

	crtc_ade = devm_kzalloc(&pdev->dev, sizeof(*crtc_ade), GFP_KERNEL);
	if (crtc_ade == NULL) {
		DRM_ERROR("failed to alloc the space\n");
		return -ENOMEM;
	}

	sema_init(&crtc_ade->sem, 1);
	init_waitqueue_head(&crtc_ade->wait_res_cmpl);

	crtc_ade->drm_dev = dev_get_platdata(&pdev->dev);
	if (crtc_ade->drm_dev == NULL) {
		DRM_ERROR("no platform data\n");
		return -EINVAL;
	}
	
	ret = hisi_drm_ade_dts_parse(pdev, crtc_ade);
	if (ret) {
		DRM_ERROR("failed to dts parse\n");
		return ret;
	}
	
	ret = hisi_drm_ade_cmdfile_buff_init(pdev, crtc_ade);
	if (ret) {
		DRM_ERROR("failed to cmd init\n");
		return ret;
	}
	
	ret = hisi_drm_crtc_create(crtc_ade);
	if (ret) {
		DRM_ERROR("failed to crtc creat\n");
		return ret;
	}

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
