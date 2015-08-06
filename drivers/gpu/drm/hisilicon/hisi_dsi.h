/*
 *  Hisilicon Terminal SoCs drm driver
 *
 *  Copyright (c) 2014-2015 Hisilicon Limited.
 *  Author:
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */
#ifndef __HISI_DRM_DSI_H__
#define __HISI_DRM_DSI_H__

struct mipi_dsi_phy_register {
	u32 clk_t_lpx;
	u32 clk_t_hs_prepare;
	u32 clk_t_hs_zero;
	u32 clk_t_hs_trial;
	u32 clk_t_wakeup;
	u32 data_t_lpx;
	u32 data_t_hs_prepare;
	u32 data_t_hs_zero;
	u32 data_t_hs_trial;
	u32 data_t_ta_go;
	u32 data_t_ta_get;
	u32 data_t_wakeup;
	u32 rg_hstx_ckg_sel;
	u32 rg_pll_fbd_div5f;
	u32 rg_pll_fbd_div1f;
	u32 rg_pll_fbd_2p;
	u32 rg_pll_enbwt;
	u32 rg_pll_fbd_p;
	u32 rg_pll_fbd_s;
	u32 rg_pll_pre_div1p;
	u32 rg_pll_pre_p;
	u32 rg_pll_vco_750M;
	u32 rg_pll_lpf_rs;
	u32 rg_pll_lpf_cs;
	u32 phy_clklp2hs_time;
	u32 phy_clkhs2lp_time;
	u32 phy_lp2hs_time;
	u32 phy_hs2lp_time;
	u32 clk_to_data_delay;
	u32 data_to_clk_delay;
	u32 lane_byte_clk_kHz;
	u32 clk_division;
	u32 burst_mode;
};

struct hisi_dsi {
	struct drm_encoder_slave base;
	struct drm_connector connector;
	struct i2c_client *client;
	struct drm_i2c_encoder_driver *drm_i2c_driver;
	struct clk *dsi_cfg_clk;
	struct videomode vm;
	int nominal_pixel_clock_kHz;
	struct drm_device *dev;

	u8 __iomem *reg_base;
	u8 color_mode;

	u32 lanes;
	u32 format;
	struct mipi_dsi_phy_register phyreg;
	u32 date_enable_pol;
	u32 vc;
	u32 mode_flags;

	bool enable;
};

#define outp32(addr, val) writel(val, addr)
#define outp16(addr, val) writew(val, addr)
#define outp8(addr, val) writeb(val, addr)
#define outp(addr, val) outp32(addr, val)

#define inp32(addr) readl(addr)
#define inp16(addr) readw(addr)
#define inp8(addr) readb(addr)
#define inp(addr) inp32(addr)

extern u8 *reg_base_mipi_dsi;
extern int hisi_drm_dsi_init(void);
extern void hisi_drm_dsi_exit(void);

void hisi_dsi_connector_destroy(struct drm_connector *connector);
enum drm_connector_status
hisi_dsi_detect(struct drm_connector *connector, bool force);
int hisi_drm_connector_mode_valid(struct drm_connector *connector,
					  struct drm_display_mode *mode);
struct drm_encoder *
hisi_dsi_best_encoder(struct drm_connector *connector);

int hisi_dsi_get_modes(struct drm_connector *connector);
void hisi_drm_encoder_destroy(struct drm_encoder *encoder);
void hisi_drm_encoder_enable(struct drm_encoder *encoder);
void hisi_drm_encoder_disable(struct drm_encoder *encoder);

bool
hisi_drm_encoder_mode_fixup(struct drm_encoder *encoder,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode);
void hisi_drm_encoder_mode_set(struct drm_encoder *encoder,
					struct drm_display_mode *mode,
					struct drm_display_mode *adjusted_mode);
#endif /* __HISI_DRM_DSI_H__ */
