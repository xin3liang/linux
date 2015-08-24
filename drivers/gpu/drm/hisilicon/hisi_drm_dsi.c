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

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/types.h>
#include <linux/of_device.h>
#include <video/mipi_display.h>
#include <video/videomode.h>

#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_helper.h>

#include "hisi_drm_drv.h"
#include "hisi_drm_encoder.h"
#include "hisi_drm_connector.h"
#include "hisi_dsi_reg.h"
#ifdef CONFIG_DRM_HISI_FBDEV
#include "hisi_drm_fbdev.h"
#endif

#define encoder_to_dsi(encoder) \
	container_of(encoder, struct hisi_dsi, hisi_encoder.base.base)

#define DEFAULT_MIPI_CLK_RATE   19200000
#define MAX_TX_ESC_CLK		   (10)
#define DSI_24BITS_1               (5)
#define DSI_NON_BURST_SYNC_PULSES  (0)
#define DSI_BURST_MODE    DSI_NON_BURST_SYNC_PULSES
#define ROUND(x, y) ((x) / (y) + ((x) % (y) * 10 / (y) >= 5 ? 1 : 0))

#define DEFAULT_MIPI_CLK_PERIOD_PS (1000000000 / (DEFAULT_MIPI_CLK_RATE / 1000))
#if 0
enum {
	DSI_16BITS_1 = 0,
	DSI_16BITS_2,
	DSI_16BITS_3,
	DSI_18BITS_1,
	DSI_18BITS_2,
	DSI_24BITS_1,
	DSI_24BITS_2,
	DSI_24BITS_3,
};
enum {
	DSI_NON_BURST_SYNC_PULSES = 0,
	DSI_NON_BURST_SYNC_EVENTS,
	DSI_BURST_SYNC_PULSES_1,
	DSI_BURST_SYNC_PULSES_2
};
#endif 
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
	struct hisi_encoder hisi_encoder;
	struct hisi_connector hisi_connector;
	struct mipi_dsi_phy_register phyreg;
	struct videomode vm;

	u32 lanes;
	u32 format;
	u32 date_enable_pol;
	u32 mode_flags;
	u8 color_mode;
	void *ctx;
};

struct hisi_dsi_context {
	struct hisi_dsi dsi;
#ifdef CONFIG_DRM_HISI_HAS_SLAVE_ENCODER
	struct i2c_client *client;
	struct drm_i2c_encoder_driver *drm_i2c_driver;
#endif
	struct clk *dsi_cfg_clk;
	struct drm_device *dev;
	
	void __iomem *base;
	int nominal_pixel_clock_kHz;
};

struct dsi_phy_seq_info {
	u32 min_range_kHz;
	u32 max_range_kHz;
	u32 rg_pll_vco_750M;
	u32 rg_hstx_ckg_sel;
};

struct dsi_phy_seq_info dphy_seq_info[] = {
	{   46000,    62000,   1,    7 },
	{   62000,    93000,   0,    7 },
	{   93000,   125000,   1,    6 },
	{  125000,   187000,   0,    6 },
	{  187000,   250000,   1,    5 },
	{  250000,   375000,   0,    5 },
	{  375000,   500000,   1,    4 },
	{  500000,   750000,   0,    4 },
	{  750000,  1000000,   1,    0 },
	{ 1000000,  1500000,   0,    0 }
};

void set_dsi_phy_rate_equal_or_faster(u32 *phy_freq_kHz,
		     struct mipi_dsi_phy_register *phyreg)
{
	u32 ui = 0;
	u32 cfg_clk_ps = DEFAULT_MIPI_CLK_PERIOD_PS;
	u32 i = 0;
	u32 q_pll = 1;
	u32 m_pll = 0;
	u32 n_pll = 0;
	u32 r_pll = 1;
	u32 m_n = 0;
	u32 m_n_int = 0;
	u64 f_kHz;
	u64 temp;

	DRM_DEBUG_DRIVER("enter (phy_freq_kHz = %u)\n", *phy_freq_kHz);
	BUG_ON(phyreg == NULL);

	do {
		f_kHz = *phy_freq_kHz;

		/* Find the PLL clock range from the table */

		for (i = 0; i < ARRAY_SIZE(dphy_seq_info); i++)
			if (f_kHz > dphy_seq_info[i].min_range_kHz &&
			    f_kHz <= dphy_seq_info[i].max_range_kHz)
				break;

		if (i == ARRAY_SIZE(dphy_seq_info)) {
			pr_err("%s: %lldkHz out of range\n", __func__, f_kHz);
			return;
		}

		phyreg->rg_pll_vco_750M = dphy_seq_info[i].rg_pll_vco_750M;
		phyreg->rg_hstx_ckg_sel = dphy_seq_info[i].rg_hstx_ckg_sel;

		if (phyreg->rg_hstx_ckg_sel <= 7 &&
		    phyreg->rg_hstx_ckg_sel >= 4)
			q_pll = 0x10 >> (7 - phyreg->rg_hstx_ckg_sel);

		temp = f_kHz * (u64)q_pll * (u64)cfg_clk_ps;
		m_n_int = temp / (u64)1000000000;
		m_n = (temp % (u64)1000000000) / (u64)100000000;

		pr_debug("%s: m_n_int = %d, m_n = %d\n",
			 __func__, m_n_int, m_n);

		if (m_n_int % 2 == 0) {
			if (m_n * 6 >= 50) {
				n_pll = 2;
				m_pll = (m_n_int + 1) * n_pll;
			} else if (m_n * 6 >= 30) {
				n_pll = 3;
				m_pll = m_n_int * n_pll + 2;
			} else {
				n_pll = 1;
				m_pll = m_n_int * n_pll;
			}
		} else {
			if (m_n * 6 >= 50) {
				n_pll = 1;
				m_pll = (m_n_int + 1) * n_pll;
			} else if (m_n * 6 >= 30) {
				n_pll = 1;
				m_pll = (m_n_int + 1) * n_pll;
			} else if (m_n * 6 >= 10) {
				n_pll = 3;
				m_pll = m_n_int * n_pll + 1;
			} else {
				n_pll = 2;
				m_pll = m_n_int * n_pll;
			}
		}

		if (n_pll == 1) {
			phyreg->rg_pll_fbd_p = 0;
			phyreg->rg_pll_pre_div1p = 1;
		} else {
			phyreg->rg_pll_fbd_p = n_pll;
			phyreg->rg_pll_pre_div1p = 0;
		}

		if (phyreg->rg_pll_fbd_2p <= 7 && phyreg->rg_pll_fbd_2p >= 4)
			r_pll = 0x10 >> (7 - phyreg->rg_pll_fbd_2p);

		if (m_pll == 2) {
			phyreg->rg_pll_pre_p = 0;
			phyreg->rg_pll_fbd_s = 0;
			phyreg->rg_pll_fbd_div1f = 0;
			phyreg->rg_pll_fbd_div5f = 1;
		} else if (m_pll >= 2 * 2 * r_pll && m_pll <= 2 * 4 * r_pll) {
			phyreg->rg_pll_pre_p = m_pll / (2 * r_pll);
			phyreg->rg_pll_fbd_s = 0;
			phyreg->rg_pll_fbd_div1f = 1;
			phyreg->rg_pll_fbd_div5f = 0;
		} else if (m_pll >= 2 * 5 * r_pll && m_pll <= 2 * 150 * r_pll) {
			if (((m_pll / (2 * r_pll)) % 2) == 0) {
				phyreg->rg_pll_pre_p =
					(m_pll / (2 * r_pll)) / 2 - 1;
				phyreg->rg_pll_fbd_s =
					(m_pll / (2 * r_pll)) % 2 + 2;
			} else {
				phyreg->rg_pll_pre_p =
					(m_pll / (2 * r_pll)) / 2;
				phyreg->rg_pll_fbd_s =
					(m_pll / (2 * r_pll)) % 2;
			}
			phyreg->rg_pll_fbd_div1f = 0;
			phyreg->rg_pll_fbd_div5f = 0;
		} else {
			phyreg->rg_pll_pre_p = 0;
			phyreg->rg_pll_fbd_s = 0;
			phyreg->rg_pll_fbd_div1f = 0;
			phyreg->rg_pll_fbd_div5f = 1;
		}

		f_kHz = (u64)1000000000 * (u64)m_pll /
			((u64)cfg_clk_ps * (u64)n_pll * (u64)q_pll);

		if (f_kHz >= *phy_freq_kHz)
			break;

		(*phy_freq_kHz) += 10;

	} while (1);

	pr_info("%s: %dkHz ->  %lldkHz\n", __func__, *phy_freq_kHz, f_kHz);

	*phy_freq_kHz = f_kHz;
	ui = 1000000 / f_kHz;

	phyreg->clk_t_lpx = ROUND(50, 8 * ui);
	phyreg->clk_t_hs_prepare = ROUND(133, 16 * ui) - 1;

	phyreg->clk_t_hs_zero = ROUND(262, 8 * ui);
	phyreg->clk_t_hs_trial = 2 * (ROUND(60, 8 * ui) - 1);
	phyreg->clk_t_wakeup = ROUND(1000000, (cfg_clk_ps / 1000) - 1);
	if (phyreg->clk_t_wakeup > 0xff)
		phyreg->clk_t_wakeup = 0xff;
	phyreg->data_t_wakeup = phyreg->clk_t_wakeup;
	phyreg->data_t_lpx = phyreg->clk_t_lpx;
	phyreg->data_t_hs_prepare = ROUND(125 + 10 * ui, 16 * ui) - 1;
	phyreg->data_t_hs_zero = ROUND(105 + 6 * ui, 8 * ui);
	phyreg->data_t_hs_trial = 2 * (ROUND(60 + 4 * ui, 8 * ui) - 1);
	phyreg->data_t_ta_go = 3;
	phyreg->data_t_ta_get = 4;

	phyreg->rg_pll_enbwt = 1;
	phyreg->phy_clklp2hs_time = ROUND(407, 8 * ui) + 12;
	phyreg->phy_clkhs2lp_time = ROUND(105 + 12 * ui, 8 * ui);
	phyreg->phy_lp2hs_time = ROUND(240 + 12 * ui, 8 * ui) + 1;
	phyreg->phy_hs2lp_time = phyreg->phy_clkhs2lp_time;
	phyreg->clk_to_data_delay = 1 + phyreg->phy_clklp2hs_time;
	phyreg->data_to_clk_delay = ROUND(60 + 52 * ui, 8 * ui) +
				phyreg->phy_clkhs2lp_time;

	phyreg->lane_byte_clk_kHz = f_kHz / 8;
	phyreg->clk_division = phyreg->lane_byte_clk_kHz / MAX_TX_ESC_CLK;
	if (phyreg->lane_byte_clk_kHz % MAX_TX_ESC_CLK)
		phyreg->clk_division++;

	phyreg->burst_mode = DSI_BURST_MODE;
	DRM_DEBUG_DRIVER("exit success.\n");
}

static void hisi_dsi_phy_tst_set(void __iomem *base,
				 u32 array, u32 phyreg_data)
{
	writel(array, base +  PHY_TST_CTRL1 );
	wmb();
	writel(0x00000002, base +  PHY_TST_CTRL0 );
	wmb();
	writel(0x00000000, base +  PHY_TST_CTRL0 );
	wmb();
	writel(phyreg_data, base +  PHY_TST_CTRL1 );
	wmb();
	writel(0x00000002, base +  PHY_TST_CTRL0 );
	wmb();
	writel(0x00000000, base +  PHY_TST_CTRL0 );
}

int dsi_mipi_init(struct hisi_dsi *dsi)
{
	struct hisi_dsi_context *ctx = dsi->ctx;
	void __iomem *base = ctx->base;

	u32 i = 0;
	u32 hline_time = 0;
	u32 hsa_time = 0;
	u32 hbp_time = 0;
	u32 pixel_clk_kHz;
	u32 delay_count = 0;
	int refresh_nom, refresh_real, htot, vtot, blc_hactive;
	int tmp, tmp1, val;
	bool is_ready = false;

	struct mipi_dsi_phy_register *phyreg = &dsi->phyreg;

	pr_info("%s: lanes %d\n", __func__, dsi->lanes);

	/* reset Core */
	writel( PWR_UP_OFF, base +  PWR_UP );

	writel((dsi->lanes - 1) | 0x30 << 8 , base +  PHY_IF_CFG );
	writel(phyreg->clk_division, base +  CLKMGR_CFG );

	writel(0x00000000, base +  PHY_RSTZ );
	writel(0x00000000, base +  PHY_TST_CTRL0 );
	writel(0x00000001, base +  PHY_TST_CTRL0 );
	writel(0x00000000, base +  PHY_TST_CTRL0 );

	/* clock lane Timing control - TLPX */
	hisi_dsi_phy_tst_set(base, 0x10010, phyreg->clk_t_lpx);

	/* clock lane Timing control - THS-PREPARE */
	hisi_dsi_phy_tst_set(base, 0x10011, phyreg->clk_t_hs_prepare);

	 /* clock lane Timing control - THS-ZERO */
	hisi_dsi_phy_tst_set(base, 0x10012, phyreg->clk_t_hs_zero);

	/* clock lane Timing control - THS-TRAIL */
	hisi_dsi_phy_tst_set(base, 0x10013, phyreg->clk_t_hs_trial);

	/* clock lane Timing control - TWAKEUP */
	hisi_dsi_phy_tst_set(base, 0x10014, phyreg->clk_t_wakeup);

	/* data lane */
	for (i = 0; i < dsi->lanes; i++) {
		/* Timing control - TLPX*/
		hisi_dsi_phy_tst_set(base, (0x10020 + (i << 4)),
								phyreg->data_t_lpx);

		/* Timing control - THS-PREPARE */
		hisi_dsi_phy_tst_set(base, (0x10021 + (i << 4)),
								phyreg->data_t_hs_prepare);

		/* Timing control - THS-ZERO */
		hisi_dsi_phy_tst_set(base, (0x10022 + (i << 4)),
								phyreg->data_t_hs_zero);

		/* Timing control - THS-TRAIL */
		hisi_dsi_phy_tst_set(base, (0x10023 + (i << 4)),
								phyreg->data_t_hs_trial);

		/* Timing control - TTA-GO */
		hisi_dsi_phy_tst_set(base, (0x10024 + (i << 4)),
								phyreg->data_t_ta_go);

		/* Timing control - TTA-GET */
		hisi_dsi_phy_tst_set(base, (0x10025 + (i << 4)),
								phyreg->data_t_ta_get);

		/*  Timing control - TWAKEUP */
		hisi_dsi_phy_tst_set(base, (0x10026 + (i << 4)),
								phyreg->data_t_wakeup);
	}

	/* physical configuration I  */
	hisi_dsi_phy_tst_set(base, 0x10060, phyreg->rg_hstx_ckg_sel);

	/* physical configuration pll II  */
	hisi_dsi_phy_tst_set(base, 0x10063, (phyreg->rg_pll_fbd_div5f << 5) +
		(phyreg->rg_pll_fbd_div1f << 4) + (phyreg->rg_pll_fbd_2p << 1) +
		phyreg->rg_pll_enbwt);

	/* physical configuration pll II  */
	hisi_dsi_phy_tst_set(base, 0x10064, phyreg->rg_pll_fbd_p);

	/* physical configuration pll III  */
	hisi_dsi_phy_tst_set(base, 0x10065, phyreg->rg_pll_fbd_s);

	/*physical configuration pll IV*/
	hisi_dsi_phy_tst_set(base, 0x10066, (phyreg->rg_pll_pre_div1p << 7) +
								phyreg->rg_pll_pre_p);

	/*physical configuration pll V*/
	hisi_dsi_phy_tst_set(base, 0x10067, (5 << 5) + (phyreg->rg_pll_vco_750M << 4) +
					(phyreg->rg_pll_lpf_rs << 2) + phyreg->rg_pll_lpf_cs);

	writel(0x00000004, base +  PHY_RSTZ );
	udelay(1);

	writel(0x00000005, base +  PHY_RSTZ );
	udelay(1);

	writel(0x00000007, base +  PHY_RSTZ );
	msleep(1);

	while (1) {
		val = readl(base +  PHY_STATUS );
		if ((0x01 & val) || delay_count > 100) {
			is_ready = (delay_count < 100) ? true : false;
			delay_count = 0;
			break;
		}

		udelay(1);
		++delay_count;
	}

	if (!is_ready)
		DRM_INFO("phylock is not ready.\n");

	while (1) {
		val = readl(base +  PHY_STATUS );
		if ((BIT(2) & val) || delay_count > 100) {
			is_ready = (delay_count < 100) ? true : false;
			break;
		}

		udelay(1);
		++delay_count;
	}

	if (!is_ready)
		DRM_INFO("phystopstateclklane is not ready.\n");

//	writel(hisi_dsi->vc, base +  DPI_VCID );
	writel(0, base +  DPI_VCID );
	writel(dsi->color_mode, base +  DPI_COLOR_CODING );
	writel(dsi->date_enable_pol | 0 << 0x3 | 0 << 0x4 |
		(dsi->vm.flags & DISPLAY_FLAGS_HSYNC_HIGH ? 0 : 1) << 0x2 |
		(dsi->vm.flags & DISPLAY_FLAGS_VSYNC_HIGH ? 0 : 1) << 0x1,
		base +  DPI_CFG_POL );

	if (dsi->format == MIPI_DSI_FMT_RGB666)
		writel(1 << 0x09, base +  DPI_COLOR_CODING );
//		writel(hisi_dsi->color_mode | 1 << 0x09, base +  DPI_COLOR_CODING );

	/*
	 * The DSI IP accepts vertical timing using lines as normal,
	 * but horizontal timing is a mixture of pixel-clocks for the
	 * active region and byte-lane clocks for the blanking-related
	 * timings.  hfp is specified as the total hline_time in byte-
	 * lane clocks minus hsa, hbp and active.
	 */

	htot = dsi->vm.hactive +dsi->vm.hsync_len +
		  dsi->vm.hfront_porch + dsi->vm.hback_porch;
	vtot = dsi->vm.vactive + dsi->vm.vsync_len +
		  dsi->vm.vfront_porch + dsi->vm.vback_porch;

	pixel_clk_kHz = dsi->vm.pixelclock;

	hsa_time = (dsi->vm.hsync_len * phyreg->lane_byte_clk_kHz) /
		   pixel_clk_kHz;
	hbp_time = (dsi->vm.hback_porch * phyreg->lane_byte_clk_kHz) /
		   pixel_clk_kHz;
	hline_time  = (((u64)htot * (u64)phyreg->lane_byte_clk_kHz)) /
		      pixel_clk_kHz;
	blc_hactive  = (((u64)dsi->vm.hactive *
			(u64)phyreg->lane_byte_clk_kHz)) / pixel_clk_kHz;

/* returns pixel clocks in dsi byte lane times, multiplied by 1000 */
#define R(x) ((u32)((((u64)(x) * (u64)1000 * (u64)dsi->vm.pixelclock) / \
	      phyreg->lane_byte_clk_kHz)))

	if ((R(hline_time) / 1000) > htot)
		hline_time--;

	if ((R(hline_time) / 1000) < htot)
		hline_time++;

	/* all specified in byte-lane clocks */
	writel(hsa_time, base +  VID_HSA_TIME );
	writel(hbp_time, base +  VID_HBP_TIME );
	writel(hline_time, base +  VID_HLINE_TIME );

	DRM_INFO("%s: pixel_clk_kHz=%d, lane_byte_clk_kHz=%d, hsa=%d, "
		 "hbp=%d, hline=%d", __func__, pixel_clk_kHz,
		 phyreg->lane_byte_clk_kHz, hsa_time, hbp_time, hline_time);

	if (dsi->vm.vsync_len > 15)
		dsi->vm.vsync_len = 15;

	writel(dsi->vm.vsync_len, base +  VID_VSA_LINES );
	writel(dsi->vm.vback_porch, base +  VID_VBP_LINES );
	writel(dsi->vm.vfront_porch, base +  VID_VFP_LINES );
	writel(dsi->vm.vactive, base +  VID_VACTIVE_LINES );
	writel(dsi->vm.hactive, base +  VID_PKT_SIZE );

	refresh_nom = ((u64)ctx->nominal_pixel_clock_kHz * 1000000) /
		      (htot * vtot);

	tmp = 1000000000 / dsi->vm.pixelclock;
	tmp1 = 1000000000 / phyreg->lane_byte_clk_kHz;

	pr_info("%s: Pixel clock: %ldkHz (%d.%03dns), "
		"DSI bytelane clock: %dkHz (%d.%03dns)\n",
		__func__,dsi->vm.pixelclock, tmp / 1000, tmp % 1000,
		phyreg->lane_byte_clk_kHz, tmp1 / 1000, tmp1 % 1000);

	pr_info("%s:       CLK   HACT VACT REFRSH    HTOT   VTOT   HFP     HSA    HBP        VFP VBP VSA\n",
		__func__);

/* returns pixel clocks in dsi byte lane times, multiplied by 1000 */
#define R(x) ((u32)((((u64)(x) * (u64)1000 * (u64)dsi->vm.pixelclock) / \
	      phyreg->lane_byte_clk_kHz)))

	refresh_real = ((u64)dsi->vm.pixelclock * (u64)1000000000) /
		      ((u64)R(hline_time) * (u64)vtot);

	pr_info("%s: nom: %6u %4u %4u %2u.%03u  %4u     %4u  %3u     %3u     %3u      %3u %3u  %3u\n",
		__func__, ctx->nominal_pixel_clock_kHz,
		dsi->vm.hactive, dsi->vm.vactive,
		refresh_nom / 1000, refresh_nom % 1000, htot, vtot,
		dsi->vm.hfront_porch, dsi->vm.hsync_len,
		dsi->vm.hback_porch, dsi->vm.vfront_porch,
		dsi->vm.vsync_len, dsi->vm.vback_porch);

	pr_info("%s: tru: %6u %4u %4u %2u.%03u  %4u.%03u %4u  %3u.%03u %3u.%03u %3u.%03u  %3u %3u  %3u\n",
		__func__, (u32)dsi->vm.pixelclock,
		dsi->vm.hactive, dsi->vm.vactive,
		refresh_real / 1000, refresh_real % 1000,
		R(hline_time) / 1000, R(hline_time) % 1000, vtot,
		R(hline_time - hbp_time - hsa_time - blc_hactive) / 1000,
		R(hline_time - hbp_time - hsa_time - blc_hactive) % 1000,
		R(hsa_time) / 1000, R(hsa_time) % 1000,
		R(hbp_time) / 1000, R(hbp_time) % 1000,
		dsi->vm.vfront_porch,
		dsi->vm.vsync_len, dsi->vm.vback_porch);

	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO) {
		/*
		 * we disable this since it affects downstream
		 * DSI -> HDMI converter output
		 */
		writel(0x00, base +  VID_MODE_CFG );

		/*VSA/VBP/VFP max transfer byte in LP mode*/
		writel(0x00, base +  DPI_LP_CMD_TIM );
		/* enable LP command transfer */
		writel(0x00, base +  VID_MODE_CFG );
		/* config max read time */
		writel(0xFF, base +  PHY_TMR_CFG );
	}

	/* Configure core's phy parameters */
	writel(0xFFF, base +  BTA_TO_CNT );
	writel(0xFF | (phyreg->phy_lp2hs_time) << 16 |
				(phyreg->phy_hs2lp_time) << 24,
				base +  PHY_TMR_CFG );
	writel((phyreg->phy_clklp2hs_time) | (phyreg->phy_clkhs2lp_time) << 16,
					base +  PHY_TMR_LPCLK_CFG );

	writel((phyreg->clk_to_data_delay) | (phyreg->data_to_clk_delay) << 8,
						base +  NO_CONTINUE );
	writel(0x00, base +  VID_MODE_CFG );
	writel(phyreg->burst_mode, base +  VID_MODE_CFG );
	writel(0x0, base +  LPCLK_CTRL );
	/* for dsi read */
	writel(0x0, base +  PCKHDL_CFG );
	/* Enable EOTP TX; Enable EDPI */
	if (dsi->mode_flags == MIPI_DSI_MODE_VIDEO)
		writel(dsi->vm.hactive, base +  EDPI_CMD_SIZE );

	/*------------DSI and D-PHY Initialization-----------------*/

	writel( VIDEO_MODE, base +  MODE_CFG );
	writel(0x1, base +  LPCLK_CTRL );
	
	writel( PWR_UP_ON, base +  PWR_UP );

	DRM_INFO("%s , exit success!\n", __func__);

	return 0;
}

static void dsi_encoder_disable(struct drm_encoder *encoder)
{
	struct hisi_dsi *dsi = encoder_to_dsi(encoder);
	struct hisi_dsi_context *ctx = dsi->ctx;
	void __iomem *base = ctx->base;
	
	writel( PWR_UP_OFF, base +  PWR_UP );
	writel(0x0, base +  LPCLK_CTRL );
	writel(0x00000000, base +  PHY_RSTZ );
	clk_disable_unprepare(ctx->dsi_cfg_clk);
}

static void dsi_encoder_enable(struct drm_encoder *encoder)
{
	struct hisi_dsi *dsi = encoder_to_dsi(encoder);
	struct hisi_dsi_context *ctx = dsi->ctx;
	int ret;

	/* mipi dphy clock enable */
	ret = clk_prepare_enable(ctx->dsi_cfg_clk);
	if (ret) {
		DRM_ERROR("failed to enable dsi_cfg_clk, error=%d!\n", ret);
		return;
	}

	ret = dsi_mipi_init(dsi);
	if (ret) {
		DRM_ERROR("failed to init mipi, error=%d!\n", ret);
		return;
	}
}

void dsi_encoder_mode_set(struct drm_encoder *encoder,
					struct drm_display_mode *mode,
					struct drm_display_mode *adjusted_mode)
{
	struct hisi_dsi *dsi = encoder_to_dsi(encoder);
	struct hisi_dsi_context *ctx = dsi->ctx;
	struct videomode *vm = &dsi->vm;
	u32 dphy_freq_kHz;

	DRM_DEBUG_DRIVER("enter.\n");
	vm->pixelclock = adjusted_mode->clock;
	ctx->nominal_pixel_clock_kHz = mode->clock;

	vm->hactive = mode->hdisplay;
	vm->vactive = mode->vdisplay;
	vm->vfront_porch = mode->vsync_start - mode->vdisplay;
	vm->vback_porch = mode->vtotal - mode->vsync_end;
	vm->vsync_len = mode->vsync_end - mode->vsync_start;
	vm->hfront_porch = mode->hsync_start - mode->hdisplay;
	vm->hback_porch = mode->htotal - mode->hsync_end;
	vm->hsync_len = mode->hsync_end - mode->hsync_start;

	dsi->lanes = 3 + !!(vm->pixelclock >= 115000);

	dphy_freq_kHz = vm->pixelclock * 24 / dsi->lanes;
	/* this avoids a less-compatible DSI rate with 1.2GHz px PLL */
	if (dphy_freq_kHz == 600000)
		dphy_freq_kHz = 640000;

	set_dsi_phy_rate_equal_or_faster(&dphy_freq_kHz, &dsi->phyreg);

	vm->flags = 0;
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		vm->flags |= DISPLAY_FLAGS_HSYNC_HIGH;
	else if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		vm->flags |= DISPLAY_FLAGS_HSYNC_LOW;
	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		vm->flags |= DISPLAY_FLAGS_VSYNC_HIGH;
	else if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		vm->flags |= DISPLAY_FLAGS_VSYNC_LOW;

	DRM_DEBUG_DRIVER("exit success: pixelclk=%dkHz, dphy_freq_kHz=%dkHz\n",
			(u32)vm->pixelclock, dphy_freq_kHz);
}

static struct drm_display_mode mode_720p = {
	.name		= "1280x720",
	.vrefresh	= 60,
	.clock		= 74250,
	.hdisplay	= 1280,
	.hsync_start	= 1390,
	.hsync_end	= 1430,
	.htotal		= 1650,
	.vdisplay	= 720,
	.vsync_start	= 725,
	.vsync_end	= 730,
	.vtotal		= 750,
	.type		= DRM_MODE_TYPE_PREFERRED | DRM_MODE_TYPE_DRIVER,
	.flags		= DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC,
	.width_mm	= 735,
	.height_mm	= 420,
};

/*
 * 800x600@60 works well, so add to defaut modes
 */
static struct drm_display_mode mode_800x600 = {
	.name		= "800x600",
	.vrefresh	= 60,
	.clock		= 40000,
	.hdisplay	= 800,
	.hsync_start	= 840,
	.hsync_end	= 968,
	.htotal		= 1056,
	.vdisplay	= 600,
	.vsync_start	= 601,
	.vsync_end	= 605,
	.vtotal		= 628,
	.type		= DRM_MODE_TYPE_PREFERRED | DRM_MODE_TYPE_DRIVER,
	.flags		= DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC,
	.width_mm	= 735,
	.height_mm	= 420,
};

static int dsi_connector_get_modes(struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	/* 1280x720@60: 720P */
	mode = drm_mode_duplicate(connector->dev, &mode_720p);
	if (!mode) {
		DRM_ERROR("failed to create a new display mode\n");
	}
	drm_mode_probed_add(connector, mode);

	/* 800x600@60 */
	mode = drm_mode_duplicate(connector->dev, &mode_800x600);
	if (!mode) {
		DRM_ERROR("failed to create a new display mode\n");
	}
	drm_mode_probed_add(connector, mode);

	return 2;
}

struct hisi_connector_funcs hisi_dsi_connector_ops = {
	.get_modes = dsi_connector_get_modes,
};

struct hisi_encoder_funcs hisi_dsi_encoder_ops = {
	.mode_set = dsi_encoder_mode_set,
	.enable = dsi_encoder_enable,
	.disable = dsi_encoder_disable,
};

static int hisi_dsi_bind(struct device *dev, struct device *master,
								void *data)
{
	struct hisi_dsi_context *ctx = dev_get_drvdata(dev);
	int ret = 0;
	
	ctx->dev = data;

	hisi_drm_encoder_init(ctx->dev, &ctx->dsi.hisi_encoder.base.base);

#ifdef CONFIG_DRM_HISI_HAS_SLAVE_ENCODER
	/* int ret; */
	ret = ctx->drm_i2c_driver->encoder_init(ctx->client, ctx->dev,
							&ctx->dsi.hisi_encoder.base);
	if (ret) {
		DRM_ERROR("drm_i2c_encoder_init error\n");
		return ret;
	}

	if (!ctx->dsi.hisi_encoder.base.slave_funcs) {
		DRM_ERROR("failed check encoder function\n");
		return -ENODEV;
	}
#endif
	hisi_drm_connector_init(ctx->dev, &ctx->dsi.hisi_encoder.base.base,
						&ctx->dsi.hisi_connector.connector);

	/* fbdev initialization should be put at last position */
#ifdef CONFIG_DRM_HISI_FBDEV
	ret = hisi_drm_fbdev_init(ctx->dev);
	if (ret) {
		DRM_ERROR("failed to initialize fbdev\n");
		return ret;
	}
#endif
	return ret;
}

static void hisi_dsi_unbind(struct device *dev, struct device *master,
							void *data)
{
	/* do nothing */
}

static const struct component_ops hisi_dsi_ops = {
	.bind	= hisi_dsi_bind,
	.unbind	= hisi_dsi_unbind,
};

static int hisi_dsi_probe(struct platform_device *pdev)
{
	int ret;
	struct hisi_dsi_context *ctx;
	struct hisi_dsi *dsi;
	struct resource *res;
	struct device_node *slave_node;
	struct device_node *np = pdev->dev.of_node;

	DRM_DEBUG_DRIVER("enter.\n");

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		dev_err(&pdev->dev, "failed to allocate hisi dsi object.\n");
		ret = -ENOMEM;
	}
	
	ctx->dsi_cfg_clk = clk_get(&pdev->dev, "pclk_dsi");
	if (IS_ERR(ctx->dsi_cfg_clk)) {
		dev_info(&pdev->dev, "failed to get dsi clock");
		ret = PTR_ERR(ctx->dsi_cfg_clk);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ctx->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ctx->base)) {
		dev_err(&pdev->dev, "failed to remap io region\n");
		ret = PTR_ERR(ctx->base);
	}

	slave_node = of_parse_phandle(np, "encoder-slave", 0);
	if (!slave_node) {
		DRM_ERROR("failed to get slave encoder node\n");
		return -EINVAL;
	}

#ifdef CONFIG_DRM_HISI_HAS_SLAVE_ENCODER
	ctx->client = of_find_i2c_device_by_node(slave_node);
	of_node_put(slave_node);
	if (!ctx->client) {
		DRM_INFO("failed to find slave encoder i2c client\n");
		return -EPROBE_DEFER;
	}

	if (!ctx->client->dev.driver) {
		DRM_INFO("%s: NULL client driver\n", __func__);
		return -EPROBE_DEFER;
	}

	ctx->drm_i2c_driver = to_drm_i2c_encoder_driver(
		to_i2c_driver(ctx->client->dev.driver));
	if (IS_ERR(ctx->drm_i2c_driver)) {
		pr_err("failed initialize encoder driver %ld\n", PTR_ERR(ctx->drm_i2c_driver));
		return -EPROBE_DEFER;
	}
#endif

	dsi = &ctx->dsi;
	dsi->ctx = ctx;
	dsi->color_mode = DSI_24BITS_1;
	dsi->date_enable_pol = 0;
	dsi->lanes = 3;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE;

	dsi->hisi_encoder.ops = &hisi_dsi_encoder_ops;
	dsi->hisi_connector.encoder = &dsi->hisi_encoder.base.base;
	dsi->hisi_connector.ops = &hisi_dsi_connector_ops;

	platform_set_drvdata(pdev, ctx);

	return component_add(&pdev->dev, &hisi_dsi_ops);
	
	DRM_DEBUG_DRIVER("exit success.\n");
}

static int hisi_dsi_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &hisi_dsi_ops);
	return 0;
}

static struct of_device_id hisi_dsi_of_match[] = {
	{.compatible = "hisilicon,hi6220-dsi"},
	{ }
};
MODULE_DEVICE_TABLE(of, hisi_dsi_of_match);

static struct platform_driver hisi_dsi_driver = {
	.probe = hisi_dsi_probe,
	.remove = hisi_dsi_remove,
	.driver = {
		.name = "hisi-dsi",
		.owner = THIS_MODULE,
		.of_match_table = hisi_dsi_of_match,
	},
};

module_platform_driver(hisi_dsi_driver);

MODULE_AUTHOR("Xinwei Kong <kong.kongxinwei@hisilicon.com>");
MODULE_AUTHOR("Xinliang Liu <z.liuxinliang@huawei.com>");
MODULE_DESCRIPTION("hisilicon SoC DRM driver");
MODULE_LICENSE("GPL v2");
