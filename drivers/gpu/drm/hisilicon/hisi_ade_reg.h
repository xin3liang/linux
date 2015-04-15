/*
 *  Hisilicon Terminal SoCs drm driver
 *
 *  Copyright (c) 2014-2015 Hisilicon Limited.
 *  Author:
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#ifndef __HISI_ADE_REG_H__
#define __HISI_ADE_REG_H__

#include "hisi_ade_cmdqueue.h"

/********** ADE Register Offset ***********/
#define ADE_CTRL_REG                (0x4)
#define ADE_CTRL1_REG               (0x8C)
#define ADE_SCL3_MUX_CFG_REG        (0x8)
#define ADE_SCL1_MUX_CFG_REG        (0xC)
#define ADE_ROT_SRC_CFG_REG         (0x10)
#define ADE_SCL2_SRC_CFG_REG        (0x14)
#define ADE_DISP_SRC_CFG_REG        (0x18)
#define ADE_WDMA2_SRC_CFG_REG       (0x1C)
#define ADE_SEC_OVLY_SRC_CFG_REG    (0x20)
#define ADE_WDMA3_SRC_CFG_REG       (0x24)
#define ADE_OVLY1_TRANS_CFG_REG     (0x2C)
#define ADE_CTRAN5_TRANS_CFG_REG    (0x40)
#define ADE_EN_REG                  (0x100)
#define INTR_MASK_CPU0_REG         (0xC10)
#define INTR_MASK_CPU1_REG         (0xC14)
#define INTR_MASK_STATE_CPU1_REG    (0xC0C)
#define INTR_CLEAR_CPU1_REG        (0xC1C)
#define ADE_FRM_DISGARD_CTRL_REG    (0xA4)
#define ADE_SOFT_RST_SEL0_REG       (0x78)
#define ADE_SOFT_RST_SEL1_REG       (0x7C)
#define ADE_RELOAD_DIS0_REG         (0xAC)
#define ADE_RELOAD_DIS1_REG         (0xB0)
#define ADE_OVLY_CTL_REG            (0x98)
#define RD_CH_DISP_CTRL_REG         (0x1404)
#define RD_CH_DISP_ADDR_REG         (0x1408)
#define RD_CH_DISP_SIZE_REG         (0x140C)
#define RD_CH_DISP_STRIDE_REG       (0x1410)
#define RD_CH_DISP_SPACE_REG        (0x1414)
#define RD_CH_DISP_EN_REG           (0x142C)
#define ADE_CTRAN5_DIS_REG          (0x5404)
#define ADE_CTRAN5_IMAGE_SIZE_REG   (0x543C)
#define ADE_CTRAN5_CFG_OK_REG       (0x5440)
#define ADE_CTRAN6_DIS_REG          (0x5504)
#define ADE_CTRAN6_IMAGE_SIZE_REG   (0x553C)
#define ADE_CTRAN6_CFG_OK_REG       (0x5540)
#define ADE_DMA_AXI_MUX_REG         (0x50)
#define RD_CH_DISP_PE_REG           (0x1400)
#define RD_CH_CMDQ1_PE_REG          (0x1500)
#define RD_CH_CMDQ1_CTRL_REG        (0x1504)
#define RD_CH_CMDQ1_ADDR_REG        (0x1508)
#define RD_CH_CMDQ1_LEN_REG         (0x150C)
#define RD_CH_CMDQ1_EN_REG          (0x1510)
#define RD_CH_CMDQ2_PE_REG          (0x1580)
#define RD_CH_CMDQ2_CTRL_REG        (0x1584)
#define RD_CH_CMDQ2_ADDR_REG        (0x1588)
#define RD_CH_CMDQ2_LEN_REG         (0x158C)
#define RD_CH_CMDQ2_EN_REG          (0x1590)

enum {
    ADE_ISR1_CMDQ1_CMPL = 0x200,
    ADE_ISR1_CMDQ2_CMPL = 0x400,
    ADE_ISR1_CMDQ1_ERR_CMD = 0x1000,
    ADE_ISR1_CMDQ2_ERR_CMD = 0x2000,
    ADE_ISR1_CMDQ1_WAIT_TIMER_OUT = 0x8000,
    ADE_ISR1_CMDQ2_WAIT_TIMER_OUT = 0x10000,
    ADE_ISR1_CMDQ1_RD_WR_TIMER_OUT = 0x40000,
    ADE_ISR1_CMDQ2_RD_WR_TIMER_OUT = 0x80000,
    ADE_ISR1_OVLY1_CMPL = 0x00200000,
    ADE_ISR1_RES_SWITCH_CMPL         = 0x80000000
};

union U_ADE_CTRL1 {
	struct
	{
		unsigned int	auto_clk_gate_en	:1;
		unsigned int	rot_buf_shr_out		:1;
		unsigned int	Reserved_44		:30;
	} bits;
	unsigned int	u32;
};

// Define the union U_ADE_DMA_AXI_MUX
union U_ADE_DMA_AXI_MUX {
    // Define the struct bits
    struct
    {
        unsigned int    rd_dma_ch1_axi_cfg    : 1   ; // [0]
        unsigned int    rd_dma_ch2_axi_cfg    : 1   ; // [1]
        unsigned int    rd_dma_ch3_axi_cfg    : 1   ; // [2]
        unsigned int    rd_dma_ch4_axi_cfg    : 1   ; // [3]
        unsigned int    rd_dma_ch5_axi_cfg    : 1   ; // [4]
        unsigned int    rd_dma_ch6_axi_cfg    : 1   ; // [5]
        unsigned int    rd_dma_disp_axi_cfg   : 1   ; // [6]
        unsigned int    rd_dma_cmdq1_axi_cfg  : 1   ; // [7]
        unsigned int    rd_dma_cmdq2_axi_cfg  : 1   ; // [8]
        unsigned int    Reserved_14           : 1   ; // [9]
        unsigned int    wr_dma_ch1_axi_cfg    : 1   ; // [10]
        unsigned int    wr_dma_ch2_axi_cfg    : 1   ; // [11]
        unsigned int    wr_dma_ch3_axi_cfg    : 1   ; // [12]
        unsigned int    Reserved_13           : 1   ; // [13]
        unsigned int    wr_dma_cmdq_axi_cfg   : 1   ; // [14]
        unsigned int    Reserved_12           : 17  ; // [31..15]
    } bits;

    // Define an unsigned member
    unsigned int    u32;

};
// Define the union U_ADE_RELOAD_DIS0
union  U_ADE_RELOAD_DIS0 {
    // Define the struct bits
    struct
    {
        unsigned int    ch1_rdma_reload_dis   : 1   ; // [0]
        unsigned int    ch2_rdma_reload_dis   : 1   ; // [1]
        unsigned int    ch3_rdma_reload_dis   : 1   ; // [2]
        unsigned int    ch4_rdma_reload_dis   : 1   ; // [3]
        unsigned int    ch5_rdma_reload_dis   : 1   ; // [4]
        unsigned int    ch6_rdma_reload_dis   : 1   ; // [5]
        unsigned int    disp_rdma_reload_dis  : 1   ; // [6]
        unsigned int    cmdq1_rdma_reload_dis  : 1   ; // [7]
        unsigned int    cmdq2_rdma_reload_dis  : 1   ; // [8]
        unsigned int    Reserved_67           : 1   ; // [9]
        unsigned int    ch1_wdma_reload_dis   : 1   ; // [10]
        unsigned int    ch2_wdma_reload_dis   : 1   ; // [11]
        unsigned int    ch3_wdma_reload_dis   : 1   ; // [12]
        unsigned int    Reserved_66           : 1   ; // [13]
        unsigned int    cmdq_wdma_reload_dis  : 1   ; // [14]
        unsigned int    clip1_reload_dis      : 1   ; // [15]
        unsigned int    clip2_reload_dis      : 1   ; // [16]
        unsigned int    clip3_reload_dis      : 1   ; // [17]
        unsigned int    clip4_reload_dis      : 1   ; // [18]
        unsigned int    clip5_reload_dis      : 1   ; // [19]
        unsigned int    clip6_reload_dis      : 1   ; // [20]
        unsigned int    scl1_reload_dis       : 1   ; // [21]
        unsigned int    scl2_reload_dis       : 1   ; // [22]
        unsigned int    scl3_reload_dis       : 1   ; // [23]
        unsigned int    ctran1_reload_dis     : 1   ; // [24]
        unsigned int    ctran2_reload_dis     : 1   ; // [25]
        unsigned int    ctran3_reload_dis     : 1   ; // [26]
        unsigned int    ctran4_reload_dis     : 1   ; // [27]
        unsigned int    ctran5_reload_dis     : 1   ; // [28]
        unsigned int    ctran6_reload_dis     : 1   ; // [29]
        unsigned int    rot_reload_dis        : 1   ; // [30]
        unsigned int    Reserved_65           : 1   ; // [31]
    } bits;

    // Define an unsigned member
    unsigned int    u32;

};

union U_ADE_SOFT_RST_SEL0 {
struct {
	unsigned int    ch1_rdma_srst_sel     :1;
	unsigned int    ch2_rdma_srst_sel     :1;
	unsigned int    ch3_rdma_srst_sel     :1;
	unsigned int    ch4_rdma_srst_sel     :1;
	unsigned int    ch5_rdma_srst_sel     :1;
	unsigned int    ch6_rdma_srst_sel     :1;
	unsigned int    disp_rdma_srst_sel    :1;
	unsigned int    cmdq1_rdma_srst_sel   :1;
	unsigned int    cmdq2_rdma_srst_sel   :1;
	unsigned int    Reserved_29           :1;
	unsigned int    ch1_wdma_srst_sel     :1;
	unsigned int    ch2_wdma_srst_sel     :1;
	unsigned int    ch3_wdma_srst_sel     :1;
	unsigned int    Reserved_28           :1;
	unsigned int    cmdq_wdma_srst_sel    :1;
	unsigned int    clip1_srst_sel        :1;
	unsigned int    clip2_srst_sel        :1;
	unsigned int    clip3_srst_sel        :1;
	unsigned int    clip4_srst_sel        :1;
	unsigned int    clip5_srst_sel        :1;
	unsigned int    clip6_srst_sel        :1;
	unsigned int    scl1_srst_sel         :1;
	unsigned int    scl2_srst_sel         :1;
	unsigned int    scl3_srst_sel         :1;
	unsigned int    ctran1_srst_sel       :1;
	unsigned int    ctran2_srst_sel       :1;
	unsigned int    ctran3_srst_sel       :1;
	unsigned int    ctran4_srst_sel       :1;
	unsigned int    ctran5_srst_sel       :1;
	unsigned int    ctran6_srst_sel       :1;
	unsigned int    rot_srst_sel          :1;
	unsigned int    Reserved_27           :1;
	} bits;
	unsigned int	u32;
};

union U_ADE_CTRL {
struct {
	unsigned int    frm_end_start         :2;
	unsigned int    dfs_buf_cfg           :1;
	unsigned int    rot_buf_cfg           :3;
	unsigned int    rd_ch5_nv             :1;
	unsigned int    rd_ch6_nv             :1;
	unsigned int    dfs_buf_unflow_lev1   :13;
	unsigned int    dfs_buf_unflow_lev2   :11;
	} bits;
	unsigned int	u32;
};


/********** ADE Register Write/Read functions ***********/
static inline void set_TOP_CTL_clk_gate_en(u8 *ade_base, u32 val)
{
	volatile union U_ADE_CTRL1   ade_ctrl1;
	u8 *addr = ade_base + ADE_CTRL1_REG;

	ade_ctrl1.u32 = readl(addr);
	ade_ctrl1.bits.auto_clk_gate_en = val;
	writel(ade_ctrl1.u32, addr);
}

static inline void set_TOP_SOFT_RST_SEL0_disp_rdma(u8 *ade_base, u32 val)
{
	volatile union U_ADE_SOFT_RST_SEL0 ade_soft_rst;
	u8 *addr = ade_base + ADE_SOFT_RST_SEL0_REG;

	ade_soft_rst.u32 = readl(addr);
	ade_soft_rst.bits.disp_rdma_srst_sel = val;
	writel(ade_soft_rst.u32, addr);
}

static inline void set_TOP_SOFT_RST_SEL0_ctran5(u8 *ade_base, u32 val)
{
	volatile union U_ADE_SOFT_RST_SEL0 ade_soft_rst;
	u8 *addr = ade_base + ADE_SOFT_RST_SEL0_REG;

	ade_soft_rst.u32 = readl(addr);
	ade_soft_rst.bits.ctran5_srst_sel = val;
	writel(ade_soft_rst.u32, addr);
}

static inline void set_TOP_SOFT_RST_SEL0_ctran6(u8 *ade_base, u32 val)
{
	volatile union U_ADE_SOFT_RST_SEL0 ade_soft_rst;
	u8 *addr = ade_base + ADE_SOFT_RST_SEL0_REG;

	ade_soft_rst.u32 = readl(addr);
	ade_soft_rst.bits.ctran6_srst_sel = val;
	writel(ade_soft_rst.u32, addr);
}

static inline void set_TOP_CTL_frm_end_start(u8 *ade_base, u32 val)
{
	volatile union U_ADE_CTRL  ade_ctrl;
	u8 *reg_addr = ade_base + ADE_CTRL_REG;

	ade_ctrl.u32 = readl(reg_addr);
	ade_ctrl.bits.frm_end_start = val;
	writel(ade_ctrl.u32, reg_addr);
}

inline void set_TOP_DISP_SRC_CFG(u8* ade_base, u32 val)
{
	u8*  reg_addr = ade_base + ADE_DISP_SRC_CFG_REG;
	writel(val, reg_addr);
}

inline void set_TOP_SOFT_RST_SEL0(u8* ade_base, u32 val)
{
	u8* addr = ade_base + ADE_SOFT_RST_SEL0_REG;     
	writel(val, addr);
}

inline void set_TOP_SOFT_RST_SEL1(u8* ade_base, u32 val)
{
	u8* addr = ade_base + ADE_SOFT_RST_SEL1_REG;
	writel(val, addr);
}
inline void set_TOP_RELOAD_DIS0(u8* ade_base, u32 val)
{
	u8* addr = ade_base + ADE_RELOAD_DIS0_REG;
	writel(val, addr);
}

inline void set_TOP_RELOAD_DIS1(u8* ade_base, u32 val)
{
	u8* addr = ade_base + ADE_RELOAD_DIS1_REG;
	writel(val, addr);
}

inline void set_TOP_SOFT_RST_SEL0_cmdq1_rdma(u8* ade_base, u32 val)
{
	u8*  addr = ade_base + ADE_SOFT_RST_SEL0_REG;
	volatile union U_ADE_SOFT_RST_SEL0 ade_soft_rst;

	ade_soft_rst.u32 = readl(addr);
	ade_soft_rst.bits.cmdq1_rdma_srst_sel = val;
	writel(ade_soft_rst.u32, addr);
}

inline void set_TOP_RELOAD_DIS0_cmdq1_rdma(u8* ade_base, u32 val)
{
	u8*          addr = ade_base + ADE_RELOAD_DIS0_REG;
	volatile union U_ADE_RELOAD_DIS0 ade_reload_dis;
	
	ade_reload_dis.u32 = readl(addr);
	ade_reload_dis.bits.cmdq1_rdma_reload_dis = val;

	writel(ade_reload_dis.u32, addr);
}

/*********** CMDQ RDMA************************/
inline void set_CMDQ_RDMA1_PE(u8* ade_base, u32 val)
{
    u8*  reg_addr = ade_base + RD_CH_CMDQ1_PE_REG;
    writel(val, reg_addr);
}
inline void set_CMDQ_RDMA1_CTRL(u8* ade_base, u32 val)
{
    u8*  reg_addr = ade_base + RD_CH_CMDQ1_CTRL_REG;
    writel(val, reg_addr);
}
inline void set_CMDQ_RDMA1_ADDR(u8* ade_base, u32 val)
{
    u8*  reg_addr = ade_base + RD_CH_CMDQ1_ADDR_REG;
    writel(val, reg_addr);
}
inline void set_CMDQ_RDMA1_LEN(u8* ade_base, u32 val)
{
    u8*  reg_addr = ade_base + RD_CH_CMDQ1_LEN_REG;
    writel(val, reg_addr);
}
inline void set_CMDQ_RDMA1_EN(u8* ade_base, u32 val)
{
    u8*  reg_addr = ade_base + RD_CH_CMDQ1_EN_REG;
    writel(val, reg_addr);
}
inline void set_CMDQ_RDMA2_PE(u8* ade_base, u32 val)
{
    u8*  reg_addr = ade_base + RD_CH_CMDQ2_PE_REG;
    writel(val, reg_addr);
}
inline void set_CMDQ_RDMA2_CTRL(u8* ade_base, u32 val)
{
    u8*  reg_addr = ade_base + RD_CH_CMDQ2_CTRL_REG;
    writel(val, reg_addr);
}
inline void set_CMDQ_RDMA2_ADDR(u8* ade_base, u32 val)
{
    u8*  reg_addr = ade_base + RD_CH_CMDQ2_ADDR_REG;
    writel(val, reg_addr);
}
inline void set_CMDQ_RDMA2_LEN(u8* ade_base, u32 val)
{
    u8*  reg_addr = ade_base + RD_CH_CMDQ2_LEN_REG;
    writel(val, reg_addr);
}
inline void set_CMDQ_RDMA2_EN(u8* ade_base, u32 val)
{
    u8*  reg_addr = ade_base + RD_CH_CMDQ2_EN_REG;
    writel(val, reg_addr);
}

inline void set_TOP_INTR_MASK_CPU1(u8* ade_base, u32 val)
{
    u8*  reg_addr = ade_base + INTR_MASK_CPU1_REG;
    writel(val, reg_addr);
}

inline void set_TOP_DMA_AXI_MUX(u8* ade_base, u32 ch_num, u32 ch_type)
{
    //return;    //Temporary modify
	u8*  reg_addr = ade_base + ADE_DMA_AXI_MUX_REG;
	u32  axi_num;
	volatile union	U_ADE_DMA_AXI_MUX dma_axi_mux;

	dma_axi_mux.u32 = readl(reg_addr);

	dma_axi_mux.bits.rd_dma_cmdq2_axi_cfg = 0;
	dma_axi_mux.bits.rd_dma_cmdq1_axi_cfg = 0;
	if (OVERLAY_PIPE_TYPE_ONLINE == ch_type) {
		axi_num = 0;
		dma_axi_mux.bits.wr_dma_ch2_axi_cfg = axi_num;
	} else {
		axi_num = 1;
		dma_axi_mux.bits.wr_dma_ch1_axi_cfg = axi_num;
		dma_axi_mux.bits.wr_dma_ch3_axi_cfg = axi_num;
	}

	switch (ch_num) {
        case OVERLAY_PIPE_ADE_CH1:
		dma_axi_mux.bits.rd_dma_ch1_axi_cfg = axi_num;
		break;
        case OVERLAY_PIPE_ADE_CH2:
		dma_axi_mux.bits.rd_dma_ch2_axi_cfg = axi_num;
		break;
        case OVERLAY_PIPE_ADE_CH3:
		dma_axi_mux.bits.rd_dma_ch3_axi_cfg = axi_num;
		break;
        case OVERLAY_PIPE_ADE_CH4:
		dma_axi_mux.bits.rd_dma_ch4_axi_cfg = axi_num;
		break;
        case OVERLAY_PIPE_ADE_CH5:
		dma_axi_mux.bits.rd_dma_ch5_axi_cfg = axi_num;
		break;
        case OVERLAY_PIPE_ADE_CH6:
		dma_axi_mux.bits.rd_dma_ch6_axi_cfg = axi_num;
		break;
        case OVERLAY_PIPE_ADE_DISP:
		dma_axi_mux.bits.rd_dma_disp_axi_cfg = 0;/* disp channel is always online */
		break;
        default:
		break;
	}

	writel(dma_axi_mux.u32, reg_addr);
}
#endif
