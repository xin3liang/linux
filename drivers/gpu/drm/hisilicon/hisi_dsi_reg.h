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
#ifndef __HISI_DSI_REG_H__
#define __HISI_DSI_REG_H__

#define  PWR_UP_ON                   (1)
#define  PWR_UP_OFF                  (0)       
#define  COMMAND_MODE                (1)
#define  VIDEO_MODE                  (0)

#define  PWR_UP                  (0x4)        /* Core power-up */
#define  PHY_IF_CFG              (0xA4)       /* D-PHY interface configuration */
#define  CLKMGR_CFG              (0x8)        /* Configuration of the internal clock dividers */
#define  PHY_RSTZ                (0xA0)       /* D-PHY reset control */
#define  PHY_TST_CTRL0           (0xB4)       /* D-PHY test interface control 0 */
#define  PHY_TST_CTRL1           (0xB8)       /* D-PHY test interface control 1 */
#define  DPI_VCID                (0xC)        /* DPI virtual channel id */
#define  DPI_COLOR_CODING        (0x10)       /* DPI color coding */
#define  DPI_CFG_POL             (0x14)       /* DPI polarity configuration */
#define  VID_HSA_TIME            (0x48)       /* Horizontal Sync Active time */
#define  VID_HBP_TIME            (0x4C)       /* Horizontal Back Porch time */
#define  VID_HLINE_TIME          (0x50)       /* Line time */
#define  VID_VSA_LINES           (0x54)       /* Vertical Sync Active period */
#define  VID_VBP_LINES           (0x58)       /* Vertical Back Porch period */
#define  VID_VFP_LINES           (0x5C)       /* Vertical Front Porch period */
#define  VID_VACTIVE_LINES       (0x60)       /* Vertical resolution */
#define  VID_PKT_SIZE            (0x3C)       /* Video packet size */
#define  VID_MODE_CFG            (0x38)       /* Video mode configuration */
#define  DPI_LP_CMD_TIM          (0x18)       /* Low-power command timing configuration */
#define  PHY_TMR_CFG             (0x9C)       /* D-PHY timing configuration for data lanes */
#define  BTA_TO_CNT              (0x8C)       /* Peripheral response timeout definition after bus turnaround */
#define  PHY_TMR_LPCLK_CFG       (0x98)       /* D-PHY timing configuration for the clock lane */
#define  NO_CONTINUE             (0xCC)
#define  LPCLK_CTRL              (0x94)       /* Low-power in clock lane */
#define  PCKHDL_CFG              (0x2C)       /* Packet handler configuration */
#define  EDPI_CMD_SIZE           (0x64)       /* Size for eDPI packets */
#define  MODE_CFG                (0x34)       /* Video or Command mode selection */
#define  PHY_STATUS              (0xB0)       /* D-PHY PPI status interface */

#endif /* __HISI_DRM_DSI_H__ */
