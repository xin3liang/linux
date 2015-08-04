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

#include <linux/io.h>
#include <linux/types.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <video/videomode.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_atomic_helper.h>

#include "hisi_dsi.h"

static struct drm_encoder_helper_funcs hisi_encoder_helper_funcs = {
	.mode_fixup	= hisi_drm_encoder_mode_fixup,
	.mode_set	= hisi_drm_encoder_mode_set,
	.enable		= hisi_drm_encoder_enable,
	.disable	= hisi_drm_encoder_disable
};

static struct drm_encoder_funcs hisi_encoder_funcs = {
	.destroy = hisi_drm_encoder_destroy
};

int hisi_drm_encoder_create(struct drm_device *dev, struct hisi_dsi *dsi)
{
	/* int ret; */
	struct drm_encoder *encoder = &dsi->base.base;
	int ret;

	DRM_DEBUG_DRIVER("enter.\n");
	dsi->enable = false;
	encoder->possible_crtcs = 1;
	
	drm_encoder_init(dev, encoder, &hisi_encoder_funcs, DRM_MODE_ENCODER_TMDS);
	drm_encoder_helper_add(encoder, &hisi_encoder_helper_funcs);

	ret = dsi->drm_i2c_driver->encoder_init(dsi->client, dev, &dsi->base);
	if (ret) {
		DRM_ERROR("drm_i2c_encoder_init error\n");
		return ret;
	}

	if (!dsi->base.slave_funcs) {
		DRM_ERROR("failed check encoder function\n");
		return -ENODEV;
	}

	return 0;
	DRM_DEBUG_DRIVER("exit success.\n");
}
