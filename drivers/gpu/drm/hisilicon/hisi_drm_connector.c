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

static struct drm_connector_helper_funcs hisi_dsi_connector_helper_funcs = {
	.get_modes = hisi_dsi_get_modes,
	.best_encoder =  hisi_dsi_best_encoder,
	.mode_valid = hisi_drm_connector_mode_valid,
};

static struct drm_connector_funcs hisi_dsi_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = hisi_dsi_detect,
	.destroy = hisi_dsi_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

void hisi_drm_connector_create(struct drm_device *dev, struct hisi_dsi *dsi)
{
	int ret;
	struct drm_encoder *encoder = &dsi->base.base;
	struct drm_connector *connector = &dsi->connector;

	DRM_DEBUG_DRIVER("enter.\n");
	connector->polled = DRM_CONNECTOR_POLL_HPD;
	connector->dpms = DRM_MODE_DPMS_OFF;
	ret = drm_connector_init(encoder->dev, connector,
					&hisi_dsi_connector_funcs,
					DRM_MODE_CONNECTOR_HDMIA);
	drm_connector_helper_add(connector, &hisi_dsi_connector_helper_funcs);
	drm_connector_register(connector);
	drm_mode_connector_attach_encoder(connector, encoder);

#ifndef CONFIG_DRM_HISI_FBDEV
	drm_reinit_primary_mode_group(dev);
#endif
	drm_mode_config_reset(dev);
	DRM_DEBUG_DRIVER("exit success.\n");
}

