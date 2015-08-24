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
#include <video/videomode.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_atomic_helper.h>

#include "hisi_drm_drv.h"
#include "hisi_drm_encoder.h"
#include "hisi_drm_connector.h"

#define to_hisi_connector(connector) \
	container_of(connector, struct hisi_connector, connector)

int hisi_drm_connector_mode_valid(struct drm_connector *connector,
					  struct drm_display_mode *mode)
{
	struct hisi_connector *hconnector = to_hisi_connector(connector);
	struct drm_encoder *encoder = hconnector->encoder;
	struct hisi_connector_funcs *ops = hconnector->ops;
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	int ret = MODE_OK;

	DRM_DEBUG_DRIVER("enter.\n");

	if (ops->modes_valid)
		ops->modes_valid(connector);

	if (sfuncs && sfuncs->mode_valid) {
		ret = sfuncs->mode_valid(encoder, mode);
		if (ret != MODE_OK)
			return ret;
	}

	DRM_DEBUG_DRIVER("exit success. ret=%d\n", ret);
	return ret;
}

struct drm_encoder *
hisi_drm_best_encoder(struct drm_connector *connector)
{
	struct hisi_connector *hconnector = to_hisi_connector(connector);
	struct drm_encoder *encoder = hconnector->encoder;

	DRM_DEBUG_DRIVER("enter.\n");
	DRM_DEBUG_DRIVER("exit success.\n");

	return encoder;
}

int hisi_drm_get_modes(struct drm_connector *connector)
{
	struct hisi_connector *hconnector = to_hisi_connector(connector);
	struct hisi_connector_funcs *ops = hconnector->ops;
	int count = 0;

	DRM_DEBUG_DRIVER("enter.\n");

#if 0
	if (sfuncs && sfuncs->get_modes)
		count = sfuncs->get_modes(encoder, connector);
#else
	if (ops->get_modes)
		count += ops->get_modes(connector);
#endif


	DRM_DEBUG_DRIVER("exit success. count=%d\n", count);

	return count;
}

static struct drm_connector_helper_funcs hisi_drm_connector_helper_funcs = {
	.get_modes = hisi_drm_get_modes,
	.best_encoder =  hisi_drm_best_encoder,
	.mode_valid = hisi_drm_connector_mode_valid,
};

void hisi_drm_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

enum drm_connector_status
hisi_drm_detect(struct drm_connector *connector, bool force)
{
	struct hisi_connector *hconnector = to_hisi_connector(connector);
	struct drm_encoder *encoder = hconnector->encoder;
	struct hisi_connector_funcs *ops = hconnector->ops;
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	enum drm_connector_status status = connector_status_unknown;

	DRM_DEBUG_DRIVER("enter.\n");

	if (ops->detect)
		ops->detect(connector);

	if (sfuncs && sfuncs->detect)
		status = sfuncs->detect(encoder, connector);

	DRM_DEBUG_DRIVER("exit success. status=%d\n", status);
	return status;
}

static struct drm_connector_funcs hisi_drm_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = hisi_drm_detect,
	.destroy = hisi_drm_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

void hisi_drm_connector_init(struct drm_device *dev, struct drm_encoder *encoder,
							struct drm_connector *connector)
{
	int ret;

	DRM_DEBUG_DRIVER("enter.\n");
	connector->polled = DRM_CONNECTOR_POLL_HPD;
	connector->dpms = DRM_MODE_DPMS_OFF;
	ret = drm_connector_init(encoder->dev, connector,
					&hisi_drm_connector_funcs,
					DRM_MODE_CONNECTOR_HDMIA);
	drm_connector_helper_add(connector, &hisi_drm_connector_helper_funcs);
	drm_connector_register(connector);
	drm_mode_connector_attach_encoder(connector, encoder);

#ifndef CONFIG_DRM_HISI_FBDEV
	drm_reinit_primary_mode_group(dev);
#endif
	drm_mode_config_reset(dev);
	DRM_DEBUG_DRIVER("exit success.\n");
}
