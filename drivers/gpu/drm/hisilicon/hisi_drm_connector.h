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

#ifndef __HISI_DRM_CONNECTOR_H__
#define __HISI_DRM_CONNECTOR_H__

struct hisi_connector_funcs {
	enum drm_connector_status (*detect)
					(struct drm_connector *connector);
	int (*get_modes)(struct drm_connector *connector);
	int (*modes_valid)(struct drm_connector *connector);
};

struct hisi_connector {
	struct drm_connector connector;
	struct drm_encoder *encoder;
	void *ops;
};

void hisi_drm_connector_init(struct drm_device *dev, struct drm_encoder *encoder,
							struct drm_connector *connector);

#endif
