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

#ifndef __HISI_DRM_DRV_H__
#define __HISI_DRM_DRV_H__

struct hisi_encoder_funcs {
	void (*destroy)(struct drm_encoder *encoder);
	bool (*mode_fixup)(struct drm_encoder *encoder,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode);
	void (*mode_set)(struct drm_encoder *encoder,
					struct drm_display_mode *mode,
					struct drm_display_mode *adjusted_mode);
	void (*enable)(struct drm_encoder *encoder);
	void (*disable)(struct drm_encoder *encoder);
};

struct hisi_encoder {
	struct drm_encoder_slave base;
	struct hisi_encoder_funcs *ops;
};

struct hisi_connector_funcs {
	enum drm_connector_status (*detect)
					(struct drm_connector *connector);
	int (*get_modes)(struct drm_connector *connector);
	int (*modes_valid)(struct drm_connector *connector);
};

struct hisi_connector {
	struct drm_connector connector;
	struct drm_encoder *encoder;
	struct hisi_connector_funcs *ops;
};

struct hisi_drm_private {
	struct hisi_drm_fbdev	*fbdev;
	struct drm_crtc		*crtc;
};

#endif /* __HISI_DRM_DRV_H__ */
