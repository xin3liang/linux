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

#ifndef __HISI_DRM_ENCODER_H__
#define __HISI_DRM_ENCODER_H__

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
	void *ops;
};

static inline struct drm_encoder_slave_funcs *
		get_slave_funcs(struct drm_encoder *enc)
{
	return to_encoder_slave(enc)->slave_funcs;
}

void hisi_drm_encoder_init(struct drm_device *dev, struct drm_encoder *encoder);

#endif
