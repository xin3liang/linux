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

	/* plane properties */
	struct drm_property *zpos_prop;
	struct drm_property *alpha_prop;
	struct drm_property *blending_prop;

	/* read only capabilities properties */
	struct drm_property *cap_scl_prop; /* 0: unsupport, 1: support */
	struct drm_property *cap_rot_prop;  /*  */
};

enum {
	/* no blending */
	ALPHA_BLENDING_NONE     = 0x0100,
	/* ONE / ONE_MINUS_SRC_ALPHA */
	ALPHA_BLENDING_PREMULT  = 0x0105,
	/* SRC_ALPHA / ONE_MINUS_SRC_ALPHA */
	ALPHA_BLENDING_COVERAGE = 0x0405
};

/* plane structs  */
struct hisi_plane_state {
	struct drm_plane_state base;
	u8 zpos;  /* z order */
	u8 alpha; /* Alpha value applied to the whole plane */
	u32 blending; /* blending cases: none, premult and coverage */
};

#endif /* __HISI_DRM_DRV_H__ */
