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

#ifndef __HISI_DRM_PLANE_H__
#define __HISI_DRM_PLANE_H__

//#define to_hisi_plane(p)        container_of(p, struct hisi_plane, base)
#define to_hisi_plane_state(s)  container_of(s, struct hisi_plane_state, base)

enum {
	/* no blending */
	ALPHA_BLENDING_NONE     = 0x0100,
	/* ONE / ONE_MINUS_SRC_ALPHA */
	ALPHA_BLENDING_PREMULT  = 0x0105,
	/* SRC_ALPHA / ONE_MINUS_SRC_ALPHA */
	ALPHA_BLENDING_COVERAGE = 0x0405
};

struct hisi_plane {
	struct drm_plane base;
	void *ops;
	void *ctx;
	u8 ch; /* channel or pipe */
//	u8 zorder;
};

struct hisi_plane_funcs {
	u32 (*get_properties)(u8 ch, const u32 **formats);
	int (*install_properties)(struct drm_device *dev,
					struct hisi_plane *hplane);
	void (*atomic_update)(struct hisi_plane *hplane,
				       struct drm_plane_state *old_state);
	void (*atomic_disable)(struct hisi_plane *hplane,
				       struct drm_plane_state *old_state);
};

/* plane structs  */
struct hisi_plane_state {
        struct drm_plane_state base;
	u8 zpos;  /* z order */
	u8 alpha; /* Alpha value applied to the whole plane */
	u32 blending; /* blending cases: none, premult and coverage */
};

int hisi_drm_plane_init(struct drm_device *dev,
			struct hisi_plane *hplane,
				enum drm_plane_type type);


#endif
