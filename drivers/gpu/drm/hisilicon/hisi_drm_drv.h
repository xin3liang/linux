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

#endif /* __HISI_DRM_DRV_H__ */
