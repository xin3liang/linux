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

#include <drm/drmP.h>

#include <drm/drm_crtc.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_atomic_helper.h>

#include "hisi_drm_plane.h"
#include "hisi_ade.h"

static const struct drm_plane_helper_funcs hisi_plane_helper_funcs = {
	.prepare_fb = hisi_plane_prepare_fb,
	.cleanup_fb = hisi_plane_cleanup_fb,
	.atomic_check = hisi_plane_atomic_check,
	.atomic_update = hisi_plane_atomic_update,
	.atomic_disable = hisi_plane_atomic_disable,
};

static struct drm_plane_funcs hisi_plane_funcs = {
	.update_plane	= drm_atomic_helper_update_plane,
	.disable_plane	= drm_atomic_helper_disable_plane,
	.set_property = drm_atomic_helper_plane_set_property,
	.destroy	= hisi_plane_destroy,
	.reset = drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

struct hisi_plane *hisi_drm_plane_init(struct drm_device *dev,
					   void *ctx,
					   u32 ch,
					   const u32 *formats,
					   u32 formats_cnt,
					   enum drm_plane_type type)
{
	struct hisi_plane *plane;
	int ret;

	plane = devm_kzalloc(dev->dev, sizeof(*plane), GFP_KERNEL);
	if (!plane)
		return ERR_PTR(-ENOMEM);
	
	plane->ctx = ctx;
	plane->ch = ch;
	plane->zorder = ch;
	
	DRM_DEBUG_DRIVER("plane init: ch=%d,type=%d, formats_count=%d\n",
			ch, type, formats_cnt);
	ret = drm_universal_plane_init(dev, &plane->base, 1,
					&hisi_plane_funcs,
					formats,
					formats_cnt,
					type);
	if (ret) {
		DRM_ERROR("fail to init plane\n");
		return ERR_PTR(ret);
	}

	drm_plane_helper_add(&plane->base, &hisi_plane_helper_funcs);

	/* install  properties */
	ade_install_plane_properties(dev, plane);
	
	return plane;
}
