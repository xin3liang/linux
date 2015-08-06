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
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>

#include "hisi_drm_drv.h"
#include "hisi_ade.h"

static const struct drm_crtc_helper_funcs crtc_helper_funcs = {
	.enable		= hisi_crtc_enable,
	.disable	= hisi_crtc_disable,
	.prepare	= hisi_drm_crtc_mode_prepare,
	.mode_fixup	= hisi_drm_crtc_mode_fixup,
	.mode_set_nofb	= hisi_drm_crtc_mode_set_nofb,
	.atomic_begin	= hisi_crtc_atomic_begin,
	.atomic_flush	= hisi_crtc_atomic_flush,
};

static const struct drm_crtc_funcs crtc_funcs = {
	.destroy	= hisi_drm_crtc_destroy,
	.set_config	= drm_atomic_helper_set_config,
	.page_flip	= drm_atomic_helper_page_flip,
	.reset		= drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
};

struct hisi_crtc *hisi_drm_crtc_create(struct drm_device  *dev,
						  void *ctx,
						  struct drm_plane *p)
{
	struct hisi_crtc *crtc;
	struct hisi_drm_private *private = dev->dev_private;
	int ret;

	crtc = devm_kzalloc(dev->dev, sizeof(*crtc), GFP_KERNEL);
	if (!crtc)
		return ERR_PTR(-ENOMEM);

	ret = drm_crtc_init_with_planes(dev, &crtc->base, p,
					NULL, &crtc_funcs);
	if (ret)
		return ERR_PTR(ret);

	drm_crtc_helper_add(&crtc->base, &crtc_helper_funcs);

	private->crtc = &crtc->base;
	crtc->ctx = ctx;
	return crtc;
}
