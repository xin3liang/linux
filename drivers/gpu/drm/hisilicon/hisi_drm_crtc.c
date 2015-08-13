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
#include "hisi_drm_plane.h"
#include "hisi_drm_crtc.h"
#include "hisi_ade.h"

#define to_hisi_crtc(c)		container_of(c, struct hisi_crtc, base)

static void  hisi_drm_crtc_enable(struct drm_crtc *crtc)
{
	struct hisi_crtc *hcrtc = to_hisi_crtc(crtc);
	struct hisi_crtc_ops *ops = hcrtc->ops;

	if (hcrtc->enable)
		return;

	if (ops->enable)
		ops->enable(hcrtc);
	drm_crtc_vblank_on(crtc);

	hcrtc->enable = true;
}

static void hisi_drm_crtc_disable(struct drm_crtc *crtc)
{
	struct hisi_crtc *hcrtc = to_hisi_crtc(crtc);
	struct hisi_crtc_ops *ops = hcrtc->ops;

	if (!hcrtc->enable)
		return;

	drm_crtc_vblank_off(crtc);
	if (ops->disable)
		ops->disable(hcrtc);

	hcrtc->enable = false;
}

static void hisi_drm_crtc_mode_prepare(struct drm_crtc *crtc)
{
	struct hisi_crtc *hcrtc = to_hisi_crtc(crtc);
	struct hisi_crtc_ops *ops = hcrtc->ops;

	if (ops->mode_prepare)
		ops->mode_prepare(hcrtc);
}

static bool hisi_drm_crtc_mode_fixup(struct drm_crtc *crtc,
			      const struct drm_display_mode *mode,
			      struct drm_display_mode *adj_mode)
{
	struct hisi_crtc *hcrtc = to_hisi_crtc(crtc);
	struct hisi_crtc_ops *ops = hcrtc->ops;
	bool ret = true;

	if (ops->mode_fixup)
		ret = ops->mode_fixup(hcrtc, mode, adj_mode);

	return ret;
}

static void hisi_drm_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct hisi_crtc *hcrtc = to_hisi_crtc(crtc);
	struct hisi_crtc_ops *ops = hcrtc->ops;

	if (ops->mode_set_nofb)
		ops->mode_set_nofb(hcrtc);
}

static void hisi_crtc_atomic_begin(struct drm_crtc *crtc)
{
	struct hisi_crtc *hcrtc = to_hisi_crtc(crtc);
	struct hisi_crtc_ops *ops = hcrtc->ops;

	if (ops->atomic_begin)
		ops->atomic_begin(hcrtc);
}

static void hisi_crtc_atomic_flush(struct drm_crtc *crtc)
{
	struct hisi_crtc *hcrtc = to_hisi_crtc(crtc);
	struct hisi_crtc_ops *ops = hcrtc->ops;

	if (ops->atomic_flush)
		ops->atomic_flush(hcrtc);
}

static const struct drm_crtc_helper_funcs crtc_helper_funcs = {
	.enable		= hisi_drm_crtc_enable,
	.disable	= hisi_drm_crtc_disable,
	.prepare	= hisi_drm_crtc_mode_prepare,
	.mode_fixup	= hisi_drm_crtc_mode_fixup,
	.mode_set_nofb	= hisi_drm_crtc_mode_set_nofb,
	.atomic_begin	= hisi_crtc_atomic_begin,
	.atomic_flush	= hisi_crtc_atomic_flush,
};

static void hisi_drm_crtc_destroy(struct drm_crtc *c)
{
	drm_crtc_cleanup(c);
}

static const struct drm_crtc_funcs crtc_funcs = {
	.destroy	= hisi_drm_crtc_destroy,
	.set_config	= drm_atomic_helper_set_config,
	.page_flip	= drm_atomic_helper_page_flip,
	.reset		= drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
};

int hisi_drm_crtc_init(struct drm_device *dev,
		       struct hisi_crtc *hcrtc,
		       struct drm_plane *plane)
{
	struct hisi_drm_private *priv = dev->dev_private;
	int ret;

	ret = drm_crtc_init_with_planes(dev, &hcrtc->base, plane,
					NULL, &crtc_funcs);
	if (ret) {
		DRM_ERROR("failed to init crtc.\n");
		return ret;
	}

	drm_crtc_helper_add(&hcrtc->base, &crtc_helper_funcs);

	priv->crtc = &hcrtc->base;
	
	return 0;
}
