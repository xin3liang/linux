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
#include <drm/drm_plane_helper.h>
#include <drm/drm_atomic_helper.h>

#include "hisi_drm_drv.h"
#include "hisi_drm_plane.h"
#include "hisi_drm_crtc.h"
#include "hisi_ade.h"

#define to_hisi_plane(plane) \
		container_of(plane, struct hisi_plane, base)

static void hisi_plane_atomic_disable(struct drm_plane *plane,
			       struct drm_plane_state *old_state)
{
	struct hisi_plane *hplane = to_hisi_plane(plane);
	struct hisi_plane_funcs *ops = hplane->ops;

	DRM_DEBUG_DRIVER("enter.\n");

	if (!old_state->crtc)
		return;
	
	if (ops->atomic_disable)
		ops->atomic_disable(hplane, old_state);

	DRM_DEBUG_DRIVER("exit success.\n");
}

static void hisi_plane_atomic_update(struct drm_plane *plane,
				       struct drm_plane_state *old_state)
{
	struct hisi_plane *hplane = to_hisi_plane(plane);
	struct hisi_plane_funcs *ops = hplane->ops;
	struct drm_plane_state	*hstate	= plane->state;

	DRM_DEBUG_DRIVER("enter.\n");
	if (!hstate->crtc)
		return;
	
	if (ops->atomic_update)
		ops->atomic_update(hplane, old_state);

	DRM_DEBUG_DRIVER("exit success.\n");
}

int hisi_plane_atomic_check(struct drm_plane *plane,
				     struct drm_plane_state *state)
{
	DRM_DEBUG_DRIVER("enter.\n");
	DRM_DEBUG_DRIVER("exit success.\n");
	return 0;
}

void hisi_plane_cleanup_fb(struct drm_plane *plane,
				struct drm_framebuffer *fb,
				const struct drm_plane_state *old_state)
{
	DRM_DEBUG_DRIVER("enter.\n");
	DRM_DEBUG_DRIVER("exit success.\n");
}

int hisi_plane_prepare_fb(struct drm_plane *p,
				struct drm_framebuffer *fb,
				const struct drm_plane_state *new_state)
{
	DRM_DEBUG_DRIVER("enter.\n");
	DRM_DEBUG_DRIVER("exit success.\n");
	return 0;
}

static const struct drm_plane_helper_funcs hisi_plane_helper_funcs = {
	.prepare_fb = hisi_plane_prepare_fb,
	.cleanup_fb = hisi_plane_cleanup_fb,
	.atomic_check = hisi_plane_atomic_check,
	.atomic_update = hisi_plane_atomic_update,
	.atomic_disable = hisi_plane_atomic_disable,
};

void hisi_plane_destroy(struct drm_plane *plane)
{
	drm_plane_cleanup(plane);
}

static void hisi_plane_atomic_reset(struct drm_plane *plane)
{
	struct hisi_plane *hplane = to_hisi_plane(plane);
	struct hisi_plane_state *hstate;

	if (plane->state && plane->state->fb)
		drm_framebuffer_unreference(plane->state->fb);

	kfree(plane->state);
	plane->state = NULL;

	hstate = kzalloc(sizeof(*hstate), GFP_KERNEL);
	if (hstate == NULL)
		return;

	/* set to default value */
	hstate->zpos = hplane->ch;
	hstate->base.rotation = BIT(DRM_ROTATE_0);
	hstate->alpha = 255;
	hstate->blending = ALPHA_BLENDING_NONE;

	plane->state = &hstate->base;
	plane->state->plane = plane;
}

static struct drm_plane_state *
hisi_plane_atomic_duplicate_state(struct drm_plane *plane)
{
	struct hisi_plane_state *hstate;
	struct hisi_plane_state *copy;

	if (WARN_ON(!plane->state))
		return NULL;

	hstate = to_hisi_plane_state(plane->state);
	copy = kmemdup(hstate, sizeof(*hstate), GFP_KERNEL);
	if (copy == NULL)
		return NULL;

	__drm_atomic_helper_plane_duplicate_state(plane, &copy->base);

	return &copy->base;
}

static void hisi_plane_atomic_destroy_state(struct drm_plane *plane,
					    struct drm_plane_state *state)
{
	__drm_atomic_helper_plane_destroy_state(plane, state);
	kfree(to_hisi_plane_state(state));
}


static int hisi_plane_atomic_set_property(struct drm_plane *plane,
					  struct drm_plane_state *state,
					  struct drm_property *property,
					  uint64_t val)
{
	struct hisi_drm_private *priv = plane->dev->dev_private;
	struct hisi_plane_state *hstate = to_hisi_plane_state(state);

	if (property == priv->zpos_prop)
		hstate->zpos = val;
	else if (property == priv->alpha_prop)
		hstate->alpha = val;
	else if (property == priv->blending_prop)
		hstate->blending = val;
	else
		return -EINVAL;

	return 0;
}

static int hisi_plane_atomic_get_property(struct drm_plane *plane,
					  const struct drm_plane_state *state,
					  struct drm_property *property,
					  uint64_t *val)
{
	struct hisi_drm_private *priv = plane->dev->dev_private;
	const struct hisi_plane_state *hstate = to_hisi_plane_state(state);

	if (property == priv->zpos_prop)
		*val = hstate->zpos;
	else if (property == priv->alpha_prop)
		*val = hstate->alpha;
	else if (property == priv->blending_prop)
		*val = hstate->blending;
	else
		return -EINVAL;

	return 0;
}

static struct drm_plane_funcs hisi_plane_funcs = {
	.update_plane	= drm_atomic_helper_update_plane,
	.disable_plane	= drm_atomic_helper_disable_plane,
	.set_property = drm_atomic_helper_plane_set_property,
	.destroy = hisi_plane_destroy,
	.reset = hisi_plane_atomic_reset,
	.atomic_duplicate_state = hisi_plane_atomic_duplicate_state,
	.atomic_destroy_state = hisi_plane_atomic_destroy_state,
	.atomic_set_property = hisi_plane_atomic_set_property,
	.atomic_get_property = hisi_plane_atomic_get_property,
};

int hisi_drm_plane_init(struct drm_device *dev,
			struct hisi_plane *hplane,
				enum drm_plane_type type)
{
	struct hisi_plane_funcs *ops = hplane->ops;
	const u32 *fmts;
	u32 fmts_cnt;
	int ret = 0;

	/* get  properties */
	fmts_cnt = ops->get_properties(hplane->ch, &fmts);
	if (ret)
		return ret;

	ret = drm_universal_plane_init(dev, &hplane->base, 1,
				&hisi_plane_funcs, fmts, fmts_cnt, type);
	if (ret) {
		DRM_ERROR("fail to init plane, ch=%d\n", hplane->ch);
		return ret;
	}

	drm_plane_helper_add(&hplane->base, &hisi_plane_helper_funcs);

	/* install  properties */
	if (ops->install_properties) {
		ret = ops->install_properties(dev, hplane);
		if (ret)
			return ret;
	}

	return 0;
}
