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

#include <drm/drm_encoder_slave.h>
#include "hisi_drm_drv.h"

#include "hisi_ade.h"
#include "hisi_drm_plane.h"

static const struct drm_plane_helper_funcs hisi_plane_helper_funcs = {
	.prepare_fb = hisi_plane_prepare_fb,
	.cleanup_fb = hisi_plane_cleanup_fb,
	.atomic_check = hisi_plane_atomic_check,
	.atomic_update = hisi_plane_atomic_update,
	.atomic_disable = hisi_plane_atomic_disable,
};

static void hisi_plane_atomic_reset(struct drm_plane *plane)
{
	struct hisi_plane *hplane = to_hisi_plane(plane);
	struct hisi_plane_state *state;

	if (plane->state && plane->state->fb)
		drm_framebuffer_unreference(plane->state->fb);

	kfree(plane->state);
	plane->state = NULL;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state == NULL)
		return;

	/* set to default value */
	state->zpos = hplane->ch;
	state->base.rotation = BIT(DRM_ROTATE_0);
	state->alpha = 255;
	state->blending = ALPHA_BLENDING_NONE;

	plane->state = &state->base;
	plane->state->plane = plane;
}

static struct drm_plane_state *
hisi_plane_atomic_duplicate_state(struct drm_plane *plane)
{
	struct hisi_plane_state *state;
	struct hisi_plane_state *copy;

	if (WARN_ON(!plane->state))
		return NULL;

	state = to_hisi_plane_state(plane->state);
	copy = kmemdup(state, sizeof(*state), GFP_KERNEL);
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
	struct hisi_plane *hplane = to_hisi_plane(plane);
	struct hisi_drm_private *priv = plane->dev->dev_private;
	struct hisi_plane_state *hstate = to_hisi_plane_state(state);

	DRM_DEBUG_DRIVER("\"%s\": channel%d, val=%d\n", property->name,
			hplane->ch, val);

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
