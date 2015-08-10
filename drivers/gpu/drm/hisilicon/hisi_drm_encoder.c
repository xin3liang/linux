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

#include <linux/io.h>
#include <linux/types.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <video/videomode.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_atomic_helper.h>

#include "hisi_dsi.h"
#include "hisi_drm_drv.h"
#include "hisi_drm_encoder.h"

#define get_hisi_encoder(encoder) \
	container_of(encoder, struct hisi_encoder, base.base)

void hisi_drm_encoder_disable(struct drm_encoder *encoder)
{
	struct hisi_encoder *hisi_encoder = get_hisi_encoder(encoder);
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	
	DRM_DEBUG_DRIVER("enter. \n");

	if (hisi_encoder->ops->enable)
		hisi_encoder->ops->disable(encoder);

	if (sfuncs && sfuncs->dpms)
		sfuncs->dpms(encoder, DRM_MODE_DPMS_OFF);

	DRM_DEBUG_DRIVER("exit success.\n");

}

void hisi_drm_encoder_enable(struct drm_encoder *encoder)
{
	struct hisi_encoder *hisi_encoder = get_hisi_encoder(encoder);
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	
	DRM_DEBUG_DRIVER("enter. \n");

	if (hisi_encoder->ops->enable)
		hisi_encoder->ops->enable(encoder);

	if (sfuncs && sfuncs->dpms)
		sfuncs->dpms(encoder, DRM_MODE_DPMS_ON);

	DRM_DEBUG_DRIVER("exit success.\n");
}

void hisi_drm_encoder_mode_set(struct drm_encoder *encoder,
					struct drm_display_mode *mode,
					struct drm_display_mode *adjusted_mode)
{

	struct hisi_encoder *hisi_encoder = get_hisi_encoder(encoder);
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);

	DRM_DEBUG_DRIVER("enter.\n");

	if (hisi_encoder->ops->mode_set)
		hisi_encoder->ops->mode_set(encoder, mode, adjusted_mode);

	if (sfuncs && sfuncs->mode_set)
		sfuncs->mode_set(encoder, mode, adjusted_mode);

	DRM_DEBUG_DRIVER("exit success");
}

bool
hisi_drm_encoder_mode_fixup(struct drm_encoder *encoder,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct hisi_encoder *hisi_encoder = get_hisi_encoder(encoder);
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);
	bool ret = true;

	DRM_DEBUG_DRIVER("enter.\n");

	if (hisi_encoder->ops->mode_fixup)
		hisi_encoder->ops->mode_fixup(encoder, mode, adjusted_mode);

	if (sfuncs && sfuncs->mode_fixup)
		ret = sfuncs->mode_fixup(encoder, mode, adjusted_mode);

	DRM_DEBUG_DRIVER("exit success.ret=%d\n", ret);

	return ret;
}

void hisi_drm_encoder_destroy(struct drm_encoder *encoder)
{
	struct hisi_encoder *hisi_encoder = get_hisi_encoder(encoder);
	struct drm_encoder_slave_funcs *sfuncs = get_slave_funcs(encoder);

	DRM_DEBUG_DRIVER("enter.\n");
	
	if (hisi_encoder->ops->destroy)
		hisi_encoder->ops->destroy(encoder);

	/*release*/
	if (sfuncs && sfuncs->destroy)
		sfuncs->destroy(encoder);

	drm_encoder_cleanup(encoder);
	
	DRM_DEBUG_DRIVER("exit success.\n");
}

static struct drm_encoder_helper_funcs hisi_encoder_helper_funcs = {
	.mode_fixup	= hisi_drm_encoder_mode_fixup,
	.mode_set	= hisi_drm_encoder_mode_set,
	.enable		= hisi_drm_encoder_enable,
	.disable	= hisi_drm_encoder_disable
};

static struct drm_encoder_funcs hisi_encoder_funcs = {
	.destroy = hisi_drm_encoder_destroy
};

void hisi_drm_encoder_init(struct drm_device *dev,
				struct drm_encoder *encoder)
{
	DRM_DEBUG_DRIVER("enter.\n");

	encoder->possible_crtcs = 1;

	drm_encoder_init(dev, encoder, &hisi_encoder_funcs, DRM_MODE_ENCODER_TMDS);
	drm_encoder_helper_add(encoder, &hisi_encoder_helper_funcs);

	DRM_DEBUG_DRIVER("exit success.\n");
}
