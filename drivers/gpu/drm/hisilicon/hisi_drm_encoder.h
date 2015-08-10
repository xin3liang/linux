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

#include "hisi_dsi.h"

static inline struct drm_encoder_slave_funcs *
		get_slave_funcs(struct drm_encoder *enc)
{
	return to_encoder_slave(enc)->slave_funcs;
}

void hisi_drm_encoder_init(struct drm_device *dev, struct drm_encoder *encoder);

#endif
