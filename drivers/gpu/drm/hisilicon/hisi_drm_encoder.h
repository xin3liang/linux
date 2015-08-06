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

int hisi_drm_encoder_create(struct drm_device *dev, struct hisi_dsi *dsi);

#endif
