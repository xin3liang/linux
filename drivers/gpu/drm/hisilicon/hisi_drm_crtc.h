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

#ifndef __HISI_DRM_CRTC_H__
#define __HISI_DRM_CRTC_H__
#include "hisi_handle.h"

struct hisi_crtc *hisi_drm_crtc_init (struct drm_device *drm_dev,
	struct hisi_handle *hisi_handle, struct drm_plane *p);

#endif
