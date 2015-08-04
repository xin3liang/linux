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

#ifndef __HISI_DRM_PLANE_H__
#define __HISI_DRM_PLANE_H__

struct hisi_plane *hisi_drm_plane_init(struct drm_device *dev,
					   void *ctx,
					   u32 ch,
					   const u32 *formats,
					   u32 formats_cnt,
					   enum drm_plane_type type);
#endif
