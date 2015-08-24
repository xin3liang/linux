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

struct hisi_crtc {
	struct drm_crtc base;
	void *ops;
	bool enable;
};

struct hisi_crtc_ops {
	void (*disable)(struct hisi_crtc *hcrtc);
	void (*enable)(struct hisi_crtc *hcrtc);
	void (*mode_prepare)(struct hisi_crtc *hcrtc);
	bool (*mode_fixup)(struct hisi_crtc *hcrtc,
			   const struct drm_display_mode *mode,
			   struct drm_display_mode *adj_mode);
	void (*mode_set_nofb)(struct hisi_crtc *hcrtc);
	void (*atomic_begin)(struct hisi_crtc *hcrtc);
	void (*atomic_flush)(struct hisi_crtc *hcrtc);
	void (*destroy)(struct hisi_crtc *hcrtc);
};

extern int hisi_drm_crtc_init(struct drm_device *dev,
		       struct hisi_crtc *crtc,
		       struct drm_plane *plane);

#endif
