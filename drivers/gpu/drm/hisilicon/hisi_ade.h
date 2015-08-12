/*
 * Hisilicon Terminal SoCs drm driver
 *
 * Copyright (c) 2014-2015 Hisilicon Limited.
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __HISI_DRM_ADE_H__
#define __HISI_DRM_ADE_H__

struct hisi_plane {
	struct drm_plane base;
	void *ctx;
	u8 ch;
};

struct hisi_crtc {
	struct drm_crtc base;
	void *ctx;
	struct drm_display_mode *dmode;
	bool enable;
	u32 ch_mask;
	u64 use_mask;
};

void hisi_crtc_atomic_flush(struct drm_crtc *c);
void hisi_crtc_atomic_begin(struct drm_crtc *c);
void hisi_drm_crtc_mode_set_nofb(struct drm_crtc *c);
bool hisi_drm_crtc_mode_fixup(struct drm_crtc *c,
				      const struct drm_display_mode *mode,
				      struct drm_display_mode *adj_mode);
void hisi_drm_crtc_mode_prepare(struct drm_crtc *c);
void hisi_crtc_disable(struct drm_crtc *c);
void hisi_crtc_enable(struct drm_crtc *c);
void hisi_drm_crtc_destroy(struct drm_crtc *c);
struct hisi_crtc *hisi_drm_crtc_create(struct drm_device  *dev,
						  void *ctx,
						  struct drm_plane *p);

void hisi_plane_atomic_disable(struct drm_plane *p,
			       struct drm_plane_state *old_state);
void hisi_plane_atomic_update(struct drm_plane *p,
				       struct drm_plane_state *old_state);
int hisi_plane_atomic_check(struct drm_plane *p,
				     struct drm_plane_state *state);
void hisi_plane_cleanup_fb(struct drm_plane *p,
				struct drm_framebuffer *fb,
				const struct drm_plane_state *old_state);
void hisi_plane_cleanup_fb(struct drm_plane *p,
				struct drm_framebuffer *fb,
				const struct drm_plane_state *old_state);
int hisi_plane_prepare_fb(struct drm_plane *p,
				struct drm_framebuffer *fb,
				const struct drm_plane_state *new_state);
void hisi_plane_destroy(struct drm_plane *p);
int ade_install_plane_properties(struct drm_device *dev,
					struct hisi_plane *plane);

extern int hisi_drm_ade_init(void);
extern void hisi_drm_ade_exit(void);

extern int hisi_drm_enable_vblank(struct drm_device *dev, int crtc);
extern void hisi_drm_disable_vblank(struct drm_device *dev, int crtc);

extern irqreturn_t hisi_drm_irq_handler(int irq, void *arg);

#endif /* __HISI_DRM_ADE_H__ */
