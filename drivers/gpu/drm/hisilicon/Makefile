ccflags-y += -I$(srctree)/drivers/staging/android
hisi-drm-y := hisi_drm_drv.o \
	      hisi_drm_ade.o \
	      hisi_drm_dsi.o \
	      hisi_drm_fb.o \
	      hisi_mipi_reg.o \
	      hisi_ade_cmdqueue.o

obj-$(CONFIG_DRM_HISI)	+= hisi-drm.o
obj-$(CONFIG_DRM_HISI_FBDEV) += hisi_drm_fbdev.o
