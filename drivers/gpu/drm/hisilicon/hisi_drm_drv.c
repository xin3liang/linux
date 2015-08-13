/*
#endif
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/of_platform.h>
#include <linux/component.h>

#include <drm/drmP.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>

#include "hisi_drm_drv.h"
#include "hisi_drm_fb.h"
#include "hisi_ade.h"
#ifdef CONFIG_DRM_HISI_FBDEV
#include "hisi_drm_fbdev.h"
#endif

#define DRIVER_NAME	"hisi-drm"

static int hisi_drm_unload(struct drm_device *dev)
{
	struct hisi_drm_private *priv = dev->dev_private;

	drm_vblank_cleanup(dev);
	drm_mode_config_cleanup(dev);
	devm_kfree(dev->dev, priv);
	dev->dev_private = NULL;
	return 0;
}

static const struct drm_mode_config_funcs hisi_drm_mode_config_funcs = {
	.fb_create = hisi_drm_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static void hisi_drm_mode_config_init(struct drm_device *dev)
{
	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	dev->mode_config.max_width = 2048;
	dev->mode_config.max_height = 2048;

	dev->mode_config.funcs = &hisi_drm_mode_config_funcs;
}

static int hisi_drm_load(struct drm_device *dev, unsigned long flags)
{
	struct hisi_drm_private *priv;
	int ret;

	/* debug setting
	drm_debug = DRM_UT_DRIVER|DRM_UT_KMS; */
	DRM_DEBUG_DRIVER("enter.\n");

	priv = devm_kzalloc(dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev->dev_private = priv;
	dev_set_drvdata(dev->dev, dev);

	/* dev->mode_config initialization */
	drm_mode_config_init(dev);
	hisi_drm_mode_config_init(dev);

	/* only support one crtc now */
	ret = drm_vblank_init(dev, 1);
	if (ret) {
		DRM_ERROR("failed to initialize vblank\n");
	}

	ret = component_bind_all(dev->dev, dev);
	if (ret)
		return ret;

	DRM_DEBUG_DRIVER("exit successfully.\n");
	return 0;
}

static const struct file_operations hisi_drm_fops = {
	.owner		= THIS_MODULE,
	.open		= drm_open,
	.release	= drm_release,
	.unlocked_ioctl	= drm_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= drm_compat_ioctl,
#endif
	.poll		= drm_poll,
	.read		= drm_read,
	.llseek		= no_llseek,
	.mmap		= drm_gem_cma_mmap,
};

static struct dma_buf *hisi_drm_gem_prime_export(struct drm_device *dev,
						struct drm_gem_object *obj,
						int flags)
{
	/* we want to be able to write in mmapped buffer */
	flags |= O_RDWR;
	return drm_gem_prime_export(dev, obj, flags);
}

static struct drm_driver hisi_drm_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_PRIME
				| DRIVER_HAVE_IRQ | DRIVER_IRQ_SHARED,
	.load			= hisi_drm_load,
	.unload                 = hisi_drm_unload,
	.fops			= &hisi_drm_fops,
	.set_busid		= drm_platform_set_busid,

	.gem_free_object	= drm_gem_cma_free_object,
	.gem_vm_ops		= &drm_gem_cma_vm_ops,
	.dumb_create		= drm_gem_cma_dumb_create,
	.dumb_map_offset	= drm_gem_cma_dumb_map_offset,
	.dumb_destroy		= drm_gem_dumb_destroy,

	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_export	= hisi_drm_gem_prime_export,
	.gem_prime_import	= drm_gem_prime_import,
	.gem_prime_get_sg_table = drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap		= drm_gem_cma_prime_vmap,
	.gem_prime_vunmap	= drm_gem_cma_prime_vunmap,
	.gem_prime_mmap		= drm_gem_cma_prime_mmap,

	.irq_handler		= hisi_drm_irq_handler,

	.get_vblank_counter	= drm_vblank_count,
	.enable_vblank		= hisi_drm_enable_vblank,
	.disable_vblank		= hisi_drm_disable_vblank,

	.name			= "hisi",
	.desc			= "Hisilicon Terminal SoCs DRM Driver",
	.date			= "20150718",
	.major			= 1,
	.minor			= 0,
};

/* -----------------------------------------------------------------------------
 * Platform driver
 */

static int compare_of(struct device *dev, void *data)
{
	return dev->of_node == data;
}

static int hisi_drm_bind(struct device *dev)
{

	dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	return drm_platform_init(&hisi_drm_driver, to_platform_device(dev));
}

static void hisi_drm_unbind(struct device *dev)
{
	drm_put_dev(dev_get_drvdata(dev));
}

static const struct component_master_ops hisi_drm_ops = {
	.bind = hisi_drm_bind,
	.unbind = hisi_drm_unbind,
};

static int hisi_drm_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct device_node *child_np;
	struct component_match *match = NULL;

	of_platform_populate(node, NULL, NULL, dev);

	child_np = of_get_next_available_child(node, NULL);
	while (child_np) {
		component_match_add(dev, &match, compare_of, child_np);
		of_node_put(child_np);
		child_np = of_get_next_available_child(node, child_np);
	}

	return component_master_add_with_match(dev, &hisi_drm_ops, match);

	return 0;
}

static int hisi_drm_platform_remove(struct platform_device *pdev)
{
	component_master_del(&pdev->dev, &hisi_drm_ops);
	of_platform_depopulate(&pdev->dev);
	return 0;
}

static const struct of_device_id hisi_drm_dt_ids[] = {
	{ .compatible = "hisilicon,display-subsystem", },
	{ /* end node */ },
};
MODULE_DEVICE_TABLE(of, hisi_drm_dt_ids);

static struct platform_driver hisi_drm_platform_driver = {
	.probe = hisi_drm_platform_probe,
	.remove = hisi_drm_platform_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
		.of_match_table = hisi_drm_dt_ids,
	},
};

module_platform_driver(hisi_drm_platform_driver);

MODULE_AUTHOR("Xinwei Kong <kong.kongxinwei@hisilicon.com>");
MODULE_AUTHOR("Xinliang Liu <z.liuxinliang@huawei.com>");
MODULE_DESCRIPTION("hisilicon SoC DRM driver");
MODULE_LICENSE("GPL v2");
