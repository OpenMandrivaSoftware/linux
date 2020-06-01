// SPDX-License-Identifier: GPL-2.0-only
/*
 * copyright (c) 2019-2020 nxp semiconductor, inc.
 *
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <drm/drm_of.h>
#include <drm/drm_vblank.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_encoder_slave.h>

#include "cdns-mhdp-phy.h"
#include "../imx-drm.h"

struct imx_mhdp_device {
	struct cdns_mhdp_device mhdp;
	struct drm_encoder encoder;
};

static const struct drm_encoder_funcs cdns_mhdp_imx_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static struct cdns_plat_data imx8mq_hdmi_drv_data = {
	.plat_name = "imx8mq-hdmi",
	.bind	= cdns_hdmi_bind,
	.unbind	= cdns_hdmi_unbind,
	.phy_set = cdns_hdmi_phy_set_imx8mq,
	.phy_video_valid = cdns_hdmi_phy_mode_valid_imx8mq,
	.lane_mapping = 0xe4,
};

static struct cdns_plat_data imx8mq_dp_drv_data = {
	.plat_name = "imx8mq-dp",
	.bind	= cdns_dp_bind,
	.unbind	= cdns_dp_unbind,
	.phy_set = cdns_dp_phy_set_imx8mq,
	.lane_mapping = 0xc6,
};

static const struct of_device_id cdns_mhdp_imx_dt_ids[] = {
	{ .compatible = "nxp,imx8mq-cdns-hdmi",
	  .data = &imx8mq_hdmi_drv_data
	},
	{ .compatible = "nxp,imx8mq-cdns-dp",
	  .data = &imx8mq_dp_drv_data
	},
	{},
};
MODULE_DEVICE_TABLE(of, cdns_mhdp_imx_dt_ids);

static int cdns_mhdp_imx_bind(struct device *dev, struct device *master,
			    void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	const struct cdns_plat_data *plat_data;
	const struct of_device_id *match;
	struct drm_device *drm = data;
	struct drm_encoder *encoder;
	struct imx_mhdp_device *imx_mhdp;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	imx_mhdp = devm_kzalloc(&pdev->dev, sizeof(*imx_mhdp), GFP_KERNEL);
	if (!imx_mhdp)
		return -ENOMEM;

	match = of_match_node(cdns_mhdp_imx_dt_ids, pdev->dev.of_node);
	plat_data = match->data;
	encoder = &imx_mhdp->encoder;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);

	/*
	 * If we failed to find the CRTC(s) which this encoder is
	 * supposed to be connected to, it's because the CRTC has
	 * not been registered yet.  Defer probing, and hope that
	 * the required CRTC is added later.
	 */
	if (encoder->possible_crtcs == 0)
		return -EPROBE_DEFER;

	drm_encoder_init(drm, encoder, &cdns_mhdp_imx_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS, NULL);


	imx_mhdp->mhdp.plat_data = plat_data;
	imx_mhdp->mhdp.dev = dev;
	ret = plat_data->bind(pdev, encoder, &imx_mhdp->mhdp);
	/*
	 * If cdns_mhdp_bind() fails we'll never call cdns_mhdp_unbind(),
	 * which would have called the encoder cleanup.  Do it manually.
	 */
	if (ret < 0)
		drm_encoder_cleanup(encoder);

	return ret;
}

static void cdns_mhdp_imx_unbind(struct device *dev, struct device *master,
			       void *data)
{
	struct imx_mhdp_device *imx_mhdp = dev_get_drvdata(dev);

	imx_mhdp->mhdp.plat_data->unbind(dev);
}

static const struct component_ops cdns_mhdp_imx_ops = {
	.bind	= cdns_mhdp_imx_bind,
	.unbind	= cdns_mhdp_imx_unbind,
};

static int cdns_mhdp_imx_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &cdns_mhdp_imx_ops);
}

static int cdns_mhdp_imx_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &cdns_mhdp_imx_ops);

	return 0;
}

static struct platform_driver cdns_mhdp_imx_platform_driver = {
	.probe  = cdns_mhdp_imx_probe,
	.remove = cdns_mhdp_imx_remove,
	.driver = {
		.name = "cdns-mhdp-imx",
		.of_match_table = cdns_mhdp_imx_dt_ids,
	},
};

module_platform_driver(cdns_mhdp_imx_platform_driver);

MODULE_AUTHOR("Sandor YU <sandor.yu@nxp.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cdnsmhdp-imx");
