// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2012-2014,2017 The Linux Foundation. All rights reserved.
 * Copyright (c) 2018, Linaro Limited
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/phy/phy.h>
#include <linux/reset.h>

/* SSPHY control registers */
#define SS_PHY_CTRL0			0x6C
#define SS_PHY_CTRL1			0x70
#define SS_PHY_CTRL2			0x74
#define SS_PHY_CTRL4			0x7C

/* SS_PHY_CTRL_REG bits */
#define REF_SS_PHY_EN			BIT(0)
#define LANE0_PWR_PRESENT		BIT(2)
#define SWI_PCS_CLK_SEL			BIT(4)
#define TEST_POWERDOWN			BIT(4)
#define SS_PHY_RESET			BIT(7)

enum ssphy_vdd_level {
	LEVEL_NONE,
	LEVEL_MIN,
	LEVEL_MAX,
	LEVEL_NUM,
};

struct ssphy_priv {
	void __iomem *base;
	struct clk_bulk_data *clks;
	int num_clks;
	struct reset_control *com_reset;
	struct reset_control *phy_reset;
	struct regulator *vdd;
	struct regulator *vdda1p8;
	unsigned int vdd_levels[LEVEL_NUM];
	struct regulator *vbus;
};

static void qcom_ssphy_write(void *base, u32 offset, u32 mask, u32 val)
{
	u32 tmp;

	tmp = readl(base + offset);
	tmp &= ~mask;
	tmp |= val;
	writel(tmp, base + offset);
}

static int qcom_ssphy_config_vdd(struct ssphy_priv *priv, int high)
{
	int min, ret;

	min = high ? 1 : 0; /* low or none? */
	ret = regulator_set_voltage(priv->vdd, priv->vdd_levels[min],
				    priv->vdd_levels[LEVEL_MAX]);
	if (ret)
		return ret;

	return 0;
}

static int qcom_ssphy_ldo_enable(struct ssphy_priv *priv, int on)
{
	int ret = 0;

	if (!on)
		goto disable_regulators;

	ret = regulator_set_load(priv->vdda1p8, 23000);
	if (ret < 0)
		return ret;

	ret = regulator_set_voltage(priv->vdda1p8, 1800000, 1800000);
	if (ret)
		goto put_vdda1p8_lpm;

	ret = regulator_enable(priv->vdda1p8);
	if (ret)
		goto unset_vdda1p8;

	return 0;

disable_regulators:
	regulator_disable(priv->vdda1p8);
unset_vdda1p8:
	regulator_set_voltage(priv->vdda1p8, 0, 1800000);
put_vdda1p8_lpm:
	regulator_set_load(priv->vdda1p8, 0);
	return ret;
}

static int qcom_ssphy_power_on(struct phy *phy)
{
	struct ssphy_priv *priv = phy_get_drvdata(phy);
	int ret;

	/* Enable VBUS supply */
	if (priv->vbus) {
		ret = regulator_enable(priv->vbus);
		if (ret)
			return ret;
	}

	ret = qcom_ssphy_config_vdd(priv, 1);
	if (ret)
		return ret;

	ret = qcom_ssphy_ldo_enable(priv, 1);
	if (ret)
		return ret;

	ret = clk_bulk_prepare_enable(priv->num_clks, priv->clks);
	if (ret)
		return ret;

	/* Use clk reset, if available; otherwise use SS_PHY_RESET bit */
	if (priv->com_reset) {
		reset_control_assert(priv->com_reset);
		reset_control_assert(priv->phy_reset);
		udelay(10);
		reset_control_deassert(priv->com_reset);
		reset_control_deassert(priv->phy_reset);
	} else {
		qcom_ssphy_write(priv->base, SS_PHY_CTRL1, SS_PHY_RESET,
				 SS_PHY_RESET);
		udelay(10); /* 10us required before de-asserting */
		qcom_ssphy_write(priv->base, SS_PHY_CTRL1, SS_PHY_RESET, 0);
	}

	writeb_relaxed(SWI_PCS_CLK_SEL, priv->base + SS_PHY_CTRL0);

	qcom_ssphy_write(priv->base, SS_PHY_CTRL4, LANE0_PWR_PRESENT,
			 LANE0_PWR_PRESENT);

	writeb_relaxed(REF_SS_PHY_EN, priv->base + SS_PHY_CTRL2);

	qcom_ssphy_write(priv->base, SS_PHY_CTRL2, REF_SS_PHY_EN, REF_SS_PHY_EN);
	qcom_ssphy_write(priv->base, SS_PHY_CTRL4, TEST_POWERDOWN, 0);

	return 0;
}

static int qcom_ssphy_power_off(struct phy *phy)
{
	struct ssphy_priv *priv = phy_get_drvdata(phy);

	qcom_ssphy_write(priv->base, SS_PHY_CTRL2, REF_SS_PHY_EN, 0);
	qcom_ssphy_write(priv->base, SS_PHY_CTRL4, TEST_POWERDOWN,
			 TEST_POWERDOWN);

	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);

	qcom_ssphy_ldo_enable(priv, 0);
	qcom_ssphy_config_vdd(priv, 0);

	/* Disable VBUS supply */
	if (priv->vbus)
		regulator_disable(priv->vbus);

	return 0;
}

static const struct phy_ops qcom_ssphy_ops = {
	.power_on = qcom_ssphy_power_on,
	.power_off = qcom_ssphy_power_off,
	.owner = THIS_MODULE,
};

static const char * const qcom_ssphy_clks[] = {
	"ref",
	"phy",
	"pipe",
};

static int qcom_ssphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct ssphy_priv *priv;
	struct resource *res;
	struct phy *phy;
	int ret;
	int i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->num_clks = ARRAY_SIZE(qcom_ssphy_clks);
	priv->clks = devm_kcalloc(dev, priv->num_clks, sizeof(*priv->clks),
				  GFP_KERNEL);
	if (!priv->clks)
		return -ENOMEM;

	for (i = 0; i < priv->num_clks; i++)
		priv->clks[i].id = qcom_ssphy_clks[i];

	ret = devm_clk_bulk_get(dev, priv->num_clks, priv->clks);
	if (ret)
		return ret;

	priv->com_reset = devm_reset_control_get_optional(dev, "com");
	if (IS_ERR(priv->com_reset))
		return PTR_ERR(priv->com_reset);

	priv->phy_reset = devm_reset_control_get_optional(dev, "phy");
	if (IS_ERR(priv->phy_reset))
		return PTR_ERR(priv->phy_reset);

	priv->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(priv->vdd))
		return PTR_ERR(priv->vdd);

	priv->vdda1p8 = devm_regulator_get(dev, "vdda1p8");
	if (IS_ERR(priv->vdda1p8))
		return PTR_ERR(priv->vdda1p8);

	ret = of_property_read_u32_array(dev->of_node, "qcom,vdd-voltage-level",
					 priv->vdd_levels,
					 ARRAY_SIZE(priv->vdd_levels));
	if (ret) {
		dev_err(dev, "failed to read qcom,vdd-voltage-level\n");
		return ret;
	}

	priv->vbus = devm_regulator_get_optional(dev, "vbus");
	if (IS_ERR(priv->vbus)) {
		if (PTR_ERR(priv->vbus) == -EPROBE_DEFER)
			return PTR_ERR(priv->vbus);
		priv->vbus = NULL;
	}

	phy = devm_phy_create(dev, dev->of_node, &qcom_ssphy_ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);

	phy_set_drvdata(phy, priv);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id qcom_ssphy_match[] = {
	{ .compatible = "qcom,usb-ssphy", },
	{ },
};
MODULE_DEVICE_TABLE(of, qcom_ssphy_match);

static struct platform_driver qcom_ssphy_driver = {
	.probe		= qcom_ssphy_probe,
	.driver = {
		.name	= "qcom_usb_ssphy",
		.of_match_table = qcom_ssphy_match,
	},
};
module_platform_driver(qcom_ssphy_driver);

MODULE_DESCRIPTION("Qualcomm Super-Speed USB PHY driver");
MODULE_LICENSE("GPL v2");
