/*
 * phy_mode.c
 *
 * Copyright 2015 Fastwel Group.
 *
 * Author: Denis Grigoryev <grigoryev@spb.prosoft.ru>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This is useful for systems with mixed controllable and
 * non-controllable regulators, as well as for allowing testing on
 * systems with no controllable regulators.
 */

#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>

struct phy_mode_data {
	struct regulator_desc desc;
	struct regulator_dev *rdev;
	int microvolts;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinstate_on;
	struct pinctrl_state *pinstate_off;
	int gpio_reset;
	int gpio_mode[4];
	int is_enabled;
	u32 mode_on;
	u32 mode_off;
};

struct phy_mode_config {
	const char *supply_name;
	const char *input_supply;
	int microvolts;
	unsigned startup_delay;
	unsigned enabled_at_boot:1;
	struct regulator_init_data *init_data;
};

static struct phy_mode_config *of_get_phy_mode_config(struct device *dev)
{
	struct phy_mode_config *config;
	struct device_node *np = dev->of_node;
	const __be32 *delay;
	struct regulator_init_data *init_data;

	config = devm_kzalloc(dev, sizeof(struct phy_mode_config), GFP_KERNEL);
	if (!config)
		return ERR_PTR(-ENOMEM);

	config->init_data = of_get_regulator_init_data(dev, dev->of_node);
	if (!config->init_data)
		return ERR_PTR(-EINVAL);

	init_data = config->init_data;
	init_data->constraints.apply_uV = 0;
#if 0
	init_data->constraints.state_mem.enabled = true;
	init_data->constraints.state_standby.enabled = true;
	init_data->constraints.state_disk.enabled = true;
#endif

	config->supply_name = init_data->constraints.name;
	if (init_data->constraints.min_uV == init_data->constraints.max_uV) {
		config->microvolts = init_data->constraints.min_uV;
	} else {
		dev_err(dev,
			"Fixed regulator specified with variable voltages\n");
		return ERR_PTR(-EINVAL);
	}

	if (init_data->constraints.boot_on)
		config->enabled_at_boot = true;

	delay = of_get_property(np, "startup-delay-us", NULL);
	if (delay)
		config->startup_delay = be32_to_cpu(*delay);

	if (of_find_property(np, "vin-supply", NULL))
		config->input_supply = "vin";

	return config;
}

static int phy_mode_get_voltage(struct regulator_dev *rdev)
{
	struct phy_mode_data *data = rdev_get_drvdata(rdev);

	if (data->microvolts)
		return data->microvolts;
	else
		return -EINVAL;
}

static int phy_mode_list_voltage(struct regulator_dev *rdev, unsigned selector)
{
	struct phy_mode_data *data = rdev_get_drvdata(rdev);

	if (selector != 0)
		return -EINVAL;

	return data->microvolts;
}

static int phy_mode_is_enabled(struct regulator_dev *rdev)
{
	struct phy_mode_data *drvdata = rdev_get_drvdata(rdev);
	return drvdata->is_enabled;
}

static void phy_set_pinstate(struct regulator_dev *rdev,
			     struct pinctrl_state *pinstate)
{
	int err;
	struct phy_mode_data *drvdata = rdev_get_drvdata(rdev);

	if (!IS_ERR(pinstate)) {
		err = pinctrl_select_state(drvdata->pinctrl, pinstate);
		if (err)
			dev_warn(&rdev->dev, "unable to set pinstate\n");
	} else
		dev_warn(&rdev->dev, "no pinstate defined\n");
}

static int phy_set_power(struct regulator_dev *rdev, bool on)
{
	struct phy_mode_data *drvdata = rdev_get_drvdata(rdev);
	unsigned long mode;
	int i;

	dev_info(&rdev->dev, "phy_set_power: %i\n", on);

	gpio_direction_output(drvdata->gpio_reset, 0);
	phy_set_pinstate(rdev, drvdata->pinstate_off);

	mode = on ? drvdata->mode_on : drvdata->mode_off;
	dev_dbg(&rdev->dev, "phy_mode = %ld\n", mode);

	for (i = 0; i < ARRAY_SIZE(drvdata->gpio_mode); i++)
		gpio_direction_output(drvdata->gpio_mode[i], test_bit(i, &mode));

	msleep(1);

	gpio_set_value(drvdata->gpio_reset, 1);

	if (on) {
		msleep(1);
		phy_set_pinstate(rdev, drvdata->pinstate_on);
	}

	drvdata->is_enabled = on;

	return 0;
}

static int phy_mode_enable(struct regulator_dev *rdev)
{
	return phy_set_power(rdev, true);
}

static int phy_mode_disable(struct regulator_dev *rdev)
{
	return phy_set_power(rdev, false);
}

static struct regulator_ops phy_mode_ops = {
	.get_voltage = phy_mode_get_voltage,
	.list_voltage = phy_mode_list_voltage,
	.is_enabled = phy_mode_is_enabled,
	.enable = phy_mode_enable,
	.disable = phy_mode_disable,
};

static int phy_mode_request_gpio(struct device *dev,
				 const char *name, int index)
{
	struct device_node *node = dev->of_node;
	int gpio;
	int ret;

	gpio = of_get_named_gpio(node, name, index);
	if (gpio_is_valid(gpio))
		ret = devm_gpio_request_one(dev,
				gpio, GPIOF_IN, "phy-regulator");
	else
		ret = -ENODEV;

	return ret == 0 ? gpio : ret;
}

static int phy_mode_voltage_probe(struct platform_device *pdev)
{
	struct phy_mode_config *config;
	struct phy_mode_data *drvdata;
	struct regulator_config cfg = { };
	int i;
	int ret = 0;

	if (pdev->dev.of_node) {
		config = of_get_phy_mode_config(&pdev->dev);
		if (IS_ERR(config)) {
			dev_err(&pdev->dev, "Probe error\n");
			return PTR_ERR(config);
		}
	} else {
		config = pdev->dev.platform_data;
	}

	if (!config) {
		dev_err(&pdev->dev, "cannot read config\n");
		return -ENOMEM;
	}

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct phy_mode_data),
			       GFP_KERNEL);
	if (drvdata == NULL) {
		dev_err(&pdev->dev, "Failed to allocate device data\n");
		ret = -ENOMEM;
		goto err;
	}

	drvdata->desc.name = kstrdup(config->supply_name, GFP_KERNEL);
	if (drvdata->desc.name == NULL) {
		dev_err(&pdev->dev, "Failed to allocate supply name\n");
		ret = -ENOMEM;
		goto err;
	}
	drvdata->desc.type = REGULATOR_VOLTAGE;
	drvdata->desc.owner = THIS_MODULE;
	drvdata->desc.ops = &phy_mode_ops;
	drvdata->desc.enable_time = config->startup_delay;

	if (config->input_supply) {
		drvdata->desc.supply_name = kstrdup(config->input_supply,
						    GFP_KERNEL);
		if (!drvdata->desc.supply_name) {
			dev_err(&pdev->dev,
				"Failed to allocate input supply\n");
			ret = -ENOMEM;
			goto err_name;
		}
	}

	drvdata->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(drvdata->pinctrl)) {
		dev_err(&pdev->dev, "cannot get pinctrl interface\n");
		ret = PTR_ERR(drvdata->pinctrl);
		goto err_input;
	}

	drvdata->pinstate_on =
	    pinctrl_lookup_state(drvdata->pinctrl, PINCTRL_STATE_DEFAULT);
	drvdata->pinstate_off =
	    pinctrl_lookup_state(drvdata->pinctrl, "state-powerdown");

	if (config->microvolts)
		drvdata->desc.n_voltages = 1;

	drvdata->microvolts = config->microvolts;

	cfg.dev = &pdev->dev;
	cfg.init_data = config->init_data;
	cfg.driver_data = drvdata;
	cfg.of_node = pdev->dev.of_node;

	ret = of_property_read_u32(pdev->dev.of_node, "phy-mode-on",
				   &drvdata->mode_on);
	if (ret)
		goto err_input;

	ret = of_property_read_u32(pdev->dev.of_node, "phy-mode-off",
				   &drvdata->mode_off);
	if (ret)
		goto err_input;

	drvdata->gpio_reset =
		phy_mode_request_gpio(&pdev->dev, "phy-reset-gpio", 0);

	if (!gpio_is_valid(drvdata->gpio_reset)) {
		dev_err(&pdev->dev, "unable to request phy-reset-gpio\n");
		ret = drvdata->gpio_reset;
		goto err_input;
	}

	for (i = 0; i < ARRAY_SIZE(drvdata->gpio_mode); i++) {
		drvdata->gpio_mode[i] =
			phy_mode_request_gpio(&pdev->dev, "phy-mode-gpios", i);
		if (!gpio_is_valid(drvdata->gpio_mode[i])) {
			dev_err(&pdev->dev, "unable to request phy-mode-gpios\n");
			ret = drvdata->gpio_mode[i];
			goto err_input;
		}
	}

	drvdata->rdev = regulator_register(&drvdata->desc, &cfg);
	if (IS_ERR(drvdata->rdev)) {
		ret = PTR_ERR(drvdata->rdev);
		dev_err(&pdev->dev, "Failed to register regulator: %d\n", ret);
		goto err_input;
	}

	platform_set_drvdata(pdev, drvdata);

	dev_dbg(&pdev->dev, "%s supplying %duV\n", drvdata->desc.name,
		drvdata->microvolts);

	return 0;

err_input:
	kfree(drvdata->desc.supply_name);
err_name:
	kfree(drvdata->desc.name);
err:
	dev_err(&pdev->dev, "probe error %i\n", ret);

	return ret;
}

static int phy_mode_voltage_remove(struct platform_device *pdev)
{
	struct phy_mode_data *drvdata = platform_get_drvdata(pdev);

	regulator_unregister(drvdata->rdev);
	kfree(drvdata->desc.supply_name);
	kfree(drvdata->desc.name);

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id phy_mode_of_match[] = {
	{.compatible = "regulator-micrel-phy",},
	{},
};

MODULE_DEVICE_TABLE(of, phy_mode_of_match);
#endif

static struct platform_driver regulator_phy_mode_voltage_driver = {
	.probe = phy_mode_voltage_probe,
	.remove = phy_mode_voltage_remove,
	.driver = {
		   .name = "reg-micrel-phy",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(phy_mode_of_match),
		   },
};

static int __init regulator_phy_mode_voltage_init(void)
{
	return platform_driver_register(&regulator_phy_mode_voltage_driver);
}

subsys_initcall(regulator_phy_mode_voltage_init);

static void __exit regulator_phy_mode_voltage_exit(void)
{
	platform_driver_unregister(&regulator_phy_mode_voltage_driver);
}

module_exit(regulator_phy_mode_voltage_exit);

MODULE_AUTHOR("Denis Grigoryev <grigoryev@spb.prosoft.ru>");
MODULE_DESCRIPTION("Micrel PHY power state regulator");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:reg-micrel-phy");
