/*
 * board-pkm01-wl1271-platdata.c
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/wl12xx.h>
#include <linux/ti_wilink_st.h>

static struct regulator *wl12xx_wifi_regulator = NULL;
static struct regulator *wl12xx_bt_regulator = NULL;

static void wl12xx_set_power(bool enable)
{
	pr_info("wl12xx_set_power %i\n", enable);

	if (IS_ERR(wl12xx_wifi_regulator))
		return;

	if (enable && !regulator_is_enabled(wl12xx_wifi_regulator)) {
		if (regulator_enable(wl12xx_wifi_regulator))
			printk(KERN_ERR "error enabling wifi\n");
		mdelay(10);
	} else if (!enable && regulator_is_enabled(wl12xx_wifi_regulator)) {
		regulator_disable(wl12xx_wifi_regulator);
	}
}

static int tiwi_bt_power_change(int enable)
{
	pr_info("tiwi_bt_power_change %i\n", enable);

	if (IS_ERR(wl12xx_bt_regulator))
		return 0;

	if (enable && !regulator_is_enabled(wl12xx_bt_regulator)) {
		if (regulator_enable(wl12xx_bt_regulator))
			printk(KERN_ERR "error enabling bluetooth\n");
		mdelay(10);
	} else if (!enable && regulator_is_enabled(wl12xx_bt_regulator)) {
		regulator_disable(wl12xx_bt_regulator);
	}

	return 0;
}


static int tiwi_bt_chip_enable(struct kim_data_s *unused)
{
	return tiwi_bt_power_change(1);
}

static int tiwi_bt_chip_disable(struct kim_data_s *unused)
{
	return tiwi_bt_power_change(0);
}

static int pkm01_bt_suspend(struct platform_device *pdev, pm_message_t pm)
{
	return 0;
}

static int pkm01_bt_resume(struct platform_device *pdev)
{
	return 0;
}

static struct ti_st_plat_data wilink_pdata = {
	.suspend = pkm01_bt_suspend,
	.resume = pkm01_bt_resume,
	.chip_enable = tiwi_bt_chip_enable,
	.chip_disable = tiwi_bt_chip_disable,
};

static struct platform_device wl12xx_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static void wl1271_add_bluetooth(struct platform_device *pdev)
{
	platform_device_register(&wl12xx_device);
	platform_device_register(&btwilink_device);
}

static int wl1271_plat_probe(struct platform_device *pdev)
{
	int ret;
	struct wl12xx_platform_data pdata;
	const void *prop;
	char const *uart_dev_name;

	dev_info(&pdev->dev, "wl1271 platform code loaded\n");

	if (!IS_ERR(wl12xx_get_platform_data()))
		return -EBUSY;

	wilink_pdata.nshutdown_gpio =
		of_get_named_gpio(pdev->dev.of_node, "bt-shutdown-gpio", 0);

	prop = of_get_property(pdev->dev.of_node, "bt-flow_cntrl", NULL);
	wilink_pdata.flow_cntrl = prop ? of_read_ulong(prop, 1) : 1;

	prop = of_get_property(pdev->dev.of_node, "bt-flow_cntrl", NULL);
	wilink_pdata.baud_rate = prop ? of_read_ulong(prop, 1) : 3000000;

	if (of_property_read_string(pdev->dev.of_node,
				"bt-dev-name", &uart_dev_name) < 0)
		uart_dev_name = "/dev/ttymxc1";
	strncpy(wilink_pdata.dev_name, uart_dev_name, UART_DEV_NAME_LEN);

	wl12xx_wifi_regulator = devm_regulator_get(&pdev->dev, "wifi");
	if (IS_ERR(wl12xx_wifi_regulator))
		dev_warn(&pdev->dev, "unable to get wifi regulator\n");

	wl12xx_bt_regulator = devm_regulator_get(&pdev->dev, "bt");
	if (IS_ERR(wl12xx_bt_regulator))
		dev_warn(&pdev->dev, "unable to get bt regulator\n");

	pdata.set_power = wl12xx_set_power;
	pdata.board_ref_clock = WL12XX_REFCLOCK_38;
	pdata.irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (pdata.irq < 0) {
		dev_warn(&pdev->dev, "unable to request wl12xx irq\n");
		pdata.irq = 0;
	}

	ret = wl12xx_set_platform_data(&pdata);
	if (ret) {
		dev_warn(&pdev->dev, "unable to set platform data\n");
		return ret;
	}

	wl1271_add_bluetooth(pdev);

	//wl12xx_set_power(1);
	//tiwi_bt_power_change(1);

	return 0;
}

static int wl1271_plat_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "wl1271 platform code removed\n");
	return 0;
}

static const struct of_device_id wl1271_plat_of_match[] = {
	{.compatible = "fastwel,wl1271-pkm01",},
	{},
};

MODULE_DEVICE_TABLE(of, wl1271_plat_of_match);

static struct platform_driver wl1271_platform_driver = {
	.probe = wl1271_plat_probe,
	.remove = wl1271_plat_remove,
	.driver = {
		   .name = "wl1271-pkm01",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(wl1271_plat_of_match),
		   },
};

static int __init wl1271_platform_init(void)
{
	return platform_driver_register(&wl1271_platform_driver);
}

module_init(wl1271_platform_init);

static void __exit wl1271_platform_exit(void)
{
	platform_driver_unregister(&wl1271_platform_driver);
}

module_exit(wl1271_platform_exit);

MODULE_AUTHOR("Denis Grigoryev <grigoryev@spb.prosoft.ru>");
MODULE_DESCRIPTION("WL1271 platform code");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:wl1271-pkm01");
