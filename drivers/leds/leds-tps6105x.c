/*
 * Driver for TPS61050/61052 led torch/flash driver.
 *
 * Copyright (C) 2015 Fastwel Group
 *
 * Author: Denis Grigoryev <linus.walleij@linaro.org>
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/regmap.h>
#include <linux/mfd/core.h>
#include <linux/mfd/tps6105x.h>

struct tps6105x_leds {
	struct platform_device *pdev;
	struct led_classdev torch_cdev;
	struct work_struct work_brightness_set;
	enum led_brightness brightness_value;
};

static int tps6105x_brightness_set(struct tps6105x_leds *leds,
				   enum led_brightness value)
{
	struct tps6105x *tps6105x = dev_get_platdata(&leds->pdev->dev);
	int val = 0;

	if (value != LED_OFF)
		val = TPS6105X_REG0_TORCHC_250_400;

	return regmap_update_bits(tps6105x->regmap, TPS6105X_REG_0,
			          val, TPS6105X_REG0_TORCHC_MASK);
}

static void tps6105x_brightness_set_work(struct work_struct *work)
{
	struct tps6105x_leds *leds =
		container_of(work, struct tps6105x_leds, work_brightness_set);

	tps6105x_brightness_set(leds, leds->brightness_value);
}

static int tps6105x_led_brightness_set_sync(struct led_classdev *led_cdev,
					    enum led_brightness value)
{
	struct tps6105x_leds *leds =
		container_of(led_cdev, struct tps6105x_leds, torch_cdev);

	return tps6105x_brightness_set(leds, value);
}

static void tps6105x_led_brightness_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	struct tps6105x_leds *leds =
		container_of(led_cdev, struct tps6105x_leds, torch_cdev);

	leds->brightness_value = value;
	schedule_work(&leds->work_brightness_set);
}

static int tps6105x_leds_probe(struct platform_device *pdev)
{
	struct tps6105x *tps6105x = dev_get_platdata(&pdev->dev);
	struct tps6105x_leds *leds;
	struct led_classdev *torch_cdev;
	//struct tps6105x_leds_config_data led_cfg;
	int ret;

	if (tps6105x->pdata->mode != TPS6105X_MODE_TORCH) {
		dev_info(&pdev->dev, "chip not in torch mode, exit probe \n");
		return -ENODEV;
	}

	leds = devm_kzalloc(&pdev->dev, sizeof(*leds), GFP_KERNEL);
	if (leds == NULL)
		return -ENOMEM;

	leds->pdev = pdev;
	torch_cdev = &leds->torch_cdev;

	torch_cdev->name = "torch";
	torch_cdev->max_brightness = 1;//led_cfg.torch_max_brightness;
	torch_cdev->brightness_set = tps6105x_led_brightness_set;
	torch_cdev->brightness_set_sync = tps6105x_led_brightness_set_sync;
	torch_cdev->default_trigger = "torch";
	torch_cdev->flags = LED_CORE_SUSPENDRESUME;

	ret = led_classdev_register(&pdev->dev, torch_cdev);
	if (ret < 0)
		goto err_register_torch;

	ret = regmap_write(tps6105x->regmap, TPS6105X_REG_0,
			TPS6105X_REG0_MODE_TORCH << TPS6105X_REG0_MODE_SHIFT);
	if (ret != 0)
		goto err_set_mode;

	INIT_WORK(&leds->work_brightness_set, tps6105x_brightness_set_work);
	platform_set_drvdata(pdev, leds);
	return 0;

err_set_mode:
	led_classdev_unregister(&leds->torch_cdev);
err_register_torch:
	dev_err(&pdev->dev, "probe error %d\n", ret);
	return ret;
}

static int tps6105x_leds_remove(struct platform_device *pdev)
{
	struct tps6105x_leds *leds = platform_get_drvdata(pdev);

	cancel_work_sync(&leds->work_brightness_set);
	led_classdev_unregister(&leds->torch_cdev);

	return 0;
}

static struct platform_driver tps6105x_torch_driver = {
	.driver = {
		.name  = "tps6105x-torch",
		.owner = THIS_MODULE,
	},
	.probe = tps6105x_leds_probe,
	.remove = tps6105x_leds_remove,
};

module_platform_driver(tps6105x_torch_driver);

MODULE_AUTHOR("Denis Grigoryev <grigoryev@fastwel.ru>");
MODULE_DESCRIPTION("TPS6105x torch driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:tps6105x-torch");
