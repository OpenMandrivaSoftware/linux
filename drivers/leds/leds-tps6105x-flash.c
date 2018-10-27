/*
 * Driver for TPS61050/61052 led torch/flash driver.
 *
 * Copyright (C) 2015 Fastwel Group
 *
 * Author: Denis Grigoryev <linus.walleij@linaro.org>
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#define DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/led-class-flash.h>
#include <linux/mfd/core.h>
#include <linux/mfd/tps6105x.h>

#define TPS6105X_REG1_FLASHC_150	0
#define TPS6105X_REG1_FLASHC_200	1
#define TPS6105X_REG1_FLASHC_300	2
#define TPS6105X_REG1_FLASHC_400	3
#define TPS6105X_REG1_FLASHC_500	4
#define TPS6105X_REG1_FLASHC_700	5
#define TPS6105X_REG1_FLASHC_900	6
#define TPS6105X_REG1_FLASHC_1200	7
#define TPS6105X_REG1_FLASHC_MASK	0x7

#define TPS6105X_REG1_FLASH_START	(1 << 3)

#define TPS6105X_REG3_STT_DEFAULT	4

struct tps6105x_leds_config_data {
	u32 flash_brightness_max;
	u32 torch_brightness_max;
};

struct tps6105x_leds {
	struct platform_device *pdev;
	struct led_classdev_flash fled_cdev;
	struct work_struct work_brightness_set;
	struct tps6105x_leds_config_data cfg;
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
				  TPS6105X_REG0_TORCHC_MASK, val);
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
	struct led_classdev_flash *fled_cdev = lcdev_to_flcdev(led_cdev);
	struct tps6105x_leds *leds =
		container_of(fled_cdev, struct tps6105x_leds, fled_cdev);

	return tps6105x_brightness_set(leds, value);
}

static void tps6105x_led_brightness_set(struct led_classdev *led_cdev,
					enum led_brightness value)
{
	struct led_classdev_flash *fled_cdev = lcdev_to_flcdev(led_cdev);
	struct tps6105x_leds *leds =
		container_of(fled_cdev, struct tps6105x_leds, fled_cdev);

	leds->brightness_value = value;
	schedule_work(&leds->work_brightness_set);
}

static int tps6105x_flash_strobe_set(struct led_classdev_flash *fled_cdev,
				     bool state)
{
	struct tps6105x_leds *leds =
		container_of(fled_cdev, struct tps6105x_leds, fled_cdev);
	struct tps6105x *tps6105x = dev_get_platdata(&leds->pdev->dev);

	dev_dbg(&leds->pdev->dev, "tps6105x_led_flash_strobe_set %d\n", state);
	if (state) {
		return regmap_update_bits(tps6105x->regmap, TPS6105X_REG_1,
				          TPS6105X_REG1_FLASH_START,
					  TPS6105X_REG1_FLASH_START);
	}

	return 0;
}

static int tps6105x_flash_timeout_set(struct led_classdev_flash *fled_cdev,
				      u32 timeout)
{
	struct tps6105x_leds *leds =
		container_of(fled_cdev, struct tps6105x_leds, fled_cdev);

	dev_dbg(&leds->pdev->dev, "tps6105x_led_flash_timeout_set %d\n", timeout);
	return 0;
}

#define in_range(_value, _min, _max) (_value >= _min && _value < _max)

static int tps6105x_flash_brightness_to_setting(u32 brightness)
{
	if (brightness < 175000)
		return TPS6105X_REG1_FLASHC_150;
	else if (in_range(brightness, 175000, 250000))
		return TPS6105X_REG1_FLASHC_200;
	else if (in_range(brightness, 250000, 350000))
		return TPS6105X_REG1_FLASHC_300;
	else if (in_range(brightness, 350000, 450000))
		return TPS6105X_REG1_FLASHC_400;
	else if (in_range(brightness, 450000, 600000))
		return TPS6105X_REG1_FLASHC_500;
	else if (in_range(brightness, 600000, 800000))
		return TPS6105X_REG1_FLASHC_700;
	else if (in_range(brightness, 800000, 1050000))
		return TPS6105X_REG1_FLASHC_900;
	else
		return TPS6105X_REG1_FLASHC_1200;
}

static int tps6105x_flash_brightness_set(struct led_classdev_flash *fled_cdev,
					 u32 brightness)
{
	struct tps6105x_leds *leds =
		container_of(fled_cdev, struct tps6105x_leds, fled_cdev);
	struct tps6105x *tps6105x = dev_get_platdata(&leds->pdev->dev);
	int setting;

	brightness = min(brightness, leds->cfg.flash_brightness_max);
	setting = tps6105x_flash_brightness_to_setting(brightness);

	dev_dbg(&leds->pdev->dev, "tps6105x_flash_brightness_set %d\n", setting);

	return regmap_update_bits(tps6105x->regmap, TPS6105X_REG_1,
				 TPS6105X_REG1_FLASHC_MASK, setting);
}

static const struct led_flash_ops flash_ops = {
	.strobe_set = tps6105x_flash_strobe_set,
	.timeout_set = tps6105x_flash_timeout_set,
	.flash_brightness_set = tps6105x_flash_brightness_set,
};

static int tps6105x_led_parse_dt(struct platform_device *pdev,
				 struct tps6105x_leds_config_data *cfg)
{
	cfg->flash_brightness_max = 700000;
	cfg->torch_brightness_max = 250000;

	return 0;
}

static int tps6105x_flash_probe(struct platform_device *pdev)
{
	struct tps6105x *tps6105x = dev_get_platdata(&pdev->dev);
	struct tps6105x_leds *leds;
	struct led_classdev *led_cdev;
	struct led_classdev_flash *fled_cdev;
	struct tps6105x_leds_config_data led_cfg;
	int ret;

	if (tps6105x->pdata->mode != TPS6105X_MODE_TORCH_FLASH) {
		dev_warn(&pdev->dev, "chip not in flash mode, exit probe \n");
		return -ENODEV;
	}

	ret = tps6105x_led_parse_dt(pdev, &led_cfg);
	if (ret < 0)
		return ret;

	leds = devm_kzalloc(&pdev->dev, sizeof(*leds), GFP_KERNEL);
	if (leds == NULL)
		return -ENOMEM;

	leds->pdev = pdev;
	memcpy(&leds->cfg, &led_cfg, sizeof(led_cfg));
	fled_cdev = &leds->fled_cdev;
	led_cdev = &fled_cdev->led_cdev;

	fled_cdev->ops = &flash_ops;
	fled_cdev->brightness.min = 150000;
	fled_cdev->brightness.max = 700000; //led_cfg.flash_brightness_max;
	fled_cdev->brightness.step = 100000;

	fled_cdev->timeout.min = 150000;
	fled_cdev->timeout.max = 700000; //led_cfg.flash_timeout_max;
	fled_cdev->timeout.step = 100000;


	led_cdev->name = "flash";
	led_cdev->max_brightness = 1;//led_cfg.torch_brightness_max;
	led_cdev->brightness_set = tps6105x_led_brightness_set;
	led_cdev->brightness_set_sync = tps6105x_led_brightness_set_sync;
	led_cdev->flags = LED_CORE_SUSPENDRESUME | LED_DEV_CAP_FLASH;

	ret = led_classdev_flash_register(&pdev->dev, fled_cdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "led_classdev_flash_register failed\n");
		goto err_register_flash;
	}

	ret = regmap_write(tps6105x->regmap, TPS6105X_REG_3,
			   TPS6105X_REG3_STT_DEFAULT);
	if (ret != 0) {
		dev_err(&pdev->dev, "regmap_write failed\n");
		goto err_set_mode;
	}

	ret = regmap_write(tps6105x->regmap, TPS6105X_REG_0,
			TPS6105X_REG0_MODE_TORCH_FLASH << TPS6105X_REG0_MODE_SHIFT);
	if (ret != 0) {
		dev_err(&pdev->dev, "regmap_write failed\n");
		goto err_set_mode;
	}

	ret = regmap_update_bits(tps6105x->regmap, TPS6105X_REG_1,
				 TPS6105X_REG1_FLASHC_MASK,
				 TPS6105X_REG1_FLASHC_700);
	if (ret != 0) {
		dev_err(&pdev->dev, "regmap_update_bits failed\n");
		goto err_config;
	}

	INIT_WORK(&leds->work_brightness_set, tps6105x_brightness_set_work);
	platform_set_drvdata(pdev, leds);
	dev_dbg(&pdev->dev, "tps6105x flash LED registered\n");

	return 0;

err_config:
	regmap_write(tps6105x->regmap, TPS6105X_REG_0, 0);
err_set_mode:
	led_classdev_flash_unregister(&leds->fled_cdev);
err_register_flash:
	return ret;
}

static int tps6105x_flash_remove(struct platform_device *pdev)
{
	struct tps6105x_leds *leds = platform_get_drvdata(pdev);

	cancel_work_sync(&leds->work_brightness_set);
	led_classdev_flash_unregister(&leds->fled_cdev);

	return 0;
}

static struct platform_driver tps6105x_flash_driver = {
	.driver = {
		.name  = "tps6105x-flash",
		.owner = THIS_MODULE,
	},
	.probe = tps6105x_flash_probe,
	.remove = tps6105x_flash_remove,
};

module_platform_driver(tps6105x_flash_driver);

MODULE_AUTHOR("Denis Grigoryev <grigoryev@fastwel.ru>");
MODULE_DESCRIPTION("TPS6105x flash LED driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:tps6105x-flash");
