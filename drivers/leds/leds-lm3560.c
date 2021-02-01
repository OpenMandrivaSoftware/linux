// SPDX-License-Identifier: PL-2.0
// TI LM3560 LED driver
// Copyright (C) 2020 Purism SPC
// http://www.ti.com/lit/ds/symlink/lm3560.pdf

#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/led-class-flash.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define LM3560_NAME "lm3560-led"

#define LM3560_ENABLE_REG		0x10
#define LM3560_ENABLE_MASK		GENMASK(1, 0)
#define LM3560_ENABLE_LED1_FLAG		BIT(3)
#define LM3560_ENABLE_LED2_FLAG		BIT(4)

#define LM3560_PRIVACY_REG		0x11
#define LM3560_PRIVACY_BRIGHT_MASK	GENMASK(3, 0)
#define LM3560_PRIVACY_BRIGHT_SHIFT	3

#define LM3560_INDICATOR_REG		0x12
#define LM3560_INDICATOR_BLK_REG	0x13
#define LM3560_PRIVACY_PWM_REG		0x14
#define LM3560_GPIO_REG			0x20
#define LM3560_V_LED_MONITOR_REG	0x30
#define LM3560_ADC_DELAY_REG		0x31
#define LM3560_V_IN_MONITOR_REG		0x80
#define LM3560_LAST_FLASH_REG		0x81

#define LM3560_TORCH_BRIGHT_REG		0xA0
#define LM3560_TORCH_BRIGHT_LED1_MASK	GENMASK(2, 0)
#define LM3560_TORCH_BRIGHT_LED1_SHIFT	0
#define LM3560_TORCH_BRIGHT_LED2_MASK	GENMASK(5, 3)
#define LM3560_TORCH_BRIGHT_LED2_SHIFT	3
#define LM3560_TORCH_BRIGHT_MIN_uA	31250
#define LM3560_TORCH_BRIGHT_STEP_uA	31250

#define LM3560_FLASH_BRIGHT_REG		0xB0
#define LM3560_FLASH_BRIGHT_LED1_MASK	GENMASK(3, 0)
#define LM3560_FLASH_BRIGHT_LED1_SHIFT	0
#define LM3560_FLASH_BRIGHT_LED2_MASK	GENMASK(7, 4)
#define LM3560_FLASH_BRIGHT_LED2_SHIFT	4
#define LM3560_FLASH_BRIGHT_MIN_uA	62500
#define LM3560_FLASH_BRIGHT_STEP_uA	62500
#define LM3560_FLASH_BRIGHT_DEFAULT	825000

#define LM3560_FLASH_DURATION_REG	0xC0
#define LM3560_FLASH_TIMEOUT_MASK	GENMASK(4, 0)
#define LM3560_FLASH_TIMEOUT_MIN_us	32000
#define LM3560_FLASH_TIMEOUT_MAX_us	1024000
#define LM3560_FLASH_TIMEOUT_STEP_us	32000
#define LM3560_FLASH_TIMEOUT_DEFAULT_us	512000

#define LM3560_FLAGS_REG		0xD0

#define LM3560_FLAGS_TIMEOUT		BIT(0)
#define LM3560_FLAGS_HOT_FLAG		BIT(1)
#define LM3560_FLAGS_LED_FAILED_FLAG	BIT(2)

#define LM3560_CONF1_REG		0xE0
#define LM3560_CONF2_REG		0xF0

enum lm3560_enable {
	LM3560_ENABLE_SHUTDOWN = 0,
	LM3560_ENABLE_PRIV_INDICATOR = BIT(0),
	LM3560_ENABLE_TORCH = BIT(1),
	LM3560_ENABLE_FLASH = BIT(0) | BIT(1),
};

struct lm3560_data {
	struct led_classdev_flash fled_cdev;
	struct i2c_client *client;
	struct regmap *regmap;

	struct gpio_desc *enable_gpio;
	struct regulator *regulator;

	unsigned int last_flag;
	unsigned int flash_timeout;
	u32 torch_current_max;
	u32 flash_current_max;
	u32 flash_timeout_max;

	struct mutex lock;
};

static const struct reg_default lm3560_reg_defaults[] = {
	{LM3560_ENABLE_REG, 0x18},
	{LM3560_PRIVACY_REG, 0x58},
	{LM3560_INDICATOR_REG, 0x00},
	{LM3560_INDICATOR_BLK_REG, 0x00},
	{LM3560_PRIVACY_PWM_REG, 0xF8},
	{LM3560_GPIO_REG, 0x80},
	{LM3560_V_LED_MONITOR_REG, 0x80},
	{LM3560_ADC_DELAY_REG, 0x90},
	{LM3560_V_IN_MONITOR_REG, 0xC0},
	{LM3560_LAST_FLASH_REG, 0x00},
	{LM3560_TORCH_BRIGHT_REG, 0x52},
	{LM3560_FLASH_BRIGHT_REG, 0xDD},
	{LM3560_FLASH_DURATION_REG, 0xEF},
	{LM3560_FLAGS_REG, 0x00},
	{LM3560_CONF1_REG, 0x6B},
	{LM3560_CONF2_REG, 0xE0}
};

static bool lm3560_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case LM3560_FLAGS_REG:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config lm3560_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = LM3560_CONF2_REG,
	.reg_defaults = lm3560_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(lm3560_reg_defaults),
	.cache_type = REGCACHE_FLAT,
	.volatile_reg = lm3560_volatile_reg,
};

static struct lm3560_data *fled_cdev_to_led(struct led_classdev_flash *fled_cdev)
{
	return container_of(fled_cdev, struct lm3560_data, fled_cdev);
}

static int lm3560_read_faults(struct lm3560_data *priv)
{
	int flags_val;
	int ret;

	ret = regmap_read(priv->regmap, LM3560_FLAGS_REG, &flags_val);
	if (ret < 0) {
		dev_err(&priv->client->dev, "Error reading flags\n");
		return -EIO;
	}

	priv->last_flag = 0;

	if (flags_val & LM3560_FLAGS_TIMEOUT)
		priv->last_flag |= LED_FAULT_TIMEOUT;

	if (flags_val & LM3560_FLAGS_HOT_FLAG)
		priv->last_flag |= LED_FAULT_OVER_TEMPERATURE;

	if (flags_val & LM3560_FLAGS_LED_FAILED_FLAG)
		priv->last_flag |= LED_FAULT_SHORT_CIRCUIT;

	return priv->last_flag;
}

static int lm3560_chip_init(struct lm3560_data *priv)
{
	int ret;
	unsigned int flags;

	if (priv->regulator) {
		ret = regulator_enable(priv->regulator);
		if (ret) {
			dev_err(&priv->client->dev,
				"failed to enable regulator: %d\n", ret);
			return ret;
		}
	}

	if (priv->enable_gpio)
		gpiod_direction_output(priv->enable_gpio, 1);

	mutex_lock(&priv->lock);
	ret = regmap_read(priv->regmap, LM3560_FLAGS_REG, &flags);
	if (ret < 0) {
		dev_err(&priv->client->dev, "Failed to read flags register\n");
		goto out;
	}

	ret = lm3560_read_faults(priv);
	if (ret < 0) {
		dev_err(&priv->client->dev, "Fault detected: 0x%x\n", ret);
		goto out;
	}

out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int lm3560_brightness_set(struct led_classdev *cdev, enum led_brightness brightness)
{
	struct led_classdev_flash *fled_cdev = lcdev_to_flcdev(cdev);
	struct lm3560_data *priv = fled_cdev_to_led(fled_cdev);
	int ret;

	mutex_lock(&priv->lock);

	ret = lm3560_read_faults(priv);
	if (ret < 0)
		goto out;

	if (brightness == LED_OFF) {
		ret = regmap_update_bits(priv->regmap, LM3560_ENABLE_REG,
					 LM3560_ENABLE_MASK,
					 LM3560_ENABLE_SHUTDOWN);
		goto out;
	}

	ret = regmap_update_bits(priv->regmap, LM3560_TORCH_BRIGHT_REG,
				 LM3560_TORCH_BRIGHT_LED2_MASK,
				 ((brightness - 1) << LM3560_TORCH_BRIGHT_LED2_SHIFT));
	if (ret < 0)
		goto out;

	ret = regmap_update_bits(priv->regmap, LM3560_ENABLE_REG,
				 LM3560_ENABLE_MASK |
				 LM3560_ENABLE_LED1_FLAG |
				 LM3560_ENABLE_LED2_FLAG,
				 LM3560_ENABLE_LED2_FLAG |
				 LM3560_ENABLE_TORCH);
out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int lm3560_strobe_set(struct led_classdev_flash *fled_cdev, bool state)
{
	struct lm3560_data *priv = fled_cdev_to_led(fled_cdev);
	int timeout_reg_val;
	int current_timeout;
	int ret;

	mutex_lock(&priv->lock);

	ret = regmap_read(priv->regmap, LM3560_FLASH_DURATION_REG, &current_timeout);
	if (ret < 0)
		goto out;
	current_timeout &= LM3560_FLASH_TIMEOUT_MASK;

	timeout_reg_val = priv->flash_timeout / LM3560_FLASH_TIMEOUT_STEP_us - 1;

	if (priv->flash_timeout != current_timeout)
		ret = regmap_update_bits(priv->regmap, LM3560_FLASH_DURATION_REG,
					 LM3560_FLASH_TIMEOUT_MASK, timeout_reg_val);

	if (state)
		ret = regmap_update_bits(priv->regmap, LM3560_ENABLE_REG,
					 LM3560_ENABLE_MASK |
					 LM3560_ENABLE_LED1_FLAG |
					 LM3560_ENABLE_LED2_FLAG,
					 LM3560_ENABLE_LED1_FLAG | LM3560_ENABLE_LED2_FLAG |
					 LM3560_ENABLE_FLASH);
	else
		ret = regmap_update_bits(priv->regmap, LM3560_ENABLE_REG,
					 LM3560_ENABLE_MASK,
					 LM3560_ENABLE_SHUTDOWN);

	ret = lm3560_read_faults(priv);
out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int lm3560_flash_brightness_set(struct led_classdev_flash *fled_cdev, u32 brightness)
{
	struct lm3560_data *priv = fled_cdev_to_led(fled_cdev);
	u8 brightness_val;
	int ret;

	mutex_lock(&priv->lock);
	ret = lm3560_read_faults(priv);
	if (ret < 0)
		goto out;

	if (brightness == LED_OFF) {
		ret = regmap_update_bits(priv->regmap, LM3560_ENABLE_REG,
					 LM3560_ENABLE_MASK,
					 LM3560_ENABLE_SHUTDOWN);
		goto out;
	}

	brightness_val = brightness / LM3560_FLASH_BRIGHT_STEP_uA;
	ret = regmap_update_bits(priv->regmap, LM3560_FLASH_BRIGHT_REG,
				 LM3560_FLASH_BRIGHT_LED2_MASK,
				 ((brightness_val - 1) << LM3560_FLASH_BRIGHT_LED1_SHIFT) |
				 ((brightness_val - 1) << LM3560_FLASH_BRIGHT_LED2_SHIFT));

out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int lm3560_flash_timeout_set(struct led_classdev_flash *fled_cdev, u32 timeout)
{
	struct lm3560_data *priv = fled_cdev_to_led(fled_cdev);

	priv->flash_timeout = timeout;
	return 0;
}

static int lm3560_strobe_get(struct led_classdev_flash *fled_cdev, bool *state)
{
	struct lm3560_data *priv = fled_cdev_to_led(fled_cdev);
	int strobe_state;
	int ret;

	mutex_lock(&priv->lock);

	ret = regmap_read(priv->regmap, LM3560_ENABLE_REG, &strobe_state);
	if (ret < 0)
		goto out;

	*state = (strobe_state & LM3560_ENABLE_MASK) == LM3560_ENABLE_FLASH;

out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int lm3560_flash_fault_get(struct led_classdev_flash *fled_cdev, u32 *fault)
{
	struct lm3560_data *priv = fled_cdev_to_led(fled_cdev);

	mutex_lock(&priv->lock);

	lm3560_read_faults(priv);
	*fault = priv->last_flag;

	mutex_unlock(&priv->lock);
	return 0;
}

static const struct led_flash_ops flash_ops = {
	.flash_brightness_set	= lm3560_flash_brightness_set,
	.strobe_set		= lm3560_strobe_set,
	.strobe_get		= lm3560_strobe_get,
	.timeout_set		= lm3560_flash_timeout_set,
	.fault_get		= lm3560_flash_fault_get,
};

static int lm3560_register_leds(struct lm3560_data *priv, struct fwnode_handle *fwnode)
{
	struct led_classdev *led_cdev;
	struct led_flash_setting *setting;
	struct led_init_data init_data = {};

	priv->fled_cdev.ops = &flash_ops;

	setting = &priv->fled_cdev.timeout;
	setting->min = LM3560_FLASH_TIMEOUT_MIN_us;
	setting->max = priv->flash_timeout_max;
	setting->step = LM3560_FLASH_TIMEOUT_STEP_us;
	setting->val = LM3560_FLASH_TIMEOUT_DEFAULT_us;

	setting = &priv->fled_cdev.brightness;
	setting->min = LM3560_FLASH_BRIGHT_MIN_uA;
	setting->max = priv->flash_current_max;
	setting->step = LM3560_FLASH_BRIGHT_STEP_uA;
	setting->val = LM3560_FLASH_BRIGHT_DEFAULT;

	led_cdev = &priv->fled_cdev.led_cdev;
	led_cdev->brightness_set_blocking = lm3560_brightness_set;
	led_cdev->max_brightness = DIV_ROUND_UP(priv->torch_current_max,
						LM3560_TORCH_BRIGHT_STEP_uA);
	led_cdev->flags |= LED_DEV_CAP_FLASH;

	init_data.fwnode = fwnode;
	init_data.devicename = priv->client->name;
	init_data.default_label = "torch";
	return devm_led_classdev_flash_register_ext(&priv->client->dev,
						    &priv->fled_cdev, &init_data);
}

static int lm3560_parse_node(struct lm3560_data *priv,
			     struct fwnode_handle **fwnode)
{
	struct fwnode_handle *child = NULL;
	int ret = -ENODEV;

	child = device_get_next_child_node(&priv->client->dev, child);
	if (!child) {
		dev_err(&priv->client->dev, "No LED Child node\n");
		return ret;
	}

	ret = fwnode_property_read_u32(child, "led-max-microamp",
					&priv->torch_current_max);
	if (ret) {
		dev_err(&priv->client->dev,
			"led-max-microamp DT property missing\n");
		goto out_err;
	}

	ret = fwnode_property_read_u32(child, "flash-max-microamp",
				&priv->flash_current_max);
	if (ret) {
		dev_err(&priv->client->dev,
			"flash-max-microamp DT property missing\n");
		goto out_err;
	}

	ret = fwnode_property_read_u32(child, "flash-max-timeout-us",
				       &priv->flash_timeout_max);
	if (ret) {
		dev_err(&priv->client->dev,
			"flash-max-timeout-us DT property missing\n");
		goto out_err;
	}

	*fwnode = child;
	return 0;

out_err:
	fwnode_handle_put(child);
	return ret;
}

static int lm3560_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lm3560_data *priv;
	struct fwnode_handle *fwnode;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	priv->client = client;
	i2c_set_clientdata(client, priv);

	priv->enable_gpio = devm_gpiod_get_optional(&priv->client->dev,
						    "enable", GPIOD_OUT_LOW);
	if (IS_ERR(priv->enable_gpio)) {
		ret = PTR_ERR(priv->enable_gpio);
		dev_err(&priv->client->dev, "Failed to get enable gpio: %d\n",
			ret);
		return ret;
	}

	priv->regulator = devm_regulator_get_optional(&priv->client->dev, "vin");
	if (IS_ERR(priv->regulator)) {
		ret = PTR_ERR(priv->regulator);
		if (ret != -ENODEV) {
			if (ret != -EPROBE_DEFER)
				dev_err(&priv->client->dev,
					"Failed to get vin regulator: %d\n",
					ret);
			return ret;
		}
		priv->regulator = NULL;
	}

	ret = lm3560_parse_node(priv, &fwnode);
	if (ret)
		return -ENODEV;

	priv->regmap = devm_regmap_init_i2c(client, &lm3560_regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		goto err;
	}

	mutex_init(&priv->lock);

	ret = lm3560_chip_init(priv);
	if (ret < 0)
		goto err;

	ret = lm3560_register_leds(priv, fwnode);
	if (ret < 0)
		goto err;

	return 0;
err:
	fwnode_handle_put(fwnode);
	return ret;
}

static int lm3560_remove(struct i2c_client *client)
{
	struct lm3560_data *priv = i2c_get_clientdata(client);

	mutex_destroy(&priv->lock);

	regmap_update_bits(priv->regmap, LM3560_ENABLE_REG,
			   LM3560_ENABLE_MASK,
			   LM3560_ENABLE_SHUTDOWN);

	if (priv->enable_gpio)
		gpiod_direction_output(priv->enable_gpio, 0);

	if (priv->regulator)
		regulator_disable(priv->regulator);

	return 0;
}

static const struct of_device_id of_lm3560_leds_match[] = {
	{ .compatible = "ti,lm3560", },
	{},
};
MODULE_DEVICE_TABLE(of, of_lm3560_leds_match);

static const struct i2c_device_id lm3560_id[] = {
	{LM3560_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, lm3560_id);

static struct i2c_driver lm3560_i2c_driver = {
	.probe = lm3560_probe,
	.remove = lm3560_remove,
	.id_table = lm3560_id,
	.driver = {
		.name = LM3560_NAME,
		.of_match_table = of_lm3560_leds_match,
	},
};
module_i2c_driver(lm3560_i2c_driver);

MODULE_DESCRIPTION("LED driver for TI LM3560");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Guido Günther <agx@sigxcpu.org>");
