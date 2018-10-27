/*
 *
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/input-polldev.h>
#include <linux/hwmon.h>
#include <linux/input.h>

#define MPL3115_DRV_NAME       "mpl3115"

#define MPL3115_STATUS 0x00
#define MPL3115_OUT_PRESS 0x01	/* MSB first, 20 bit */
#define MPL3115_OUT_TEMP 0x04	/* MSB first, 12 bit */
#define MPL3115_WHO_AM_I 0x0c
#define MPL3115_CTRL_REG1 0x26

#define MPL3115_DEVICE_ID 0xc4

#define MPL3115_STATUS_PRESS_RDY BIT(2)
#define MPL3115_STATUS_TEMP_RDY BIT(1)

#define MPL3115_CTRL_RESET BIT(2)	/* software reset */
#define MPL3115_CTRL_OST BIT(1)	/* initiate measurement */
#define MPL3115_CTRL_ACTIVE BIT(0)	/* continuous measurement */
#define MPL3115_CTRL_OS_258MS (BIT(5) | BIT(4))	/* 64x oversampling */

#define POLL_INTERVAL_MAX       10000
#define POLL_INTERVAL_MIN       0
#define POLL_INTERVAL           1000
/* if sensor is standby ,set POLL_STOP_TIME to slow down the poll */
#define POLL_STOP_TIME          5001

struct mpl3115_data {
	struct i2c_client *client;
	struct mutex lock;
	u8 ctrl_reg1;
	bool data_ready;
	wait_queue_head_t waitq;
	struct input_polled_dev *poll_dev;
	unsigned long saved_interval;
};

/*
 * Initialization function
 */
static int mpl3115_init_device(struct mpl3115_data *data)
{
	int ret;
	/* software reset, I2C transfer is aborted (fails) */
	ret = i2c_smbus_write_byte_data(data->client, MPL3115_CTRL_REG1,
					MPL3115_CTRL_RESET);
	if (ret)
		return ret;

	msleep(50);
	data->ctrl_reg1 = MPL3115_CTRL_OS_258MS;

	return i2c_smbus_write_byte_data(data->client, MPL3115_CTRL_REG1,
					 data->ctrl_reg1);
}

static int mpl3115_request(struct mpl3115_data *data)
{
	int ret, tries = 15;

	dev_dbg(&data->client->dev, "mpl3115_request\n");

	/* trigger measurement */
	ret = i2c_smbus_write_byte_data(data->client, MPL3115_CTRL_REG1,
					data->ctrl_reg1 | MPL3115_CTRL_OST);
	if (ret < 0) {
		dev_err(&data->client->dev, "write error %i\n", ret);
		return ret;
	}

	while (tries-- > 0) {
		ret = i2c_smbus_read_byte_data(data->client, MPL3115_CTRL_REG1);
		if (ret < 0) {
			dev_err(&data->client->dev, "read error %i\n", ret);
			return ret;
		}

		dev_dbg(&data->client->dev, "MPL3115_CTRL_REG1: 0x%x\n", ret);

		msleep(20);

		/* wait for data ready, i.e. OST cleared */
		if (!(ret & MPL3115_CTRL_OST))
			break;
	}

	if (tries < 0) {
		dev_err(&data->client->dev, "data not ready\n");
		return -ETIME;
	}

	return 0;
}

static int mpl3115_read_raw(struct mpl3115_data *data, int *press, int *temp)
{
	s32 tmp = 0;
	int ret;

	ret = mpl3115_request(data);
	if (ret < 0)
		goto out_err;

	ret = i2c_smbus_read_i2c_block_data(data->client,
					    MPL3115_OUT_PRESS, 3, (u8 *) & tmp);
	if (ret < 0) {
		dev_err(&data->client->dev, "read block status %i\n", ret);
		goto out_err;
	}

	*press = sign_extend32(be32_to_cpu(tmp) >> 12, 23);

	ret = i2c_smbus_read_i2c_block_data(data->client,
					    MPL3115_OUT_TEMP, 2, (u8 *) & tmp);
	if (ret < 0) {
		dev_err(&data->client->dev, "read block status %i\n", ret);
		goto out_err;
	}

	*temp = sign_extend32(be32_to_cpu(tmp) >> 20, 15);

	return 0;
out_err:
	return ret;
}

static void mpl3115_report_data(struct mpl3115_data *data)
{

	struct input_polled_dev *poll_dev = data->poll_dev;
	struct input_dev *idev = poll_dev->input;
	int press, temp;

	mutex_lock(&data->lock);
	if (mpl3115_read_raw(data, &press, &temp) != 0) {
		data->saved_interval = poll_dev->poll_interval;
		poll_dev->poll_interval = POLL_STOP_TIME;
		goto out;
	} else if (poll_dev->poll_interval == POLL_STOP_TIME)
		poll_dev->poll_interval = data->saved_interval;

	idev = data->poll_dev->input;
	input_report_abs(idev, ABS_PRESSURE, press);
	input_report_abs(idev, ABS_MISC, temp);
	input_sync(idev);
out:
	mutex_unlock(&data->lock);
}

static void mpl3115_dev_poll(struct input_polled_dev *dev)
{
	struct mpl3115_data *data = (struct mpl3115_data *)dev->private;

	dev_dbg(&dev->input->dev, "poll\n");
	mpl3115_report_data(data);
}

static int mpl3115_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct input_dev *idev;
	struct mpl3115_data *data;
	int ret = 0;

	ret = i2c_smbus_read_byte_data(client, MPL3115_WHO_AM_I);
	if (ret < 0)
		return ret;
	if (ret != MPL3115_DEVICE_ID) {
		dev_err(&client->dev,
			"read chip ID 0x%x is not equal to 0x%x!\n", ret,
			MPL3115_DEVICE_ID);
		return -ENODEV;
	}

	data = kzalloc(sizeof(struct mpl3115_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	mutex_init(&data->lock);
	i2c_set_clientdata(client, data);
	/* Init queue */
	init_waitqueue_head(&data->waitq);

	/*input poll device register */
	data->poll_dev = input_allocate_polled_device();
	if (!data->poll_dev) {
		dev_err(&client->dev, "alloc poll device failed!\n");
		ret = -ENOMEM;
		goto error_alloc_poll_dev;
	}
	data->saved_interval = POLL_INTERVAL;
	data->poll_dev->poll = mpl3115_dev_poll;
	data->poll_dev->poll_interval = POLL_INTERVAL;
	data->poll_dev->poll_interval_max = POLL_INTERVAL_MAX;
	data->poll_dev->poll_interval_min = POLL_INTERVAL_MIN;
	data->poll_dev->private = data;
	idev = data->poll_dev->input;
	idev->name = MPL3115_DRV_NAME;
	idev->id.bustype = BUS_I2C;
	idev->evbit[0] = BIT_MASK(EV_ABS);
	input_set_abs_params(idev, ABS_PRESSURE, 0, 0xfffff, 0, 0);
	input_set_abs_params(idev, ABS_MISC, -0x7fff, 0x7fff, 0, 0);
	ret = input_register_polled_device(data->poll_dev);
	if (ret) {
		dev_err(&client->dev, "register poll device failed!\n");
		goto error_reg_poll_dev;
	}

	/* Initialize mpl3115 chip */
	mpl3115_init_device(data);

	dev_info(&client->dev, "mpl3115 is probed\n");
	return 0;

	input_unregister_polled_device(data->poll_dev);
error_reg_poll_dev:
	input_free_polled_device(data->poll_dev);
error_alloc_poll_dev:
	kfree(data);
	return ret;
}

static int mpl3115_standby(struct mpl3115_data *data)
{
	return i2c_smbus_write_byte_data(data->client, MPL3115_CTRL_REG1,
					 data->ctrl_reg1 &
					 ~MPL3115_CTRL_ACTIVE);
}

static int mpl3115_remove(struct i2c_client *client)
{
	struct mpl3115_data *data = i2c_get_clientdata(client);

	mpl3115_standby(data);
	input_unregister_polled_device(data->poll_dev);
	input_free_polled_device(data->poll_dev);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int mpl3115_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return mpl3115_standby(i2c_get_clientdata(client));

}

static int mpl3115_resume(struct i2c_client *client)
{
	struct mpl3115_data *data = i2c_get_clientdata(client);

	return i2c_smbus_write_byte_data(client, MPL3115_CTRL_REG1,
					 data->ctrl_reg1);
}

#else
#define mpl3115_suspend        NULL
#define mpl3115_resume         NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id mpl3115_id[] = {
	{MPL3115_DRV_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, mpl3115_id);
static struct i2c_driver mpl3115_driver = {
	.driver = {.name = MPL3115_DRV_NAME,
		   .owner = THIS_MODULE,},
	.suspend = mpl3115_suspend,
	.resume = mpl3115_resume,
	.probe = mpl3115_probe,
	.remove = mpl3115_remove,
	.id_table = mpl3115_id,
};

static int __init mpl3115_init(void)
{
	return i2c_add_driver(&mpl3115_driver);
}

static void __exit mpl3115_exit(void)
{
	i2c_del_driver(&mpl3115_driver);
}

module_init(mpl3115_init);
module_exit(mpl3115_exit);
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale mpl3115 pressure sensor driver");
MODULE_LICENSE("GPL");
