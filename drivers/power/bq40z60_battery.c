/*
 * drivers/power/bq40z60_battery.c
 *
 * Gas Gauge driver for TI's BQ40Z60
 *
 * Copyright (c) 2015, FASTWEL Group.
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/pm_runtime.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#ifdef CONFIG_WAKELOCK
#include <linux/wakelock.h>
#endif
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/busfreq-imx.h>

#include <asm/unaligned.h>

#define BATLOW_IRQ_INDEX	1
#define BATTERY_POLLING_RATE	5
#define KELVIN_TO_CELCIUS(x)	((x) - 2731)

/* Commands */
#define BQ40Z60_MAC		0x0
#define BQ40Z60_MAC_READ	0x23
#define BQ40Z60_ALT_MAC		0x44

#define BQ40Z60_MAC_SHUTDOWN	0x10

/* Registers */
#define BQ40Z60_MAC_ID		0x1
#define BQ40Z60_MAC_ID_VALUE	0x4600

#define BQ40Z60_TEMPERATURE	0x8
#define BQ40Z60_RSOC		0xd
#define BQ40Z60_CURRENT_NOW	0xa
#define BQ40Z60_CURRENT_AVG	0xb
#define BQ40Z60_REMAIN_CAP	0xf
#define BQ40Z60_FULL_CAP	0x10
#define BQ40Z60_AVG_TTE		0x12
#define BQ40Z60_AVG_TTF		0x13
#define BQ40Z60_STATUS		0x16
#define BQ40Z60_STATUS_FC	(1 << 5) /* Fully Charged */
#define BQ40Z60_STATUS_DSG	(1 << 6) /* Discharge or Relaxing */
#define BQ40Z60_BATT_STATUS	0x16
#define BQ40Z60_CYCLE_COUNT	0x17
#define BQ40Z60_DESIGN_CAP	0x18
#define BQ40Z60_CELL_VOLTAGE_4	0x3c
#define BQ40Z60_CELL_VOLTAGE_3	0x3d
#define BQ40Z60_CELL_VOLTAGE_2	0x3e
#define BQ40Z60_CELL_VOLTAGE_1	0x3f
#define BQ40Z60_INIT_DISCHARGE_SET 0x4a
#define BQ40Z60_STATE_OF_HEALTH	0x4f
#define BQ40Z60_CHARGING_STATUS	0x55

#define LOW_CHARGE_mAh		150

#define BQ40Z60_DATAFLASH_START	0x4000
#define BQ40Z60_DATAFLASH_END	0x6000
#define BQ40Z60_DATAFLASH_SIZE	(BQ40Z60_DATAFLASH_END - \
				 BQ40Z60_DATAFLASH_START)
#define BQ40Z60_MFDATA_START	0x4040
#define BQ40Z60_MFDATA_END	0x4061
#define BQ40Z60_MFDATA_SIZE	(BQ40Z60_MFDATA_END - BQ40Z60_MFDATA_START)

#define BQ40Z60_OP_STATUS		0x54
#define BQ40Z60_OP_STATUS_FULL_ACCESS	(0x1 << 8)
#define BQ40Z60_OP_STATUS_UNSEALED	(0x2 << 8)
#define BQ40Z60_OP_STATUS_SEALED	(0x3 << 8)
#define BQ40Z60_OP_STATUS_SEALED_MASK	(0x3 << 8)

#define BQ40Z60_FULL_ACCESS	0
#define BQ40Z60_UNSEALED	1
#define BQ40Z60_SEALED		2

#ifdef DEBUG
#define dbg_dump(_buf, _size) \
	print_hex_dump(KERN_DEBUG, \
		       "data: ", DUMP_PREFIX_NONE, 16, 1, \
		       _buf, _size, 1);
#else
#define dbg_dump(_buf, _size) while (0) {}
#endif

struct prop_cache_struct {
	unsigned retry_count;
	int intval;
};

enum prop_cache_enum {
	PROP_CACHE_VOLT_NOW = 0,
	PROP_CACHE_CAPACITY,
	PROP_CACHE_TTE,
	PROP_CACHE_TTF,
	PROP_CACHE_CHARGE_NOW,
	PROP_CACHE_CHARGE_FULL,
	PROP_CACHE_CURRENT_NOW,
	PROP_CACHE_CURRENT_AVG,
	PROP_CACHE_TEMP,
	PROP_CACHE_STATUS,
	PROP_CACHE_HEALTH,
	PROP_CACHE_DESIGN_CAP,
	PROP_CACHE_MAX,
};

struct bq40z60_device {
	struct i2c_client *client;
	struct delayed_work poll_work;

	struct power_supply battery_supply;
	struct power_supply ac_supply;
#ifdef CONFIG_WAKELOCK
	struct wake_lock batlow_wakelock;
#endif
	int n_charger_prsnt_gpio;
	int n_charging_gpio;
	int batlow_gpio;
	int batlow_irq;
	unsigned int old_capacity;
	unsigned int cap_err;
	unsigned int check_interval;
	bool is_batlow;
	struct mutex lock;
	int dc_ok;
	struct workqueue_struct *battery_wq;

	struct prop_cache_struct prop_cache[PROP_CACHE_MAX];
};

static enum power_supply_property bq40z60_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};

static enum power_supply_property bq40z60_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int bq40z60_ac_get_property(struct power_supply *psy,
			      enum power_supply_property psp,
			      union power_supply_propval *val)
{
	struct bq40z60_device *bqdev =
		container_of(psy, struct bq40z60_device, ac_supply);
	int err = 0;

	dev_dbg(&bqdev->client->dev, "bq40z60_ac_get_property: %d\n", psp);

	pm_runtime_get_sync(&bqdev->client->dev);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bqdev->dc_ok;
		break;
	default:
		err = -EINVAL;
		break;
	}
	pm_runtime_put_sync(&bqdev->client->dev);

	return err;
}

static char *bq40z60_supply_list[] = {
	"battery",
	"ac",
};

static int bq40z60_read_unlocked(struct bq40z60_device *bqdev,
				 u8 reg, int *value)
{
	struct i2c_client *client = bqdev->client;
	s32 rc;

	rc = i2c_smbus_read_word_data(client, reg);
	if (rc < 0)
		return rc;

	*value = rc;
	return 0;
}

static int bq40z60_write_unlocked(struct bq40z60_device *bqdev,
				  u8 reg, int value)
{
	struct i2c_client *client = bqdev->client;
	return i2c_smbus_write_word_data(client, reg, value);
}

static int bq40z60_read(struct bq40z60_device *bqdev, u8 reg, int *value)
{
	int rc;

	mutex_lock(&bqdev->lock);
	rc = bq40z60_read_unlocked(bqdev, reg, value);
	mutex_unlock(&bqdev->lock);

	return rc;
}

static int bq40z60_write(struct bq40z60_device *bqdev, u8 reg, int value)
{
	int rc;

	mutex_lock(&bqdev->lock);
	rc = bq40z60_write_unlocked(bqdev, reg, value);
	mutex_unlock(&bqdev->lock);

	return rc;
}

static int bq40z60_read_block(struct bq40z60_device *bqdev,
		u16 addr, void *buf, ssize_t size)
{
	struct i2c_client *client = bqdev->client;
	int err;

	mutex_lock(&bqdev->lock);

	dev_dbg(&bqdev->client->dev,
			"bq40z60_read_block addr=0x%x size=%d\n", addr, size);

	if (size > I2C_SMBUS_BLOCK_MAX)
		size = I2C_SMBUS_BLOCK_MAX;

	err = bq40z60_write_unlocked(bqdev, BQ40Z60_MAC, addr);
	if (err != 0)
		goto error;

	err = i2c_smbus_read_block_data(client, BQ40Z60_MAC_READ, buf);
	if (err < 0)
		goto error;

	size = min(size, err);
	err = size;

	dbg_dump(buf, size);
error:
	mutex_unlock(&bqdev->lock);
	return err;
}

static int bq40z60_write_block(struct bq40z60_device *bqdev,
		u16 addr, const void *buf, ssize_t size)
{
	struct i2c_client *client = bqdev->client;
	union i2c_smbus_data data;
	int err;

	mutex_lock(&bqdev->lock);

	if (size > I2C_SMBUS_BLOCK_MAX - 2)
		size = I2C_SMBUS_BLOCK_MAX - 2;

	dev_dbg(&bqdev->client->dev,
			"bq40z60_write_block addr=0x%x size=%d\n", addr, size);
	data.block[0] = size + sizeof(addr);
	data.block[1] = addr;
	data.block[2] = addr >> 8;
	memcpy(data.block + 3, buf, size);

	dbg_dump(data.block, size + 2);

	err = i2c_smbus_xfer(client->adapter,
			client->addr, client->flags | I2C_CLIENT_PEC, I2C_SMBUS_WRITE,
			BQ40Z60_ALT_MAC, I2C_SMBUS_BLOCK_DATA, &data);
	if (err == 0)
		err = size;
	else
		dev_err(&client->dev,
				"i2c_smbus_xfer MAC failed with status %d\n", err);

	mutex_unlock(&bqdev->lock);

	/*
	 * Sometimes BQ40Z60 answers NAK to next write command, so wait
	 * some time to complete current write operation.
	 */
	msleep(10);

	return err;
}

static int bq40z60_write_mfdata(struct bq40z60_device *bqdev,
		off_t addr, const void *data, size_t size)
{
	int rc;
	int written = 0;
	int remain = size;

	while (remain) {
		rc = bq40z60_write_block(bqdev,
				addr + written, data + written, remain);
		if (rc < 0) {
			dev_err(&bqdev->client->dev,
					"unable to write mfdata: %d\n", rc);
			return rc;
		}

		remain -= rc;
		written += rc;
	}

	return written;
}

static int bq40z60_read_mfdata(struct bq40z60_device *bqdev,
		off_t addr, void *data, size_t size)
{
	int rc;
	int bytes_read = 0;
	int remain = size;

	while (remain) {
		rc = bq40z60_read_block(bqdev, addr + bytes_read,
				data + bytes_read, remain);
		if (rc < 0) {
			dev_err(&bqdev->client->dev,
					"unable to read mfdata: %d\n", rc);
			return rc;
		}

		remain -= rc;
		bytes_read += rc;
	}

	return bytes_read;
}

#ifdef CONFIG_BATTERY_BQ40Z60_FIRMWARE_ACCESS

static ssize_t bq40z60_firmware_read(struct file *filp, struct kobject *kobj,
			   struct bin_attribute *bin_attr,
			   char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = to_i2c_client(container_of(kobj, struct device, kobj));
        struct bq40z60_device *bqdev = i2c_get_clientdata(client);
	u16 addr = BQ40Z60_DATAFLASH_START + off;

	if (addr < BQ40Z60_DATAFLASH_START || addr >= BQ40Z60_DATAFLASH_END)
		return -EINVAL;

	if (addr + count > BQ40Z60_DATAFLASH_END)
		count = BQ40Z60_DATAFLASH_END - addr;

	return bq40z60_read_mfdata(bqdev, addr, buf, count);
}

static ssize_t bq40z60_firmware_write(struct file *filp, struct kobject *kobj,
			   struct bin_attribute *bin_attr,
			   char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = to_i2c_client(container_of(kobj, struct device, kobj));
        struct bq40z60_device *bqdev = i2c_get_clientdata(client);
	u16 addr = BQ40Z60_DATAFLASH_START + off;

	if (addr < BQ40Z60_DATAFLASH_START || addr >= BQ40Z60_DATAFLASH_END)
		return -EINVAL;

	if (addr + count > BQ40Z60_DATAFLASH_END)
		count = BQ40Z60_DATAFLASH_END - addr;

	return bq40z60_write_mfdata(bqdev, addr, buf, count);
}

static struct bin_attribute firmware_attr = {
	.attr = {
		.name = "firmware",
		.mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
	},
	.size = BQ40Z60_DATAFLASH_SIZE,
	.read = bq40z60_firmware_read,
	.write = bq40z60_firmware_write,
};

#endif /* CONFIG_BATTERY_BQ40Z60_FIRMWARE_ACCESS */

static ssize_t bq40z60_eeprom_read(struct file *filp, struct kobject *kobj,
			   struct bin_attribute *bin_attr,
			   char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = to_i2c_client(container_of(kobj, struct device, kobj));
        struct bq40z60_device *bqdev = i2c_get_clientdata(client);
	u16 addr = BQ40Z60_MFDATA_START + off;

	if (addr < BQ40Z60_MFDATA_START || addr >= BQ40Z60_MFDATA_END)
		return -EINVAL;

	if (addr + count > BQ40Z60_MFDATA_END)
		count = BQ40Z60_MFDATA_END - addr;

	return bq40z60_read_mfdata(bqdev, addr, buf, count);
}


static ssize_t bq40z60_eeprom_write(struct file *filp, struct kobject *kobj,
			   struct bin_attribute *bin_attr,
			   char *buf, loff_t off, size_t count)
{
	struct i2c_client *client = to_i2c_client(container_of(kobj, struct device, kobj));
        struct bq40z60_device *bqdev = i2c_get_clientdata(client);
	u16 addr = BQ40Z60_MFDATA_START + off;

	if (addr < BQ40Z60_MFDATA_START || addr >= BQ40Z60_MFDATA_END)
		return -EINVAL;

	if (addr + count > BQ40Z60_MFDATA_END)
		count = BQ40Z60_MFDATA_END - addr;

	return bq40z60_write_mfdata(bqdev, addr, buf, count);
}

static struct bin_attribute eeprom_attr = {
	.attr = {
		.name = "eeprom",
		.mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP,
	},
	.size = BQ40Z60_MFDATA_SIZE,
	.read = bq40z60_eeprom_read,
	.write = bq40z60_eeprom_write,
};

static int bq40z60_get_sealed(struct bq40z60_device *bqdev)
{
	int rc;
	u32 op_status;

	rc = bq40z60_read_block(bqdev,
			BQ40Z60_OP_STATUS, &op_status, sizeof(op_status));
	if (rc < 0) {
		dev_err(&bqdev->client->dev,
				"bq40z60_read_block failed (%d)\n", rc);
		return rc;
	}
	if (rc < sizeof(op_status))
		return -EFAULT;

	switch (op_status & BQ40Z60_OP_STATUS_SEALED_MASK) {
	case BQ40Z60_OP_STATUS_SEALED:
		return BQ40Z60_SEALED;
	case BQ40Z60_OP_STATUS_UNSEALED:
		return BQ40Z60_UNSEALED;
	case BQ40Z60_OP_STATUS_FULL_ACCESS:
		return BQ40Z60_FULL_ACCESS;
	default:
		break;
	}

	return -EFAULT;
}

static int bq40z60_set_sealed(struct bq40z60_device *bqdev, int status)
{
	switch (status) {
	case BQ40Z60_SEALED:
		bq40z60_write(bqdev, 0, 0x30);
		return 0;
	case BQ40Z60_UNSEALED:
		mutex_lock(&bqdev->lock);
		bq40z60_write_unlocked(bqdev, 0, 0x414);
		bq40z60_write_unlocked(bqdev, 0, 0x3672);
		mutex_unlock(&bqdev->lock);
		return 0;
	case BQ40Z60_FULL_ACCESS:
		mutex_lock(&bqdev->lock);
		bq40z60_write_unlocked(bqdev, 0, 0xffff);
		bq40z60_write_unlocked(bqdev, 0, 0xffff);
		mutex_unlock(&bqdev->lock);
		return 0;
	}
	return 0;
}

static const char *bq40z60_get_sealed_string(int code)
{
	switch (code) {
	case BQ40Z60_SEALED:
		return "sealed";
	case BQ40Z60_UNSEALED:
		return "unsealed";
	case BQ40Z60_FULL_ACCESS:
		return "full";
	default:
		return "unknown";
	}
}

static ssize_t bq40z60_show_sealed(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
        struct bq40z60_device *bqdev = i2c_get_clientdata(client);
	int rc;

	rc = bq40z60_get_sealed(bqdev);
	if (rc < 0)
		return rc;

	return sprintf(buf, "%s\n", bq40z60_get_sealed_string(rc));
}

static ssize_t bq40z60_store_sealed(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
        struct bq40z60_device *bqdev = i2c_get_clientdata(client);
	int status;
	int rc;

	status = bq40z60_get_sealed(bqdev);
	if (status < 0)
		return status;

	if (strncmp(buf, "sealed", 6) == 0 && status != BQ40Z60_SEALED) {
		rc = bq40z60_set_sealed(bqdev, BQ40Z60_SEALED);
	} else if (strncmp(buf, "unsealed", 8) == 0 && status != BQ40Z60_UNSEALED) {
		rc = bq40z60_set_sealed(bqdev, BQ40Z60_UNSEALED);
	} else if (strncmp(buf, "full", 4) == 0 && status == BQ40Z60_UNSEALED) {
		rc = bq40z60_set_sealed(bqdev, BQ40Z60_FULL_ACCESS);
	} else
		return -EINVAL;

	if (rc < 0)
		return rc;

	return count;
}

static int bq40z60_get_cell_voltage(struct bq40z60_device *bqdev,
				    int *cell_volt);

static ssize_t bq40z60_show_cell_voltage(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
        struct bq40z60_device *bqdev = i2c_get_clientdata(client);
	int cell_volt[4];
	int ret;

	memset(cell_volt, 0, sizeof(cell_volt));
	ret = bq40z60_get_cell_voltage(bqdev, cell_volt);

	return ret < 0 ? ret : sprintf(buf, "%d\n%d\n%d\n%d\n",
		       cell_volt[0], cell_volt[1], cell_volt[2], cell_volt[3]);
}

static ssize_t shutdown_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct bq40z60_device *bqdev = i2c_get_clientdata(client);
	u16 data = BQ40Z60_MAC_SHUTDOWN;

	dev_info(&client->dev, "shutdown_store: %s\n", buf);

	if (strncmp(buf, "shutdown", 8) == 0) {
		dev_warn(&client->dev, "Shutting down gas gauge!\n");
		ret = bq40z60_write_block(bqdev, 0x10, &data, sizeof(data));
		if (ret != sizeof(data)) {
			dev_err(&client->dev, "Shutdown failed with error %d\n", ret);
			return -EIO;
		}
		return count;
	}

	return -EINVAL;
}

static ssize_t soh_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
        struct bq40z60_device *bqdev = i2c_get_clientdata(client);
	int soh;
	int ret;

	ret = bq40z60_read(bqdev, BQ40Z60_STATE_OF_HEALTH, &soh);

	return ret < 0 ? ret : sprintf(buf, "%d\n", soh);
}

static DEVICE_ATTR(sealed, 0644, bq40z60_show_sealed, bq40z60_store_sealed);
static DEVICE_ATTR(cell_voltage, 0444, bq40z60_show_cell_voltage, NULL);
static DEVICE_ATTR_WO(shutdown);
static DEVICE_ATTR_RO(soh);

static void bq40z60_set_dc_ok(struct bq40z60_device *bqdev, unsigned cable_state)
{
	dev_info(&bqdev->client->dev, "DC %sconnected\n", cable_state ? "" : "dis");

	bqdev->dc_ok = cable_state;
	power_supply_changed(&bqdev->ac_supply);
	mod_delayed_work(bqdev->battery_wq, &bqdev->poll_work, 2 * HZ);
}

static int bq40z60_check_status(struct bq40z60_device *bqdev)
{
	int ret;
	int status;
	int charging;

	ret = bq40z60_read(bqdev, BQ40Z60_STATUS, &status);
	if (ret != 0)
		return ret;

	charging = !(status & BQ40Z60_STATUS_DSG);
	if (charging != bqdev->dc_ok)
		bq40z60_set_dc_ok(bqdev, charging);

	return ret;
}

static void battery_status_poll(struct work_struct *work)
{
	struct bq40z60_device *bqdev =
	    container_of(work, struct bq40z60_device, poll_work.work);

	pm_runtime_get_sync(&bqdev->client->dev);
	queue_delayed_work(bqdev->battery_wq,
			&bqdev->poll_work, bqdev->check_interval * HZ);
	power_supply_changed(&bqdev->battery_supply);
	bq40z60_check_status(bqdev);
	pm_runtime_put_sync(&bqdev->client->dev);
}

static irqreturn_t bq40z60_low_battery_isr(int irq, void *dev_id)
{
	struct bq40z60_device *bqdev = dev_id;

	dev_warn(&bqdev->client->dev, "low battery interrupt\n");

	disable_irq_nosync(bqdev->batlow_irq);
#ifdef CONFIG_WAKELOCK
	wake_lock_timeout(&bqdev->batlow_wakelock, 10 * HZ);
#endif
	mod_delayed_work(bqdev->battery_wq, &bqdev->poll_work, 0);

	return IRQ_HANDLED;
}

static irqreturn_t bq40z60_charger_changed_irq(int irq, void *data)
{
	struct bq40z60_device *bqdev = data;

	dev_info(&bqdev->client->dev, "Charger IRQ\n");
#ifdef CONFIG_WAKELOCK
	wake_lock_timeout(&bqdev->batlow_wakelock, 4 * HZ);
#endif
	mod_delayed_work(bqdev->battery_wq, &bqdev->poll_work, 2 * HZ);

	return IRQ_HANDLED;
}

static inline int trim_capacity(int x)
{
	if (x < 0)
		return 0;
	else if (x >= 100)
		return 100;
	else if (x < 84)
		return (84*x - 840)/74;
	else
		return x;
}

static void bq40z60_ack_batlow_irq(struct bq40z60_device *bqdev)
{
	dev_info(&bqdev->client->dev, "batlow acknowlege\n");
	bq40z60_write(bqdev, BQ40Z60_INIT_DISCHARGE_SET, LOW_CHARGE_mAh);
}

static int bq40z60_get_remain_cap(struct bq40z60_device *bqdev)
{
	int ret, val;

	ret = bq40z60_read(bqdev, BQ40Z60_REMAIN_CAP, &val);
	return ret == 0 ? val : ret;
}

static int bq40z60_is_batlow(struct bq40z60_device *bqdev)
{
	int is_batlow = 0;
	int capacity;

	if (!gpio_is_valid(bqdev->batlow_gpio))
		return 0;

	if (gpio_get_value(bqdev->batlow_gpio) == 0) {
		capacity = bq40z60_get_remain_cap(bqdev);
		if (capacity <= LOW_CHARGE_mAh)
			is_batlow = 1;
		else
			bq40z60_ack_batlow_irq(bqdev);

	}

	if (!is_batlow && bqdev->is_batlow)
		enable_irq(bqdev->batlow_irq);

	bqdev->is_batlow = is_batlow;

	return is_batlow;
}

static int bq40z60_health_by_soh(struct bq40z60_device *bqdev, int soh)
{
	if (soh <= 10)
		return POWER_SUPPLY_HEALTH_DEAD;
	else
		return POWER_SUPPLY_HEALTH_GOOD;
}

static int bq40z60_get_health(struct bq40z60_device *bqdev, u8 arg,
			      union power_supply_propval *val)
{
	int err;
	int soh;

	err = bq40z60_read(bqdev, BQ40Z60_STATE_OF_HEALTH, &soh);
	if (err >= 0) {
		dev_dbg(&bqdev->client->dev, "Battery SoH: %d\n", soh);
		val->intval = bq40z60_health_by_soh(bqdev, soh);
	}
	return err;
}

static int bq40z60_get_capacity(struct bq40z60_device *bqdev, u8 arg,
				union power_supply_propval *val)
{
	int err;
	int trim_cap;
	int cap = 0;

	err = bq40z60_read(bqdev, BQ40Z60_RSOC, &cap);
	if (err < 0) {
		dev_err(&bqdev->client->dev, "Capacity read failed\n");

		if (bqdev->cap_err > 5 || bqdev->old_capacity == 0xff)
			return -EINVAL;
		else {
			val->intval = bqdev->old_capacity;
			bqdev->cap_err++;
			dev_warn(&bqdev->client->dev,
					"cap_err=%u use old capacity=%u\n",
					bqdev->cap_err, val->intval);
			return 0;
		}
	}

	trim_cap = trim_capacity(cap);

	if (bq40z60_is_batlow(bqdev))
		trim_cap = 0;
	else if (trim_cap <= 0)
		trim_cap = 1;

#if 0
	if (trim_cap <= 3 && bqdev->old_capacity > 3) {
		bqdev->check_interval = 10;
	} else if (trim_cap <= 9 && bqdev->old_capacity > 9) {
		bqdev->check_interval = 60;
	} else {
		if ((trim_cap > 4) && (bqdev->check_interval == 1)) {
			bqdev->check_interval = 60;
		} else if (trim_cap > 10 && bqdev->check_interval != 60) {
			bqdev->check_interval = 120;
		}
	}
#endif

	val->intval = bqdev->old_capacity = trim_cap;
	bqdev->cap_err = 0;

	dev_dbg(&bqdev->client->dev, "= %u%% ret= %u\n", val->intval, cap);

	return 0;
}

static int bq40z60_read_u16(struct bq40z60_device *bqdev,
		u8 reg, union power_supply_propval *val)
{
	return bq40z60_read(bqdev, reg, &val->intval);
}

static int bq40z60_read_s16(struct bq40z60_device *bqdev,
		u8 reg, union power_supply_propval *val)
{
	int ret;
	int value;

	ret = bq40z60_read(bqdev, reg, &value);
	val->intval = (s16)value;

	return ret;
}

static int bq40z60_read_time_sec(struct bq40z60_device *bqdev, u8 reg,
				 union power_supply_propval *val)
{
	int err;
	int minutes;

	err = bq40z60_read(bqdev, reg, &minutes);
	if (err == 0)
		val->intval = minutes * 60;
	else
		dev_err(&bqdev->client->dev, "time read failed\n");

	return err;
}

static int bq40z60_get_status(struct bq40z60_device *bqdev,
			      u8 arg, union power_supply_propval *val)
{
	int ret;
	int status;

#if 0
	if (!gpio_get_value(bqdev->n_charging_gpio)) {
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	}

	if (gpio_get_value(bqdev->n_charging_gpio) && bqdev->dc_ok) {
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}
#endif

	ret = bq40z60_read(bqdev, BQ40Z60_STATUS, &status);
	if (ret != 0)
		return ret;

	if (!(status & BQ40Z60_STATUS_DSG)) {
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	}

	if (status & BQ40Z60_STATUS_FC) {
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	val->intval = POWER_SUPPLY_STATUS_DISCHARGING;

	return 0;
}

static int bq40z60_get_temp(struct bq40z60_device *bqdev, u8 reg,
			    union power_supply_propval *val)
{
	int ret;
	int value;

	ret = bq40z60_read(bqdev, BQ40Z60_TEMPERATURE, &value);
	if (ret == 0)
		val->intval = KELVIN_TO_CELCIUS(value);
	else
		dev_err(&bqdev->client->dev, "temperature read failed\n");

	return ret;
}

static int bq40z60_get_cell_voltage(struct bq40z60_device *bqdev,
				    int *cell_volt)
{
	int ret;

	ret = bq40z60_read(bqdev, BQ40Z60_CELL_VOLTAGE_1, &cell_volt[0]);
	if (ret)
		return ret;
	ret = bq40z60_read(bqdev, BQ40Z60_CELL_VOLTAGE_2, &cell_volt[1]);
	if (ret)
		return ret;
	ret = bq40z60_read(bqdev, BQ40Z60_CELL_VOLTAGE_3, &cell_volt[2]);
	if (ret)
		return ret;
	return bq40z60_read(bqdev, BQ40Z60_CELL_VOLTAGE_4, &cell_volt[3]);
}

static int bq40z60_get_voltage_now(struct bq40z60_device *bqdev, u8 reg,
				   union power_supply_propval *val)
{
	int ret;
	int cell_volt[4];

	memset(cell_volt, 0, sizeof(cell_volt));

	ret = bq40z60_get_cell_voltage(bqdev, cell_volt);
	if (ret == 0)
		val->intval = cell_volt[0] + cell_volt[1] + \
			      cell_volt[2] + cell_volt[3];

	return ret;
}

#define MAX_READ_RETRY 3

typedef int (*prop_read_fn_t)(struct bq40z60_device *bqdev, u8 reg,
			      union power_supply_propval *val);

static int bq40z60_read_retry(struct bq40z60_device *bqdev,
			      prop_read_fn_t read_fn,
			      u8 arg,
			      enum prop_cache_enum prop_id,
			      union power_supply_propval *val)
{
	int err;
	struct prop_cache_struct *cache = &bqdev->prop_cache[prop_id];

	err = read_fn(bqdev, arg, val);
	if (err == 0) {
		cache->intval = val->intval;
		cache->retry_count = 0;
		return 0;
	}

	if (++cache->retry_count > MAX_READ_RETRY) {
		dev_err(&bqdev->client->dev,
			"Property %d max read attempts reached, giving up\n",
			prop_id);
		return err;
	}

	dev_warn(&bqdev->client->dev,
		 "Property %d read failed\n", prop_id);

	val->intval = cache->intval;
	return 0;
}

static int bq40z60_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	int err = 0;
	struct bq40z60_device *bqdev =
		container_of(psy, struct bq40z60_device, battery_supply);

	dev_dbg(&bqdev->client->dev, "bq40z60_get_property: %d\n", psp);

	pm_runtime_get_sync(&bqdev->client->dev);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		err = bq40z60_read_retry(bqdev, bq40z60_get_health,
					 0, PROP_CACHE_HEALTH, val);
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		err = bq40z60_read_retry(bqdev, bq40z60_get_capacity,
					 0, PROP_CACHE_CAPACITY, val);
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		err = bq40z60_read_retry(bqdev, bq40z60_read_s16,
					 BQ40Z60_CURRENT_NOW,
					 PROP_CACHE_CURRENT_NOW, val);
		break;

	case POWER_SUPPLY_PROP_CURRENT_AVG:
		err = bq40z60_read_retry(bqdev, bq40z60_read_s16,
				         BQ40Z60_CURRENT_AVG,
					 PROP_CACHE_CURRENT_AVG, val);
		break;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		err = bq40z60_read_retry(bqdev, bq40z60_read_u16,
					 BQ40Z60_REMAIN_CAP,
					 PROP_CACHE_CHARGE_NOW, val);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		err = bq40z60_read_retry(bqdev, bq40z60_read_u16,
					 BQ40Z60_FULL_CAP,
					 PROP_CACHE_CHARGE_FULL, val);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		err = bq40z60_read_retry(bqdev, bq40z60_read_u16,
					 BQ40Z60_DESIGN_CAP,
					 PROP_CACHE_DESIGN_CAP, val);
		break;

	case POWER_SUPPLY_PROP_CHARGE_EMPTY_DESIGN:
		val->intval = 150;
		err = 0;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		err = bq40z60_read_retry(bqdev, bq40z60_get_voltage_now,
					 0, PROP_CACHE_VOLT_NOW, val);
		break;

	case POWER_SUPPLY_PROP_STATUS:
		err = bq40z60_read_retry(bqdev, bq40z60_get_status,
					 0, PROP_CACHE_STATUS, val);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		err = bq40z60_read_retry(bqdev, bq40z60_get_temp,
					 0, PROP_CACHE_TEMP, val);
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		err = bq40z60_read_retry(bqdev, bq40z60_read_time_sec,
					 BQ40Z60_AVG_TTE, PROP_CACHE_TTE, val);
		break;

	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		err = bq40z60_read_retry(bqdev, bq40z60_read_time_sec,
					 BQ40Z60_AVG_TTF, PROP_CACHE_TTF, val);
		break;

	default:
		dev_err(&bqdev->client->dev, "Invalid property psp=%u\n", psp);
		err = -EINVAL;
		break;
	}

	pm_runtime_put_sync(&bqdev->client->dev);

	return err;
}

static int bq40z60_request_irq_wake(struct bq40z60_device *bqdev,
		int gpio, int irq_index, irq_handler_t thread_fn, const char *function)
{

	int irq, err;

	err = gpio_request(gpio, function);
	if (err) {
		dev_err(&bqdev->client->dev, "gpio_request failed\n");
		return err;
	}

	gpio_direction_input(gpio);

	irq = irq_of_parse_and_map(bqdev->client->dev.of_node, irq_index);
	if (irq < 0) {
		dev_err(&bqdev->client->dev, "No irq found!\n");
		return irq;
	}

	dev_info(&bqdev->client->dev, "gpio %i irq %i\n", gpio, irq);

	err = request_threaded_irq(irq,
			NULL, thread_fn,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			function, bqdev);
	if (err) {
		dev_err(&bqdev->client->dev, "failed request %s irq\n", function);
		gpio_free(gpio);
		return err;
	}

	enable_irq_wake(irq);

	return irq;
}

static int bq40z60_request_batlow_irq(struct bq40z60_device *bqdev)
{

	int irq, err;
	int gpio;

	gpio = bqdev->batlow_gpio;
	if (!gpio_is_valid(gpio))
		return 0;

	err = gpio_request(gpio, "battery-low");
	if (err) {
		dev_err(&bqdev->client->dev, "gpio_request failed\n");
		return err;
	}

	gpio_direction_input(gpio);

	irq = irq_of_parse_and_map(bqdev->client->dev.of_node, BATLOW_IRQ_INDEX);
	if (irq < 0) {
		dev_err(&bqdev->client->dev, "No irq found!\n");
		return irq;
	}

	bqdev->batlow_irq = irq;

	err = request_irq(irq, bq40z60_low_battery_isr,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, "battery-low", bqdev);
	if (err) {
		dev_err(&bqdev->client->dev, "failed request batlow irq\n");
		gpio_free(gpio);
		return err;
	}

	enable_irq_wake(irq);

	return 0;
}


static void bq40z60_free_irq_wake(struct bq40z60_device *bqdev,
		int gpio)
{
	int irq;

	if (gpio_is_valid(gpio)) {
		irq = gpio_to_irq(gpio);
		disable_irq_wake(irq);
		free_irq(irq, bqdev);
		gpio_free(gpio);
	}
}

static int bq40z60_detect(struct bq40z60_device *bqdev)
{
	struct i2c_client *client = bqdev->client;
	u16 id = 0;
	int ret;

	ret = bq40z60_read_block(bqdev, BQ40Z60_MAC_ID, &id, sizeof(id));
	if (ret < 0) {
		dev_err(&client->dev, "read ID failed with status %d\n", ret);
		return ret;
	}

	if (id != BQ40Z60_MAC_ID_VALUE) {
		dev_err(&client->dev, "read ID 0x%x, should be 0x%x\n",
				id, BQ40Z60_MAC_ID_VALUE);
		return -ENODEV;
	}

	return 0;
}

static int bq40z60_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret, irq;
	struct device_node *of_node = client->dev.of_node;
	struct bq40z60_device *bqdev;

	dev_dbg(&client->dev, "client addr %02x\n", client->addr);

	pinctrl_pm_select_default_state(&client->dev);

	bqdev = kzalloc(sizeof(*bqdev), GFP_KERNEL);
	if (!bqdev)
		return -ENOMEM;

	client->flags |= I2C_CLIENT_PEC;
	bqdev->client = client;

	mutex_init(&bqdev->lock);

	request_bus_freq(BUS_FREQ_HIGH);
	pm_runtime_enable(&client->dev);
	pm_runtime_resume(&client->dev);

	ret = bq40z60_detect(bqdev);
	if (ret != 0) {
		dev_err(&client->dev, "No BQ40Z60 device found\n");
		goto out_err_no_dev;
	}

	bqdev->old_capacity = 100;
	bqdev->check_interval = BATTERY_POLLING_RATE;

	bqdev->batlow_gpio = of_get_named_gpio(of_node, "gpio-batlow", 0);
	bqdev->n_charger_prsnt_gpio = of_get_named_gpio(of_node, "gpio-ncharger-prsnt", 0);
	bqdev->n_charging_gpio = of_get_named_gpio(of_node, "gpio-ncharging", 0);

	if (!gpio_is_valid(bqdev->batlow_gpio) ||
			!gpio_is_valid(bqdev->n_charger_prsnt_gpio) ||
			!gpio_is_valid(bqdev->n_charging_gpio)) {
		ret = -ENODEV;
		goto out_err_charging_gpio;
	}

	gpio_request(bqdev->n_charging_gpio, "ncharging");
	gpio_direction_input(bqdev->n_charging_gpio);

	bqdev->battery_supply.name = "battery";
	bqdev->battery_supply.type = POWER_SUPPLY_TYPE_BATTERY;
	bqdev->battery_supply.properties = bq40z60_properties;
	bqdev->battery_supply.num_properties = ARRAY_SIZE(bq40z60_properties);
	bqdev->battery_supply.get_property = bq40z60_get_property;

	ret = power_supply_register(&client->dev, &bqdev->battery_supply);
	if (ret) {
		dev_err(&client->dev, "Failed to register batterry supply\n");
		goto out_err_battery_supply;
	}

	bqdev->ac_supply.name = "ac";
	bqdev->ac_supply.type = POWER_SUPPLY_TYPE_MAINS;
	bqdev->ac_supply.supplied_to = bq40z60_supply_list;
	bqdev->ac_supply.num_supplicants = ARRAY_SIZE(bq40z60_supply_list);
	bqdev->ac_supply.properties = bq40z60_ac_properties;
	bqdev->ac_supply.num_properties = ARRAY_SIZE(bq40z60_ac_properties);
	bqdev->ac_supply.get_property = bq40z60_ac_get_property;

	ret = power_supply_register(&client->dev, &bqdev->ac_supply);
	if (ret) {
		dev_err(&client->dev, "Failed to register power supply\n");
		goto out_err_ac_supply;
	}

	bqdev->battery_wq = create_singlethread_workqueue("battery_wq");

	INIT_DELAYED_WORK(&bqdev->poll_work,
			  battery_status_poll);
	cancel_delayed_work(&bqdev->poll_work);

#ifdef CONFIG_WAKELOCK
	wake_lock_init(&bqdev->batlow_wakelock,
		       WAKE_LOCK_SUSPEND, "batlow_detect");
#endif
	i2c_set_clientdata(client, bqdev);

	irq = bq40z60_request_irq_wake(bqdev, bqdev->n_charger_prsnt_gpio, 0,
			bq40z60_charger_changed_irq, "charger-irq");
	if (irq < 0) {
		ret = irq;
		goto out_err_charger_irq;
	}

	ret = bq40z60_request_batlow_irq(bqdev);
	if (ret < 0)
		goto out_err_batlow_irq;

	bq40z60_write(bqdev, BQ40Z60_INIT_DISCHARGE_SET, 7000);

	queue_delayed_work(bqdev->battery_wq, &bqdev->poll_work, 15 * HZ);

	ret = sysfs_create_bin_file(&client->dev.kobj, &eeprom_attr);
	if (ret)
		dev_warn(&client->dev, "unable to register eeprom\n");

#ifdef CONFIG_BATTERY_BQ40Z60_FIRMWARE_ACCESS
	ret = sysfs_create_bin_file(&client->dev.kobj, &firmware_attr);
	if (ret)
		dev_warn(&client->dev, "unable to register firmware eeprom\n");
#endif
	device_create_file(&client->dev, &dev_attr_soh);
	device_create_file(&client->dev, &dev_attr_shutdown);
	device_create_file(&client->dev, &dev_attr_sealed);
	device_create_file(&client->dev, &dev_attr_cell_voltage);

	dev_info(&client->dev, "%s driver registered\n", client->name);

	return 0;

out_err_batlow_irq:
	bq40z60_free_irq_wake(bqdev, bqdev->batlow_gpio);
out_err_charger_irq:
	power_supply_unregister(&bqdev->ac_supply);
out_err_ac_supply:
	power_supply_unregister(&bqdev->battery_supply);
out_err_battery_supply:
	gpio_free(bqdev->n_charging_gpio);
out_err_charging_gpio:
out_err_no_dev:
	kfree(bqdev);
	pm_runtime_disable(&client->dev);
	release_bus_freq(BUS_FREQ_HIGH);

	return ret;
}

static int bq40z60_remove(struct i2c_client *client)
{
	struct bq40z60_device *bqdev = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_sealed);
	device_remove_file(&client->dev, &dev_attr_cell_voltage);
	device_remove_file(&client->dev, &dev_attr_shutdown);
	device_remove_file(&client->dev, &dev_attr_soh);
	sysfs_remove_bin_file(&client->dev.kobj, &eeprom_attr);
#ifdef CONFIG_BATTERY_BQ40Z60_FIRMWARE_ACCESS
	sysfs_remove_bin_file(&client->dev.kobj, &firmware_attr);
#endif
	bq40z60_free_irq_wake(bqdev, bqdev->n_charger_prsnt_gpio);
	bq40z60_free_irq_wake(bqdev, bqdev->batlow_gpio);

	cancel_delayed_work_sync(&bqdev->poll_work);

	power_supply_unregister(&bqdev->battery_supply);
	power_supply_unregister(&bqdev->ac_supply);

	gpio_free(bqdev->n_charging_gpio);
#ifdef CONFIG_WAKELOCK
	wake_lock_destroy(&bqdev->batlow_wakelock);
#endif
	pm_runtime_disable(&client->dev);
	release_bus_freq(BUS_FREQ_HIGH);
	kfree(bqdev);

	return 0;
}

#if defined (CONFIG_PM)
static int bq40z60_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq40z60_device *bqdev = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&bqdev->poll_work);
	flush_workqueue(bqdev->battery_wq);
	release_bus_freq(BUS_FREQ_HIGH);

	return 0;
}

static int bq40z60_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq40z60_device *bqdev = i2c_get_clientdata(client);

	request_bus_freq(BUS_FREQ_HIGH);
	cancel_delayed_work(&bqdev->poll_work);
	queue_delayed_work(bqdev->battery_wq, &bqdev->poll_work, 2 * HZ);

	return 0;
}

#if defined CONFIG_PM_SLEEP

static int bq40z60_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "bq40z60_runtime_suspend\n");
	release_bus_freq(BUS_FREQ_HIGH);
	return 0;
}

static int bq40z60_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "bq40z60_runtime_resume\n");
	request_bus_freq(BUS_FREQ_HIGH);
	return 0;
}

#endif

static const struct dev_pm_ops bq40z60_pm_ops = {
	SET_RUNTIME_PM_OPS(bq40z60_runtime_suspend, bq40z60_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(bq40z60_suspend, bq40z60_resume)
};

#define BQ40Z60_PM_OPS (&bq40z60_pm_ops)

#else /* CONFIG_PM */
#define BQ40Z60_PM_OPS NULL
#endif

static const struct i2c_device_id bq40z60_id[] = {
	{"bq40z60", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, bq40z60_id);

static struct i2c_driver bq40z60_battery_driver = {
	.probe = bq40z60_probe,
	.remove = bq40z60_remove,
	.id_table = bq40z60_id,
	.driver = {
		.name = "bq40z60-battery",
		.owner = THIS_MODULE,
		.pm = BQ40Z60_PM_OPS,
	},
};

static int __init bq40z60_battery_init(void)
{
	int ret;

	ret = i2c_add_driver(&bq40z60_battery_driver);
	if (ret)
		pr_err("%s: i2c_add_driver failed\n", __func__);

	return ret;
}

module_init(bq40z60_battery_init);

static void __exit bq40z60_battery_exit(void)
{
	i2c_del_driver(&bq40z60_battery_driver);
}

module_exit(bq40z60_battery_exit);

MODULE_AUTHOR("Denis Grigoryev");
MODULE_DESCRIPTION("bq40z60 battery monitor driver");
MODULE_LICENSE("GPL");
