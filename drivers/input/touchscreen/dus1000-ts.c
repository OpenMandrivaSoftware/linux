/*
 * DUS1000 touchscreen driver
 *
 * Copyright (c) 2014 Fastwel Co.
 * Author: Denis Grigoryev <grigoryev@spb.prosoft.ru>
 *
 * Based on MicroTouch driver (drivers/input/touchscreen/mtouch.c)
 * Copyright (c) 2004 Vojtech Pavlik
 * and Dan Streetman <ddstreet@ieee.org>
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/regulator/consumer.h>

#include <asm/unaligned.h>

#define DRIVER_DESC	"DUS1000 serial touchscreen driver"

MODULE_AUTHOR("Denis Grigoryev <grigoryev@spb.prosoft.ru>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

#define DMC_REPORT_LENGTH    6
#define DMC_MAX_LENGTH       80

#define DMC_TOUCH_COMMAND_ID 0x2
#define DMC_TOUCH_REPORT_ID  0x4

#define DMC_TOUCH_COMMAND    0x4c

#define DMC_MAX_X 6399
#define DMC_MAX_Y 4607
#define MAX_FINGERS 3

#define DMC_POWER_uV 5000000

struct dmc_finger {
	__s32 x, y;
	__s32 contactid;
	__u16 trkid;
	bool touch_state;
	bool prev_touch_state;
	bool valid;
	bool delimiter;
};

struct dmc_dev {
	struct input_dev *dev;
	int idx;
	char phys[32];
	struct dmc_finger f[MAX_FINGERS];
	struct dmc_finger curdata;
	struct completion cmd_done;
	struct mutex cmd_lock;
	int started;
	 irqreturn_t(*handle_sym) (struct serio *, unsigned char sym);
	struct delayed_work delayed_work;
	struct serio *serio;
	__u8 data[DMC_REPORT_LENGTH];
	__u8 command;
	__u8 *buffer;
	int reply_len;
	int reply_max_len;
	__u8 num_received;
	__u16 trkid;
	struct regulator *power;
};

static __inline__ __u32 extract(__u8 * report, unsigned offset, unsigned n)
{
	u64 x;

	if (n > 32)
		printk(KERN_WARNING
		       "dus1000-ts: extract() called with n (%d) > 32! (%s)\n",
		       n, current->comm);

	report += offset >> 3;	/* adjust byte index */
	offset &= 7;		/* now only need bit offset into one byte */
	x = get_unaligned_le64(report);
	x = (x >> offset) & ((1ULL << n) - 1);	/* extract bit field */
	return (u32) x;
}

static void dmc_emit_event(struct dmc_dev *tsdev, struct input_dev *input)
{
	int count = 0;
	struct dmc_finger *oldest = 0;
	int i;

	for (i = 0; i < MAX_FINGERS; i++) {
		struct dmc_finger *f = &tsdev->f[i];
		if (!f->valid)
			continue;
		if (f->touch_state) {
			dev_dbg(&input->dev, "finger%d %d:%d\n", i, f->x, f->y);

			if (!f->prev_touch_state)
				f->trkid = tsdev->trkid++;
			input_report_abs(input, ABS_MT_TRACKING_ID, i);
			input_report_abs(input, ABS_MT_POSITION_X, f->x);
			input_report_abs(input, ABS_MT_POSITION_Y, f->y);
			input_mt_sync(input);
			count++;

			/* touchscreen emulation: pick the oldest contact */
			if (!oldest
			    || ((f->trkid - oldest->trkid) & (SHRT_MAX + 1)))
				oldest = f;
		}
		f->valid = 0;
	}
	/* touchscreen emulation */
	if (oldest) {
		input_event(input, EV_KEY, BTN_TOUCH, 1);
		input_event(input, EV_ABS, ABS_X, oldest->x);
		input_event(input, EV_ABS, ABS_Y, oldest->y);
	} else {
		input_event(input, EV_KEY, BTN_TOUCH, 0);
	}
	if (!count)
		input_mt_sync(input);

	input_sync(input);
	tsdev->num_received = 0;
}

static inline void dmc_set_idle(struct dmc_dev *tsdev)
{
	tsdev->handle_sym = NULL;
	tsdev->idx = 0;
}

static irqreturn_t dmc_in_report(struct serio *serio, unsigned char sym)
{
	__u8 *data;
	struct dmc_dev *tsdev = serio_get_drvdata(serio);
	struct input_dev *input = tsdev->dev;

	tsdev->data[tsdev->idx++] = sym;
	if (tsdev->idx < DMC_REPORT_LENGTH)
		return IRQ_HANDLED;

	data = &tsdev->data[1];

	tsdev->curdata.prev_touch_state = tsdev->curdata.touch_state;
	tsdev->curdata.touch_state = extract(data, 0, 1);
	tsdev->curdata.contactid = extract(data, 2, 4);
	tsdev->curdata.delimiter = extract(data, 6, 1);
	tsdev->curdata.valid = extract(data, 7, 1);
	tsdev->curdata.x = extract(data, 8, 16);
	tsdev->curdata.y = extract(data, 24, 16);

	tsdev->f[tsdev->curdata.contactid] = tsdev->curdata;
	tsdev->num_received++;

	if (tsdev->num_received > 0 && tsdev->curdata.delimiter)
		dmc_emit_event(tsdev, input);

	dmc_set_idle(tsdev);

	return IRQ_HANDLED;
}

static irqreturn_t dmc_in_data(struct serio *serio, unsigned char sym);

static irqreturn_t dmc_in_reply(struct serio *serio, unsigned char sym)
{
	struct dmc_dev *tsdev = serio_get_drvdata(serio);

	tsdev->data[tsdev->idx++] = sym;

	switch (tsdev->idx) {
	case 1:
		if (sym != DMC_TOUCH_COMMAND_ID) {
			dev_warn(&serio->dev, "invalid prefix 0x%.2x\n", sym);
			dmc_set_idle(tsdev);
		}

		break;
	case 2:
		if (sym != DMC_TOUCH_COMMAND) {
			dev_warn(&serio->dev, "invalid cmd ID 0x%.2x\n", sym);
			dmc_set_idle(tsdev);
		}

		break;
	case 3:
		/* At least one byte should be received */
		if (!sym || sym >= DMC_MAX_LENGTH) {
			dev_warn(&serio->dev, "invalid reply length %i\n", sym);
			dmc_set_idle(tsdev);
		}

		break;
	default:
		if (tsdev->command != sym) {
			dev_warn(&serio->dev, "invalid argument 0x%.2x\n", sym);
			dmc_set_idle(tsdev);
			break;
		}

		tsdev->reply_len = tsdev->data[2] - 1;

		if (tsdev->reply_len > tsdev->reply_max_len)
			tsdev->reply_len = tsdev->reply_max_len;

		dev_dbg(&serio->dev, "reply len: %i buffer: %p\n",
			tsdev->reply_len, tsdev->buffer);

		if (tsdev->reply_len && tsdev->buffer) {
			tsdev->idx = 0;
			tsdev->handle_sym = dmc_in_data;
		} else {
			complete(&tsdev->cmd_done);
			dmc_set_idle(tsdev);
		}

		break;
	}

	return IRQ_HANDLED;
}

static irqreturn_t dmc_in_data(struct serio *serio, unsigned char sym)
{
	struct dmc_dev *tsdev = serio_get_drvdata(serio);

	dev_dbg(&serio->dev, "%i: 0x%.2x\n", tsdev->idx, sym);

	tsdev->buffer[tsdev->idx++] = sym;
	if (tsdev->idx >= tsdev->reply_len) {
		complete(&tsdev->cmd_done);
		dmc_set_idle(tsdev);
	}

	return IRQ_HANDLED;
}

static irqreturn_t dmc_interrupt(struct serio *serio,
				 unsigned char sym, unsigned int flags)
{
	struct dmc_dev *tsdev = serio_get_drvdata(serio);

	/* Ignore other reports */
	if (!tsdev->handle_sym && !tsdev->idx) {
		switch (sym) {
		case DMC_TOUCH_REPORT_ID:
			tsdev->idx = 0;
			tsdev->handle_sym = dmc_in_report;
			break;
		case DMC_TOUCH_COMMAND_ID:
			tsdev->idx = 0;
			tsdev->handle_sym = dmc_in_reply;
			break;
		default:
			dev_dbg(&serio->dev, "Invalid character: 0x%.2x\n",
				sym);
			return IRQ_HANDLED;
		}
	}

	if (tsdev->handle_sym)
		return tsdev->handle_sym(serio, sym);

	return IRQ_HANDLED;
}

static int dmc_serio_write(struct serio *serio, __u8 * blk, size_t len)
{
	int i;
	int err;

	for (i = 0; i < len; i++) {
		err = serio_write(serio, blk[i]);
		if (err) {
			dev_err(&serio->dev, "failed to send command: %i\n",
				err);
			return err;
		}
	}

	return 0;
}

static int __dmc_send_command(struct serio *serio, __u8 cmd, __u8 arg,
			      int has_arg)
{
	__u8 command[5];

	command[0] = DMC_TOUCH_COMMAND_ID;
	command[1] = DMC_TOUCH_COMMAND;
	command[2] = has_arg ? 2 : 1;
	command[3] = cmd;
	command[4] = arg;

	dev_dbg(&serio->dev, "command: %.2x:%.2x:%.2x:%.2x:%.2x\n",
		command[0], command[1], command[2], command[3], command[4]);

	return dmc_serio_write(serio, command, has_arg ? 5 : 4);
}

static int dmc_send_command(struct serio *serio, __u8 cmd, __u8 arg,
			    int has_arg)
{
	int rc;
	struct dmc_dev *tsdev = serio_get_drvdata(serio);

	mutex_lock(&tsdev->cmd_lock);
	rc = __dmc_send_command(serio, cmd, arg, has_arg);
	mutex_unlock(&tsdev->cmd_lock);

	return rc;
}

static int dmc_send_command_reply(struct serio *serio,
				  __u8 cmd, __u8 arg, int has_arg,
				  __u8 * reply, size_t reply_len,
				  unsigned long timeout)
{
	int rc;
	struct dmc_dev *tsdev = serio_get_drvdata(serio);

	if (!reply && !reply_len && !timeout) {
		WARN_ON(1);
		return -EINVAL;
	}

	mutex_lock(&tsdev->cmd_lock);

	tsdev->command = cmd;
	tsdev->reply_max_len = reply_len;
	tsdev->buffer = reply;
	init_completion(&tsdev->cmd_done);

	rc = __dmc_send_command(serio, cmd, arg, has_arg);
	if (rc == 0) {
		rc = tsdev->reply_len;
		if (!wait_for_completion_timeout(&tsdev->cmd_done, timeout)) {
			dev_warn(&serio->dev, "command timeout\n");
			dmc_set_idle(tsdev);
			rc = -ETIMEDOUT;
		} else
			rc = tsdev->reply_len;
	}

	mutex_unlock(&tsdev->cmd_lock);

	return rc;
}

static int dmc_adjust_offset(struct serio *serio)
{
	int rc;
	__u8 reply;

	rc = dmc_send_command_reply(serio, 0x17, 0, 0, &reply, 1, 5 * HZ);
	if (rc != 1) {
		dev_err(&serio->dev, "CMD17 error: %i\n", rc);
		return rc;
	}

	if (reply != 1) {
		dev_err(&serio->dev, "CMD17 invalid reply: %i\n", reply);
		return -EIO;
	}

	return 0;
}

static int dmc_calibrate_offset(struct serio *serio)
{
	int rc;
	__u8 reply;

	rc = dmc_send_command_reply(serio, 0x1, 0, 0, &reply, 1, 20 * HZ);
	if (rc != 1) {
		dev_err(&serio->dev, "CMD01 error: %i\n", rc);
		return rc;
	}

	if (reply != 1) {
		dev_err(&serio->dev, "CMD01 invalid reply: %i\n", reply);
		return -EIO;
	}

	return 0;
}

static int dmc_start(struct serio *serio)
{
	struct dmc_dev *tsdev = serio_get_drvdata(serio);

	tsdev->started = 1;
	return dmc_send_command(serio, 0x81, 0x1, 1);
}

static int dmc_stop(struct serio *serio)
{
	struct dmc_dev *tsdev = serio_get_drvdata(serio);

	tsdev->started = 0;
	return dmc_send_command(serio, 0x81, 0x0, 1);
}

static int dmc_read_version(struct serio *serio, char *buffer, size_t size)
{
	return dmc_send_command_reply(serio, 0x4, 0, 1, buffer, size, HZ);
}

static int dmc_read_firmware_details(struct serio *serio, char *buffer,
				     size_t size)
{
	return dmc_send_command_reply(serio, 0x6, 0, 1, buffer, size, HZ);
}

static int dmc_calibration(struct serio *serio)
{
	struct dmc_dev *tsdev = serio_get_drvdata(serio);

	if (tsdev->started)
		return -EBUSY;

	return dmc_adjust_offset(serio) || dmc_calibrate_offset(serio);
}

static ssize_t dmc_show_version(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct serio *serio = container_of(dev, struct serio, dev);

	return dmc_read_version(serio, buf, 255);
}

static ssize_t dmc_show_firmware(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct serio *serio = container_of(dev, struct serio, dev);

	return dmc_read_firmware_details(serio, buf, 255);
}

static ssize_t dmc_show_calibration(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	int status;
	struct serio *serio = container_of(dev, struct serio, dev);

	status = dmc_calibration(serio);

	return sprintf(buf, "%i\n", status);
}

static ssize_t dmc_show_start(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct serio *serio = container_of(dev, struct serio, dev);
	struct dmc_dev *tsdev = serio_get_drvdata(serio);

	return sprintf(buf, "%i\n", tsdev->started);
}

static ssize_t dmc_store_start(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int err;
	struct serio *serio = container_of(dev, struct serio, dev);

	if (buf[0] == '0')
		err = dmc_stop(serio);
	else
		err = dmc_start(serio);

	return err < 0 ? err : count;
}

static DEVICE_ATTR(version, 0666, dmc_show_version, NULL);
static DEVICE_ATTR(firmware, 0666, dmc_show_firmware, NULL);
static DEVICE_ATTR(start, 0666, dmc_show_start, dmc_store_start);
static DEVICE_ATTR(calibration, 0666, dmc_show_calibration, NULL);

static struct attribute *dmc_device_attrs[] = {
	&dev_attr_version.attr,
	&dev_attr_firmware.attr,
	&dev_attr_start.attr,
	&dev_attr_calibration.attr,
	NULL,
};

static struct attribute_group dmc_device_attr_group = {
	.attrs = dmc_device_attrs,
};

static void dmc_disconnect(struct serio *serio)
{
	struct dmc_dev *tsdev = serio_get_drvdata(serio);

	cancel_delayed_work_sync(&tsdev->delayed_work);
	sysfs_remove_group(&serio->dev.kobj, &dmc_device_attr_group);
	input_get_device(tsdev->dev);
	input_unregister_device(tsdev->dev);
	serio_close(serio);
	serio_set_drvdata(serio, NULL);
	input_put_device(tsdev->dev);
	kfree(tsdev);
}

static void dmc_print_version(struct serio *serio)
{
	int rc;
	char version[DMC_MAX_LENGTH + 1];

	rc = dmc_read_version(serio, version, sizeof(version));
	if (rc > 0 && rc < DMC_MAX_LENGTH) {
		version[rc] = 0;
		dev_info(&serio->dev, "version %s\n", version);
	} else
		dev_warn(&serio->dev, "failed to read version rc=%i\n", rc);
}

static void dmc_resume_work(struct work_struct *work)
{
	struct dmc_dev *tsdev =
	    container_of(work, struct dmc_dev, delayed_work.work);

	dmc_start(tsdev->serio);
}

static int dmc_connect(struct serio *serio, struct serio_driver *drv)
{
	struct dmc_dev *tsdev;
	struct input_dev *input_dev;
	struct regulator *pwr;
	int err;

	tsdev = kzalloc(sizeof(struct dmc_dev), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!tsdev || !input_dev) {
		err = -ENOMEM;
		goto fail1;
	}

	pwr = devm_regulator_get(&serio->dev, "ts-pwr");
	if (pwr != NULL) {
		if (regulator_can_change_voltage(pwr))
			err = regulator_set_voltage(pwr, DMC_POWER_uV, DMC_POWER_uV);
		else
			err = 0;

		if (err == 0)
			err = regulator_enable(pwr);
		else
			dev_err(&serio->dev, "regulator_set_voltage failed: %d\n", err);

		if (err == 0) {
			msleep(50);
			tsdev->power = pwr;
		} else {
			dev_err(&serio->dev, "Unable to power-up touchscreen, error %d\n", err);
			devm_regulator_put(pwr);
		}

	}

	tsdev->dev = input_dev;
	snprintf(tsdev->phys, sizeof(tsdev->phys), "%s/input0", serio->phys);
	input_dev->name = "DUS1000 Serial TouchScreen";
	input_dev->phys = tsdev->phys;
	input_dev->id.bustype = BUS_RS232;
	input_dev->id.vendor = SERIO_DMC;
	input_dev->id.product = 0;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &serio->dev;

	init_completion(&tsdev->cmd_done);
	mutex_init(&tsdev->cmd_lock);

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, DMC_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, DMC_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TRACKING_ID, 0, MAX_FINGERS - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, DMC_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_X, 0, DMC_MAX_X, 0, 0);

	serio_set_drvdata(serio, tsdev);

	err = serio_open(serio, drv);
	if (err)
		goto fail2;

	err = sysfs_create_group(&serio->dev.kobj, &dmc_device_attr_group);
	if (err)
		goto fail3;

	err = input_register_device(tsdev->dev);
	if (err)
		goto fail4;

	if (dmc_start(serio)) {
		dev_warn(&serio->dev, "Could not start device\n");
		goto fail5;
	}

	tsdev->serio = serio;
	INIT_DELAYED_WORK(&tsdev->delayed_work, dmc_resume_work);

	dmc_print_version(serio);

	return 0;
fail5:
	input_unregister_device(tsdev->dev);
fail4:
	sysfs_remove_group(&serio->dev.kobj, &dmc_device_attr_group);
fail3:
	serio_close(serio);
fail2:
	serio_set_drvdata(serio, NULL);
fail1:
	input_free_device(input_dev);
	kfree(tsdev);
	return err;
}

/*
 * The serio driver structure.
 */

static struct serio_device_id dmc_serio_ids[] = {
	{
	 .type = SERIO_RS232,
	 .proto = SERIO_DMC,
	 .id = SERIO_ANY,
	 .extra = SERIO_ANY,
	 },
	{0}
};

MODULE_DEVICE_TABLE(serio, dmc_serio_ids);

static void dmc_cleanup(struct serio *serio)
{
	struct dmc_dev *tsdev = serio_get_drvdata(serio);

	cancel_delayed_work_sync(&tsdev->delayed_work);
	dmc_stop(serio);

	/* Ensure last character is sent */
	msleep(1);

	if (tsdev->power) {
		if (regulator_disable(tsdev->power))
			dev_err(&serio->dev, "Unable to power-down touchscreen\n");
	}
}

static int dmc_reconnect(struct serio *serio)
{
	struct dmc_dev *tsdev = serio_get_drvdata(serio);

	if (tsdev->power) {
		if (regulator_enable(tsdev->power) != 0)
			dev_err(&serio->dev, "Unable to power-up touchscreen\n");
	}

	/* 100 ms controller power-up delay */
	schedule_delayed_work(&tsdev->delayed_work, msecs_to_jiffies(100));

	return 0;
}

static struct serio_driver dmc_drv = {
	.driver = {
		   .name = "dus1000-ts",
		   },
	.description = DRIVER_DESC,
	.id_table = dmc_serio_ids,
	.interrupt = dmc_interrupt,
	.connect = dmc_connect,
	.reconnect = dmc_reconnect,
	.disconnect = dmc_disconnect,
	.cleanup = dmc_cleanup,
};

static int __init dmc_init(void)
{
	return serio_register_driver(&dmc_drv);
}

static void __exit dmc_exit(void)
{
	serio_unregister_driver(&dmc_drv);
}

module_init(dmc_init);
module_exit(dmc_exit);
