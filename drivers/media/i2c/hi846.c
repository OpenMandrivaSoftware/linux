// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020 Purism SPC

#include <asm/unaligned.h>
#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/pm.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define HI846_LINK_FREQ_437MHZ		437000000ULL
#define HI846_LINK_FREQ_288MHZ		288000000ULL
#define HI846_LINK_FREQ_200MHZ		200000000ULL

#define HI846_MEDIA_BUS_FORMAT		MEDIA_BUS_FMT_SBGGR8_1X8
//#define HI846_MEDIA_BUS_FORMAT		MEDIA_BUS_FMT_SGBRG8_1X8
//#define HI846_MEDIA_BUS_FORMAT		MEDIA_BUS_FMT_SGRBG8_1X8
//#define HI846_MEDIA_BUS_FORMAT		MEDIA_BUS_FMT_SRGGB8_1X8

//#define HI846_MEDIA_BUS_FORMAT		MEDIA_BUS_FMT_SBGGR10_1X10
//#define HI846_MEDIA_BUS_FORMAT		MEDIA_BUS_FMT_SGBRG10_1X10
//#define HI846_MEDIA_BUS_FORMAT		MEDIA_BUS_FMT_SGRBG10_1X10
//#define HI846_MEDIA_BUS_FORMAT		MEDIA_BUS_FMT_SRGGB10_1X10

//#define HI846_MEDIA_BUS_FORMAT		MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_BE
//#define HI846_MEDIA_BUS_FORMAT		MEDIA_BUS_FMT_SBGGR10_2X8_PADHI_LE
//#define HI846_MEDIA_BUS_FORMAT		MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_BE
//#define HI846_MEDIA_BUS_FORMAT		MEDIA_BUS_FMT_SBGGR10_2X8_PADLO_LE

#define HI846_LINE_LENGTH		3800

#define HI846_DATA_LANES		4
#define HI846_RGB_DEPTH			8

#define DEFAULT_FPS			30

#define HI846_REG_CHIP_ID_L		0x0f16
#define HI846_REG_CHIP_ID_H		0x0f17
#define HI846_CHIP_ID_L			0x46
#define HI846_CHIP_ID_H			0x08

/* Digital gain controls from sensor */
#define HI846_REG_MWB_GR_GAIN_H		0x0078
#define HI846_REG_MWB_GR_GAIN_L		0x0079
#define HI846_REG_MWB_GB_GAIN_H		0x007a
#define HI846_REG_MWB_GB_GAIN_L		0x007b
#define HI846_REG_MWB_R_GAIN_H		0x007c
#define HI846_REG_MWB_R_GAIN_L		0x007d
#define HI846_REG_MWB_B_GAIN_H		0x007e
#define HI846_REG_MWB_B_GAIN_L		0x007f
#define HI846_DGTL_GAIN_MIN		0
#define HI846_DGTL_GAIN_MAX		8191
#define HI846_DGTL_GAIN_STEP		1
#define HI846_DGTL_GAIN_DEFAULT		256

/* Analog gain controls from sensor */
#define HI846_REG_ANALOG_GAIN		0x0077 /* 0x0076 ? */
#define HI846_ANAL_GAIN_MIN		0
#define HI846_ANAL_GAIN_MAX		240
#define HI846_ANAL_GAIN_STEP		8

/* Test Pattern Control */
#define HI846_REG_ISP			0X0a05
#define HI846_REG_ISP_TPG_EN		0x01
#define HI846_REG_TEST_PATTERN		0x020a /* 1-9 */

/* ISP Common */
#define HI846_REG_MODE_SELECT		0x0a00
#define HI846_MODE_STANDBY		0x00
#define HI846_MODE_STREAMING		0x01


enum {
	HI846_LINK_FREQ_437MHZ_INDEX,
};

struct hi846_reg {
	u16 address;
	u16 val;
};

struct hi846_reg_list {
	u32 num_of_regs;
	const struct hi846_reg *regs;
};

/* hi846_res */
struct hi846_mode {
	/* Frame width in pixels */
	u32 width;

	/* Frame height in pixels */
	u32 height;

	/* Link frequency needed for this resolution */
	u32 link_freq;

	u16 fps;

	u16 frame_len;

	const struct hi846_reg_list reg_list;
};

#define to_hi846(_sd) container_of(_sd, struct hi846, sd)

static const struct hi846_reg hi846_init_regs[] = {
//#include "hi846_init_android_patch.txt"
#include "hi846_init_android_github1.txt"
};

static const struct hi846_reg mode_640x480_regs[] = {
/* 2lane */
#include "hi846_640_android_patch.txt"
};

static const struct hi846_reg mode_1280x720_regs[] = {
//#include "hi846_1280_android_patch.txt"
#include "hi846_1280_android_github1.txt"
};

static const struct hi846_reg mode_1632x1224_regs[] = {
//#include "hi846_1632_android_patch.txt"
#include "hi846_1632_android_github1.txt"
};

static const struct hi846_reg mode_3264x2448_regs[] = {
//#include "hi846_3264_android_patch.txt"
#include "hi846_3264_android_github1.txt"
};

static const struct hi846_reg hi846_set_test_pattern[] = {
	{0x0A00, 0x0000},
	{0x0a04, 0x0141},
	{0x020a, 0x0700},
	{0x0A00, 0x0100},
};

static const char * const hi846_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"100% Colour Bars",
	"Fade To Grey Colour Bars",
	"PN9",
	"Gradient Horizontal",
	"Gradient Vertical",
	"Check Board",
	"Slant Pattern",
	"Resolution Pattern",
};

static const s64 link_freq_menu_items[] = {
	HI846_LINK_FREQ_200MHZ,
};

static const struct hi846_reg_list hi846_init_regs_list = {
	.num_of_regs = ARRAY_SIZE(hi846_init_regs),
	.regs = hi846_init_regs,
};

static const struct hi846_reg_list hi846_set_test_pattern_regs_list = {
	.num_of_regs = ARRAY_SIZE(hi846_set_test_pattern),
	.regs = hi846_set_test_pattern,
};

/* hi846_valid_res */
static const struct hi846_mode supported_modes[] = {
#if 0
	/* the reg_list does 2-lane settings */
	{
		.width = 640,
		.height = 480,
		.link_freq = 80000000,
		.fps = 120,
		.frame_len = 631,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_640x480_regs),
			.regs = mode_640x480_regs,
		},
	},
#endif
	{
		.width = 1280,
		.height = 720,
		.link_freq = 200000000, /* 288000000 or 200000000 */
		.fps = 30,
		.frame_len = 842,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1280x720_regs),
			.regs = mode_1280x720_regs,
		},
	},
	{
		.width = 1632,
		.height = 1224,
		.link_freq = 200000000,
		.fps = 30,
		.frame_len = 2526,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_1632x1224_regs),
			.regs = mode_1632x1224_regs,
		},
	},
	{
		.width = 3264,
		.height = 2448,
		.link_freq = 300000000,
		.fps = 30,
		.frame_len = 2526,
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(mode_3264x2448_regs),
			.regs = mode_3264x2448_regs,
		},
	}
};

struct hi846_gpio {
	int gpio;
	int level;
};

enum hi846_gpio_id {
	RST,
	NUM_GPIOS,
};

struct hi846_datafmt {
	u32 code;
	enum v4l2_colorspace colorspace;
};

struct hi846 {
	struct hi846_gpio gpios[NUM_GPIOS];
	struct regulator *vdd1_regulator;
	struct regulator *vdd_regulator;
	struct clk *clock;
	struct hi846_datafmt *fmt;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_pix_format pix;
	struct v4l2_captureparm streamcap;

	/* V4L2 Controls */
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;

	/* Current mode */
	const struct hi846_mode *cur_mode;

	/* To serialize asynchronus callbacks */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
};

static struct hi846_datafmt hi846_colour_fmts[] = {
	{HI846_MEDIA_BUS_FORMAT, V4L2_COLORSPACE_RAW},
};

static struct hi846_datafmt *hi846_find_datafmt(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hi846_colour_fmts); i++)
		if (hi846_colour_fmts[i].code == code)
			return hi846_colour_fmts + i;

	return NULL;
}

static int get_capturemode(int width, int height)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		if ((supported_modes[i].width == width) &&
		    (supported_modes[i].height == height))
			return i;
	}

	return -1;
}

static u64 to_pixel_rate(u32 f_index)
{
	u64 pixel_rate = link_freq_menu_items[f_index] * 2 * HI846_DATA_LANES;

	do_div(pixel_rate, HI846_RGB_DEPTH);

	return pixel_rate;
}

static int hi846_read_reg(struct hi846 *hi846, u16 reg, u8 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&hi846->sd);
	struct i2c_msg msgs[2];
	u8 addr_buf[2];
	u8 data_buf[1] = {0};
	int ret;

	put_unaligned_be16(reg, addr_buf);
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(addr_buf);
	msgs[0].buf = addr_buf;
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = &data_buf[0];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "i2c read error: %d\n", ret);
		return -EIO;
	}

	*val = data_buf[0];

	return 0;
}

static int hi556_write_reg(struct hi846 *hi846, u16 reg, u16 len, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&hi846->sd);
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << 8 * (4 - len), buf + 2);
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int hi556_write_reg_list(struct hi846 *hi846,
				const struct hi846_reg_list *r_list)
{
	struct i2c_client *client = v4l2_get_subdevdata(&hi846->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < r_list->num_of_regs; i++) {
		ret = hi556_write_reg(hi846, r_list->regs[i].address,
				      2,
				      r_list->regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "failed to write reg 0x%4.4x. error = %d",
					    r_list->regs[i].address, ret);
			return ret;
		}
	}

	return 0;
}

static int hi846_write_reg(struct hi846 *hi846, u16 reg, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&hi846->sd);
	u8 buf[3] = { reg >> 8, reg & 0xff, val };
	int ret;

	struct i2c_msg msg[] = {
		{ .addr = client->addr, .flags = 0,
		  .len = ARRAY_SIZE(buf), .buf = buf },
	};

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		dev_err(&client->dev, "i2c write error\n");
		return -EIO;
	}

	return 0;
}

static int hi846_update_digital_gain(struct hi846 *hi846, u16 d_gain)
{
	int ret;

	ret = hi846_write_reg(hi846, HI846_REG_MWB_GR_GAIN_H, d_gain >> 8);
	if (ret)
		return ret;
	ret = hi846_write_reg(hi846, HI846_REG_MWB_GR_GAIN_L, d_gain & 0xff);
	if (ret)
		return ret;

	ret = hi846_write_reg(hi846, HI846_REG_MWB_GB_GAIN_H, d_gain >> 8);
	if (ret)
		return ret;
	ret = hi846_write_reg(hi846, HI846_REG_MWB_GB_GAIN_L, d_gain & 0xff);
	if (ret)
		return ret;

	ret = hi846_write_reg(hi846, HI846_REG_MWB_R_GAIN_H, d_gain >> 8);
	if (ret)
		return ret;
	ret = hi846_write_reg(hi846, HI846_REG_MWB_R_GAIN_L, d_gain & 0xff);
	if (ret)
		return ret;

	ret = hi846_write_reg(hi846, HI846_REG_MWB_B_GAIN_H, d_gain >> 8);
	if (ret)
		return ret;
	return hi846_write_reg(hi846, HI846_REG_MWB_B_GAIN_L, d_gain & 0xff);
}

static int hi846_test_pattern(struct hi846 *hi846, u32 pattern)
{
	int ret;
	u8 val;

	if (pattern) {
		ret = hi846_read_reg(hi846, HI846_REG_ISP, &val);
		if (ret)
			return ret;

		ret = hi846_write_reg(hi846, HI846_REG_ISP,
				      val | HI846_REG_ISP_TPG_EN);
		if (ret)
			return ret;
	}

	return hi846_write_reg(hi846, HI846_REG_TEST_PATTERN, pattern);
}

static int hi846_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct hi846 *hi846 = container_of(ctrl->handler,
					     struct hi846, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&hi846->sd);
	int ret = 0;

	/* V4L2 controls values will be applied only when power is already up */

	ret = pm_runtime_get_sync(&client->dev);
	if (ret < 0) {
		dev_err(&client->dev, "%s: pm_runtime_get failed: %d\n",
			__func__, ret);
		pm_runtime_put_noidle(&client->dev);
		return ret;
	}

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		ret = hi846_write_reg(hi846, HI846_REG_ANALOG_GAIN, ctrl->val);
		break;

	case V4L2_CID_DIGITAL_GAIN:
		ret = hi846_update_digital_gain(hi846, ctrl->val);
		break;
#if 0
	case V4L2_CID_EXPOSURE:
		/* TODO integ_time registers */
		break;
#endif

	case V4L2_CID_TEST_PATTERN:
		ret = hi846_test_pattern(hi846, ctrl->val);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_mark_last_busy(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops hi846_ctrl_ops = {
	.s_ctrl = hi846_set_ctrl,
};

static int hi846_init_controls(struct hi846 *hi846)
{
	struct v4l2_ctrl_handler *ctrl_hdlr;
	int ret;

	ctrl_hdlr = &hi846->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 8);
	if (ret)
		return ret;

	ctrl_hdlr->lock = &hi846->mutex;

	hi846->pixel_rate = v4l2_ctrl_new_std
			    (ctrl_hdlr, &hi846_ctrl_ops,
			     V4L2_CID_PIXEL_RATE, 0,
			     to_pixel_rate(HI846_LINK_FREQ_437MHZ_INDEX),
			     1,
			     to_pixel_rate(HI846_LINK_FREQ_437MHZ_INDEX));
	v4l2_ctrl_new_std(ctrl_hdlr, &hi846_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  HI846_ANAL_GAIN_MIN, HI846_ANAL_GAIN_MAX,
			  HI846_ANAL_GAIN_STEP, HI846_ANAL_GAIN_MIN);
	v4l2_ctrl_new_std(ctrl_hdlr, &hi846_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  HI846_DGTL_GAIN_MIN, HI846_DGTL_GAIN_MAX,
			  HI846_DGTL_GAIN_STEP, HI846_DGTL_GAIN_DEFAULT);
	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &hi846_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(hi846_test_pattern_menu) - 1,
				     0, 0, hi846_test_pattern_menu);
	if (ctrl_hdlr->error)
		return ctrl_hdlr->error;

	hi846->sd.ctrl_handler = ctrl_hdlr;

	return 0;
}

static int hi846_start_streaming(struct hi846 *hi846)
{
	struct i2c_client *client = v4l2_get_subdevdata(&hi846->sd);
	//int link_freq_index;
	int ret;

	ret = __v4l2_ctrl_handler_setup(hi846->sd.ctrl_handler);
	if (ret)
		return ret;

#if 0
	ret = hi846_write_reg(hi846, HI846_REG_MODE_SELECT,
			      HI846_MODE_STREAMING);
	if (ret) {
		dev_err(&client->dev, "failed to start stream");
		return ret;
	}
	mdelay(10);
#endif

#if 1
	ret = hi556_write_reg_list(hi846, &hi846_set_test_pattern_regs_list);
	if (ret) {
		dev_err(&client->dev, "failed to set test pattern: %d\n", ret);
		return ret;
	}
	mdelay(10);
#endif

	dev_dbg(&client->dev, "%s: STARTED\n", __func__);

	return 0;
}

static void hi846_stop_streaming(struct hi846 *hi846)
{
	struct i2c_client *client = v4l2_get_subdevdata(&hi846->sd);

	dev_dbg(&client->dev, "%s\n", __func__);

	if (hi846_write_reg(hi846, HI846_REG_MODE_SELECT, HI846_MODE_STANDBY))
		dev_err(&client->dev, "failed to stop stream");
}

static int hi846_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct hi846 *hi846 = to_hi846(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	dev_dbg(&client->dev, "%s\n", __func__);

	if (hi846->streaming == enable)
		return 0;

	mutex_lock(&hi846->mutex);
	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			mutex_unlock(&hi846->mutex);
			return ret;
		}

		ret = hi846_start_streaming(hi846);
		if (ret) {
			enable = 0;
			hi846_stop_streaming(hi846);
			pm_runtime_mark_last_busy(&client->dev);
			pm_runtime_put_autosuspend(&client->dev);
		}
	} else {
		hi846_stop_streaming(hi846);
		pm_runtime_mark_last_busy(&client->dev);
		pm_runtime_put_autosuspend(&client->dev);
	}

	hi846->streaming = enable;
	mutex_unlock(&hi846->mutex);

	return ret;
}

static int hi846_regulator_enable(struct hi846 *hi846)
{
	int ret = 0;

	ret = regulator_enable(hi846->vdd_regulator);
	if (ret)
		return ret;

	return regulator_enable(hi846->vdd1_regulator);
}

static int __maybe_unused hi846_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hi846 *hi846 = to_hi846(sd);

	dev_dbg(dev, "%s\n", __func__);

	//mutex_lock(&hi846->mutex);
	if (hi846->streaming)
		hi846_stop_streaming(hi846);

	if (!IS_ERR(hi846->clock))
		clk_disable_unprepare(hi846->clock);

	regulator_disable(hi846->vdd1_regulator);
	regulator_disable(hi846->vdd_regulator);
	//mutex_unlock(&hi846->mutex);

	return 0;
}

static int __maybe_unused hi846_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hi846 *hi846 = to_hi846(sd);
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	ret = hi846_regulator_enable(hi846);
	if (ret) {
		dev_err(dev, "enable regulator failed\n");
		return ret;
	}

	ret = clk_prepare_enable(hi846->clock);
	if (ret < 0)
		goto error_regulator;

	msleep(500);

	//mutex_lock(&hi846->mutex);
	if (hi846->streaming) {
		ret = hi846_start_streaming(hi846);
		if (ret) {
			dev_err(dev, "%s: start streaming failed\n", __func__);
			goto error;
		}
	}

	//mutex_unlock(&hi846->mutex);

	return 0;

error:
	hi846_stop_streaming(hi846);
	dev_err(dev, "%s: stopped streaming\n", __func__);
	hi846->streaming = 0;
	//mutex_unlock(&hi846->mutex);

error_regulator:
	regulator_disable(hi846->vdd1_regulator);
	regulator_disable(hi846->vdd_regulator);
	return ret;
}

static int hi846_set_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *f_interval)
{
	struct hi846 *hi846 = to_hi846(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct v4l2_fract *timeperframe = &f_interval->interval;

	dev_dbg(&client->dev, "starting %s: %d / %d\n", __func__,
		timeperframe->numerator, timeperframe->denominator);

	if ((timeperframe->numerator == 0) ||
	    (timeperframe->denominator == 0)) {
		timeperframe->denominator = DEFAULT_FPS;
		timeperframe->numerator = 1;
	}

/* FIXME */
dev_dbg(&client->dev, "overwriting with default\n");
timeperframe->denominator = DEFAULT_FPS;
timeperframe->numerator = 1;

	hi846->streamcap.timeperframe = f_interval->interval;

	return 0;
}

static int hi846_get_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *f_interval)
{
	struct hi846 *hi846 = to_hi846(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "starting %s\n", __func__);

	memset(f_interval, 0, sizeof(*f_interval));

	f_interval->interval = hi846->streamcap.timeperframe;

	return 0;
}

static int hi846_set_video_mode(struct hi846 *hi846, int fps)
{
	int frame_length;
	int ret;

	frame_length = hi846->cur_mode->link_freq / fps / HI846_LINE_LENGTH;
	if (frame_length < hi846->cur_mode->frame_len)
		frame_length = hi846->cur_mode->frame_len;

	pr_debug("%s: frame length calculated: %d\n", __func__, frame_length);

	ret = hi556_write_reg(hi846, 0x0006, 2, frame_length & 0xFFFF);
	ret = hi556_write_reg(hi846, 0x0008, 2, HI846_LINE_LENGTH & 0xFFFF);

	return ret;
}

static int hi846_set_format(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *format)
{
	struct hi846 *hi846 = to_hi846(sd);
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct v4l2_fract *timeperframe = &hi846->streamcap.timeperframe;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hi846_datafmt *fmt = hi846_find_datafmt(mf->code);
	int capturemode;
	u32 tgt_fps;
	int ret;

	dev_dbg(&client->dev, "------------ starting %s ---------\n", __func__);
	if (!fmt) {
		mf->code = hi846_colour_fmts[0].code;
		mf->colorspace = hi846_colour_fmts[0].colorspace;
		fmt = &hi846_colour_fmts[0];
	}

	mf->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	hi846->fmt = fmt;

	/* TODO remove when v4l2 below works */
	capturemode = get_capturemode(mf->width, mf->height);
	dev_dbg(&client->dev, "%s: found mode %d: %dx%d\n", __func__, capturemode, mf->width, mf->height);
	if (capturemode < 0) {
		dev_err(&client->dev, "%s: capturemode: %d wrong\n", __func__, capturemode);
		return -EINVAL;
	}

	hi846->cur_mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes), width,
				      height, mf->width,
				      mf->height);
	dev_dbg(&client->dev, "%s: found mode: %dx%d\n", __func__, hi846->cur_mode->width, hi846->cur_mode->height);

	/* TODO then use cur_mode here */
	tgt_fps = timeperframe->denominator / timeperframe->numerator;
	dev_dbg(&client->dev, "%s: target fps: %d\n", __func__, tgt_fps);

	ret = pm_runtime_get_sync(&client->dev);
	if (ret < 0) {
		dev_err(&client->dev, "%s: pm_runtime_get failed: %d\n",
			__func__, ret);
		pm_runtime_put_noidle(&client->dev);
		return ret;
	}

	dev_dbg(&client->dev, "writing regs now\n");
	ret = hi556_write_reg_list(hi846, &hi846_init_regs_list);
	if (ret) {
		dev_err(&client->dev, "failed to set plls: %d\n", ret);
		return ret;
	}

	ret = hi556_write_reg_list(hi846, &hi846->cur_mode->reg_list);
	if (ret) {
		dev_err(&client->dev, "failed to set mode: %d\n", ret);
		return ret;
	}

#if 0
	if (tgt_fps == 15)
		hi846_capture_setting_normal_video_2(hi846);
	else if (tgt_fps == 30)
		hi846_capture_setting_normal_video(hi846);
	else {
		dev_err(&client->dev, "frame rate %d not supported\n", tgt_fps);
		return -EINVAL;
	}
#endif

	/* TODO line length pck and frame length lines */
	hi846_set_video_mode(hi846, tgt_fps);

	hi846->streamcap.capturemode = capturemode;
	hi846->pix.width = mf->width;
	hi846->pix.height = mf->height;
	dev_dbg(&client->dev, "Set format w=%d h=%d mode=%d code=0x%x colorspace=0x%x\n",
		mf->width, mf->height, capturemode,
		fmt->code, fmt->colorspace);

	pm_runtime_mark_last_busy(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	dev_dbg(&client->dev, "--------------------------------------------\n");
	return 0;
}

static int hi846_get_format(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *format)
{
	struct hi846 *hi846 = to_hi846(sd);
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (format->pad)
		return -EINVAL;

	dev_dbg(&client->dev, "hi846: starting %s\n", __func__);

	mutex_lock(&hi846->mutex);
	mf->code        = HI846_MEDIA_BUS_FORMAT;
	mf->colorspace  = V4L2_COLORSPACE_RAW;
	mf->field       = V4L2_FIELD_NONE;
	mf->width       = hi846->cur_mode->width;
	mf->height      = hi846->cur_mode->height;
	mutex_unlock(&hi846->mutex);
	dev_dbg(&client->dev, "Get format w=%d h=%d code=0x%x colorspace=0x%x\n",
                        mf->width, mf->height,
                        mf->code, mf->colorspace);

	return 0;
}

static int hi846_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	pr_debug("%s: starting. pad: %d code index: %d (only 0 exists)\n",
		__func__, code->pad, code->index);
	if (code->pad || code->index >= ARRAY_SIZE(hi846_colour_fmts)) {
		pr_debug("%s: error: code pad set or index too high\n", __func__);
		return -EINVAL;
	}

	code->code = hi846_colour_fmts[code->index].code;
	pr_debug("%s: code: 0x%x\n", __func__, code->code);

	return 0;
}

static int hi846_enum_frame_size(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_frame_size_enum *fse)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s frame size index: %d\n", __func__, fse->index);

	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != HI846_MEDIA_BUS_FORMAT) {
		dev_err(&client->dev, "frame size enum not matching\n");
		return -EINVAL;
	}

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	dev_dbg(&client->dev, "%s: max width: %d max height: %d\n", __func__,
		fse->max_width, fse->max_height);

	return 0;
}

static int hi846_enum_frame_intervals(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_frame_interval_enum *fie)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;

	dev_dbg(dev, "starting %s index %d\n", __func__, fie->index);

	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	fie->interval.numerator = 1;
/* TODO FIXME for index 1?*/
	fie->interval.denominator = DEFAULT_FPS;

	return 0;
}

static int hi846_s_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static const struct v4l2_subdev_video_ops hi846_video_ops = {
	.s_stream = hi846_set_stream,
	.s_frame_interval = hi846_set_frame_interval,
	.g_frame_interval = hi846_get_frame_interval,
};

static const struct v4l2_subdev_pad_ops hi846_pad_ops = {
	.enum_frame_size = hi846_enum_frame_size,
	.enum_frame_interval = hi846_enum_frame_intervals,
	.enum_mbus_code = hi846_enum_mbus_code,
	.set_fmt = hi846_set_format,
	.get_fmt = hi846_get_format,
};

static struct v4l2_subdev_core_ops hi846_core_ops = {
	.s_power = hi846_s_power,
};

static const struct v4l2_subdev_ops hi846_subdev_ops = {
	.core = &hi846_core_ops,
	.video = &hi846_video_ops,
	.pad = &hi846_pad_ops,
};

static void hi846_gpio_assert(struct hi846 *hi846, int id)
{
	struct hi846_gpio *gpio = &hi846->gpios[id];
	if (gpio == 0)
		return;

	gpio_set_value(gpio->gpio, gpio->level);
}

#if 0
static void hi846_gpio_deassert(struct hi846 *hi846, int id)
{
	struct hi846_gpio *gpio = &hi846->gpios[id];
	if (gpio == 0)
		return;

	gpio_set_value(gpio->gpio, !gpio->level);
}
#endif

static int hi846_parse_gpios(struct hi846_gpio *gpios, struct device *dev)
{
	static const char * const names[] = {
		"rst-gpios",
	};
	struct device_node *node = dev->of_node;
	enum of_gpio_flags flags;
	int ret, i;

	for (i = 0; i < NUM_GPIOS; ++i) {
		ret = of_get_named_gpio_flags(node, names[i], 0, &flags);
		if (ret < 0) {
			dev_err(dev, "no %s GPIO pin provided\n", names[i]);
			return ret;
		}
		gpios[i].gpio = ret;
		gpios[i].level = !(flags & OF_GPIO_ACTIVE_LOW);
	}

	return 0;
}

static int hi846_identify_module(struct hi846 *hi846)
{
	struct i2c_client *client = v4l2_get_subdevdata(&hi846->sd);
	int ret;
	u8 val;

	ret = hi846_read_reg(hi846, HI846_REG_CHIP_ID_L, &val);
	if (ret)
		return ret;

	if (val != HI846_CHIP_ID_L) {
		dev_err(&client->dev, "wrong chip id low byte: %x", val);
		return -ENXIO;
	} else {
		dev_info(&client->dev, "chip id low byte correct: %X\n", val);
	}

	ret = hi846_read_reg(hi846, HI846_REG_CHIP_ID_H, &val);
	if (ret)
		return ret;

	if (val != HI846_CHIP_ID_H) {
		dev_err(&client->dev, "wrong chip id high byte: %x", val);
		return -ENXIO;
	} else {
		dev_info(&client->dev, "chip id high byte correct: %X\n", val);
	}

	return 0;
}

static int hi846_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *ep;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = 0;

	if (!fwnode)
		return -ENXIO;

	ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return ret;

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != HI846_DATA_LANES) {
		dev_err(dev, "number of CSI2 data lanes %d is not supported",
			bus_cfg.bus.mipi_csi2.num_data_lanes);
		ret = -EINVAL;
		goto check_hwcfg_error;
	}

check_hwcfg_error:
	v4l2_fwnode_endpoint_free(&bus_cfg);

	return ret;
}

static int hi846_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hi846 *hi846 = to_hi846(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	pm_runtime_put_noidle(&client->dev);
	mutex_destroy(&hi846->mutex);
	regulator_disable(hi846->vdd1_regulator);
	regulator_disable(hi846->vdd_regulator);

	return 0;
}

static int hi846_probe(struct i2c_client *client)
{
	struct hi846 *hi846;
	int ret;

	ret = hi846_check_hwcfg(&client->dev);
	if (ret) {
		dev_err(&client->dev, "failed to check HW configuration: %d",
			ret);
		return ret;
	}

	hi846 = devm_kzalloc(&client->dev, sizeof(*hi846), GFP_KERNEL);
	if (!hi846)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&hi846->sd, client, &hi846_subdev_ops);

	ret = hi846_parse_gpios(hi846->gpios, &client->dev);
	if (ret < 0) {
		dev_err(&client->dev, "parse gpios failed\n");
		return ret;
	}

	hi846->vdd_regulator = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(hi846->vdd_regulator))
		dev_warn(&client->dev, "cannot get voltage regulator\n");

	hi846->vdd1_regulator = devm_regulator_get(&client->dev, "vdd1");
	if (IS_ERR(hi846->vdd1_regulator))
		dev_warn(&client->dev, "cannot get voltage regulator\n");

	ret = hi846_regulator_enable(hi846);
	if (ret) {
		dev_err(&client->dev, "regulator enable failed: %d\n", ret);
		return ret;
	}

	hi846->clock = devm_clk_get(hi846->sd.dev, "mclk");
	if (IS_ERR(hi846->clock)) {
		dev_err(&client->dev, "get clk failed\n");
		ret = -EPROBE_DEFER;
		goto probe_error_regulator;
	}

	ret = clk_prepare_enable(hi846->clock);
	if (ret < 0)
		goto probe_error_regulator;

	msleep(500);

	hi846->streamcap.capability = V4L2_MODE_HIGHQUALITY |
					V4L2_CAP_TIMEPERFRAME;
	hi846->streamcap.capturemode = 0;
	hi846->streamcap.timeperframe.denominator = DEFAULT_FPS;
	hi846->streamcap.timeperframe.numerator = 1;

	hi846_gpio_assert(hi846, RST);

	ret = hi846_identify_module(hi846);
	if (ret) {
		dev_err(&client->dev, "failed to find sensor: %d", ret);
		goto probe_error_regulator;
	}

	mutex_init(&hi846->mutex);

	hi846->cur_mode = &supported_modes[0];
	ret = hi846_init_controls(hi846);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d", ret);
		goto probe_error_v4l2_ctrl_handler_free;
	}

	hi846->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = v4l2_async_register_subdev_sensor_common(&hi846->sd);
	if (ret < 0) {
		dev_err(&client->dev, "failed to register V4L2 subdev: %d",
			ret);
		goto probe_error_media_entity_cleanup;
	}

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);

	pm_runtime_set_autosuspend_delay(&client->dev, 3000);
	pm_runtime_use_autosuspend(&client->dev);

	return 0;

probe_error_media_entity_cleanup:

probe_error_v4l2_ctrl_handler_free:
	v4l2_ctrl_handler_free(hi846->sd.ctrl_handler);
	mutex_destroy(&hi846->mutex);

probe_error_regulator:
	regulator_disable(hi846->vdd1_regulator);
	regulator_disable(hi846->vdd_regulator);

	return ret;
}

UNIVERSAL_DEV_PM_OPS(hi846_pm_ops, hi846_suspend, hi846_resume, NULL);

static const struct of_device_id hi846_of_match[] = {
	{ .compatible = "hynix,hi846", },
	{},
};
MODULE_DEVICE_TABLE(of, hi846_of_match);

static struct i2c_driver hi846_i2c_driver = {
	.driver = {
		.name = "hi846",
		.pm = &hi846_pm_ops,
		.acpi_match_table = ACPI_PTR(hi846_acpi_ids),
		.of_match_table = of_match_ptr(hi846_of_match),
	},
	.probe_new = hi846_probe,
	.remove = hi846_remove,
};

module_i2c_driver(hi846_i2c_driver);

MODULE_AUTHOR("Angus Ainslie <angus@akkea.ca>");
MODULE_DESCRIPTION("Hynix HI846 sensor driver");
MODULE_LICENSE("GPL v2");
