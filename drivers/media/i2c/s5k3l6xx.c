// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Samsung S5K3L6XX 1/3" 13M CMOS Image Sensor.
 *
 * Copyright (C) 2020, Purism SPC
 *
 * Based on S5K5BAF driver
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/pm.h>
#include <linux/slab.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-fwnode.h>

static int debug;
module_param(debug, int, 0644);

#define S5K5BAF_DRIVER_NAME		"s5k3l6xx"
#define S5K5BAF_DEFAULT_MCLK_FREQ	24000000U
#define S5K5BAF_CLK_NAME		"mclk"

#define S5K3L6_REG_MODEL_ID_L		0x0000
#define S5K3L6_REG_MODEL_ID_H		0x0001
#define S5K3L6_MODEL_ID_L		0x30
#define S5K3L6_MODEL_ID_H		0xc6

#define S5K3L6_REG_REVISION_NUMBER	0x0002
#define S5K3L6_REVISION_NUMBER		0xb0
#define S5K3L6_REG_MANUFACTURER_ID	0x0003
#define S5K3L6_MANUFACTURER_ID		0x00

#define S5K3L6_REG_FRAME_COUNT		0x0005

#define S5K3L6_REG_TEST_PATTERN_MODE	0x0601
#define S5K3L6_TEST_PATTERN_COLOR_BAR	0x02

#define S5K3L6_REG_TEST_DATA_RED	0x0602
#define S5K3L6_REG_TEST_DATA_GREENR	0x0604
#define S5K3L6_REG_TEST_DATA_BLUE	0x0606
#define S5K3L6_REG_TEST_DATA_GREENB	0x0608

#define S5K3L6_REG_MODE_SELECT		0x100
#define S5K3L6_MODE_STREAMING		0x1
#define S5K3L6_MODE_STANDBY		0x0

#define S5K3L6_REG_DATA_FORMAT		0x0112
#define S5K3L6_DATA_FORMAT_RAW8		0x0808

#define S5K3L6_CIS_WIDTH		4208
#define S5K3L6_CIS_HEIGHT		3120

#define S5K5BAF_CIS_WIDTH		1600
#define S5K5BAF_CIS_HEIGHT		1200
#define S5K5BAF_WIN_WIDTH_MIN		8
#define S5K5BAF_WIN_HEIGHT_MIN		8
#define S5K5BAF_GAIN_RED_DEF		127
#define S5K5BAF_GAIN_GREEN_DEF		95
#define S5K5BAF_GAIN_BLUE_DEF		180

/* Default number of MIPI CSI-2 data lanes used */
#define S5K5BAF_DEF_NUM_LANES		1

/*
 * H/W register Interface (PAGE_IF_HW)
 */
#define REG_SW_LOAD_COMPLETE		0x0014
#define REG_CMDWR_PAGE			0x0028
#define REG_CMDWR_ADDR			0x002a
#define REG_CMDRD_PAGE			0x002c
#define REG_CMD_BUF			0x0f12
#define REG_SET_HOST_INT		0x1000
#define REG_CLEAR_HOST_INT		0x1030
#define REG_PATTERN_SET			0x3100
#define REG_PATTERN_WIDTH		0x3118
#define REG_PATTERN_HEIGHT		0x311a
#define REG_PATTERN_PARAM		0x311c

/*
 * S/W register interface (PAGE_IF_SW)
 */

/* Initialization parameters */
/* Master clock frequency in KHz */
#define REG_I_INCLK_FREQ_L		0x01b8
#define REG_I_INCLK_FREQ_H		0x01ba
#define  MIN_MCLK_FREQ_KHZ		6000U
#define  MAX_MCLK_FREQ_KHZ		48000U
#define REG_I_USE_NPVI_CLOCKS		0x01c6
#define  NPVI_CLOCKS			1
#define REG_I_USE_NMIPI_CLOCKS		0x01c8
#define  NMIPI_CLOCKS			1
#define REG_I_BLOCK_INTERNAL_PLL_CALC	0x01ca

/* Clock configurations, n = 0..2. REG_I_* frequency unit is 4 kHz. */
#define REG_I_OPCLK_4KHZ(n)		((n) * 6 + 0x01cc)
#define REG_I_MIN_OUTRATE_4KHZ(n)	((n) * 6 + 0x01ce)
#define REG_I_MAX_OUTRATE_4KHZ(n)	((n) * 6 + 0x01d0)
#define  SCLK_PVI_FREQ			24000
#define  SCLK_MIPI_FREQ			48000
#define  PCLK_MIN_FREQ			6000
#define  PCLK_MAX_FREQ			48000
#define REG_I_USE_REGS_API		0x01de
#define REG_I_INIT_PARAMS_UPDATED	0x01e0
#define REG_I_ERROR_INFO		0x01e2

/* General purpose parameters */
#define REG_USER_BRIGHTNESS		0x01e4
#define REG_USER_CONTRAST		0x01e6
#define REG_USER_SATURATION		0x01e8
#define REG_USER_SHARPBLUR		0x01ea

#define REG_G_SPEC_EFFECTS		0x01ee
#define REG_G_ENABLE_PREV		0x01f0
#define REG_G_ENABLE_PREV_CHG		0x01f2
#define REG_G_NEW_CFG_SYNC		0x01f8
#define REG_G_PREVREQ_IN_WIDTH		0x01fa
#define REG_G_PREVREQ_IN_HEIGHT		0x01fc
#define REG_G_PREVREQ_IN_XOFFS		0x01fe
#define REG_G_PREVREQ_IN_YOFFS		0x0200
#define REG_G_PREVZOOM_IN_WIDTH		0x020a
#define REG_G_PREVZOOM_IN_HEIGHT	0x020c
#define REG_G_PREVZOOM_IN_XOFFS		0x020e
#define REG_G_PREVZOOM_IN_YOFFS		0x0210
#define REG_G_INPUTS_CHANGE_REQ		0x021a
#define REG_G_ACTIVE_PREV_CFG		0x021c
#define REG_G_PREV_CFG_CHG		0x021e
#define REG_G_PREV_OPEN_AFTER_CH	0x0220
#define REG_G_PREV_CFG_ERROR		0x0222
#define  CFG_ERROR_RANGE		0x0b
#define REG_G_PREV_CFG_BYPASS_CHANGED	0x022a
#define REG_G_ACTUAL_P_FR_TIME		0x023a
#define REG_G_ACTUAL_P_OUT_RATE		0x023c
#define REG_G_ACTUAL_C_FR_TIME		0x023e
#define REG_G_ACTUAL_C_OUT_RATE		0x0240

/* Preview control section. n = 0...4. */
#define PREG(n, x)			((n) * 0x26 + x)
#define REG_P_OUT_WIDTH(n)		PREG(n, 0x0242)
#define REG_P_OUT_HEIGHT(n)		PREG(n, 0x0244)
#define REG_P_FMT(n)			PREG(n, 0x0246)
#define REG_P_MAX_OUT_RATE(n)		PREG(n, 0x0248)
#define REG_P_MIN_OUT_RATE(n)		PREG(n, 0x024a)
#define REG_P_PVI_MASK(n)		PREG(n, 0x024c)
#define  PVI_MASK_MIPI			0x52
#define REG_P_CLK_INDEX(n)		PREG(n, 0x024e)
#define  CLK_PVI_INDEX			0
#define  CLK_MIPI_INDEX			NPVI_CLOCKS
#define REG_P_FR_RATE_TYPE(n)		PREG(n, 0x0250)
#define  FR_RATE_DYNAMIC		0
#define  FR_RATE_FIXED			1
#define  FR_RATE_FIXED_ACCURATE		2
#define REG_P_FR_RATE_Q_TYPE(n)		PREG(n, 0x0252)
#define  FR_RATE_Q_DYNAMIC		0
#define  FR_RATE_Q_BEST_FRRATE		1 /* Binning enabled */
#define  FR_RATE_Q_BEST_QUALITY		2 /* Binning disabled */
/* Frame period in 0.1 ms units */
#define REG_P_MAX_FR_TIME(n)		PREG(n, 0x0254)
#define REG_P_MIN_FR_TIME(n)		PREG(n, 0x0256)
#define  S5K5BAF_MIN_FR_TIME		333  /* x100 us */
#define  S5K5BAF_MAX_FR_TIME		6500 /* x100 us */
/* The below 5 registers are for "device correction" values */
#define REG_P_SATURATION(n)		PREG(n, 0x0258)
#define REG_P_SHARP_BLUR(n)		PREG(n, 0x025a)
#define REG_P_GLAMOUR(n)		PREG(n, 0x025c)
#define REG_P_COLORTEMP(n)		PREG(n, 0x025e)
#define REG_P_GAMMA_INDEX(n)		PREG(n, 0x0260)
#define REG_P_PREV_MIRROR(n)		PREG(n, 0x0262)
#define REG_P_CAP_MIRROR(n)		PREG(n, 0x0264)
#define REG_P_CAP_ROTATION(n)		PREG(n, 0x0266)

/* Extended image property controls */
/* Exposure time in 10 us units */
#define REG_SF_USR_EXPOSURE_L		0x03bc
#define REG_SF_USR_EXPOSURE_H		0x03be
#define REG_SF_USR_EXPOSURE_CHG		0x03c0
#define REG_SF_USR_TOT_GAIN		0x03c2
#define REG_SF_USR_TOT_GAIN_CHG		0x03c4
#define REG_SF_RGAIN			0x03c6
#define REG_SF_RGAIN_CHG		0x03c8
#define REG_SF_GGAIN			0x03ca
#define REG_SF_GGAIN_CHG		0x03cc
#define REG_SF_BGAIN			0x03ce
#define REG_SF_BGAIN_CHG		0x03d0
#define REG_SF_WBGAIN_CHG		0x03d2
#define REG_SF_FLICKER_QUANT		0x03d4
#define REG_SF_FLICKER_QUANT_CHG	0x03d6

/* Output interface (parallel/MIPI) setup */
#define REG_OIF_EN_MIPI_LANES		0x03f2
#define REG_OIF_EN_PACKETS		0x03f4
#define  EN_PACKETS_CSI2		0xc3
#define REG_OIF_CFG_CHG			0x03f6

/* Auto-algorithms enable mask */
#define REG_DBG_AUTOALG_EN		0x03f8
#define  AALG_ALL_EN			BIT(0)
#define  AALG_AE_EN			BIT(1)
#define  AALG_DIVLEI_EN			BIT(2)
#define  AALG_WB_EN			BIT(3)
#define  AALG_USE_WB_FOR_ISP		BIT(4)
#define  AALG_FLICKER_EN		BIT(5)
#define  AALG_FIT_EN			BIT(6)
#define  AALG_WRHW_EN			BIT(7)

/* Pointers to color correction matrices */
#define REG_PTR_CCM_HORIZON		0x06d0
#define REG_PTR_CCM_INCANDESCENT	0x06d4
#define REG_PTR_CCM_WARM_WHITE		0x06d8
#define REG_PTR_CCM_COOL_WHITE		0x06dc
#define REG_PTR_CCM_DL50		0x06e0
#define REG_PTR_CCM_DL65		0x06e4
#define REG_PTR_CCM_OUTDOOR		0x06ec

#define REG_ARR_CCM(n)			(0x2800 + 36 * (n))

static const char * const s5k5baf_supply_names[] = {
	"vddio",	/* I/O power supply 1.8V (1.65V to 1.95V)
			   or 2.8V (2.5V to 3.1V) */
	"vdda",		/* Analog power supply 2.8V (2.6V to 3.0V) */
	"vddreg",	/* Regulator input power supply 1.8V (1.7V to 1.9V)
			   or 2.8V (2.6V to 3.0) */
};
#define S5K5BAF_NUM_SUPPLIES 1
// #define S5K5BAF_NUM_SUPPLIES ARRAY_SIZE(s5k5baf_supply_names)


static const u16 sensor_3l6_pllinfo_A_4128x3096_30fps[] = {
       // EXT_CLK_Mhz * 1000 * 1000, /* ext_clk */
       0x0301, 0x02, 0x01, /* vt_pix_clk_div   (0x0301) */
       0x0303, 0x01, 0x01, /* vt_sys_clk_div   (0x0303) */ // not in doc!
       0x0305, 0x03, 0x01, /* pre_pll_clk_div  (0x0305) */
       0x0307, 0x53, 0x01, /* pll_multiplier   (0x0307) */
       0x0309, 0x08, 0x01, /* op_pix_clk_div   (0x0309) */
       0x030b, 0x01, 0x01, /* op_sys_clk_div   (0x030B) */

       0x030d, 0x04, 0x01, /* secnd_pre_pll_clk_div    (0x030D) */
       0x030f, 0x5C, 0x01, /* secnd_pll_multiplier     (0x030F) */

	// WARNING: don't just copy, those are 2-bytes and the comment is about the high byte
       0x0340, 0x0CC1, 0x02, /* frame_length_lines     (0x0341) */
       0x0342, 0x1320, 0x02,/* line_length_pck (0x0343) */
};

static const u16 sensor_3l6_pllinfo_B_2064x1160_30fps[] = {
	// EXT_CLK_Mhz * 1000 * 1000, /* ext_clk */
	0x0301, 0x02, 0x01, /* vt_pix_clk_div	(0x0301) */
	0x0303, 0x01, 0x01, /* vt_sys_clk_div	(0x0303) */
	0x0305, 0x03, 0x01, /* pre_pll_clk_div	(0x0305) */
	0x0307, 0x53, 0x01, /* pll_multiplier	(0x0307) */
	0x0309, 0x08, 0x01, /* op_pix_clk_div	(0x0309) */
	0x030b, 0x04, 0x01, /* op_sys_clk_div	(0x030B) */

	0x030d, 0x04, 0x01, /* secnd_pre_pll_clk_div	(0x030D) */
	0x030f, 0x5C, 0x01, /* secnd_pll_multiplier	(0x030F) */
	0x0340, 0x0CC1, 0x02, /* frame_length_lines	(0x0341) */
	0x0342, 0x1320, 0x02, /* line_length_pck	(0x0343) */
};


static const u16 sensor_3l6_setfile_A_4128x3096_30fps[] = {
       //mode change   Single Still Capture (4:3)
       0x314A, 0x5F00, 0x02,
       0x3064, 0xFFCF, 0x02,
       0x3066, 0x7E00, 0x02,
       0x309C, 0x0640, 0x02,
       0x380C, 0x001A, 0x02,
       0x32B2, 0x0000, 0x02,
       0x32B4, 0x0000, 0x02,
       0x32B6, 0x0000, 0x02,
       0x32B8, 0x0000, 0x02,
       0x3090, 0x8800, 0x02,
       0x3238, 0x000C, 0x02,
       0x0100, 0x0000, 0x02,
       0x0344, 0x0030, 0x02,
       0x0348, 0x104F, 0x02,
       0x0346, 0x0014, 0x02,
       0x034A, 0x0C2B, 0x02,
       0x034C, 0x1020, 0x02,
       0x034E, 0x0C18, 0x02,
       0x0202, 0x03DE, 0x02,
       0x3400, 0x0000, 0x02,
       0x3402, 0x4E42, 0x02,
       0x0136, 0x1A00, 0x02,
       0x0304, 0x0003, 0x02,
       0x0306, 0x0053, 0x02,
       0x030C, 0x0004, 0x02,
       0x030E, 0x005C, 0x02,
       0x3C16, 0x0000, 0x02,
       0x0342, 0x1320, 0x02,
       0x0340, 0x0CC1, 0x02,
       0x0900, 0x0000, 0x02,
       0x0386, 0x0001, 0x02,
       0x3452, 0x0000, 0x02,
       0x345A, 0x0000, 0x02,
       0x345C, 0x0000, 0x02,
       0x345E, 0x0000, 0x02,
       0x3460, 0x0000, 0x02,
       0x38C4, 0x0009, 0x02,
       0x38D8, 0x002A, 0x02,
       0x38DA, 0x000A, 0x02,
       0x38DC, 0x000B, 0x02,
       0x38C2, 0x000A, 0x02,
       0x38C0, 0x000F, 0x02,
       0x38D6, 0x000A, 0x02,
       0x38D4, 0x0009, 0x02,
       0x38B0, 0x000F, 0x02,
       0x3932, 0x1000, 0x02,
       0x0820, 0x04AC, 0x02,
       0x3C34, 0x0008, 0x02,
       0x3C36, 0x2800, 0x02,
       0x3C38, 0x0028, 0x02,
       0x393E, 0x4000, 0x02,
	// the following not present in set B (otherwise identical)
       0x3C1E, 0x0100, 0x02, // system pll pd (what does it mean? power down?)
       0x0100, 0x0100, 0x02, // start streaming
       0x3C1E, 0x0000, 0x02,
};

static const u16 sensor_3l6_setfile_B_2064x1160_30fps[] = {
	/* VT Call 16:9 ratio */
	0x314A, 0x5F02, 0x02,
	0x3064, 0xFFCF, 0x02,
	0x3066, 0x7E00, 0x02,
	0x309C, 0x0640, 0x02,
	0x380C, 0x0090, 0x02,
	0x32B2, 0x0003, 0x02,
	0x32B4, 0x0003, 0x02,
	0x32B6, 0x0003, 0x02,
	0x32B8, 0x0003, 0x02,
	0x3090, 0x8000, 0x02,
	0x3238, 0x000B, 0x02,
	0x0100, 0x0000, 0x02, // stop stream
	0x0344, 0x0030, 0x02, // x_addr_start
	0x0348, 0x104F, 0x02, // x_addr_end
	0x0346, 0x0198, 0x02, // y_addr_start
	0x034A, 0x0AA7, 0x02, // y_addr_end
	0x034C, 0x0810, 0x02, // x_output_size
	0x034E, 0x0488, 0x02, // y_output_size
	0x0202, 0x0656, 0x02, // coarse_integration_time
	0x3400, 0x0000, 0x02, // can also be 0x1
	0x3402, 0x4E46, 0x02,
	// This part actually covers the same as pllinfo.
	0x0136, 0x1A00, 0x02, // extclk freq
	0x0304, 0x0003, 0x02, // pre_pll_clk_div
	0x0306, 0x0023, 0x02, // pll_multiplier
	0x030C, 0x0004, 0x02, // op_pre_pll_clk_div
	0x030E, 0x005C, 0x02, // op_pll_multiplier
	// not the next line
	0x3C16, 0x0000, 0x02,
	/// I don't get it, they are the same regardless of width. Frame length changes... with fps.
	0x0342, 0x1320, 0x02, // line_length pixel clocks = 4896??? gives 2832 pixels of time of blanking
	0x0340, 0x0CC1, 0x02, // frame_length_lines = 3265?? 2105 lines of blanking
	// presumes 480MHz pixel clock for 30fps
	// end pllinfo part
	0x0900, 0x0122, 0x02, // binning enable, 2:2
	0x0386, 0x0003, 0x02, // y_odd_inc
	0x3452, 0x0000, 0x02,
	0x345A, 0x0000, 0x02,
	0x345C, 0x0000, 0x02,
	0x345E, 0x0000, 0x02,
	0x3460, 0x0000, 0x02,
	0x38C4, 0x0009, 0x02,
	0x38D8, 0x002A, 0x02,
	0x38DA, 0x000A, 0x02,
	0x38DC, 0x000B, 0x02,
	0x38C2, 0x000A, 0x02,
	0x38C0, 0x000F, 0x02,
	0x38D6, 0x000A, 0x02,
	0x38D4, 0x0009, 0x02,
	0x38B0, 0x000F, 0x02,
	0x3932, 0x1000, 0x02,
	0x0820, 0x04AC, 0x02, // requested_link_rate freq of MIPI lane
	0x3C34, 0x0008, 0x02,
	0x3C36, 0x2800, 0x02,
	0x3C38, 0x0028, 0x02,
	0x393E, 0x4000, 0x02,
};

static const u16 sensor_3l6_setfile_A_Global[] = {
	// None of this is in the documentation. Skip?
       0x306A, 0x2F4C, 0x02,
       0x306C, 0xCA01, 0x02,
       0x307A, 0x0D20, 0x02,
       0x309E, 0x002D, 0x02,
       0x3072, 0x0013, 0x02,
       0x3074, 0x0977, 0x02,
       0x3076, 0x9411, 0x02,
       0x3024, 0x0016, 0x02,
       0x3002, 0x0E00, 0x02,
       0x3006, 0x1000, 0x02,
       0x300A, 0x0C00, 0x02,
       0x3018, 0xC500, 0x02,
       0x303A, 0x0204, 0x02,
       0x3266, 0x0001, 0x02,
       0x38DA, 0x000A, 0x02, // duplicated in resolution
       0x38DC, 0x000B, 0x02, // and this
       0x38D6, 0x000A, 0x02, // and this
       0x3070, 0x3D00, 0x02,
       0x3084, 0x1314, 0x02,
       0x3086, 0x0CE7, 0x02,
       0x3004, 0x0800, 0x02,
};

static const u16 sensor_3l6_setfile_B_Global[] = {
	// None of this is in the documentation. Skip?
	0x306A,	0x2F4C, 0x02,
	0x306C,	0xCA01, 0x02,
	0x307A,	0x0D20, 0x02,
	0x309E,	0x002D, 0x02,
	0x3072,	0x0013, 0x02,
	0x3074,	0x0977, 0x02,
	0x3076,	0x9411, 0x02,
	0x3024,	0x0016, 0x02,
	0x3002,	0x0E00, 0x02,
	0x3006,	0x1000, 0x02,
	0x300A,	0x0C00, 0x02,
	0x3018,	0xC500, 0x02,
	0x303A,	0x0204, 0x02,
	0x3266,	0x0001, 0x02,
	0x38DA,	0x000E, 0x02, // different: 0x000a
	0x38DC,	0x000B, 0x02,
	0x38D6,	0x000A, 0x02,
	0x3070,	0x3D00, 0x02,
	0x3084,	0x1314, 0x02,
	0x3086,	0x0CE7, 0x02,
	0x3004,	0x0800, 0x02,
	0x3C66,	0x0100, 0x02,	// Master mode disable ?? what is this even. Different than A
};

struct s5k5baf_gpio {
	int gpio;
	int level;
};

enum s5k5baf_gpio_id {
	RST,
	NUM_GPIOS,
};

#define PAD_CIS 0
#define PAD_OUT 1
#define NUM_CIS_PADS 1
#define NUM_ISP_PADS 2

struct s5k5baf_pixfmt {
	u32 code;
	u32 colorspace;
	/* REG_P_FMT(x) register value */
	u16 reg_p_fmt;
};

struct s5k3l6_frame {
	u32 width;
	u32 height;
	u32 code;
	const u16 *pllregs;
	u16 pllregcount;
	const u16 *streamregs;
	u16 streamregcount;
};

struct s5k5baf_ctrls {
	struct v4l2_ctrl_handler handler;
	struct { /* Auto / manual white balance cluster */
		struct v4l2_ctrl *awb;
		struct v4l2_ctrl *gain_red;
		struct v4l2_ctrl *gain_blue;
	};
	struct { /* Mirror cluster */
		struct v4l2_ctrl *hflip;
		struct v4l2_ctrl *vflip;
	};
	struct { /* Auto exposure / manual exposure and gain cluster */
		struct v4l2_ctrl *auto_exp;
		struct v4l2_ctrl *exposure;
		struct v4l2_ctrl *gain;
	};
};

enum {
	S5K5BAF_FW_ID_PATCH,
	S5K5BAF_FW_ID_CCM,
	S5K5BAF_FW_ID_CIS,
};

struct s5k5baf_fw {
	u16 count;
	struct {
		u16 id;
		u16 offset;
	} seq[0];
	u16 data[];
};

struct s5k5baf {
	struct s5k5baf_gpio gpios[NUM_GPIOS];
	enum v4l2_mbus_type bus_type;
	u8 nlanes;
	struct regulator_bulk_data supplies[S5K5BAF_NUM_SUPPLIES];

	struct clk *clock;
	u32 mclk_frequency;

	struct s5k5baf_fw *fw;

	struct v4l2_subdev cis_sd;
	struct media_pad cis_pad;

	struct v4l2_subdev sd;
	struct media_pad pads[NUM_ISP_PADS];

	/* protects the struct members below */
	struct mutex lock;

	int error;

	struct v4l2_rect crop_sink;
	struct v4l2_rect compose;
	struct v4l2_rect crop_source;
	/* index to s5k3l6_frames array */
	int frame_fmt_idx;
	/* actual frame interval in 100us */
	u16 fiv;
	/* requested frame interval in 100us */
	u16 req_fiv;
	/* cache for REG_DBG_AUTOALG_EN register */
	u16 auto_alg;

	struct s5k5baf_ctrls ctrls;

	unsigned int streaming:1;
	unsigned int apply_cfg:1;
	unsigned int apply_crop:1;
	unsigned int valid_auto_alg:1;
	unsigned int power;
};

static const struct s5k5baf_pixfmt s5k5baf_formats[] = {
	{ MEDIA_BUS_FMT_VYUY8_2X8,	V4L2_COLORSPACE_JPEG,	5 },
	/* range 16-240 */
	{ MEDIA_BUS_FMT_VYUY8_2X8,	V4L2_COLORSPACE_REC709,	6 },
	{ MEDIA_BUS_FMT_RGB565_2X8_BE,	V4L2_COLORSPACE_JPEG,	0 },
};

static const struct s5k5baf_pixfmt s5k3l6_formats[] = {
	// invalid, included to push through mx6s_vidioc_enum_fmt_vid_cap "no more fmt" error
	{ MEDIA_BUS_FMT_SBGGR8_1X8,	V4L2_COLORSPACE_RAW,	5 },
	// 8bit Bayer entry (BGGR?)
	// 10 bit Bayer entry
};

// Frame sizes are only available in RAW, so this effectively replaces pixfmt.
static const struct s5k3l6_frame s5k3l6_frames[] = {
	{
		.width = 2064, .height = 1160,
		.pllregs = sensor_3l6_pllinfo_B_2064x1160_30fps,
		.pllregcount = ARRAY_SIZE(sensor_3l6_pllinfo_B_2064x1160_30fps) / 3,
		.streamregs = sensor_3l6_setfile_B_2064x1160_30fps,
		.streamregcount = ARRAY_SIZE(sensor_3l6_setfile_B_2064x1160_30fps) / 3,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
	},
	{
		.width = 4128, .height = 3096,
		.pllregs = sensor_3l6_pllinfo_A_4128x3096_30fps,
		.pllregcount = ARRAY_SIZE(sensor_3l6_pllinfo_A_4128x3096_30fps) / 3,
		.streamregs = sensor_3l6_setfile_A_4128x3096_30fps,
		.streamregcount = ARRAY_SIZE(sensor_3l6_setfile_A_4128x3096_30fps) / 3,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
	},
};

static struct v4l2_rect s5k5baf_cis_rect = {
	0, 0, S5K5BAF_CIS_WIDTH, S5K5BAF_CIS_HEIGHT
};

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct s5k5baf, ctrls.handler)->sd;
}

static inline bool s5k5baf_is_cis_subdev(struct v4l2_subdev *sd)
{
	return sd->entity.function == MEDIA_ENT_F_CAM_SENSOR;
}

static inline struct s5k5baf *to_s5k5baf(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k5baf, sd);
}

static u8 s5k5baf_i2c_read(struct s5k5baf *state, u16 addr)
{
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);
	__be16 w;
	u8 res;
	struct i2c_msg msg[] = {
		{ .addr = c->addr, .flags = 0,
		  .len = 2, .buf = (u8 *)&w },
		{ .addr = c->addr, .flags = I2C_M_RD,
		  .len = 1, .buf = (u8 *)&res },
	};
	int ret;

	if (state->error)
		return 0;

	w = cpu_to_be16(addr);
	ret = i2c_transfer(c->adapter, msg, 2);

	v4l2_dbg(3, debug, c, "i2c_read: 0x%04x : 0x%02x\n", addr, res);

	if (ret != 2) {
		v4l2_err(c, "i2c_read: error during transfer (%d)\n", ret);
		state->error = ret;
	}
	return res;
}

static void s5k5baf_i2c_write(struct s5k5baf *state, u16 addr, u8 val)
{
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);
	u8 buf[3] = { addr >> 8, addr & 0xFF, val };

	struct i2c_msg msg[] = {
		{ .addr = c->addr, .flags = 0,
		  .len = 3, .buf = buf },
	};
	int ret;

	if (state->error)
		return;

	ret = i2c_transfer(c->adapter, msg, 1);

	v4l2_dbg(3, debug, c, "i2c_write to 0x%04x : 0x%02x\n", addr, val);

	if (ret != 1) {
		v4l2_err(c, "i2c_write: error during transfer (%d)\n", ret);
		state->error = ret;
	}
	// Not sure if actually needed. So really debugging code at the moment.
	int actual = s5k5baf_i2c_read(state, addr);
	if (actual != val) {
		v4l2_err(c, "i2c_write: value didn't stick. 0x%04x = 0x%02x != 0x%02x", addr, actual, val);
	}
}

static void s5k5baf_i2c_write2(struct s5k5baf *state, u16 addr, u16 val)
{
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);
	u8 buf[4] = { addr >> 8, addr & 0xFF, (val >> 8) & 0xff, val & 0xff };

	struct i2c_msg msg[] = {
		{ .addr = c->addr, .flags = 0,
		  .len = 4, .buf = buf },
	};
	int ret;

	if (state->error)
		return;

	ret = i2c_transfer(c->adapter, msg, 1);

	v4l2_dbg(3, debug, c, "i2c_write to 0x%04x : 0x%02x\n", addr, val);

	if (ret != 1) {
		v4l2_err(c, "i2c_write: error during transfer (%d)\n", ret);
		state->error = ret;
	}
}

static void s5k3l6_submit_regs(struct s5k5baf *state, const u16 *regs, u16 regcount) {
       unsigned i;
       for (i = 0; i < regcount; i+=3) {
	       if (regs[i+2] == 2)
		       s5k5baf_i2c_write2(state, regs[i], regs[i+1]);
	       else
		       s5k5baf_i2c_write(state, regs[i], (u8)regs[i+1]);
       }
}

static u8 s5k5baf_read(struct s5k5baf *state, u16 addr)
{
	return s5k5baf_i2c_read(state, addr);
}

static void s5k5baf_write(struct s5k5baf *state, u16 addr, u8 val)
{
	s5k5baf_i2c_write(state, addr, val);
}

#if 0
static void s5k5baf_synchronize(struct s5k5baf *state, int timeout, u16 addr)
{
	unsigned long end = jiffies + msecs_to_jiffies(timeout);
	u16 reg;

	s5k5baf_write(state, addr, 1);
	do {
		reg = s5k5baf_read(state, addr);
		if (state->error || !reg)
			return;
		usleep_range(5000, 10000);
	} while (time_is_after_jiffies(end));

	v4l2_err(&state->sd, "timeout on register synchronize (%#x)\n", addr);
	state->error = -ETIMEDOUT;
}
#endif

static void s5k3l6_hw_set_clocks(struct s5k5baf *state) {
	const struct s5k3l6_frame *frame = &s5k3l6_frames[state->frame_fmt_idx];
	s5k3l6_submit_regs(state, frame->pllregs, frame->pllregcount);
}

static void s5k3l6_hw_set_cis(struct s5k5baf *state) {
}

#if 0
static void s5k5baf_hw_sync_cfg(struct s5k5baf *state)
{
	s5k5baf_write(state, REG_G_PREV_CFG_CHG, 1);
	if (state->apply_crop) {
		s5k5baf_write(state, REG_G_INPUTS_CHANGE_REQ, 1);
		s5k5baf_write(state, REG_G_PREV_CFG_BYPASS_CHANGED, 1);
	}
	s5k5baf_synchronize(state, 500, REG_G_NEW_CFG_SYNC);
}
#endif

static int s5k3l6_find_pixfmt(struct v4l2_mbus_framefmt *mf)
{
	int i, c = -1;

	for (i = 0; i < ARRAY_SIZE(s5k3l6_frames); i++) {
		if ((mf->colorspace != V4L2_COLORSPACE_DEFAULT)
				&& (mf->colorspace != V4L2_COLORSPACE_RAW))
			continue;
		if ((mf->width != s5k3l6_frames[i].width) || (mf->height != s5k3l6_frames[i].height))
			continue;
		if (mf->code == s5k3l6_frames[i].code)
			return i;
	}
	return c;
}

static int s5k5baf_clear_error(struct s5k5baf *state)
{
	int ret = state->error;

	state->error = 0;
	return ret;
}

/*
static int s5k3l6_hw_set_video_bus(struct s5k5baf *state)
{
	s5k3l6_submit_regs(state, sensor_3l6_setfile_A_Global,
			   ARRAY_SIZE(sensor_3l6_setfile_A_Global) / 3);

	return s5k5baf_clear_error(state);
}*/

static const u16 setstream[] = {
	S5K3L6_REG_DATA_FORMAT, S5K3L6_DATA_FORMAT_RAW8, 2,
};

static void s5k3l6_hw_set_config(struct s5k5baf *state) {
	const struct s5k3l6_frame *frame_fmt = &s5k3l6_frames[state->frame_fmt_idx];
	v4l2_err(&state->sd, "Setting frame format %d", state->frame_fmt_idx);
	s5k3l6_submit_regs(state, frame_fmt->streamregs, frame_fmt->streamregcount);

	// This may mess up PLL settings...
	// If the above already enabled streaming (setfile A), we're also in trouble.
	s5k3l6_submit_regs(state, setstream, ARRAY_SIZE(setstream) / 3);
}

static void s5k3l6_hw_set_test_pattern(struct s5k5baf *state, int id)
{
	s5k5baf_write(state, S5K3L6_REG_TEST_PATTERN_MODE, (u8)id);
}


static void s5k5baf_gpio_assert(struct s5k5baf *state, int id)
{
	struct s5k5baf_gpio *gpio = &state->gpios[id];

	if (gpio == 0)
		return;

	gpio_set_value(gpio->gpio, gpio->level);
}

static void s5k5baf_gpio_deassert(struct s5k5baf *state, int id)
{
	struct s5k5baf_gpio *gpio = &state->gpios[id];

	if (gpio == 0)
		return;

	gpio_set_value(gpio->gpio, !gpio->level);
}

static int s5k5baf_power_on(struct s5k5baf *state)
{
	int ret;

	ret = regulator_bulk_enable(S5K5BAF_NUM_SUPPLIES, state->supplies);
	if (ret < 0)
		goto err;

	ret = clk_set_rate(state->clock, state->mclk_frequency);
	if (ret < 0)
		goto err_reg_dis;

	ret = clk_prepare_enable(state->clock);
	if (ret < 0)
		goto err_reg_dis;

	v4l2_dbg(1, debug, &state->sd, "clock frequency: %ld\n",
		 clk_get_rate(state->clock));

	usleep_range(50, 100);
	s5k5baf_gpio_deassert(state, RST);
	return 0;

err_reg_dis:
	regulator_bulk_disable(S5K5BAF_NUM_SUPPLIES, state->supplies);
err:
	v4l2_err(&state->sd, "%s() failed (%d)\n", __func__, ret);
	return ret;
}

static int s5k5baf_power_off(struct s5k5baf *state)
{
	int ret;

	state->streaming = 0;
	state->apply_cfg = 0;
	state->apply_crop = 0;

	s5k5baf_gpio_assert(state, RST);

	if (!IS_ERR(state->clock))
		clk_disable_unprepare(state->clock);

	ret = regulator_bulk_disable(S5K5BAF_NUM_SUPPLIES,
					state->supplies);
	if (ret < 0)
		v4l2_err(&state->sd, "failed to disable regulators\n");

	return 0;
}

/*
 * V4L2 subdev core and video operations
 */

static int s5k5baf_set_power(struct v4l2_subdev *sd, int on)
{
	struct s5k5baf *state = to_s5k5baf(sd);
	int ret = 0;

	mutex_lock(&state->lock);

	if (state->power != !on)
		goto out;

	if (on) {
		//s5k5baf_initialize_data(state);

		ret = s5k5baf_power_on(state); // TODO: test this
		if (ret < 0)
			goto out;

		/* TODO runtime pm or remove the s_power callback */

		// load global register set
		//ret = s5k3l6_hw_set_video_bus(state);
		if (ret < 0)
			goto out;
		// load pllinfo (will get overridden on stream start anyway)
		s5k3l6_hw_set_clocks(state);
		s5k3l6_hw_set_cis(state);
		//s5k5baf_hw_set_ccm(state);

		ret = s5k5baf_clear_error(state);
		if (!ret)
			state->power++;
	} else {
		s5k5baf_power_off(state);
		state->power--;
	}

out:
	mutex_unlock(&state->lock);

	if (!ret && on)
		ret = v4l2_ctrl_handler_setup(&state->ctrls.handler);

	return ret;
}

static void s5k3l6_hw_set_stream(struct s5k5baf *state, int enable)
{
	v4l2_err(&state->sd, "set_dtream sd %px", (void*)&state->sd);
	s5k5baf_i2c_write(state, S5K3L6_REG_MODE_SELECT, enable ? S5K3L6_MODE_STREAMING : S5K3L6_MODE_STANDBY);
}

static int s5k5baf_s_stream(struct v4l2_subdev *sd, int on)
{
	struct s5k5baf *state = to_s5k5baf(sd);
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);
	int ret = 0;

	mutex_lock(&state->lock);

	if (state->streaming == !!on) {
		ret = 0;
		goto out;
	}

	if (on) {
		ret = pm_runtime_get_sync(&c->dev);
		if (ret < 0) {
			dev_err(&c->dev, "%s: pm_runtime_get failed: %d\n",
				__func__, ret);
			pm_runtime_put_noidle(&c->dev);
			goto out;
		}

		//ret = s5k5baf_hw_set_crop_rects(state);
		s5k3l6_hw_set_config(state);
		if (ret < 0)
			goto out;
		s5k3l6_hw_set_stream(state, 1);
	} else {
		s5k3l6_hw_set_stream(state, 0);
		pm_runtime_mark_last_busy(&c->dev);
		pm_runtime_put_autosuspend(&c->dev);
	}
	ret = s5k5baf_clear_error(state);
	if (!ret)
		state->streaming = !state->streaming;

out:
	mutex_unlock(&state->lock);

	return ret;
}

/*
 * V4L2 subdev pad level and video operations
 */
static int s5k5baf_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	v4l2_err(sd, "enum_mbus idx %d", code->index);
	if (code->index >= ARRAY_SIZE(s5k3l6_frames))
		return -EINVAL;
	code->code = s5k3l6_frames[code->index].code;
	return 0;
}

static int s5k5baf_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	int i;

	if (fse->index > 0)
		return -EINVAL;

	i = ARRAY_SIZE(s5k3l6_frames);
	while (--i)
		if (fse->code == s5k3l6_frames[i].code)
			break;
	fse->code = s5k3l6_frames[i].code;
	fse->min_width = s5k3l6_frames[i].width;
	fse->max_width = s5k3l6_frames[i].width;
	fse->max_height = s5k3l6_frames[i].height;
	fse->min_height = s5k3l6_frames[i].height;

	return 0;
}

static int s5k3l6_try_cis_format(struct v4l2_mbus_framefmt *mf)
{
	int pixfmt;
/*
	v4l_bound_align_image(&mf->width, S5K5BAF_WIN_WIDTH_MIN,
			      S5K5BAF_CIS_WIDTH, 1,
			      &mf->height, S5K5BAF_WIN_HEIGHT_MIN,
			      S5K5BAF_CIS_HEIGHT, 1, 0);
*/

	struct s5k3l6_frame *mode = v4l2_find_nearest_size(s5k3l6_frames,
				      ARRAY_SIZE(s5k3l6_frames),
				      width, height,
				      mf->width, mf->height);
	mf->width = mode->width;
	mf->height = mode->height;

	pixfmt = s5k3l6_find_pixfmt(mf);
	if (pixfmt < 0)
		return pixfmt;

	mf->colorspace = V4L2_COLORSPACE_RAW;
	mf->code = s5k3l6_frames[pixfmt].code;
	mf->field = V4L2_FIELD_NONE;

	return pixfmt;
}

static int s5k5baf_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct s5k5baf *state = to_s5k5baf(sd);
	const struct s5k5baf_pixfmt *pixfmt;
	struct v4l2_mbus_framefmt *mf;

	v4l2_err(sd, "get_fmt");
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		fmt->format = *mf;
		return 0;
	}

	mf = &fmt->format;
	if (fmt->pad == PAD_CIS) {
		s5k3l6_try_cis_format(mf);
		return 0;
	}
	// DEAD
	v4l2_err(sd, "PAD IS NOT CIS");
	mf->field = V4L2_FIELD_NONE;
	mutex_lock(&state->lock);
	pixfmt = &s5k5baf_formats[state->frame_fmt_idx];
	mf->width = state->crop_source.width;
	mf->height = state->crop_source.height;
	mf->code = pixfmt->code;
	mf->colorspace = pixfmt->colorspace;
	mutex_unlock(&state->lock);

	return 0;
}

static int s5k5baf_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct s5k5baf *state = to_s5k5baf(sd);
	int pixfmt_idx = 0;
	mf->field = V4L2_FIELD_NONE;

	v4l2_err(sd, "set_fmt %p", (void*)sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = *mf;
		return 0;
	}

	mutex_lock(&state->lock);

	if (state->streaming) {
		mutex_unlock(&state->lock);
		return -EBUSY;
	}

	pixfmt_idx = s5k3l6_try_cis_format(mf);
	if (pixfmt_idx == -1) {
		v4l2_err(sd, "set_fmt choice unsupported");
		mutex_unlock(&state->lock);
		return -EINVAL; // could not find the format. Unsupported
	}
	state->frame_fmt_idx = pixfmt_idx;
	mf->code = s5k3l6_frames[state->frame_fmt_idx].code;
	mf->colorspace = V4L2_COLORSPACE_RAW;
	mf->width = s5k3l6_frames[state->frame_fmt_idx].width;
	mf->height = s5k3l6_frames[state->frame_fmt_idx].height;

	mutex_unlock(&state->lock);
	return 0;
}

enum selection_rect { R_CIS, R_CROP_SINK, R_COMPOSE, R_CROP_SOURCE, R_INVALID };

static const struct v4l2_subdev_pad_ops s5k5baf_cis_pad_ops = {
	.enum_mbus_code		= s5k5baf_enum_mbus_code,
	.enum_frame_size	= s5k5baf_enum_frame_size,
	.get_fmt		= s5k5baf_get_fmt,
	.set_fmt		= s5k5baf_set_fmt,
};

static const struct v4l2_subdev_pad_ops s5k5baf_pad_ops = {
	.enum_mbus_code		= s5k5baf_enum_mbus_code,
	.enum_frame_size	= s5k5baf_enum_frame_size,
//	.enum_frame_interval	= s5k5baf_enum_frame_interval,
	// doesn't seem to be used... ioctl(3, VIDIOC_S_FMT, ...)
	// instead seems to call enum_fmt, which does enum_mbus_code here.
	.get_fmt		= s5k5baf_get_fmt,
	.set_fmt		= s5k5baf_set_fmt,
//	.get_selection		= s5k5baf_get_selection,
//	.set_selection		= s5k5baf_set_selection,
};

static const struct v4l2_subdev_video_ops s5k5baf_video_ops = {
	//.g_frame_interval	= s5k5baf_g_frame_interval,
	//.s_frame_interval	= s5k5baf_s_frame_interval,
	.s_stream		= s5k5baf_s_stream,
};

/*
 * V4L2 subdev controls
 */

static int s5k5baf_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct s5k5baf *state = to_s5k5baf(sd);
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);
	int ret;

	v4l2_dbg(1, debug, sd, "ctrl: %s, value: %d\n", ctrl->name, ctrl->val);

	mutex_lock(&state->lock);

	ret = pm_runtime_get_sync(&c->dev);
	if (ret < 0) {
		dev_err(&c->dev, "%s: pm_runtime_get failed: %d\n",
			__func__, ret);
		pm_runtime_put_noidle(&c->dev);
		return ret;
	}

	if (state->power == 0)
		goto unlock;

	switch (ctrl->id) { /*
	case V4L2_CID_AUTO_WHITE_BALANCE:
		s5k5baf_hw_set_awb(state, ctrl->val);
		break;

	case V4L2_CID_BRIGHTNESS:
		s5k5baf_write(state, REG_USER_BRIGHTNESS, ctrl->val);
		break;

	case V4L2_CID_COLORFX:
		s5k5baf_hw_set_colorfx(state, ctrl->val);
		break;

	case V4L2_CID_CONTRAST:
		s5k5baf_write(state, REG_USER_CONTRAST, ctrl->val);
		break;

	case V4L2_CID_EXPOSURE_AUTO:
		s5k5baf_hw_set_auto_exposure(state, ctrl->val);
		break;

	case V4L2_CID_HFLIP:
		s5k5baf_hw_set_mirror(state);
		break;

	case V4L2_CID_POWER_LINE_FREQUENCY:
		s5k5baf_hw_set_anti_flicker(state, ctrl->val);
		break;

	case V4L2_CID_SATURATION:
		s5k5baf_write(state, REG_USER_SATURATION, ctrl->val);
		break;

	case V4L2_CID_SHARPNESS:
		s5k5baf_write(state, REG_USER_SHARPBLUR, ctrl->val);
		break;

	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		s5k5baf_write(state, REG_P_COLORTEMP(0), ctrl->val);
		if (state->apply_cfg)
			s5k5baf_hw_sync_cfg(state);
		break;
*/
	case V4L2_CID_TEST_PATTERN:
		v4l2_err(sd, "Setting pattern %d", ctrl->val);
		s5k3l6_hw_set_test_pattern(state, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN_RED:
		s5k5baf_i2c_write2(state, S5K3L6_REG_TEST_DATA_RED, (u16)ctrl->val & 0x3f);
		break;
	case V4L2_CID_TEST_PATTERN_GREENR:
		s5k5baf_i2c_write2(state, S5K3L6_REG_TEST_DATA_GREENR, (u16)ctrl->val & 0x3f);
		break;
	case V4L2_CID_TEST_PATTERN_BLUE:
		s5k5baf_i2c_write2(state, S5K3L6_REG_TEST_DATA_BLUE, (u16)ctrl->val & 0x3f);
		break;
	case V4L2_CID_TEST_PATTERN_GREENB:
		s5k5baf_i2c_write2(state, S5K3L6_REG_TEST_DATA_GREENB, (u16)ctrl->val & 0x3f);
		break;
	}
unlock:
	ret = s5k5baf_clear_error(state);

	pm_runtime_mark_last_busy(&c->dev);
	pm_runtime_put_autosuspend(&c->dev);

	mutex_unlock(&state->lock);
	return ret;
}

static const struct v4l2_ctrl_ops s5k5baf_ctrl_ops = {
	.s_ctrl	= s5k5baf_s_ctrl,
};

static const char * const s5k5baf_test_pattern_menu[] = {
	"Disabled",
	"Blank",
	"Bars",
	"Gradients",
	"Textile",
	"Textile2",
	"Squares"
};

static const char * const s5k3l6_test_pattern_menu[] = {
	"Disabled",
	"Solid", // Color selectable
	"Bars", // 8 bars 100% saturation: black, blue, red, magents, green, cyan, yellow, white
	"Fade", // Bars fading towards 50% at the bottom. 512px high. Subdivided into left smooth and right quantized halves.
	"White",
	"PN9", // pseudo-random noise
	"LFSR32",
	"Address",
};

static int s5k5baf_initialize_ctrls(struct s5k5baf *state)
{
	const struct v4l2_ctrl_ops *ops = &s5k5baf_ctrl_ops;
	struct s5k5baf_ctrls *ctrls = &state->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 16);
	if (ret < 0) {
		v4l2_err(&state->sd, "cannot init ctrl handler (%d)\n", ret);
		return ret;
	}

	/* Auto white balance cluster */
	/*
	ctrls->awb = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_AUTO_WHITE_BALANCE,
				       0, 1, 1, 1);
	ctrls->gain_red = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_RED_BALANCE,
					    0, 255, 1, S5K5BAF_GAIN_RED_DEF);
	ctrls->gain_blue = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BLUE_BALANCE,
					     0, 255, 1, S5K5BAF_GAIN_BLUE_DEF);
	v4l2_ctrl_auto_cluster(3, &ctrls->awb, 0, false);

	ctrls->hflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	ctrls->vflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_cluster(2, &ctrls->hflip);

	ctrls->auto_exp = v4l2_ctrl_new_std_menu(hdl, ops,
				V4L2_CID_EXPOSURE_AUTO,
				V4L2_EXPOSURE_MANUAL, 0, V4L2_EXPOSURE_AUTO);
	// Exposure time: x 1 us
	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					    0, 6000000U, 1, 100000U);
	// Total gain: 256 <=> 1x
	ctrls->gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_GAIN,
					0, 256, 1, 256);
	v4l2_ctrl_auto_cluster(3, &ctrls->auto_exp, 0, false);

	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_POWER_LINE_FREQUENCY,
			       V4L2_CID_POWER_LINE_FREQUENCY_AUTO, 0,
			       V4L2_CID_POWER_LINE_FREQUENCY_AUTO);

	v4l2_ctrl_new_std_menu(hdl, ops, V4L2_CID_COLORFX,
			       V4L2_COLORFX_SKY_BLUE, ~0x6f, V4L2_COLORFX_NONE);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_WHITE_BALANCE_TEMPERATURE,
			  0, 256, 1, 0);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_SATURATION, -127, 127, 1, 0);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_BRIGHTNESS, -127, 127, 1, 0);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_CONTRAST, -127, 127, 1, 0);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_SHARPNESS, -127, 127, 1, 0);
*/
	v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k3l6_test_pattern_menu) - 1,
				     0, 0, s5k3l6_test_pattern_menu);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_TEST_PATTERN_RED, 0, 1023, 1, 512);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_TEST_PATTERN_GREENR, 0, 1023, 1, 512);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_TEST_PATTERN_BLUE, 0, 1023, 1, 512);
	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_TEST_PATTERN_GREENB, 0, 1023, 1, 512);

	if (hdl->error) {
		v4l2_err(&state->sd, "error creating controls (%d)\n",
			 hdl->error);
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	state->sd.ctrl_handler = hdl;
	return 0;
}

/*
 * V4L2 subdev internal operations
 */
static int s5k5baf_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *mf;

	mf = v4l2_subdev_get_try_format(sd, fh->pad, PAD_CIS);
	s5k3l6_try_cis_format(mf);

	if (s5k5baf_is_cis_subdev(sd))
		return 0;
// DEAD CODE
	mf = v4l2_subdev_get_try_format(sd, fh->pad, PAD_OUT);
	mf->colorspace = s5k5baf_formats[0].colorspace;
	mf->code = s5k5baf_formats[0].code;
	mf->width = s5k5baf_cis_rect.width;
	mf->height = s5k5baf_cis_rect.height;
	mf->field = V4L2_FIELD_NONE;

	*v4l2_subdev_get_try_crop(sd, fh->pad, PAD_CIS) = s5k5baf_cis_rect;
	*v4l2_subdev_get_try_compose(sd, fh->pad, PAD_CIS) = s5k5baf_cis_rect;
	*v4l2_subdev_get_try_crop(sd, fh->pad, PAD_OUT) = s5k5baf_cis_rect;

	return 0;
}

static const struct v4l2_subdev_ops s5k5baf_cis_subdev_ops = {
	.pad	= &s5k5baf_cis_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k5baf_cis_subdev_internal_ops = {
	.open = s5k5baf_open,
};

static const struct v4l2_subdev_internal_ops s5k5baf_subdev_internal_ops = {
	// .registered = s5k5baf_registered, unneeded; was only registering CIS subdev
	// .unregistered = s5k5baf_unregistered, unneeded, was unregistering CIS
	.open = s5k5baf_open,
};

static const struct v4l2_subdev_core_ops s5k5baf_core_ops = {
	.s_power = s5k5baf_set_power,
	.log_status = v4l2_ctrl_subdev_log_status,
};

static const struct v4l2_subdev_ops s5k5baf_subdev_ops = {
	.core = &s5k5baf_core_ops,
	.pad = &s5k5baf_pad_ops,
	.video = &s5k5baf_video_ops,
};

static int __maybe_unused s5k3l6_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k5baf *state = to_s5k5baf(sd);

	dev_dbg(dev, "%s\n", __func__);

	if (state->streaming)
		s5k3l6_hw_set_stream(state, 0);

	s5k5baf_power_off(state);

	return 0;
}

static int __maybe_unused s5k3l6_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k5baf *state = to_s5k5baf(sd);
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	ret = s5k5baf_power_on(state);
	msleep(500);

	if (state->streaming)
		s5k3l6_hw_set_stream(state, 1);

	return 0;
}

static int s5k5baf_configure_gpios(struct s5k5baf *state)
{
	static const char * const name[] = { "S5K5BAF_RST" };
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);
	struct s5k5baf_gpio *g = state->gpios;
	int ret, i;

	for (i = 0; i < NUM_GPIOS; ++i) {
		int flags = GPIOF_DIR_OUT;
		if (g[i].level)
			flags |= GPIOF_INIT_HIGH;
		ret = devm_gpio_request_one(&c->dev, g[i].gpio, flags, name[i]);
		if (ret < 0) {
			v4l2_err(c, "failed to request gpio %s\n", name[i]);
			return ret;
		}
	}
	return 0;
}

static int s5k5baf_parse_gpios(struct s5k5baf_gpio *gpios, struct device *dev)
{
	static const char * const names[] = {
		"rstn-gpios",
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

static int s5k5baf_parse_device_node(struct s5k5baf *state, struct device *dev)
{
	struct device_node *node = dev->of_node;
	struct device_node *node_ep;
	struct v4l2_fwnode_endpoint ep = { .bus_type = 0 };
	int ret;

	if (!node) {
		dev_err(dev, "no device-tree node provided\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(node, "clock-frequency",
				   &state->mclk_frequency);
	if (ret < 0) {
		state->mclk_frequency = S5K5BAF_DEFAULT_MCLK_FREQ;
		dev_warn(dev, "using default %u Hz clock frequency\n",
			 state->mclk_frequency);
	}

	ret = s5k5baf_parse_gpios(state->gpios, dev);
	if (ret < 0) {
		dev_err(dev, "parse gpios failed\n");
		return ret;
	}

	node_ep = of_graph_get_next_endpoint(node, NULL);
	if (!node_ep) {
		dev_err(dev, "no endpoint defined at node %pOF\n", node);
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(node_ep), &ep);
	of_node_put(node_ep);
	if (ret) {
		dev_err(dev, "fwnode endpoint parse failed\n");
		return ret;
	}

	state->bus_type = ep.bus_type;

	switch (state->bus_type) {
	case V4L2_MBUS_CSI2_DPHY:
		state->nlanes = (unsigned char)ep.bus.mipi_csi2.num_data_lanes;
		break;
	default:
		dev_err(dev, "unsupported bus %d in endpoint defined at node %pOF\n",
			state->bus_type, (void*)node);
		return -EINVAL;
	}

	return 0;
}

static int s5k5baf_configure_subdevs(struct s5k5baf *state,
				     struct i2c_client *c)
{
	struct v4l2_subdev *sd;
	int ret;

	/*sd = &state->cis_sd;
	v4l2_subdev_init(sd, &s5k5baf_cis_subdev_ops);
	sd->owner = THIS_MODULE;
	v4l2_set_subdevdata(sd, state);
	snprintf(sd->name, sizeof(sd->name), "S5K5BAF-CIS %d-%04x",
		 i2c_adapter_id(c->adapter), c->addr);

	sd->internal_ops = &s5k5baf_cis_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	state->cis_pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, NUM_CIS_PADS, &state->cis_pad);
	if (ret < 0)
		goto err;*/
	v4l2_info(&state->sd, "probe sd %px", (void*)&state->sd);
	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, c, &s5k5baf_subdev_ops);
	snprintf(sd->name, sizeof(sd->name), "S5K3L6-CIS %d-%04x",
		 i2c_adapter_id(c->adapter), c->addr);
	v4l2_info(sd, "probe i2c %px", (void*)c);

	sd->internal_ops = &s5k5baf_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	state->cis_pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, NUM_CIS_PADS, &state->cis_pad);

	if (!ret)
		return 0;

	//media_entity_cleanup(&state->cis_sd.entity);
//err:
	dev_err(&c->dev, "cannot init media entity %s\n", sd->name);
	return ret;
}

static int s5k5baf_configure_regulators(struct s5k5baf *state)
{
	struct i2c_client *c = v4l2_get_subdevdata(&state->sd);
	int ret;
	int i;

	for (i = 0; i < S5K5BAF_NUM_SUPPLIES; i++)
		state->supplies[i].supply = s5k5baf_supply_names[i];

	ret = devm_regulator_bulk_get(&c->dev, S5K5BAF_NUM_SUPPLIES,
				      state->supplies);
	if (ret < 0)
		v4l2_err(c, "failed to get regulators\n");
	return ret;
}

static int s5k5baf_probe(struct i2c_client *c)
{
	struct s5k5baf *state;
	int ret;
	u8 test;

	state = devm_kzalloc(&c->dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	mutex_init(&state->lock);
	state->crop_sink = s5k5baf_cis_rect;
	state->compose = s5k5baf_cis_rect;
	state->crop_source = s5k5baf_cis_rect;

	ret = s5k5baf_parse_device_node(state, &c->dev);
	if (ret < 0) {
		pr_err("s5k5baf_parse_device_node: failed");
		return ret;
	}

	ret = s5k5baf_configure_subdevs(state, c);
	if (ret < 0) {
		pr_err("s5k5baf_configure_subdevs: failed");
		return ret;
	}

	ret = s5k5baf_configure_gpios(state);
	if (ret < 0) {
		pr_err("s5k5baf_parse_device_node: failed");
		goto err_me;
	}

	ret = s5k5baf_configure_regulators(state);
	if (ret < 0) {
		pr_err("s5k5baf_parse_device_node: failed");
		goto err_me;
	}

	state->clock = devm_clk_get(state->sd.dev, S5K5BAF_CLK_NAME);
	if (IS_ERR(state->clock)) {
		pr_err("get clk failed: failed");
		ret = -EPROBE_DEFER;
		goto err_me;
	}

	ret = s5k5baf_power_on(state);
	if (ret < 0) {
		pr_err("s5k5baf_power_on: failed");
		ret = -EPROBE_DEFER;
		goto err_me;
	}
	state->power = 1;

	test = s5k5baf_read(state, S5K3L6_REG_MODEL_ID_L);
	if (test != S5K3L6_MODEL_ID_L) {
		dev_err(&c->dev, "model mismatch: 0x%X != 0x30\n", test);
	} else {
		dev_info(&c->dev, "model low: 0x%X\n", test);
	}

	test = s5k5baf_read(state, S5K3L6_REG_MODEL_ID_H);
	if (test != S5K3L6_MODEL_ID_H) {
		dev_err(&c->dev, "model mismatch: 0x%X != 0xC6\n", test);
	} else {
		dev_info(&c->dev, "model high: 0x%X\n", test);
	}

	test = s5k5baf_read(state, S5K3L6_REG_REVISION_NUMBER);
	if (test != S5K3L6_REVISION_NUMBER) {
		dev_err(&c->dev, "revision mismatch: 0x%X != 0xB0\n", test);
	} else {
		dev_info(&c->dev, "revision number: 0x%X\n", test);
	}

	ret = s5k5baf_initialize_ctrls(state);
	if (ret < 0)
		goto err_me;
	v4l2_info(&state->sd, "pre-register sd %p ctrls %p",
		  (void*)&state->sd, (void*)&state->sd.ctrl_handler);
	ret = v4l2_async_register_subdev(&state->sd);
	if (ret < 0)
		goto err_ctrl;

	pm_runtime_set_active(&c->dev);
	pm_runtime_enable(&c->dev);
	pm_runtime_set_autosuspend_delay(&c->dev, 3000);
	pm_runtime_use_autosuspend(&c->dev);

	return 0;

err_ctrl:
	v4l2_ctrl_handler_free(state->sd.ctrl_handler);
err_me:
	media_entity_cleanup(&state->sd.entity);
	media_entity_cleanup(&state->cis_sd.entity);
	return ret;
}

static int s5k5baf_remove(struct i2c_client *c)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(c);
	struct s5k5baf *state = to_s5k5baf(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	media_entity_cleanup(&sd->entity);

	sd = &state->cis_sd;
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);

	pm_runtime_disable(&c->dev);
	pm_runtime_set_suspended(&c->dev);
	pm_runtime_put_noidle(&c->dev);

	return 0;
}

UNIVERSAL_DEV_PM_OPS(s5k3l6_pm_ops, s5k3l6_suspend, s5k3l6_resume, NULL);

static const struct i2c_device_id s5k5baf_id[] = {
	{ S5K5BAF_DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, s5k5baf_id);

static const struct of_device_id s5k5baf_of_match[] = {
	{ .compatible = "samsung,s5k3l6xx" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5k5baf_of_match);

static struct i2c_driver s5k5baf_i2c_driver = {
	.driver = {
		.pm = &s5k3l6_pm_ops,
		.of_match_table = s5k5baf_of_match,
		.name = S5K5BAF_DRIVER_NAME
	},
	.probe_new	= s5k5baf_probe,
	.remove		= s5k5baf_remove,
	.id_table	= s5k5baf_id,
};

module_i2c_driver(s5k5baf_i2c_driver);

MODULE_DESCRIPTION("Samsung S5K3L6XX 13M camera driver");
MODULE_AUTHOR("Martin Kepplinger <martin.kepplinger@puri.sm>");
MODULE_AUTHOR("Dorota Czaplejewicz <dorota.czaplejewicz@puri.sm>");
MODULE_LICENSE("GPL v2");
