// SPDX-License-Identifier: GPL-2.0
/*
 * gc2607 sensor driver
 *
 * Copyright (C) 2020 Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 first version.
 * V0.0X01.0X01 add quick stream on/off
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>

#include <media/v4l2-async.h>
#include <media/media-entity.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#define DRIVER_VERSION          KERNEL_VERSION(0, 0x01, 0x01)
#define GC2607_NAME             "gc2607"
#define GC2607_MEDIA_BUS_FMT    MEDIA_BUS_FMT_SGRBG10_1X10

#define MIPI_FREQ_297M          336000000
#define GC2607_XVCLK_FREQ       24000000



#define GC2607_REG_CHIP_ID_H    0x03f0
#define GC2607_REG_CHIP_ID_L    0x03f1

#define GC2607_REG_EXP_H        0x0202
#define GC2607_REG_EXP_L        0x0203

#define GC2607_REG_VTS_H        0x0340
#define GC2607_REG_VTS_L        0x0341

#define GC2607_REG_CTRL_MODE    0x0117
#define GC2607_MODE_SW_STANDBY  0x11
#define GC2607_MODE_STREAMING   0x91

#define REG_NULL                0xFF

#define GC2607_CHIP_ID          0x2607

#define GC2607_VTS_MAX          0x3FFF
#define GC2607_HTS_MAX          0xFFF

#define GC2607_EXPOSURE_MAX     0x3FFF
#define GC2607_EXPOSURE_MIN     1
#define GC2607_EXPOSURE_STEP    1

#define GC2607_GAIN_MIN         0x40
#define GC2607_GAIN_MAX         0x400
#define GC2607_GAIN_STEP        1
#define GC2607_GAIN_DEFAULT     64

#define GC2607_LANES            2

#define GC2607_REG_VALUE_08BIT		1
#define GC2607_REG_VALUE_16BIT		2
#define GC2607_REG_VALUE_24BIT		3
#define OF_CAMERA_PINCTRL_STATE_DEFAULT "rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP   "rockchip,camera_sleep"

#define SENSOR_ID(_msb, _lsb)   ((_msb) << 8 | (_lsb))

#define GC2607_FLIP_MIRROR_REG  0x0101

#define GC_MIRROR_BIT_MASK      BIT(0)
#define GC_FLIP_BIT_MASK        BIT(1)

static const char * const gc2607_supply_names[] = {
	"dovdd",    /* Digital I/O power */
	"avdd",     /* Analog power */
	"dvdd",     /* Digital core power */
};

#define GC2607_NUM_SUPPLIES ARRAY_SIZE(gc2607_supply_names)

#define to_gc2607(sd) container_of(sd, struct gc2607, subdev)

/*enum gc2607_max_pad {
	//PAD0,
	//PAD_MAX,
};*/

struct regval {
	u16 addr;
	u8 val;
};

struct gc2607_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 vc[PAD_MAX];
};

struct gc2607 {
	struct i2c_client   *client;
	struct clk      *xvclk;
	struct gpio_desc    *reset_gpio;
	struct gpio_desc    *pwdn_gpio;
	struct gpio_desc    *power_gpio;
	struct regulator_bulk_data supplies[GC2607_NUM_SUPPLIES];

	struct pinctrl      	*pinctrl;
	struct pinctrl_state    *pins_default;
	struct pinctrl_state    *pins_sleep;

	struct v4l2_subdev  subdev;
	struct media_pad    pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl    *exposure;
	struct v4l2_ctrl    *anal_gain;
	struct v4l2_ctrl    *hblank;
	struct v4l2_ctrl    *vblank;
	struct v4l2_ctrl    *h_flip;
	struct v4l2_ctrl    *v_flip;
	struct mutex        mutex;
	bool            streaming;
	bool			power_on;
	const struct gc2607_mode *cur_mode;
	unsigned int        lane_num;
	unsigned int        cfg_num;
	unsigned int        pixel_rate;

	u32         module_index;
	const char      *module_facing;
	const char      *module_name;
	const char      *len_name;
	struct rkmodule_awb_cfg awb_cfg;
	struct rkmodule_lsc_cfg lsc_cfg;
	u8			flip;
};

 
static const struct regval gc2607_1920x1080_regs_2lane[] = {
	//window_size=1920*1080,mipi 2lane
//mclk=24mhz,pclk=84Mhz,row_time=24.3810us,mipi_data_rate=672Mbps
//linelength=2048,framelength=1367
//max_framerate=30fps	
{0x03fe,0xf0},
{0x03fe,0xf0},
{0x03fe,0x00},
{0x03fe,0x00},		
{0x03fe,0x00},
{0x03fe,0x00},
{0x0d06,0x01},
{0x0315,0xd4},
{0x0d82,0x14},
{0x0a70,0x80},
{0x0134,0x5b},
{0x0110,0x01},
{0x0dd1,0x56},
{0x0137,0x03},
{0x0135,0x01},
{0x0136,0x2a},
{0x0130,0x08},
{0x0132,0x01},
{0x031c,0x93},
{0x0218,0x00},
{0x0340,0x05},
{0x0341,0x37},
{0x0342,0x08},
{0x0343,0x00},
{0x0220,0x05},
{0x0221,0x37},
{0x0af4,0x2b},
{0x0002,0x30},		
{0x00c3,0x3c},	
{0x0101,0x00},
{0x0d05,0xcc},
{0x0218,0x00},
{0x005e,0x84},
{0x0007,0x15},
{0x0350,0x01},
{0x00c0,0x07},
{0x00c1,0x90},
{0x0346,0x00},
{0x0347,0x02},
{0x034a,0x04},
{0x034b,0x40},
{0x021f,0x12},
{0x034c,0x07},
{0x034d,0x80},
{0x0353,0x00},
{0x0354,0x04},
{0x0d11,0x10},
{0x0d22,0x00},
{0x03f6,0x4d},
{0x03f5,0x3c}, 
{0x03f3,0x54},
{0x0d07,0xdd}, 
{0x0e71,0x00},
{0x0e72,0x10},
{0x0e17,0x26},
{0x0e22,0x0d},
{0x0e23,0x20},
{0x0e1b,0x30},
{0x0e3a,0x15},
{0x0e0a,0x00},
{0x0e0b,0x00},
{0x0e0e,0x00},
{0x0e2a,0x08}, 
{0x0e2b,0x08},
{0x0d02,0x73},
{0x0d22,0x38},
{0x0d25,0x00},
{0x0e6a,0x39},
{0x0050,0x05},
{0x0089,0x03},
{0x0070,0x40},
{0x0071,0x40},
{0x0072,0x40},
{0x0073,0x40},
{0x0040,0x82},
{0x0030,0x80},
{0x0031,0x80},
{0x0032,0x80},
{0x0033,0x80},
{0x0202,0x04},
{0x0203,0x38},
{0x02b3,0x00},
{0x02b3,0x00},
{0x02b4,0x00},
{0x0208,0x04},
{0x0209,0x00},
{0x009e,0x01},
{0x009f,0xa0},
{0x0db8,0x08},
{0x0db6,0x02},
{0x0db4,0x05},
{0x0db5,0x16},
{0x0db9,0x09},
{0x0d93,0x05},
{0x0d94,0x06},
{0x0d95,0x0b},
{0x0d99,0x10},
{0x0082,0x03},
{0x0107,0x05},
{0x0117,0x01},
{0x0d80,0x07},
{0x0d81,0x02},
{0x0d84,0x09},
{0x0d85,0x60},
{0x0d86,0x04},
{0x0d87,0xb1},
{0x0222,0x00},
{0x0223,0x01},
{0x0117,0x91},
{0x03f4,0x38},
{0x0e69,0x00},
{0x00d6,0x00},
{0x00d0,0x0d},
{0x00d1,0x13},
{0x00e0,0x18},
{0x00e1,0x18},
{0x00e2,0x18},
{0x00e3,0x18},
{0x00e4,0x18},
{0x00e5,0x18},
{0x00e6,0x18},
{0x00e7,0x18},
	{REG_NULL, 0x00},
};


static const struct gc2607_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x460,
		.hts_def = 2048,
		.vts_def = 1367,
		.reg_list = gc2607_1920x1080_regs_2lane,
		.hdr_mode = NO_HDR,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const s64 link_freq_menu_items[] = {
	MIPI_FREQ_297M
};
static int gc2607_write_reg(struct i2c_client *client, u16 reg,
			    u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
	{
		dev_err(&client->dev,"%s,(%d)\n",__func__,__LINE__);
		return -EIO;
	}
	return 0;
}



/* sensor register write */


static int gc2607_write_array(struct i2c_client *client,
				  const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = gc2607_write_reg(client, regs[i].addr,
				       GC2607_REG_VALUE_08BIT, regs[i].val);

	return ret;
}

/* sensor register read */
static int gc2607_read_reg(struct i2c_client *client, u16 reg,
		unsigned int len,	  u32 *val)
{
	struct i2c_msg msgs[2];

	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}






static int gc2607_get_reso_dist(const struct gc2607_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
		   abs(mode->height - framefmt->height);
}

static const struct gc2607_mode *
gc2607_find_best_fit(struct gc2607 *gc2607, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < gc2607->cfg_num; i++) {
		dist = gc2607_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist <= cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static uint8_t gain_reg_table[17][4] = {    //0x02b3 0x02b4 0x020c 0x020d
	                            {0x00,   0x00,  0x00,  0x40},
                                {0x05,   0x00,  0x00,  0x4b},
                                {0x00,   0x01,  0x00,  0x59},
                                {0x05,   0x01,  0x00,  0x6a},
                                {0x00,   0x02,  0x00,  0x80},
                                {0x05,   0x02,  0x00,  0x97},	
                                {0x00,   0x03,  0x00,  0xb3},
                                {0x05,   0x03,  0x00,  0xd4},
                                {0x00,   0x04,  0x01,  0x00},
                                {0x05,   0x04,  0x01,  0x2f},
                                {0x00,   0x05,  0x01,  0x66},				
                                {0x05,   0x05,  0x01,  0xa8},
                                {0x00,   0x06,  0x02,  0x00},
                                {0x05,   0x06,  0x02,  0x5e},
                                {0x09,   0x26,  0x02,  0xcc},
                                {0x0c,   0xb6,  0x03,  0x50},							
                                {0x10,   0x06,  0x04,  0x00},
   						};;

static  uint32_t gain_level_table[18] = {

									  64,
									  76,
									  93,
									  111,
									  130,
									  156,
									  184,
									  221,
									  253,
									  304,
									  367,
									  434,
									  510, 
									  607,
									  717,
									  847,
									  1012,
									  0xffffffff,
							   };
;

static int gc2607_set_gain(struct gc2607 *gc2607, u32 gain)
{
	int ret;
	uint8_t i = 0;
	uint8_t total = 0;
	uint32_t temp = 0;

	total = sizeof(gain_level_table) / sizeof(u32) - 1;
	for (i = 0; i < total; i++) {
		if ((gain_level_table[i] <= gain) && (gain < gain_level_table[i+1]))
			break;
	}


	temp = 1024 * gain / gain_level_table[i];

ret = gc2607_write_reg(gc2607->client, 0x02b3, GC2607_REG_VALUE_08BIT,gain_reg_table[i][0]);

ret = gc2607_write_reg(gc2607->client, 0x02b4,  GC2607_REG_VALUE_08BIT,gain_reg_table[i][1]);
ret = gc2607_write_reg(gc2607->client, 0x020c, GC2607_REG_VALUE_08BIT, gain_reg_table[i][2]);
ret = gc2607_write_reg(gc2607->client, 0x020d, GC2607_REG_VALUE_08BIT, gain_reg_table[i][3]);



	
ret = gc2607_write_reg(gc2607->client, 0x0208,  GC2607_REG_VALUE_08BIT,(temp>>8));
ret = gc2607_write_reg(gc2607->client, 0x0209, GC2607_REG_VALUE_08BIT, (temp));


	return ret;
}





static int gc2607_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gc2607 *gc2607 = container_of(ctrl->handler,
						 struct gc2607, ctrl_handler);
	struct i2c_client *client = gc2607->client;
	s64 max;
	int ret = 0;
	u32 vts = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = gc2607->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(gc2607->exposure,
					 gc2607->exposure->minimum, max,
					 gc2607->exposure->step,
					 gc2607->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = gc2607_write_reg(gc2607->client, GC2607_REG_EXP_H,GC2607_REG_VALUE_08BIT,
					   (ctrl->val >> 8) & 0x3f);
		ret |= gc2607_write_reg(gc2607->client, GC2607_REG_EXP_L,GC2607_REG_VALUE_08BIT,
					   ctrl->val & 0xff);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		gc2607_set_gain(gc2607, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		vts = ctrl->val + gc2607->cur_mode->height;
		ret = gc2607_write_reg(gc2607->client, GC2607_REG_VTS_H, GC2607_REG_VALUE_08BIT,(vts >> 8) & 0x3f);
		ret |= gc2607_write_reg(gc2607->client, GC2607_REG_VTS_L, GC2607_REG_VALUE_08BIT,vts & 0xff);
		break;
	case V4L2_CID_HFLIP:
		if (ctrl->val)
			gc2607->flip |= GC_MIRROR_BIT_MASK;
		else
			gc2607->flip &= ~GC_MIRROR_BIT_MASK;
		break;
	case V4L2_CID_VFLIP:
		if (ctrl->val)
			gc2607->flip |= GC_FLIP_BIT_MASK;
		else
			gc2607->flip &= ~GC_FLIP_BIT_MASK;
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);
	return ret;
}

static const struct v4l2_ctrl_ops gc2607_ctrl_ops = {
	.s_ctrl = gc2607_set_ctrl,
};

static int gc2607_configure_regulators(struct gc2607 *gc2607)
{
	unsigned int i;

	for (i = 0; i < GC2607_NUM_SUPPLIES; i++)
		gc2607->supplies[i].supply = gc2607_supply_names[i];

	return devm_regulator_bulk_get(&gc2607->client->dev,
					   GC2607_NUM_SUPPLIES,
					   gc2607->supplies);
}

static int gc2607_parse_of(struct gc2607 *gc2607)
{
	struct device *dev = &gc2607->client->dev;
	struct device_node *endpoint;
	struct fwnode_handle *fwnode;
	int rval;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}
	fwnode = of_fwnode_handle(endpoint);
	rval = fwnode_property_read_u32_array(fwnode, "data-lanes", NULL, 0);
	if (rval <= 0) {
		dev_warn(dev, " Get mipi lane num failed!\n");
		return -1;
	}

	gc2607->lane_num = rval;
	if (2 == gc2607->lane_num) {
		gc2607->cur_mode = &supported_modes[0];
		gc2607->cfg_num = ARRAY_SIZE(supported_modes);

		/*pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
		gc2607->pixel_rate = MIPI_FREQ_297M * 2U * (gc2607->lane_num) / 10U;
		dev_info(dev, "lane_num(%d)  pixel_rate(%u)\n",
				 gc2607->lane_num, gc2607->pixel_rate);
	} else {
		dev_info(dev, "gc2607 can not support the lane num(%d)\n", gc2607->lane_num);
	}
	return 0;
}

static int gc2607_initialize_controls(struct gc2607 *gc2607)
{
	const struct gc2607_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &gc2607->ctrl_handler;
	mode = gc2607->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;
	handler->lock = &gc2607->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
					  0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, gc2607->pixel_rate, 1, gc2607->pixel_rate);

	h_blank = mode->hts_def - mode->width;
	gc2607->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (gc2607->hblank)
		gc2607->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	gc2607->vblank = v4l2_ctrl_new_std(handler, &gc2607_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				GC2607_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 4;
	gc2607->exposure = v4l2_ctrl_new_std(handler, &gc2607_ctrl_ops,
				V4L2_CID_EXPOSURE, GC2607_EXPOSURE_MIN,
				exposure_max, GC2607_EXPOSURE_STEP,
				mode->exp_def);

	gc2607->anal_gain = v4l2_ctrl_new_std(handler, &gc2607_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, GC2607_GAIN_MIN,
				GC2607_GAIN_MAX, GC2607_GAIN_STEP,
				GC2607_GAIN_DEFAULT);

	gc2607->h_flip = v4l2_ctrl_new_std(handler, &gc2607_ctrl_ops,
				V4L2_CID_HFLIP, 0, 1, 1, 0);

	gc2607->v_flip = v4l2_ctrl_new_std(handler, &gc2607_ctrl_ops,
				V4L2_CID_VFLIP, 0, 1, 1, 0);
	gc2607->flip = 0;

	if (handler->error) {
		ret = handler->error;
		dev_err(&gc2607->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	gc2607->subdev.ctrl_handler = handler;
	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);
	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 gc2607_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, GC2607_XVCLK_FREQ / 1000 / 1000);
}

static int __gc2607_power_on(struct gc2607 *gc2607)
{
	int ret;
	u32 delay_us;
	struct device *dev = &gc2607->client->dev;

	if (!IS_ERR_OR_NULL(gc2607->pins_default)) {
		ret = pinctrl_select_state(gc2607->pinctrl,
					   gc2607->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

	ret = clk_set_rate(gc2607->xvclk, GC2607_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(gc2607->xvclk) != GC2607_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(gc2607->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	ret = regulator_bulk_enable(GC2607_NUM_SUPPLIES, gc2607->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}
	if (!IS_ERR(gc2607->power_gpio)) {
		gpiod_set_value_cansleep(gc2607->power_gpio, 1);
		usleep_range(100, 200);
	}
	if (!IS_ERR(gc2607->reset_gpio)) {
		gpiod_set_value_cansleep(gc2607->reset_gpio, 1);
		usleep_range(100, 200);
	}
	if (!IS_ERR(gc2607->pwdn_gpio))
		gpiod_set_value_cansleep(gc2607->pwdn_gpio, 0);

	if (!IS_ERR(gc2607->reset_gpio))
		gpiod_set_value_cansleep(gc2607->reset_gpio, 0);
	usleep_range(3000, 6000);
	/* 8192 cycles prior to first SCCB transaction */
	delay_us = gc2607_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);
	return 0;
#if 1
disable_clk:
	clk_disable_unprepare(gc2607->xvclk);
#endif
	return ret;
}

static void __gc2607_power_off(struct gc2607 *gc2607)
{
#if 1
	int ret;
	struct device *dev = &gc2607->client->dev;

	if (!IS_ERR(gc2607->pwdn_gpio))
		gpiod_set_value_cansleep(gc2607->pwdn_gpio, 1);
	clk_disable_unprepare(gc2607->xvclk);

	if (!IS_ERR(gc2607->reset_gpio))
		gpiod_set_value_cansleep(gc2607->reset_gpio, 1);

	if (!IS_ERR_OR_NULL(gc2607->pins_sleep)) {
		ret = pinctrl_select_state(gc2607->pinctrl,
					   gc2607->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	if (!IS_ERR(gc2607->power_gpio))
		gpiod_set_value_cansleep(gc2607->power_gpio, 0);
	regulator_bulk_disable(GC2607_NUM_SUPPLIES, gc2607->supplies);
#endif
}

static int gc2607_check_sensor_id(struct gc2607 *gc2607,
				   struct i2c_client *client)
{
	struct device *dev = &gc2607->client->dev;
	u32 pid = 0, ver = 0;
	u16 id = 0;
	int ret = 0;

	/* Check sensor revision */
	ret = gc2607_read_reg(client, GC2607_REG_CHIP_ID_H, GC2607_REG_VALUE_08BIT,&pid);
	ret |= gc2607_read_reg(client, GC2607_REG_CHIP_ID_L,GC2607_REG_VALUE_08BIT,&ver);
	if (ret) {
		dev_err(&client->dev, "gc2607_read_reg failed (%d)\n", ret);
		return ret;
	}

	id = SENSOR_ID(pid, ver);
	if (id != GC2607_CHIP_ID) {
		dev_err(&client->dev,
				"Sensor detection failed (%04X,%d)\n",
				id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected GC%04x sensor\n", id);
	return 0;
}

static int gc2607_set_flip(struct gc2607 *gc2607, u8 mode)
{
	u32 match_reg = 0;

	gc2607_read_reg(gc2607->client, GC2607_FLIP_MIRROR_REG,GC2607_REG_VALUE_08BIT, &match_reg);

	if (mode == GC_FLIP_BIT_MASK) {
		match_reg |= GC_FLIP_BIT_MASK;
		match_reg &= ~GC_MIRROR_BIT_MASK;
	} else if (mode == GC_MIRROR_BIT_MASK) {
		match_reg |= GC_MIRROR_BIT_MASK;
		match_reg &= ~GC_FLIP_BIT_MASK;
	} else if (mode == (GC_MIRROR_BIT_MASK |
		GC_FLIP_BIT_MASK)) {
		match_reg |= GC_FLIP_BIT_MASK;
		match_reg |= GC_MIRROR_BIT_MASK;
	} else {
		match_reg &= ~GC_FLIP_BIT_MASK;
		match_reg &= ~GC_MIRROR_BIT_MASK;
	}
	return gc2607_write_reg(gc2607->client, GC2607_FLIP_MIRROR_REG,GC2607_REG_VALUE_08BIT,match_reg);
}

static int __gc2607_start_stream(struct gc2607 *gc2607)
{
	int ret;

	ret = gc2607_write_array(gc2607->client, gc2607->cur_mode->reg_list);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&gc2607->mutex);
	ret = v4l2_ctrl_handler_setup(&gc2607->ctrl_handler);
	mutex_lock(&gc2607->mutex);

	ret = gc2607_set_flip(gc2607, gc2607->flip);
	if (ret)
		return ret;
	return gc2607_write_reg(gc2607->client, GC2607_REG_CTRL_MODE,GC2607_REG_VALUE_08BIT,
							GC2607_MODE_STREAMING);
}

static int __gc2607_stop_stream(struct gc2607 *gc2607)
{
	return gc2607_write_reg(gc2607->client, GC2607_REG_CTRL_MODE,GC2607_REG_VALUE_08BIT,
							GC2607_MODE_SW_STANDBY);
}

static void gc2607_get_module_inf(struct gc2607 *gc2607,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, GC2607_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, gc2607->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, gc2607->len_name, sizeof(inf->base.lens));
}

static void gc2607_set_awb_cfg(struct gc2607 *gc2607,
				   struct rkmodule_awb_cfg *cfg)
{
	mutex_lock(&gc2607->mutex);
	memcpy(&gc2607->awb_cfg, cfg, sizeof(*cfg));
	mutex_unlock(&gc2607->mutex);
}

static void gc2607_set_lsc_cfg(struct gc2607 *gc2607,
				   struct rkmodule_lsc_cfg *cfg)
{
	mutex_lock(&gc2607->mutex);
	memcpy(&gc2607->lsc_cfg, cfg, sizeof(*cfg));
	mutex_unlock(&gc2607->mutex);
}

static long gc2607_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct gc2607 *gc2607 = to_gc2607(sd);
	long ret = 0;
	struct rkmodule_hdr_cfg *hdr_cfg;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = gc2607->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
	case RKMODULE_SET_CONVERSION_GAIN:
		break;
	case RKMODULE_GET_MODULE_INFO:
		gc2607_get_module_inf(gc2607, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_AWB_CFG:
		gc2607_set_awb_cfg(gc2607, (struct rkmodule_awb_cfg *)arg);
		break;
	case RKMODULE_LSC_CFG:
		gc2607_set_lsc_cfg(gc2607, (struct rkmodule_lsc_cfg *)arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = gc2607_write_reg(gc2607->client, GC2607_REG_CTRL_MODE,GC2607_REG_VALUE_08BIT,
					       GC2607_MODE_STREAMING);
		else
			ret = gc2607_write_reg(gc2607->client, GC2607_REG_CTRL_MODE,GC2607_REG_VALUE_08BIT,
					       GC2607_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOTTY;
		break;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long gc2607_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *awb_cfg;
	struct rkmodule_lsc_cfg *lsc_cfg;
	struct rkmodule_hdr_cfg *hdr;
	long ret = 0;
	u32 cg = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = gc2607_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		awb_cfg = kzalloc(sizeof(*awb_cfg), GFP_KERNEL);
		if (!awb_cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(awb_cfg, up, sizeof(*awb_cfg));
		if (!ret)
			ret = gc2607_ioctl(sd, cmd, awb_cfg);
		kfree(awb_cfg);
		break;
	case RKMODULE_LSC_CFG:
		lsc_cfg = kzalloc(sizeof(*lsc_cfg), GFP_KERNEL);
		if (!lsc_cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(lsc_cfg, up, sizeof(*lsc_cfg));
		if (!ret)
			ret = gc2607_ioctl(sd, cmd, lsc_cfg);
		kfree(lsc_cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = gc2607_ioctl(sd, cmd, hdr);
		if (!ret)
			ret = copy_to_user(up, hdr, sizeof(*hdr));
		kfree(hdr);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdr, up, sizeof(*hdr));
		if (!ret)
			ret = gc2607_ioctl(sd, cmd, hdr);
		kfree(hdr);
		break;
	case RKMODULE_SET_CONVERSION_GAIN:
		ret = copy_from_user(&cg, up, sizeof(cg));
		if (!ret)
			ret = gc2607_ioctl(sd, cmd, &cg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		ret = copy_from_user(&stream, up, sizeof(u32));
		if (!ret)
			ret = gc2607_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOTTY;
		break;
	}
	return ret;
}
#endif

static int gc2607_s_stream(struct v4l2_subdev *sd, int on)
{
	struct gc2607 *gc2607 = to_gc2607(sd);
	struct i2c_client *client = gc2607->client;
	int ret = 0;

	mutex_lock(&gc2607->mutex);
	on = !!on;
	if (on == gc2607->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __gc2607_start_stream(gc2607);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__gc2607_stop_stream(gc2607);
		pm_runtime_put(&client->dev);
	}

	gc2607->streaming = on;

unlock_and_return:
	mutex_unlock(&gc2607->mutex);
	return 0;
}

static int gc2607_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct gc2607 *gc2607 = to_gc2607(sd);
	const struct gc2607_mode *mode = gc2607->cur_mode;

	mutex_lock(&gc2607->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&gc2607->mutex);

	return 0;
}

/*static int gc2607_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	struct gc2607 *gc2607 = to_gc2607(sd);
	const struct gc2607_mode *mode = gc2607->cur_mode;
	u32 val = 0;

	if (mode->hdr_mode == NO_HDR)
		val = 1 << (GC2607_LANES - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	config->type = V4L2_MBUS_CSI2;
	config->flags = val;
	return 0;
}*/
static int gc2607_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = GC2607_MEDIA_BUS_FMT;
	return 0;
}

static int gc2607_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct gc2607 *gc2607 = to_gc2607(sd);

	if (fse->index >= gc2607->cfg_num)
		return -EINVAL;

	if (fse->code != GC2607_MEDIA_BUS_FMT)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;
	return 0;
}

static int gc2607_enum_frame_interval(struct v4l2_subdev *sd,
						  struct v4l2_subdev_pad_config *cfg,
						  struct v4l2_subdev_frame_interval_enum *fie)
{
	struct gc2607 *gc2607 = to_gc2607(sd);

	if (fie->index >= gc2607->cfg_num)
		return -EINVAL;

	fie->code = GC2607_MEDIA_BUS_FMT;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static int gc2607_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc2607 *gc2607 = to_gc2607(sd);
	const struct gc2607_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&gc2607->mutex);

	mode = gc2607_find_best_fit(gc2607, fmt);
	fmt->format.code = GC2607_MEDIA_BUS_FMT;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&gc2607->mutex);
		return -ENOTTY;
#endif
	} else {
		gc2607->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(gc2607->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(gc2607->vblank, vblank_def,
					 GC2607_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&gc2607->mutex);
	return 0;
}

static int gc2607_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct gc2607 *gc2607 = to_gc2607(sd);
	const struct gc2607_mode *mode = gc2607->cur_mode;

	mutex_lock(&gc2607->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&gc2607->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = GC2607_MEDIA_BUS_FMT;
		fmt->format.field = V4L2_FIELD_NONE;

		/* format info: width/height/data type/virctual channel */
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];

	}
	mutex_unlock(&gc2607->mutex);
	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int gc2607_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gc2607 *gc2607 = to_gc2607(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct gc2607_mode *def_mode = &supported_modes[0];

	mutex_lock(&gc2607->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = GC2607_MEDIA_BUS_FMT;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&gc2607->mutex);
	/* No crop or compose */
	return 0;
}
#endif

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops gc2607_internal_ops = {
	.open = gc2607_open,
};
#endif

static int gc2607_s_power(struct v4l2_subdev *sd, int on)
{
	struct gc2607 *gc2607 = to_gc2607(sd);
	struct i2c_client *client = gc2607->client;
	int ret = 0;

	mutex_lock(&gc2607->mutex);

	/* If the power state is not modified - no work to do. */
	if (gc2607->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		gc2607->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		gc2607->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&gc2607->mutex);

	return ret;
}

static const struct v4l2_subdev_core_ops gc2607_core_ops = {
	.s_power = gc2607_s_power,
	.ioctl = gc2607_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = gc2607_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops gc2607_video_ops = {
	.s_stream = gc2607_s_stream,
	.g_frame_interval = gc2607_g_frame_interval,
	//.g_mbus_config = gc2607_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops gc2607_pad_ops = {
	.enum_mbus_code = gc2607_enum_mbus_code,
	.enum_frame_size = gc2607_enum_frame_sizes,
	.enum_frame_interval = gc2607_enum_frame_interval,
	.get_fmt = gc2607_get_fmt,
	.set_fmt = gc2607_set_fmt,
};

static const struct v4l2_subdev_ops gc2607_subdev_ops = {
	.core   = &gc2607_core_ops,
	.video  = &gc2607_video_ops,
	.pad    = &gc2607_pad_ops,
};

static int gc2607_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc2607 *gc2607 = to_gc2607(sd);

	__gc2607_power_on(gc2607);
	return 0;
}

static int gc2607_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc2607 *gc2607 = to_gc2607(sd);

	__gc2607_power_off(gc2607);
	return 0;
}

static const struct dev_pm_ops gc2607_pm_ops = {
	SET_RUNTIME_PM_OPS(gc2607_runtime_suspend,
					   gc2607_runtime_resume, NULL)
};

static int gc2607_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct gc2607 *gc2607;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	gc2607 = devm_kzalloc(dev, sizeof(*gc2607), GFP_KERNEL);
	if (!gc2607)
		return -ENOMEM;

	gc2607->client = client;
	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &gc2607->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
					   &gc2607->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
					   &gc2607->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
					   &gc2607->len_name);
	if (ret) {
		dev_err(dev,
			"could not get module information!\n");
		return -EINVAL;
	}

	gc2607->xvclk = devm_clk_get(&client->dev, "xvclk");
	if (IS_ERR(gc2607->xvclk)) {
		dev_err(&client->dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	gc2607->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gc2607->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	gc2607->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(gc2607->pwdn_gpio))
		dev_info(dev, "Failed to get pwdn-gpios, maybe no used\n");

	gc2607->power_gpio = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(gc2607->power_gpio))
		dev_warn(dev, "Failed to get power-gpios\n");

	ret = gc2607_configure_regulators(gc2607);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	ret = gc2607_parse_of(gc2607);
	if (ret != 0)
		return -EINVAL;

	gc2607->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(gc2607->pinctrl)) {
		gc2607->pins_default =
			pinctrl_lookup_state(gc2607->pinctrl,
						 OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(gc2607->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		gc2607->pins_sleep =
			pinctrl_lookup_state(gc2607->pinctrl,
						 OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(gc2607->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	mutex_init(&gc2607->mutex);

	sd = &gc2607->subdev;
	v4l2_i2c_subdev_init(sd, client, &gc2607_subdev_ops);
	ret = gc2607_initialize_controls(gc2607);
	if (ret)
		goto err_destroy_mutex;

	ret = __gc2607_power_on(gc2607);
	if (ret)
		goto err_free_handler;
	//while(1){
	ret = gc2607_check_sensor_id(gc2607, client);
	//}
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &gc2607_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	gc2607->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &gc2607->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(gc2607->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 gc2607->module_index, facing,
		 GC2607_NAME, dev_name(sd->dev));

	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif

err_power_off:
	__gc2607_power_off(gc2607);
err_free_handler:
	v4l2_ctrl_handler_free(&gc2607->ctrl_handler);

err_destroy_mutex:
	mutex_destroy(&gc2607->mutex);
	return ret;
}

static int gc2607_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gc2607 *gc2607 = to_gc2607(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&gc2607->ctrl_handler);
	mutex_destroy(&gc2607->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__gc2607_power_off(gc2607);
	pm_runtime_set_suspended(&client->dev);
	return 0;
}

static const struct i2c_device_id gc2607_match_id[] = {
	{ "gc2607", 0 },
	{ },
};

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id gc2607_of_match[] = {
	{ .compatible = "galaxycore,gc2607" },
	{},
};
MODULE_DEVICE_TABLE(of, gc2607_of_match);
#endif

static struct i2c_driver gc2607_i2c_driver = {
	.driver = {
		.name = GC2607_NAME,
		.pm = &gc2607_pm_ops,
		.of_match_table = of_match_ptr(gc2607_of_match),
	},
	.probe      = &gc2607_probe,
	.remove     = &gc2607_remove,
	.id_table   = gc2607_match_id,
};
//
static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&gc2607_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&gc2607_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("GC2035 CMOS Image Sensor driver");
MODULE_LICENSE("GPL v2");
