#define DEBUG 1
/*
 * Copyright (C) 2017 Vigilate
 * Author: Davide Ciminaghi <ciminaghi@gnudd.com>
 *
 * Driver for mtm9031 parallel sensor (mxc platform), comes from
 * ev76c570 driver in this directory (FIXME: DUPLICATED CODE).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"

/* Registers defines */
#define R0x3000 0x3000
#define R0x300e 0x300e
#define R0x31fc 0x31fc

struct mt9m031_priv {
	struct sensor_data sen;
	struct regmap *map8;
	struct regmap *map16;
	int reset_gpio;
	struct i2c_client *i2c;
	struct v4l2_int_device *vd;
	char vdname[20];
};

#define to_priv(a) container_of(a, struct mt9m031_priv, sen)

struct mt9m031_platform_data {
	int reset_gpio;
	enum of_gpio_flags reset_gpio_flags;
};

static inline int mt9m031_read_reg(struct mt9m031_priv *priv, int reg,
				   unsigned int *val)
{
	struct regmap *map = reg == R0x300e ? priv->map8 : priv->map16;
	return regmap_read(map, reg, val);
}

static inline int mt9m031_write_reg(struct mt9m031_priv *priv, int reg,
				     unsigned int val)
{
	struct regmap *map =  reg == R0x300e ? priv->map8 : priv->map16;
	return regmap_write(map, reg, val);
}

static bool mt9m031_is_volatile_reg(struct device *dev, unsigned int reg)
{
	return true;
}

static const struct regmap_config mt9m031_regmap_config8 = {
	.name = "mt9m031-config-8",
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = R0x300e,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_config mt9m031_regmap_config16 = {
	.name = "mt9m031-config-16",
	.reg_bits = 16,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.max_register = R0x31fc,
	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = mt9m031_is_volatile_reg,
};

static struct mt9m031_platform_data *setup_platdata(struct i2c_client *i2c)
{
	struct device_node *n = i2c->dev.of_node;
	struct mt9m031_platform_data *out;
	struct device *dev = &i2c->dev;

	if (!n) {
		dev_err(dev, "no device node\n");
		return ERR_PTR(-ENODEV);
	}
	out = devm_kzalloc(dev, sizeof(*out), GFP_KERNEL);
	if (!out) {
		dev_err(dev, "not enough memory for platform data\n");
		return ERR_PTR(-ENOMEM);
	}
	out->reset_gpio = of_get_named_gpio_flags(n,
						  "reset-gpio", 0,
						  &out->reset_gpio_flags);
	dev_dbg(&i2c->dev, "%s %d: gpio flags = %d\n", __func__, __LINE__,
		out->reset_gpio_flags);
	if (gpio_is_valid(out->reset_gpio)) {
		int v, ret = devm_gpio_request(dev, out->reset_gpio, "RST");
		if (ret) {
			dev_err(dev, "cannot get RST gpio\n");
			return ERR_PTR(ret);
		}
		/* gpio is an output, also keep it non-active */
		v = out->reset_gpio_flags & OF_GPIO_ACTIVE_LOW ?
			1 : 0;
		dev_dbg(&i2c->dev, "%s %d: setting rst gpio to %d\n", __func__, __LINE__, v);
		gpio_direction_output(out->reset_gpio, v);
	}
	return out;
}

static int mt9m031_reset(struct i2c_client *i2c)
{
	struct mt9m031_platform_data *plat = dev_get_platdata(&i2c->dev);
	int v;

	/* Assumes plat is a valid pointer */
	v = plat->reset_gpio_flags & OF_GPIO_ACTIVE_LOW ? 0 : 1;
	dev_dbg(&i2c->dev, "%s %d: setting rst gpio to %d\n", __func__, __LINE__, v);
	gpio_set_value(plat->reset_gpio, v);
	v = v ? 0 : 1;
	mdelay(10);
	dev_dbg(&i2c->dev, "%s %d: setting rst gpio to %d\n", __func__, __LINE__, v);
	gpio_set_value(plat->reset_gpio, v);
	return 0;
}

/*
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
struct mt9m031_reg {
	int index;
	uint16_t val;
};

static const struct mt9m031_reg regs_init_tab[] = {
	{
		.index = -1,
	},
};

#define INPUT_CLK 33  
#define MT9M031_PLL_M  142
#define MT9M031_PLL_N  8
#define MT9M031_PLL_P1 2 
#define MT9M031_PLL_P2 4
#define MT9M031_OUT_CLK ((INPUT_CLK * MT9M031_PLL_M) / (MT9M031_PLL_N * MT9M031_PLL_P1 * MT9M031_PLL_P2))

//#define INPUT_CLK 24  
//#define PLL_M  99				32<= M  <=384
//#define PLL_N  4				1< = N  <=64		
//#define PLL_P1 1				1< = P1 <=16
//#define PLL_P2 8				4< = P2 <=16
//#define OUT_CLK ((INPUT_CLK 24 * PLL_M 99) / (PLL_N 4 * PLL_P1 1 * PLL_P2 8)) ---> 74.25Mhz

const unsigned short MT9M031_Parallel[]=
{
// embedia reset out of parallel 
0x301A, 0x00D9, 	// RESET_REGISTER
0x3088, 0x8000, 	// SEQ_CTRL_PORT
0x3086, 0x3227, 	// SEQ_DATA_PORT
0x3086, 0x0101, 	// SEQ_DATA_PORT
0x3086, 0x0F25, 	// SEQ_DATA_PORT
0x3086, 0x0808, 	// SEQ_DATA_PORT
0x3086, 0x0227, 	// SEQ_DATA_PORT
0x3086, 0x0101, 	// SEQ_DATA_PORT
0x3086, 0x0837, 	// SEQ_DATA_PORT
0x3086, 0x2700, 	// SEQ_DATA_PORT
0x3086, 0x0138, 	// SEQ_DATA_PORT
0x3086, 0x2701, 	// SEQ_DATA_PORT
0x3086, 0x013A, 	// SEQ_DATA_PORT
0x3086, 0x2700, 	// SEQ_DATA_PORT
0x3086, 0x0125, 	// SEQ_DATA_PORT
0x3086, 0x0020, 	// SEQ_DATA_PORT
0x3086, 0x3C25, 	// SEQ_DATA_PORT
0x3086, 0x0040, 	// SEQ_DATA_PORT
0x3086, 0x3427, 	// SEQ_DATA_PORT
0x3086, 0x003F, 	// SEQ_DATA_PORT
0x3086, 0x2500, 	// SEQ_DATA_PORT
0x3086, 0x2037, 	// SEQ_DATA_PORT
0x3086, 0x2540, 	// SEQ_DATA_PORT
0x3086, 0x4036, 	// SEQ_DATA_PORT
0x3086, 0x2500, 	// SEQ_DATA_PORT
0x3086, 0x4031, 	// SEQ_DATA_PORT
0x3086, 0x2540, 	// SEQ_DATA_PORT
0x3086, 0x403D, 	// SEQ_DATA_PORT
0x3086, 0x6425, 	// SEQ_DATA_PORT
0x3086, 0x2020, 	// SEQ_DATA_PORT
0x3086, 0x3D64, 	// SEQ_DATA_PORT
0x3086, 0x2510, 	// SEQ_DATA_PORT
0x3086, 0x1037, 	// SEQ_DATA_PORT
0x3086, 0x2520, 	// SEQ_DATA_PORT
0x3086, 0x2010, 	// SEQ_DATA_PORT
0x3086, 0x2510, 	// SEQ_DATA_PORT
0x3086, 0x100F, 	// SEQ_DATA_PORT
0x3086, 0x2708, 	// SEQ_DATA_PORT
0x3086, 0x0802, 	// SEQ_DATA_PORT
0x3086, 0x2540, 	// SEQ_DATA_PORT
0x3086, 0x402D, 	// SEQ_DATA_PORT
0x3086, 0x2608, 	// SEQ_DATA_PORT
0x3086, 0x280D, 	// SEQ_DATA_PORT
0x3086, 0x1709, 	// SEQ_DATA_PORT
0x3086, 0x2600, 	// SEQ_DATA_PORT
0x3086, 0x2805, 	// SEQ_DATA_PORT
0x3086, 0x26A7, 	// SEQ_DATA_PORT
0x3086, 0x2807, 	// SEQ_DATA_PORT
0x3086, 0x2580, 	// SEQ_DATA_PORT
0x3086, 0x8029, 	// SEQ_DATA_PORT
0x3086, 0x1705, 	// SEQ_DATA_PORT
0x3086, 0x2500, 	// SEQ_DATA_PORT
0x3086, 0x4027, 	// SEQ_DATA_PORT
0x3086, 0x2222, 	// SEQ_DATA_PORT
0x3086, 0x1616, 	// SEQ_DATA_PORT
0x3086, 0x2726, 	// SEQ_DATA_PORT
0x3086, 0x2617, 	// SEQ_DATA_PORT
0x3086, 0x3626, 	// SEQ_DATA_PORT
0x3086, 0xA617, 	// SEQ_DATA_PORT
0x3086, 0x0326, 	// SEQ_DATA_PORT
0x3086, 0xA417, 	// SEQ_DATA_PORT
0x3086, 0x1F28, 	// SEQ_DATA_PORT
0x3086, 0x0526, 	// SEQ_DATA_PORT
0x3086, 0x2028, 	// SEQ_DATA_PORT
0x3086, 0x0425, 	// SEQ_DATA_PORT
0x3086, 0x2020, 	// SEQ_DATA_PORT
0x3086, 0x2700, 	// SEQ_DATA_PORT
0x3086, 0x2625, 	// SEQ_DATA_PORT
0x3086, 0x0000, 	// SEQ_DATA_PORT
0x3086, 0x171E, 	// SEQ_DATA_PORT
0x3086, 0x2500, 	// SEQ_DATA_PORT
0x3086, 0x0425, 	// SEQ_DATA_PORT
0x3086, 0x0020, 	// SEQ_DATA_PORT
0x3086, 0x2117, 	// SEQ_DATA_PORT
0x3086, 0x121B, 	// SEQ_DATA_PORT
0x3086, 0x1703, 	// SEQ_DATA_PORT
0x3086, 0x2726, 	// SEQ_DATA_PORT
0x3086, 0x2617, 	// SEQ_DATA_PORT
0x3086, 0x2828, 	// SEQ_DATA_PORT
0x3086, 0x0517, 	// SEQ_DATA_PORT
0x3086, 0x1A26, 	// SEQ_DATA_PORT
0x3086, 0x6017, 	// SEQ_DATA_PORT
0x3086, 0xAE25, 	// SEQ_DATA_PORT
0x3086, 0x0080, 	// SEQ_DATA_PORT
0x3086, 0x2700, 	// SEQ_DATA_PORT
0x3086, 0x2626, 	// SEQ_DATA_PORT
0x3086, 0x1828, 	// SEQ_DATA_PORT
0x3086, 0x002E, 	// SEQ_DATA_PORT
0x3086, 0x2A28, 	// SEQ_DATA_PORT
0x3086, 0x081E, 	// SEQ_DATA_PORT
0x3086, 0x4127, 	// SEQ_DATA_PORT
0x3086, 0x1010, 	// SEQ_DATA_PORT
0x3086, 0x0214, 	// SEQ_DATA_PORT
0x3086, 0x6060, 	// SEQ_DATA_PORT
0x3086, 0x0A14, 	// SEQ_DATA_PORT
0x3086, 0x6060, 	// SEQ_DATA_PORT
0x3086, 0x0B14, 	// SEQ_DATA_PORT
0x3086, 0x6060, 	// SEQ_DATA_PORT
0x3086, 0x0C14, 	// SEQ_DATA_PORT
0x3086, 0x6060, 	// SEQ_DATA_PORT
0x3086, 0x0D14, 	// SEQ_DATA_PORT
0x3086, 0x6060, 	// SEQ_DATA_PORT
0x3086, 0x0217, 	// SEQ_DATA_PORT
0x3086, 0x3C14, 	// SEQ_DATA_PORT
0x3086, 0x0060, 	// SEQ_DATA_PORT
0x3086, 0x0A14, 	// SEQ_DATA_PORT
0x3086, 0x0060, 	// SEQ_DATA_PORT
0x3086, 0x0B14, 	// SEQ_DATA_PORT
0x3086, 0x0060, 	// SEQ_DATA_PORT
0x3086, 0x0C14, 	// SEQ_DATA_PORT
0x3086, 0x0060, 	// SEQ_DATA_PORT
0x3086, 0x0D14, 	// SEQ_DATA_PORT
0x3086, 0x0060, 	// SEQ_DATA_PORT
0x3086, 0x0811, 	// SEQ_DATA_PORT
0x3086, 0x2500, 	// SEQ_DATA_PORT
0x3086, 0x1027, 	// SEQ_DATA_PORT
0x3086, 0x0010, 	// SEQ_DATA_PORT
0x3086, 0x2F6F, 	// SEQ_DATA_PORT
0x3086, 0x0F3E, 	// SEQ_DATA_PORT
0x3086, 0x2500, 	// SEQ_DATA_PORT
0x3086, 0x0827, 	// SEQ_DATA_PORT
0x3086, 0x0008, 	// SEQ_DATA_PORT
0x3086, 0x3066, 	// SEQ_DATA_PORT
0x3086, 0x3225, 	// SEQ_DATA_PORT
0x3086, 0x0008, 	// SEQ_DATA_PORT
0x3086, 0x2700, 	// SEQ_DATA_PORT
0x3086, 0x0830, 	// SEQ_DATA_PORT
0x3086, 0x6631, 	// SEQ_DATA_PORT
0x3086, 0x3D64, 	// SEQ_DATA_PORT
0x3086, 0x2508, 	// SEQ_DATA_PORT
0x3086, 0x083D, 	// SEQ_DATA_PORT
0x3086, 0xFF3D, 	// SEQ_DATA_PORT
0x3086, 0x2A27, 	// SEQ_DATA_PORT
0x3086, 0x083F, 	// SEQ_DATA_PORT
0x3086, 0x2C00, 	// SEQ_DATA_PORT
0x301A, 0x00D8, 	// RESET_REGISTER

0x30D4, 0x0007, 	// COLUMN_CORRECTION
0x301A, 0x00DC, 	// RESET_REGISTER
0x301A, 0x0000, 	// RESET_REGISTER
0x30D4, 0x8000, 	// COLUMN_CORRECTION
0x301A, 0x0004, 	// RESET_REGISTER
0x307A, 0x0000, 	// TEST_RAW_MODE
0x30EA, 0x0C00, 	// RESERVED_MFR_30EA
0x3044, 0x0404, 	// DARK_CONTROL
0x301E, 0x012C, 	// DATA_PEDESTAL
0x3180, 0x8000, 	// RESERVED_MFR_3180
0x30D4, 0xE007, 	// COLUMN_CORRECTION
0x3014, 0x0380, 	// FINE_INTEGRATION_TIME
0x3ED6, 0x00FD, 	// RESERVED_MFR_3ED6
0x3ED8, 0x0FFF, 	// RESERVED_MFR_3ED8
0x3EDA, 0x0003, 	// RESERVED_MFR_3EDA
0x3EDC, 0xF87A, 	// RESERVED_MFR_3EDC
0x3EDE, 0xE075, 	// RESERVED_MFR_3EDE
0x3EE0, 0x077C, 	// RESERVED_MFR_3EE0
0x3EE2, 0xA4EB, 	// RESERVED_MFR_3EE2
0x3EE4, 0xD208, 	// RESERVED_MFR_3EE4
/*
0x302C, 0x0001, 	// VT_SYS_CLK_DIV
0x302A, 0x0008, 	// VT_PIX_CLK_DIV
0x302E, 0x0002, 	// PRE_PLL_CLK_DIV
0x3030, 0x002C, 	// PLL_MULTIPLIER
*/
0x302C, MT9M031_PLL_P1, // VT_SYS_CLK_DIV
0x302A, MT9M031_PLL_P2, // VT_PIX_CLK_DIV
0x302E, MT9M031_PLL_N, 	// PRE_PLL_CLK_DIV
0x3030, MT9M031_PLL_M, 	// PLL_MULTIPLIER

0x30B0, 0x0000, 	// DIGITAL_TEST

0x3070, 0x0000,		// TEST PATTERN

0x3100, 0x0000,		// ANALOG GAIN AND AEG

// active pixel w= 1280 h=960 
// streched syncs w=1920 h=1080
0x3002, 0x0004, 	// Y_ADDR_START HD (4)     						
0x3004, 0x0002, 	// X_ADDR_START_HD (2)
0x3006, 0x03c3, 	// Y_ADDR_END HD  (963)
0x3008, 0x0501, 	// X_ADDR_END_HD (1281)
0x300A, 0x03de, 	// FRAME_LENGTH_LINES
0x300C, 0x0672, 	// LINE_LENGTH_PCK 
0x3012, 0x0ff, 		// COARSE_INTEGRATION_TIME_HD

0x30A2, 0x0001, 	// X_ODD_INC
0x30A6, 0x0001, 	// Y_ODD_INC
0x3040, 0xc000, 	// READ_MODE
//0x3032, 0x0020, 	// DIGITAL_BINNING
0x3032, 0x0000, 	// DIGITAL_BINNING
0x3028, 0x0010, 	// ROW_SPEED
0x301A, 0x10DC 		// RESET_REGISTER
};

static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct mt9m031_priv *data = to_priv(s->priv);
	/*const struct mt9m031_reg *ptr;

	dev_dbg(&data->i2c->dev, "%s", __func__);
	for (ptr = regs_init_tab; ptr->index != -1; ptr++) {
		unsigned int v;

		mt9m031_write_reg(data, ptr->index, ptr->val);
		dev_dbg(&data->i2c->dev, "%s: writing 0x%04x to 0x%04x\n", __func__, ptr->val, ptr->index);
		mt9m031_read_reg(data, ptr->index, &v);
		dev_dbg(&data->i2c->dev, "%s: read 0x%04x from 0x%04x\n", __func__, v, ptr->index);
	}*/

    	int status = 0;
 	int j;

	for(j=0; j < sizeof(MT9M031_Parallel)/sizeof(unsigned short); j=j+2)
	{
		unsigned int v;

		mt9m031_write_reg(data, MT9M031_Parallel[j], MT9M031_Parallel[j+1]);
		dev_dbg(&data->i2c->dev, "%s: writing 0x%04x to 0x%04x\n", __func__, MT9M031_Parallel[j+1], MT9M031_Parallel[j]);
		mt9m031_read_reg(data, MT9M031_Parallel[j], &v);
		dev_dbg(&data->i2c->dev, "%s: read 0x%04x from 0x%04x\n", __func__, v, MT9M031_Parallel[j]);
	
	   	/*status = mt9m031_write(sd, MT9M031_Parallel[j], MT9M031_Parallel[j+1]);
	   	if(status != 0)
		{
		  v4l2_err(sd,"I2C write Error,index:%d\n",j);
		   return status;
		}*/
		if(MT9M031_Parallel[j] == 0x301A )	
		   msleep_interruptible(200);
		else
		   msleep_interruptible(1);	
	}

	return 0;
}

/*
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	struct mt9m031_priv *data = to_priv(s->priv);

	dev_dbg(&data->i2c->dev, "%s", __func__);
	return 0;
}

/*
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct mt9m031_priv *data = to_priv(s->priv);

	dev_dbg(&data->i2c->dev, "%s (%d)", __func__, on);
	return 0;
}

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct mt9m031_priv *data;
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	data = to_priv(s->priv);
	memset(p, 0, sizeof(*p));
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	/*
	 * HACK: force csi_param.clk_mode = IPU_CSI_CLK_MODE_GATED_CLK in
	 * mxc_v4l2_s_param
	 */
	p->u.bt656.clock_curr = -1;
	/* everything else is 0. Is this OK ? */
	return 0;
}

/*
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	struct mt9m031_priv *data = to_priv(s->priv);

	dev_dbg(&data->i2c->dev, "%s\n", __func__);
	return 0;
}

/*
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	if (fmt->index > 0)
		return -EINVAL;
	/* Monochrome only supported at the moment */
	fmt->pixelformat = V4L2_PIX_FMT_GREY;
	return 0;
}

/*
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensdata = s->priv;
	struct mt9m031_priv *data = to_priv(s->priv);

	dev_dbg(&data->i2c->dev, "%s (format type = %d)\n", __func__, f->type);
	f->fmt.pix.width = sensdata->pix.width;
	f->fmt.pix.height = sensdata->pix.height;
	f->fmt.pix.pixelformat = sensdata->pix.pixelformat;
	return 0;
}

/*
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct mt9m031_priv *data = to_priv(s->priv);
	struct v4l2_captureparm *cparm = &a->parm.capture;

	dev_dbg(&data->i2c->dev, "%s", __func__);
	switch (a->type) {
		/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		dev_dbg(&data->i2c->dev,
			"%s, type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n", __func__);
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = V4L2_MODE_HIGHQUALITY |
			V4L2_CAP_TIMEPERFRAME;
		cparm->timeperframe.numerator = 50;
		cparm->timeperframe.denominator = 1;
		/* FIXME ?? */
		cparm->capturemode = 0;
		break;

	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("ioctl_g_parm:type is unknown %d\n", a->type);
		break;
	}
	return 0;
}

/*
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct mt9m031_priv *data = to_priv(s->priv);

	dev_dbg(&data->i2c->dev, "%s\n", __func__);
	/* Currently unsupported */
	return 0;
}

/*
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	/* Currently unsupported */
	return -EPERM;
}

/*
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	return -EPERM;
}

/*
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	return -EINVAL;
}

/*
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	/* Currently unsupported */
	return -EINVAL;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
	       "mt9m031_sensor");

	return 0;
}

/*
 * v4l2 ioctls
 */
static struct v4l2_int_ioctl_desc mt9m031_ioctl_desc[] = {
	{ vidioc_int_dev_init_num,
	  (v4l2_int_ioctl_func *)ioctl_dev_init },
	{ vidioc_int_dev_exit_num,
	  ioctl_dev_exit},
	{ vidioc_int_s_power_num,
	  (v4l2_int_ioctl_func *)ioctl_s_power },
	{ vidioc_int_g_ifparm_num,
	  (v4l2_int_ioctl_func *)ioctl_g_ifparm },
	{ vidioc_int_init_num,
	  (v4l2_int_ioctl_func *)ioctl_init },
	{ vidioc_int_enum_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap },
	{ vidioc_int_g_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_g_fmt_cap },
	{ vidioc_int_g_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_g_parm },
	{ vidioc_int_s_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_s_parm },
	{ vidioc_int_g_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_g_ctrl },
	{ vidioc_int_s_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_s_ctrl },
	{ vidioc_int_enum_framesizes_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_framesizes },
	{ vidioc_int_enum_frameintervals_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_frameintervals },
	{ vidioc_int_g_chip_ident_num,
	  (v4l2_int_ioctl_func *)ioctl_g_chip_ident },
};

static int mt9m031_v4l2_write_reg(struct v4l2_int_device *s, u16 reg, u16 val)
{
	struct mt9m031_priv *data = to_priv(s->priv);
	unsigned r = reg, v = val;

	pr_debug("%s reg = %u, val = %u\n", __func__, r, v);

	return mt9m031_write_reg(data, r, v);
}

static int mt9m031_v4l2_read_reg(struct v4l2_int_device *s, u16 reg, u16 *val)
{
	struct mt9m031_priv *data = to_priv(s->priv);
	unsigned r = reg, v;
	int ret;

	ret = mt9m031_read_reg(data, r, &v);
	if (ret < 0)
		return ret;
	*val = v;
	return ret;
}

static const struct v4l2_int_slave_ops slave_ops = {
	.write_reg = mt9m031_v4l2_write_reg,
	.read_reg = mt9m031_v4l2_read_reg
};

static struct v4l2_int_slave mt9m031_slave = {
	.ioctls = mt9m031_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(mt9m031_ioctl_desc),
	.ops = &slave_ops,
};

static const struct v4l2_int_device mt9m031_int_device = {
	.module = THIS_MODULE,
	.name = "mt9m031",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &mt9m031_slave,
	},
};

static int mt9m031_remove(struct i2c_client *i2c)
{
	struct mt9m031_priv *data = dev_get_drvdata(&i2c->dev);

	v4l2_int_device_unregister(data->vd);
	dev_info(&data->i2c->dev, "v4l2 device registered\n");
	return 0;
}

static int mt9m031_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct mt9m031_priv *data = devm_kzalloc(&i2c->dev, sizeof(*data),
						 GFP_KERNEL);
	struct mt9m031_platform_data *plat = dev_get_platdata(&i2c->dev);
	struct v4l2_int_device *vd;
	struct v4l2_int_slave *slave;
	unsigned int chip_id;
	static int instance_id = 0;
	int ret;

	if (!data) {
		dev_err(&i2c->dev, "cannot allocate private data\n");
		return -ENOMEM;
	}
	if (!plat) {
		plat = setup_platdata(i2c);
		if (IS_ERR(plat)) {
			dev_err(&i2c->dev, "no platform data\n");
			return PTR_ERR(plat);
		}
		i2c->dev.platform_data = plat;
	}
	if (gpio_is_valid(plat->reset_gpio)) {
		ret = mt9m031_reset(i2c);
		msleep_interruptible(7);
		if (ret < 0)return ret;
	}
	data->sen.streamcap.timeperframe.denominator = 50;
	data->sen.streamcap.timeperframe.numerator = 1;
	data->sen.pix.width = 1280;
	data->sen.pix.height = 960;
	data->sen.pix.pixelformat = V4L2_PIX_FMT_GREY;
	ret = of_property_read_u32(i2c->dev.of_node, "ipu_id", &data->sen.ipu_id);
	if (ret) {
		dev_err(&i2c->dev, "ipu_id missing or invalid\n");
		return ret;
	}

	ret = of_property_read_u32(i2c->dev.of_node, "csi_id", &data->sen.csi);
	if (ret) {
		dev_err(&i2c->dev, "csi_id missing or invalid\n");
		return ret;
	}
	snprintf(data->vdname, sizeof(data->vdname) - 1, "mt9m031.%d\n",
		 instance_id++);
	data->i2c = i2c;
	data->map8 = devm_regmap_init_i2c(i2c, &mt9m031_regmap_config8);
	if (IS_ERR(data->map8))
		return PTR_ERR(data->map8);
	data->map16 = devm_regmap_init_i2c(i2c, &mt9m031_regmap_config16);
	if (IS_ERR(data->map16))
		return PTR_ERR(data->map16);
	ret = mt9m031_read_reg(data, R0x3000, &chip_id);
	if (ret < 0) {
		dev_err(&i2c->dev, "error reading chip id\n");
		return ret;
	}
	dev_info(&i2c->dev, "detected chip id 0x%04x\n", chip_id);
	vd = devm_kzalloc(&i2c->dev, sizeof(*vd), GFP_KERNEL);
	if (!vd) {
		dev_err(&i2c->dev, "not enough memory for v4l2_int_dev\n");
		return -ENOMEM;
	}
	slave = devm_kzalloc(&i2c->dev, sizeof(*slave), GFP_KERNEL);
	if (!slave) {
		dev_err(&i2c->dev, "not enough memory for v4l2_int slave data\n");
		return -ENOMEM;
	}
	*vd = mt9m031_int_device;
	strncpy(vd->name, data->vdname, sizeof(vd->name));
	*slave = mt9m031_slave;
	vd->u.slave = slave;
	vd->priv = &data->sen;
	data->vd = vd;
	ret = v4l2_int_device_register(vd);
	if (ret < 0) {
		dev_err(&i2c->dev, "error registering v4l2 device\n");
		return ret;
	}
	dev_info(&i2c->dev, "v4l2 device registered\n");
	dev_set_drvdata(&i2c->dev, data);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mt9m031_dt_ids[] = {
	{ .compatible = "aptina,mt9m031" },
	{},
};

MODULE_DEVICE_TABLE(of, mt9m031_dt_ids);
#endif

static const struct i2c_device_id mt9m031_id[] = {
	{ "mt9m031", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mma8452_id);


static struct i2c_driver mt9m031_driver = {
	.driver = {
		.name	= "mt9m031",
		.of_match_table = of_match_ptr(mt9m031_dt_ids),
		.owner	= THIS_MODULE,
	},
	.probe		= mt9m031_probe,
	.remove		= mt9m031_remove,
	.id_table	= mt9m031_id,
};

module_i2c_driver(mt9m031_driver);

MODULE_AUTHOR("Davide Ciminaghi <ciminaghi@gnudd.com>");
MODULE_DESCRIPTION("mt9m031 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:mt9m031");
