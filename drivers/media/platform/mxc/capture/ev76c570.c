#define DEBUG 1
/*
 * Copyright (C) 2017 Vigilate
 * Author: Davide Ciminaghi <ciminaghi@gnudd.com>
 *
 * Driver for ev76c570 parallel sensor (mxc platform). Some code comes from
 * the ov5640 driver in this directory.
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
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"

/* Registers defines */
#define EV76C570_REG0 0
#define EV76C570_REG_SOFT_RESET 1
#define EV76C570_CALIB_MBX 2
#define EV76C570_ABORT_MBX 3
#define EV76C570_REG_LINE_CFG 4
#define EV76C570_CHIP_ID 0x7f

struct ev76c570_priv {
	struct sensor_data sen;
	struct regmap *map8;
	struct regmap *map16;
	int reset_gpio;
	struct spi_device *spi;
	struct v4l2_int_device *vd;
};

#define to_priv(a) container_of(a, struct ev76c570_priv, sen)

struct ev76c570_platform_data {
	int reset_gpio;
	enum of_gpio_flags reset_gpio_flags;
};

static inline int ev76c570_read_reg(struct ev76c570_priv *priv, int reg,
				    unsigned int *val)
{
	struct regmap *map = reg <= EV76C570_ABORT_MBX ? priv->map8 :
		priv->map16;
	return regmap_read(map, reg, val);
}

static inline int ev76c570_write_reg(struct ev76c570_priv *priv, int reg,
				     unsigned int val)
{
	struct regmap *map = reg <= EV76C570_ABORT_MBX ? priv->map8 :
		priv->map16;
	return regmap_write(map, reg, val);
}

static bool ev76c570_is_volatile_reg(struct device *dev, unsigned int reg)
{
	return true;
}

static const struct regmap_config ev76c570_regmap_config8 = {
	.reg_bits = 8,
	.val_bits = 8,
	.val_format_endian = REGMAP_ENDIAN_BIG,

	.max_register = EV76C570_ABORT_MBX,
	.cache_type = REGCACHE_NONE,

	.volatile_reg = ev76c570_is_volatile_reg,
};

static const struct regmap_config ev76c570_regmap_config16 = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_BIG,

	.max_register = EV76C570_CHIP_ID,
	.cache_type = REGCACHE_RBTREE,

	.volatile_reg = ev76c570_is_volatile_reg,
};

static struct ev76c570_platform_data *setup_platdata(struct spi_device *spi)
{
	struct device_node *n = spi->dev.of_node;
	struct ev76c570_platform_data *out;
	struct device *dev = &spi->dev;

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
	dev_dbg(&spi->dev, "%s %d: gpio flags = %d\n", __func__, __LINE__,
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
		dev_dbg(&spi->dev, "%s %d: setting rst gpio to %d\n", __func__, __LINE__, v);
		gpio_direction_output(out->reset_gpio, v);
	}
	return out;
}

static int ev76c570_reset(struct spi_device *spi)
{
	struct ev76c570_platform_data *plat = dev_get_platdata(&spi->dev);
	int v;

	/* Assumes plat is a valid pointer */
	v = plat->reset_gpio_flags & OF_GPIO_ACTIVE_LOW ? 0 : 1;
	dev_dbg(&spi->dev, "%s %d: setting rst gpio to %d\n", __func__, __LINE__, v);
	gpio_set_value(plat->reset_gpio, v);
	v = v ? 0 : 1;
	mdelay(5);
	dev_dbg(&spi->dev, "%s %d: setting rst gpio to %d\n", __func__, __LINE__, v);
	gpio_set_value(plat->reset_gpio, v);
	return 0;
}

/*
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct ev76c570_priv *data = to_priv(s->priv);

	dev_dbg(&data->spi->dev, "%s", __func__);
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
	struct ev76c570_priv *data = to_priv(s->priv);

	dev_dbg(&data->spi->dev, "%s", __func__);
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
	struct ev76c570_priv *data = to_priv(s->priv);

	dev_dbg(&data->spi->dev, "%s (%d)", __func__, on);
	return 0;
}

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct ev76c570_priv *data;
	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	data = to_priv(s->priv);
	memset(p, 0, sizeof(*p));
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_10BIT;
	/* everything else is 0. Is this OK ? */
	return 0;
}

/*
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	struct ev76c570_priv *data = to_priv(s->priv);

	dev_dbg(&data->spi->dev, "%s\n", __func__);
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
	struct ev76c570_priv *data = to_priv(s->priv);

	dev_dbg(&data->spi->dev, "%s (format type = %d)\n", __func__, f->type);
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
	struct ev76c570_priv *data = to_priv(s->priv);
	struct v4l2_captureparm *cparm = &a->parm.capture;

	dev_dbg(&data->spi->dev, "%s", __func__);
	switch (a->type) {
		/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		dev_dbg(&data->spi->dev,
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
	struct ev76c570_priv *data = to_priv(s->priv);

	dev_dbg(&data->spi->dev, "%s\n", __func__);
	/* Currently unsupported */
	return -EINVAL;
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
	       "ev76c570_sensor");

	return 0;
}

/*
 * v4l2 ioctls
 */
static struct v4l2_int_ioctl_desc ev76c570_ioctl_desc[] = {
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

static struct v4l2_int_slave ev76c570_slave = {
	.ioctls = ev76c570_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(ev76c570_ioctl_desc),
};

static const struct v4l2_int_device ev76c570_int_device = {
	.module = THIS_MODULE,
	.name = "ev76c570",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &ev76c570_slave,
	},
};

static int ev76c570_remove(struct spi_device *spi)
{
	struct ev76c570_priv *data = dev_get_drvdata(&spi->dev);

	v4l2_int_device_unregister(data->vd);
	dev_info(&data->spi->dev, "v4l2 device registered\n");
	return 0;
}

static int ev76c570_probe(struct spi_device *spi)
{
	struct ev76c570_priv *data = devm_kzalloc(&spi->dev, sizeof(*data),
						  GFP_KERNEL);
	struct ev76c570_platform_data *plat = dev_get_platdata(&spi->dev);
	struct v4l2_int_device *vd;
	unsigned int chip_id;
	int ret;

	if (!data) {
		dev_err(&spi->dev, "cannot allocate private data\n");
		return -ENOMEM;
	}
	if (!plat) {
		plat = setup_platdata(spi);
		if (IS_ERR(plat)) {
			dev_err(&spi->dev, "no platform data\n");
			return PTR_ERR(plat);
		}
		spi->dev.platform_data = plat;
	}
	ret = ev76c570_reset(spi);
	if (ret < 0)
		return ret;
	data->sen.streamcap.timeperframe.denominator = 50;
	data->sen.streamcap.timeperframe.numerator = 1;
	data->sen.pix.width = 1600;
	data->sen.pix.height = 1200;
	data->sen.pix.pixelformat = V4L2_PIX_FMT_GREY;
	data->spi = spi;
	data->map8 = devm_regmap_init_spi(spi, &ev76c570_regmap_config8);
	if (IS_ERR(data->map8))
		return PTR_ERR(data->map8);
	data->map16 = devm_regmap_init_spi(spi, &ev76c570_regmap_config16);
	if (IS_ERR(data->map16))
		return PTR_ERR(data->map16);
	ret = ev76c570_read_reg(data, EV76C570_CHIP_ID, &chip_id);
	if (ret < 0) {
		dev_err(&spi->dev, "error reading chip id\n");
		return ret;
	}
	dev_info(&spi->dev, "detected chip id 0x%04x\n", chip_id);
	vd = devm_kzalloc(&spi->dev, sizeof(*vd), GFP_KERNEL);
	if (!vd) {
		dev_err(&spi->dev, "not enough memory for v4l2_int_dev\n");
		return -ENOMEM;
	}
	*vd = ev76c570_int_device;
	vd->priv = &data->sen;
	data->vd = vd;
	ret = v4l2_int_device_register(vd);
	if (ret < 0) {
		dev_err(&spi->dev, "error registering v4l2 device\n");
		return ret;
	}
	dev_info(&spi->dev, "v4l2 device registered\n");
	dev_set_drvdata(&spi->dev, data);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id ev76c570_dt_ids[] = {
	{ .compatible = "e2v,ev76c570" },
	{},
};

MODULE_DEVICE_TABLE(of, ev76c570_dt_ids);
#endif

static struct spi_driver ev76c570_driver = {
	.driver = {
		.name	= "ev76c570",
		.of_match_table = of_match_ptr(ev76c570_dt_ids),
		.owner	= THIS_MODULE,
	},
	.probe		= ev76c570_probe,
	.remove		= ev76c570_remove,
};

module_spi_driver(ev76c570_driver);

MODULE_AUTHOR("Davide Ciminaghi <ciminaghi@gnudd.com>");
MODULE_DESCRIPTION("ev76c570 driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:ev76c570");
