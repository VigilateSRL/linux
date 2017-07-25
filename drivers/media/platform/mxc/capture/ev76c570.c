/*
 * Copyright (C) 2017 Vigilate
 * Author: Davide Ciminaghi <ciminaghi@gnudd.com>
 *
 * Driver for ev76c570 parallel sensor (mxc platform)
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

/* Registers defines */
#define EV76C570_REG0 0
#define EV76C570_REG_SOFT_RESET 1
#define EV76C570_CALIB_MBX 2
#define EV76C570_ABORT_MBX 3
#define EV76C570_REG_LINE_CFG 4
#define EV76C570_CHIP_ID 0x7f

struct ev76c570_priv {
	struct regmap *map8;
	struct regmap *map16;
	int reset_gpio;
};

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

static int ev76c570_remove(struct spi_device *spi)
{
	return 0;
}

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
	if (gpio_is_valid(out->reset_gpio)) {
		int v, ret = devm_gpio_request(dev, out->reset_gpio, "RST");
		if (ret) {
			dev_err(dev, "cannot get RST gpio\n");
			return ERR_PTR(ret);
		}
		/* gpio is an output, also keep it non-active */
		v = out->reset_gpio_flags & OF_GPIO_ACTIVE_LOW ?
			1 : 0;
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
	gpio_set_value(plat->reset_gpio, v);
	v = v ? 0 : 1;
	mdelay(5);
	gpio_set_value(plat->reset_gpio, v);
	return 0;
}

static int ev76c570_probe(struct spi_device *spi)
{
	struct ev76c570_priv *data = devm_kzalloc(&spi->dev, sizeof(*data),
						  GFP_KERNEL);
	struct ev76c570_platform_data *plat = dev_get_platdata(&spi->dev);
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
