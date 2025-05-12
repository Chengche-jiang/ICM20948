// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Invensense, Inc.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/of.h>

#include "inv_icm20948.h"

static int inv_icm20948_i2c_bus_setup(struct inv_icm20948_state *st)
{
	return 0;
}

static int inv_icm20948_i2c_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct regmap *regmap;
	int ret;

	regmap = devm_regmap_init_i2c(client, &inv_icm20948_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "Failed to initialize I2C regmap\n");
		return PTR_ERR(regmap);
	}

	ret = inv_icm20948_core_probe(regmap, client->irq, inv_icm20948_i2c_bus_setup);
	if (ret) {
		dev_err(&client->dev, "Failed to probe device\n");
		return ret;
	}

	return 0;
}

static const struct i2c_device_id inv_icm20948_i2c_id[] = {
	{ "icm20948", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, inv_icm20948_i2c_id);

static const struct of_device_id inv_icm20948_i2c_of_match[] = {
	{ .compatible = "invensense,icm20948" },
	{ }
};
MODULE_DEVICE_TABLE(of, inv_icm20948_i2c_of_match);

static struct i2c_driver inv_icm20948_i2c_driver = {
	.driver = {
		.name = "inv_icm20948_i2c",
		.of_match_table = inv_icm20948_i2c_of_match,
	},
	.probe = inv_icm20948_i2c_probe,
	.id_table = inv_icm20948_i2c_id,
};
module_i2c_driver(inv_icm20948_i2c_driver);

MODULE_AUTHOR("InvenSense, Inc.");
MODULE_DESCRIPTION("InvenSense ICM-20948 I2C driver");
MODULE_LICENSE("GPL v2"); 