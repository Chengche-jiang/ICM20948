// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Invensense, Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "inv_icm20948.h"

/* ICM-20948 temperature has a sensitivity of 333.87 LSB/°C
 * and an offset of 0°C = 21°C.
 */
#define INV_ICM20948_TEMP_SCALE 333870
#define INV_ICM20948_TEMP_OFFSET 21000

static const struct iio_chan_spec inv_icm20948_temp_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				     BIT(IIO_CHAN_INFO_SCALE) |
				     BIT(IIO_CHAN_INFO_OFFSET) |
				     BIT(IIO_CHAN_INFO_PROCESSED),
		.scan_index = -1,
	},
};

static int inv_icm20948_temp_read_raw(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan,
				     int *val, int *val2, long mask)
{
	struct inv_icm20948_state *st;
	struct device *dev;
	int ret;
	__be16 data;
	unsigned int bank_val;
	int raw_temp;

	/* 检查输入参数 */
	if (!indio_dev)
		return -EINVAL;

	st = iio_device_get_drvdata(indio_dev);

	/* 安全检查：确保状态结构体和regmap有效 */
	if (!st) {
		pr_err("ICM20948: 温度传感器状态结构为空\n");
		return -EINVAL;
	}

	if (!st->map) {
		pr_err("ICM20948: 温度传感器regmap为空\n");
		return -EINVAL;
	}

	/* 使用pr_debug而不是dev_info，避免依赖设备指针 */
	pr_debug("ICM20948: 温度传感器读取开始\n");

	/* 获取设备指针前先确保map有效 */
	dev = regmap_get_device(st->map);
	if (!dev) {
		pr_err("ICM20948: 无法获取温度传感器设备指针\n");
		return -EINVAL;
	}

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		/* 读取原始温度值 */
		dev_info(dev, "温度传感器处理值读取开始\n");
		mutex_lock(&st->lock);
		
		/* 确保在Bank 0 */
		ret = inv_icm20948_set_bank(st, INV_ICM20948_BANK_0);
		if (ret) {
			dev_err(dev, "切换到Bank 0失败: %d\n", ret);
			mutex_unlock(&st->lock);
			return ret;
		}

		/* 添加延时，确保Bank切换完成 */
		msleep(10);

		/* 读取温度寄存器 */
		ret = regmap_bulk_read(st->map, INV_ICM20948_REG_TEMP_OUT_H,
				      &data, sizeof(data));
		mutex_unlock(&st->lock);

		if (ret) {
			dev_err(dev, "温度数据读取失败: %d\n", ret);
			return ret;
		}

		/* 获取原始温度值 */
		raw_temp = be16_to_cpu(data);

		/* 计算实际温度值: 原始值/333.87 + 21 */
		/* 使用整数计算: 原始值 * 1000000 / 333870 + 21000 (单位: 毫度) */
		*val = (raw_temp * 1000000 / INV_ICM20948_TEMP_SCALE) + INV_ICM20948_TEMP_OFFSET;
		*val2 = 0;
		dev_info(dev, "温度处理值: %d 毫度\n", *val);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_RAW:
		dev_info(dev, "温度传感器读取开始，准备获取互斥锁\n");
		mutex_lock(&st->lock);
		dev_info(dev, "已获取互斥锁，准备切换到Bank 0\n");

		/* 读取当前Bank值 */
		ret = regmap_read(st->map, INV_ICM20948_REG_BANK_SEL, &bank_val);
		if (ret) {
			dev_err(dev, "读取当前Bank失败: %d\n", ret);
			mutex_unlock(&st->lock);
			return ret;
		}
		dev_info(dev, "当前Bank值: 0x%02x\n", bank_val);

		/* Ensure we are in bank 0 */
		ret = inv_icm20948_set_bank(st, INV_ICM20948_BANK_0);
		if (ret) {
			dev_err(dev, "切换到Bank 0失败: %d\n", ret);
			mutex_unlock(&st->lock);
			return ret;
		}

		/* 验证Bank切换 */
		ret = regmap_read(st->map, INV_ICM20948_REG_BANK_SEL, &bank_val);
		if (ret) {
			dev_err(dev, "切换后读取Bank失败: %d\n", ret);
			mutex_unlock(&st->lock);
			return ret;
		}
		dev_info(dev, "切换后Bank值: 0x%02x\n", bank_val);

		/* 添加延时，确保Bank切换完成 */
		msleep(10);
		dev_info(dev, "Bank切换延时完成，准备读取温度数据\n");

		ret = regmap_bulk_read(st->map, INV_ICM20948_REG_TEMP_OUT_H,
				      &data, sizeof(data));
		dev_info(dev, "温度数据读取完成，准备释放互斥锁\n");
		mutex_unlock(&st->lock);
		dev_info(dev, "互斥锁已释放\n");

		if (ret) {
			dev_err(dev, "温度数据读取失败: %d\n", ret);
			return ret;
		}

		*val = be16_to_cpu(data);
		dev_info(dev, "温度原始值: %d\n", *val);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = INV_ICM20948_TEMP_SCALE;
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_OFFSET:
		*val = INV_ICM20948_TEMP_OFFSET;
		*val2 = 0;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static const struct iio_info inv_icm20948_temp_info = {
	.read_raw = inv_icm20948_temp_read_raw,
};

struct iio_dev *inv_icm20948_temp_init(struct inv_icm20948_state *st)
{
	struct iio_dev *indio_dev;
	struct device *dev;

	/* 安全检查 */
	if (!st) {
		pr_err("ICM20948: 温度传感器初始化失败，状态结构为空\n");
		return ERR_PTR(-EINVAL);
	}

	if (!st->map) {
		pr_err("ICM20948: 温度传感器初始化失败，regmap为空\n");
		return ERR_PTR(-EINVAL);
	}

	dev = regmap_get_device(st->map);
	if (!dev) {
		pr_err("ICM20948: 温度传感器初始化失败，无法获取设备指针\n");
		return ERR_PTR(-EINVAL);
	}

	/* 分配IIO设备，不需要私有数据空间 */
	indio_dev = devm_iio_device_alloc(dev, 0);
	if (!indio_dev) {
		dev_err(dev, "温度传感器初始化失败，无法分配IIO设备\n");
		return ERR_PTR(-ENOMEM);
	}

	indio_dev->name = "icm20948_temp";
	indio_dev->info = &inv_icm20948_temp_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = inv_icm20948_temp_channels;
	indio_dev->num_channels = ARRAY_SIZE(inv_icm20948_temp_channels);

	/* 设置驱动数据，使用主驱动的状态结构体 */
	iio_device_set_drvdata(indio_dev, st);

	dev_info(dev, "温度传感器初始化成功\n");
	return indio_dev;
}
EXPORT_SYMBOL_GPL(inv_icm20948_temp_init);