// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Invensense, Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#include "inv_icm20948.h"

/* AK09916 Registers */
#define INV_ICM20948_REG_MAG_WIA1		0x00
#define INV_ICM20948_REG_MAG_WIA2		0x01
#define INV_ICM20948_MAG_WIA1_VAL		0x48
#define INV_ICM20948_MAG_WIA2_VAL		0x09

#define INV_ICM20948_REG_MAG_ST1		0x10
#define INV_ICM20948_REG_MAG_HXL		0x11
#define INV_ICM20948_REG_MAG_HXH		0x12
#define INV_ICM20948_REG_MAG_HYL		0x13
#define INV_ICM20948_REG_MAG_HYH		0x14
#define INV_ICM20948_REG_MAG_HZL		0x15
#define INV_ICM20948_REG_MAG_HZH		0x16
#define INV_ICM20948_REG_MAG_ST2		0x18

#define INV_ICM20948_REG_MAG_CNTL2		0x31
#define INV_ICM20948_MAG_MODE_POWER_DOWN	0x00
#define INV_ICM20948_MAG_MODE_SINGLE		0x01
#define INV_ICM20948_MAG_MODE_CONT1		0x02
#define INV_ICM20948_MAG_MODE_CONT2		0x04
#define INV_ICM20948_MAG_MODE_CONT3		0x06
#define INV_ICM20948_MAG_MODE_CONT4		0x08
#define INV_ICM20948_MAG_MODE_SELF_TEST		0x10

#define INV_ICM20948_MAGN_CHAN(_type, _axis, _index) {			\
	.type = _type,							\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			     BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _index,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 16,						\
		.storagebits = 16,						\
		.endianness = IIO_LE,					\
	},									\
}

static const struct iio_chan_spec inv_icm20948_magn_channels[] = {
	INV_ICM20948_MAGN_CHAN(IIO_MAGN, X, 0),
	INV_ICM20948_MAGN_CHAN(IIO_MAGN, Y, 1),
	INV_ICM20948_MAGN_CHAN(IIO_MAGN, Z, 2),
	IIO_CHAN_SOFT_TIMESTAMP(3),
};

/* AK09916 magnetometer has a fixed scale of 0.15 μT/LSB */
static const int inv_icm20948_magn_scale = 150; /* 0.15 μT/LSB * 1000 */

static int inv_icm20948_magn_read_raw(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan,
				     int *val, int *val2, long mask)
{
	struct inv_icm20948_state *st;
	struct device *dev;
	int ret;
	__le16 data;

	/* 检查输入参数 */
	if (!indio_dev)
		return -EINVAL;

	st = iio_device_get_drvdata(indio_dev);

	/* 安全检查：确保状态结构体和regmap有效 */
	if (!st) {
		pr_err("ICM20948: 磁力计传感器状态结构为空\n");
		return -EINVAL;
	}

	if (!st->map) {
		pr_err("ICM20948: 磁力计传感器regmap为空\n");
		return -EINVAL;
	}

	/* 使用pr_debug而不是dev_info，避免依赖设备指针 */
	pr_debug("ICM20948: 磁力计传感器读取开始\n");

	/* 获取设备指针前先确保map有效 */
	dev = regmap_get_device(st->map);
	if (!dev) {
		pr_err("ICM20948: 无法获取磁力计传感器设备指针\n");
		return -EINVAL;
	}

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		dev_info(dev, "磁力计读取开始，准备获取互斥锁\n");
		mutex_lock(&st->lock);
		dev_info(dev, "已获取互斥锁，准备切换到Bank 0\n");

		/* Ensure we are in bank 0 */
		ret = inv_icm20948_set_bank(st, INV_ICM20948_BANK_0);
		if (ret) {
			dev_err(dev, "切换到Bank 0失败: %d\n", ret);
			mutex_unlock(&st->lock);
			return ret;
		}

		/* 添加延时，确保Bank切换完成 */
		msleep(10);
		dev_info(dev, "Bank切换延时完成，准备读取磁力计数据\n");

		/* Magnetometer data starts at EXT_SLV_SENS_DATA_01 (skipping ST1 status) */
		ret = regmap_bulk_read(st->map, 
		                      INV_ICM20948_REG_EXT_SLV_SENS_DATA_01 + chan->scan_index * 2,
		                      &data, sizeof(data));
		dev_info(dev, "磁力计数据读取完成，准备释放互斥锁\n");
		mutex_unlock(&st->lock);
		dev_info(dev, "互斥锁已释放\n");

		if (ret) {
			dev_err(dev, "磁力计数据读取失败: %d\n", ret);
			return ret;
		}

		*val = le16_to_cpu(data);
		dev_info(dev, "磁力计原始值: %d\n", *val);
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = inv_icm20948_magn_scale;
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = 100;  /* Fixed 100Hz (using continuous measurement mode 4) */
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int inv_icm20948_magn_write_raw(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      int val, int val2, long mask)
{
	struct inv_icm20948_state *st = iio_device_get_drvdata(indio_dev);
	struct device *dev = regmap_get_device(st->map);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		/* Only 100Hz is currently supported */
		if (val != 100) {
			dev_err(dev, "磁力计仅支持100Hz采样率，当前设置: %dHz\n", val);
			return -EINVAL;
		}
		dev_info(dev, "设置磁力计采样率: 100Hz\n");
		return 0;

	case IIO_CHAN_INFO_SCALE:
		/* 磁力计的量程是固定的，但我们可以提供友好的错误信息 */
		if (val2 == 0) {
			/* 如果用户尝试直接设置μT值 */
			dev_info(dev, "磁力计量程固定为4912μT，不支持修改为%dμT\n", val);
		} else {
			dev_info(dev, "磁力计量程固定，不支持修改\n");
		}
		return -EINVAL;

	default:
		return -EINVAL;
	}
}

static int inv_icm20948_magn_validate_trigger(struct iio_dev *indio_dev,
                                            struct iio_trigger *trig)
{
	struct inv_icm20948_state *st = iio_device_get_drvdata(indio_dev);

	if (st->trig != trig)
		return -EINVAL;

	return 0;
}

static const struct iio_info inv_icm20948_magn_info = {
	.read_raw = inv_icm20948_magn_read_raw,
	.write_raw = inv_icm20948_magn_write_raw,
	.validate_trigger = inv_icm20948_magn_validate_trigger,
};

static int inv_icm20948_read_fifo_magn(struct inv_icm20948_state *st,
                                      __le16 *buffer)
{
	int ret;
	u8 d[8];

	/* Read magnetometer data from external sensor data registers */
	ret = regmap_bulk_read(st->map, INV_ICM20948_REG_EXT_SLV_SENS_DATA_00, d, sizeof(d));
	if (ret)
		return ret;

	/* Check data ready bit in ST1 register */
	if (!(d[0] & BIT(0)))
		return -EAGAIN;

	/* Skip ST1 status byte and copy 3 axes of mag data (HXL,HXH,HYL,HYH,HZL,HZH) */
	memcpy(buffer, &d[1], 6);

	return 0;
}

/* 将trigger_handler改为非静态函数，使其在buffer.c中可见 */
irqreturn_t inv_icm20948_magn_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct inv_icm20948_state *st = iio_device_get_drvdata(indio_dev);
	__le16 buffer[8]; /* 3 x 16-bit channels + timestamp */
	int ret;

	mutex_lock(&st->lock);
	
	if (st->use_fifo) {
		ret = inv_icm20948_read_fifo_magn(st, buffer);
		if (ret) {
			/* On error, fall back to direct register read */
			ret = inv_icm20948_set_bank(st, INV_ICM20948_BANK_0);
			if (ret) {
				mutex_unlock(&st->lock);
				goto done;
			}
			
			ret = regmap_bulk_read(st->map, INV_ICM20948_REG_EXT_SLV_SENS_DATA_01,
			                      buffer, 3 * sizeof(__le16));
			if (ret) {
				mutex_unlock(&st->lock);
				goto done;
			}
		}
	} else {
		ret = inv_icm20948_set_bank(st, INV_ICM20948_BANK_0);
		if (ret) {
			mutex_unlock(&st->lock);
			goto done;
		}
		
		ret = regmap_bulk_read(st->map, INV_ICM20948_REG_EXT_SLV_SENS_DATA_01,
		                      buffer, 3 * sizeof(__le16));
		if (ret) {
			mutex_unlock(&st->lock);
			goto done;
		}
	}
	
	st->timestamp.magn = iio_get_time_ns(indio_dev);
	mutex_unlock(&st->lock);

	iio_push_to_buffers_with_timestamp(indio_dev, buffer, st->timestamp.magn);

done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

struct iio_dev *inv_icm20948_magn_init(struct inv_icm20948_state *st)
{
	struct iio_dev *indio_dev;
	struct inv_icm20948_sensor_state *sensor;
	int ret;

	indio_dev = devm_iio_device_alloc(regmap_get_device(st->map),
					 sizeof(*sensor));
	if (!indio_dev)
		return ERR_PTR(-ENOMEM);

	sensor = iio_priv(indio_dev);

	indio_dev->name = "icm20948_magn";
	indio_dev->info = &inv_icm20948_magn_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = inv_icm20948_magn_channels;
	indio_dev->num_channels = ARRAY_SIZE(inv_icm20948_magn_channels);

	/* Store the same state pointer for all sensors */
	iio_device_set_drvdata(indio_dev, st);

	/* 只关联触发器，不设置触发缓冲区，这由inv_icm20948_setup_trigger统一处理 */
	if (st->trig) {
		indio_dev->trig = st->trig;
	}

	return indio_dev;
}
EXPORT_SYMBOL_GPL(inv_icm20948_magn_init);