// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Invensense, Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/slab.h>

#include "inv_icm20948.h"

#define INV_ICM20948_FIFO_CHUNK_SIZE 16
#define INV_ICM20948_FIFO_MAX_SIZE 1024

/* 声明在其他文件中定义的传感器特定触发处理程序函数 */
extern irqreturn_t inv_icm20948_gyro_trigger_handler(int irq, void *p);
extern irqreturn_t inv_icm20948_accel_trigger_handler(int irq, void *p);
extern irqreturn_t inv_icm20948_magn_trigger_handler(int irq, void *p);

static void inv_icm20948_fifo_flush(struct inv_icm20948_state *st)
{
	unsigned int reg;

	mutex_lock(&st->lock);
	inv_icm20948_set_bank(st, INV_ICM20948_BANK_0);
	regmap_write(st->map, INV_ICM20948_REG_FIFO_RST, 0x1F);
	regmap_write(st->map, INV_ICM20948_REG_FIFO_RST, 0x00);
	regmap_read(st->map, INV_ICM20948_REG_INT_STATUS, &reg);
	mutex_unlock(&st->lock);
}

/* Parse FIFO data for gyroscope */
static int inv_icm20948_parse_fifo_gyro(struct iio_dev *indio_dev, uint8_t *data, int64_t timestamp)
{
	struct iio_buffer *buffer = indio_dev->buffer;
	s16 x, y, z;
	s8 *buf;
	int i;

	if (!buffer)
		return -EINVAL;

	/* Get raw values */
	x = (s16)(data[0] << 8 | data[1]);
	y = (s16)(data[2] << 8 | data[3]);
	z = (s16)(data[4] << 8 | data[5]);

	/* Allocate buffer for scan */
	buf = kmalloc(indio_dev->scan_bytes, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	/* Fill buffer based on active channels */
	for (i = 0; i < 3; i++) {
		if (test_bit(i, indio_dev->active_scan_mask)) {
			/* Copy correct value to appropriate position */
			if (i == 0)
				memcpy(buf + indio_dev->channels[i].scan_index * sizeof(s16), &x, sizeof(x));
			else if (i == 1)
				memcpy(buf + indio_dev->channels[i].scan_index * sizeof(s16), &y, sizeof(y));
			else if (i == 2)
				memcpy(buf + indio_dev->channels[i].scan_index * sizeof(s16), &z, sizeof(z));
		}
	}

	/* Push to buffer */
	iio_push_to_buffers_with_timestamp(indio_dev, buf, timestamp);

	kfree(buf);
	return 0;
}

/* Parse FIFO data for accelerometer */
static int inv_icm20948_parse_fifo_accel(struct iio_dev *indio_dev, uint8_t *data, int64_t timestamp)
{
	struct iio_buffer *buffer = indio_dev->buffer;
	s16 x, y, z;
	s8 *buf;
	int i;

	if (!buffer)
		return -EINVAL;

	/* Get raw values */
	x = (s16)(data[0] << 8 | data[1]);
	y = (s16)(data[2] << 8 | data[3]);
	z = (s16)(data[4] << 8 | data[5]);

	/* Allocate buffer for scan */
	buf = kmalloc(indio_dev->scan_bytes, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	/* Fill buffer based on active channels */
	for (i = 0; i < 3; i++) {
		if (test_bit(i, indio_dev->active_scan_mask)) {
			/* Copy correct value to appropriate position */
			if (i == 0)
				memcpy(buf + indio_dev->channels[i].scan_index * sizeof(s16), &x, sizeof(x));
			else if (i == 1)
				memcpy(buf + indio_dev->channels[i].scan_index * sizeof(s16), &y, sizeof(y));
			else if (i == 2)
				memcpy(buf + indio_dev->channels[i].scan_index * sizeof(s16), &z, sizeof(z));
		}
	}

	/* Push to buffer */
	iio_push_to_buffers_with_timestamp(indio_dev, buf, timestamp);

	kfree(buf);
	return 0;
}

/* Parse FIFO data for magnetometer */
static int inv_icm20948_parse_fifo_magn(struct iio_dev *indio_dev, uint8_t *data, int64_t timestamp)
{
	struct iio_buffer *buffer = indio_dev->buffer;
	s16 x, y, z;
	s8 *buf;
	int i;

	if (!buffer)
		return -EINVAL;

	/* Get raw values */
	x = (s16)(data[1] << 8 | data[0]);
	y = (s16)(data[3] << 8 | data[2]);
	z = (s16)(data[5] << 8 | data[4]);

	/* Allocate buffer for scan */
	buf = kmalloc(indio_dev->scan_bytes, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	/* Fill buffer based on active channels */
	for (i = 0; i < 3; i++) {
		if (test_bit(i, indio_dev->active_scan_mask)) {
			/* Copy correct value to appropriate position */
			if (i == 0)
				memcpy(buf + indio_dev->channels[i].scan_index * sizeof(s16), &x, sizeof(x));
			else if (i == 1)
				memcpy(buf + indio_dev->channels[i].scan_index * sizeof(s16), &y, sizeof(y));
			else if (i == 2)
				memcpy(buf + indio_dev->channels[i].scan_index * sizeof(s16), &z, sizeof(z));
		}
	}

	/* Push to buffer */
	iio_push_to_buffers_with_timestamp(indio_dev, buf, timestamp);

	kfree(buf);
	return 0;
}

/* Read FIFO data */
int inv_icm20948_read_fifo(struct inv_icm20948_state *st)
{
	__be16 fifo_count;
	uint8_t data[INV_ICM20948_FIFO_MAX_SIZE];
	uint8_t *ptr;
	unsigned int word;
	int ret;
	int64_t timestamp = iio_get_time_ns(st->indio_gyro);
	bool has_gyro, has_accel, has_magn;
	bool accel_done = false, gyro_done = false, magn_done = false;

	mutex_lock(&st->lock);

	/* Check which sensors are enabled in FIFO */
	ret = inv_icm20948_set_bank(st, INV_ICM20948_BANK_0);
	if (ret)
		goto end;

	ret = regmap_read(st->map, INV_ICM20948_REG_FIFO_EN_2, &word);
	if (ret)
		goto end;

	has_gyro = word & INV_ICM20948_FIFO_GYRO_EN_ALL;
	has_accel = word & INV_ICM20948_BIT_ACCEL_FIFO_EN;

	ret = regmap_read(st->map, INV_ICM20948_REG_FIFO_EN_1, &word);
	if (ret)
		goto end;

	has_magn = word & INV_ICM20948_BIT_SLV_0_FIFO_EN;

	/* Read FIFO count */
	ret = regmap_bulk_read(st->map, INV_ICM20948_REG_FIFO_COUNT_H,
			      &fifo_count, sizeof(fifo_count));
	if (ret)
		goto end;

	word = be16_to_cpu(fifo_count);
	if (!word) {
		ret = 0;
		goto end;
	}

	if (word > INV_ICM20948_FIFO_MAX_SIZE) {
		/* FIFO overrun, reset FIFO */
		inv_icm20948_fifo_flush(st);
		ret = -EIO;
		goto end;
	}

	/* Read FIFO data */
	ret = regmap_bulk_read(st->map, INV_ICM20948_REG_FIFO_R_W,
			      data, word);
	if (ret)
		goto end;

	/* Parse FIFO data */
	ptr = data;
	while (ptr < data + word) {
		if (has_accel && !accel_done) {
			ret = inv_icm20948_parse_fifo_accel(st->indio_accel, ptr, timestamp);
			if (ret)
				goto end;
			ptr += 6; /* 3 axes x 2 bytes */
			accel_done = true;
		}

		if (has_gyro && !gyro_done) {
			ret = inv_icm20948_parse_fifo_gyro(st->indio_gyro, ptr, timestamp);
			if (ret)
				goto end;
			ptr += 6; /* 3 axes x 2 bytes */
			gyro_done = true;
		}

		if (has_magn && !magn_done && st->indio_magn) {
			ret = inv_icm20948_parse_fifo_magn(st->indio_magn, ptr, timestamp);
			if (ret)
				goto end;
			ptr += 6; /* 3 axes x 2 bytes */
			magn_done = true;
		}

		/* If all sensors have been processed, start a new batch */
		if ((accel_done || !has_accel) && 
		    (gyro_done || !has_gyro) && 
		    (magn_done || !has_magn || !st->indio_magn)) {
			accel_done = false;
			gyro_done = false;
			magn_done = false;
		}
	}

end:
	mutex_unlock(&st->lock);
	return ret;
}

/* Callback for triggered buffer mode */
static irqreturn_t inv_icm20948_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct inv_icm20948_state *st = iio_priv(indio_dev);
	
	if (st->use_fifo) {
		/* Read data from FIFO */
		inv_icm20948_read_fifo(st);
	} else {
		/* Direct register read for each sensor will be handled by 
		 * sensor-specific trigger handlers
		 */
	}
	
	iio_trigger_notify_done(indio_dev->trig);
	
	return IRQ_HANDLED;
}

int inv_icm20948_setup_trigger(struct inv_icm20948_state *st)
{
	int ret;
	struct device *dev = regmap_get_device(st->map);
	
	if (!st->indio_gyro || !st->indio_accel) {
		dev_err(dev, "陀螺仪或加速度计设备未初始化\n");
		return -EINVAL;
	}
	
	/* 确保触发器已正确分配和注册 */
	if (!st->trig) {
		dev_err(dev, "触发器未初始化\n");
		return -EINVAL;
	}
	
	dev_info(dev, "设置传感器触发缓冲区\n");
	
	/* 为陀螺仪设置触发缓冲区 */
	dev_info(dev, "为陀螺仪设置触发缓冲区\n");
	st->indio_gyro->modes |= INDIO_BUFFER_TRIGGERED;
	ret = iio_triggered_buffer_setup(st->indio_gyro, NULL,
				        inv_icm20948_gyro_trigger_handler, NULL);
	if (ret) {
		dev_err(dev, "陀螺仪触发缓冲区设置失败: %d\n", ret);
		return ret;
	}
	
	/* 为加速度计设置触发缓冲区 */
	dev_info(dev, "为加速度计设置触发缓冲区\n");
	st->indio_accel->modes |= INDIO_BUFFER_TRIGGERED;
	ret = iio_triggered_buffer_setup(st->indio_accel, NULL,
				        inv_icm20948_accel_trigger_handler, NULL);
	if (ret) {
		dev_err(dev, "加速度计触发缓冲区设置失败: %d\n", ret);
		goto error_accel;
	}
	
	/* 为磁力计设置触发缓冲区（如果存在） */
	if (st->indio_magn) {
		dev_info(dev, "为磁力计设置触发缓冲区\n");
		st->indio_magn->modes |= INDIO_BUFFER_TRIGGERED;
		ret = iio_triggered_buffer_setup(st->indio_magn, NULL,
					        inv_icm20948_magn_trigger_handler, NULL);
		if (ret) {
			dev_err(dev, "磁力计触发缓冲区设置失败: %d\n", ret);
			goto error_magn;
		}
	}
	
	dev_info(dev, "所有传感器触发缓冲区设置成功\n");
	return 0;

error_magn:
	iio_triggered_buffer_cleanup(st->indio_accel);
error_accel:
	iio_triggered_buffer_cleanup(st->indio_gyro);
	
	return ret;
}
EXPORT_SYMBOL_GPL(inv_icm20948_setup_trigger);

void inv_icm20948_cleanup_buffer(struct inv_icm20948_state *st)
{
	if (st->indio_magn)
		iio_triggered_buffer_cleanup(st->indio_magn);
	
	iio_triggered_buffer_cleanup(st->indio_accel);
	iio_triggered_buffer_cleanup(st->indio_gyro);
}
EXPORT_SYMBOL_GPL(inv_icm20948_cleanup_buffer); 