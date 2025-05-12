// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Invensense, Inc.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c-smbus.h>
#include <linux/workqueue.h>

#include "inv_icm20948.h"

/* 触发器操作结构体 */
static const struct iio_trigger_ops inv_icm20948_trigger_ops = {
	.owner = THIS_MODULE,
};

/* 配置和功能设置 */
#define INV_ICM20948_WHO_AM_I_VAL     0xEA
#define INV_ICM20948_REG_WHO_AM_I     0x00  /* 这是Bank 0中的WHO_AM_I寄存器地址 */

static const struct regmap_range inv_icm20948_readable_ranges[] = {
    // Bank 0
    regmap_reg_range(INV_ICM20948_REG_WHO_AM_I, INV_ICM20948_REG_WHO_AM_I),
    regmap_reg_range(INV_ICM20948_REG_USER_CTRL, INV_ICM20948_REG_PWR_MGMT_2),
    regmap_reg_range(INV_ICM20948_REG_INT_PIN_CFG, INV_ICM20948_REG_INT_ENABLE_3),
    regmap_reg_range(INV_ICM20948_REG_INT_STATUS, INV_ICM20948_REG_INT_STATUS_3),
    regmap_reg_range(INV_ICM20948_REG_ACCEL_XOUT_H, INV_ICM20948_REG_TEMP_OUT_L),
    regmap_reg_range(INV_ICM20948_REG_EXT_SLV_SENS_DATA_00, INV_ICM20948_REG_EXT_SLV_SENS_DATA_06),
    regmap_reg_range(INV_ICM20948_REG_FIFO_EN_1, INV_ICM20948_REG_FIFO_EN_2),
    regmap_reg_range(INV_ICM20948_REG_FIFO_RST, INV_ICM20948_REG_FIFO_RST),
    regmap_reg_range(INV_ICM20948_REG_FIFO_COUNT_H, INV_ICM20948_REG_FIFO_R_W),
    regmap_reg_range(INV_ICM20948_REG_BANK_SEL, INV_ICM20948_REG_BANK_SEL),

    // Bank 2
    regmap_reg_range(INV_ICM20948_REG_GYRO_SMPLRT_DIV, INV_ICM20948_REG_GYRO_CONFIG_2),
    regmap_reg_range(INV_ICM20948_REG_ACCEL_SMPLRT_DIV_1, INV_ICM20948_REG_ACCEL_SMPLRT_DIV_2),
    regmap_reg_range(INV_ICM20948_REG_ACCEL_CONFIG, INV_ICM20948_REG_ACCEL_CONFIG_2),

    // Bank 3
    regmap_reg_range(INV_ICM20948_REG_I2C_MST_CTRL, INV_ICM20948_REG_I2C_SLV0_DO),
    regmap_reg_range(INV_ICM20948_REG_I2C_SLV0_CTRL, INV_ICM20948_REG_I2C_SLV0_CTRL),
};

const struct regmap_access_table inv_icm20948_readable_regs = {
	.yes_ranges = inv_icm20948_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(inv_icm20948_readable_ranges),
};

static const struct regmap_range inv_icm20948_writable_ranges[] = {
	// Bank 0
	regmap_reg_range(INV_ICM20948_REG_USER_CTRL, INV_ICM20948_REG_PWR_MGMT_2),
	regmap_reg_range(INV_ICM20948_REG_INT_PIN_CFG, INV_ICM20948_REG_INT_ENABLE_3),
	regmap_reg_range(INV_ICM20948_REG_FIFO_EN_1, INV_ICM20948_REG_FIFO_EN_2),
	regmap_reg_range(INV_ICM20948_REG_FIFO_RST, INV_ICM20948_REG_FIFO_RST),
	regmap_reg_range(INV_ICM20948_REG_FIFO_R_W, INV_ICM20948_REG_FIFO_R_W),
	regmap_reg_range(INV_ICM20948_REG_BANK_SEL, INV_ICM20948_REG_BANK_SEL),

	// Bank 2
	regmap_reg_range(INV_ICM20948_REG_GYRO_SMPLRT_DIV, INV_ICM20948_REG_GYRO_CONFIG_2),
	regmap_reg_range(INV_ICM20948_REG_ACCEL_SMPLRT_DIV_1, INV_ICM20948_REG_ACCEL_SMPLRT_DIV_2),
	regmap_reg_range(INV_ICM20948_REG_ACCEL_CONFIG, INV_ICM20948_REG_ACCEL_CONFIG_2),

	// ✅ Bank 3 - I2C Master 控制寄存器（确保加进去）
	regmap_reg_range(INV_ICM20948_REG_I2C_MST_CTRL, INV_ICM20948_REG_I2C_SLV0_DO),
	regmap_reg_range(INV_ICM20948_REG_I2C_SLV0_CTRL, INV_ICM20948_REG_I2C_SLV0_CTRL),
};


const struct regmap_access_table inv_icm20948_writable_regs = {
	.yes_ranges = inv_icm20948_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(inv_icm20948_writable_ranges),
};

const struct regmap_config inv_icm20948_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x7F,
	.rd_table = &inv_icm20948_readable_regs,
	.wr_table = &inv_icm20948_writable_regs,
	.cache_type = REGCACHE_NONE,
};
EXPORT_SYMBOL_GPL(inv_icm20948_regmap_config);

/**
 * inv_icm20948_set_bank - Set register bank
 * @st: Driver state
 * @bank: Bank to select (0-3)
 *
 * Select the register bank to access registers in different banks.
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_set_bank(struct inv_icm20948_state *st, u8 bank)
{
	int ret;
	struct device *dev = regmap_get_device(st->map);
	unsigned int read_val;

	/* 切换到目标Bank */
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, bank);
	if (ret) {
		dev_err(dev, "切换到Bank 0x%02x失败: %d\n", bank, ret);
		return ret;
	}

	/* 使用msleep替代usleep_range，确保更长的延迟 */
	msleep(10);

	/* 验证Bank切换成功 */
	ret = regmap_read(st->map, INV_ICM20948_REG_BANK_SEL, &read_val);
	if (ret) {
		dev_err(dev, "验证Bank切换失败: %d\n", ret);
		return ret;
	}

	if ((read_val & INV_ICM20948_BANK_SEL_MASK) != bank) {
		dev_err(dev, "Bank切换验证失败，期望: 0x%02x, 实际: 0x%02x\n",
		       bank, read_val & INV_ICM20948_BANK_SEL_MASK);
		return -EIO;
	}

	return 0;
}

/**
 * inv_icm20948_write_reg - Write to a register in a specific bank
 * @st: Driver state
 * @bank: Bank number (0-3)
 * @reg: Register address
 * @val: Value to write
 *
 * Select bank, write to register, and restore bank 0.
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
    int inv_icm20948_write_reg(struct inv_icm20948_state *st, u8 bank, u8 reg, u8 val)
    {
        int ret;
        struct device *dev = regmap_get_device(st->map);
        unsigned int current_bank, read_val;

        /* 先读取当前Bank */
        ret = regmap_read(st->map, INV_ICM20948_REG_BANK_SEL, &current_bank);
        if (ret) {
            dev_err(dev, "读取当前Bank失败: %d\n", ret);
            return ret;
        }

        dev_info(dev, "当前Bank原始值: 0x%02x, 掩码后: 0x%02x, 目标Bank: 0x%02x\n",
                 current_bank, current_bank & INV_ICM20948_BANK_SEL_MASK, bank);

        if ((current_bank & INV_ICM20948_BANK_SEL_MASK) != bank) {
            /* 切换到目标Bank */
            ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, bank);
            if (ret) {
                dev_err(dev, "切换到Bank 0x%02x失败: %d\n", bank, ret);
                return ret;
            }

            /* 使用msleep替代usleep_range，确保更长的延迟 */
            msleep(20);

            /* 验证Bank切换成功 */
            ret = regmap_read(st->map, INV_ICM20948_REG_BANK_SEL, &read_val);
            if (ret) {
                dev_err(dev, "验证Bank切换失败: %d\n", ret);
                return ret;
            }

            dev_info(dev, "Bank切换后原始值: 0x%02x, 掩码后: 0x%02x\n",
                     read_val, read_val & INV_ICM20948_BANK_SEL_MASK);

            if ((read_val & INV_ICM20948_BANK_SEL_MASK) != bank) {
                dev_err(dev, "Bank切换验证失败，期望: 0x%02x, 实际: 0x%02x\n",
                       bank, read_val & INV_ICM20948_BANK_SEL_MASK);
                return -EIO;
            }
        }

        /* 写入寄存器值 */
        ret = regmap_write(st->map, reg, val);
        if (ret) {
            dev_err(dev, "写入寄存器0x%02x失败: %d\n", reg, ret);
            /* 不返回错误，尝试恢复Bank 0 */
            regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
            msleep(10);
            return ret;
        }

        /* 等待写入完成 */
        msleep(5);

        /* 恢复到Bank 0 */
        if (bank != INV_ICM20948_BANK_0) {
            ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
            if (ret) {
                dev_err(dev, "恢复到Bank 0失败: %d\n", ret);
                return ret;
            }
            msleep(10);
        }

        return 0;
    }


/**
 * inv_icm20948_read_reg - Read from a register in a specific bank
 * @st: Driver state
 * @bank: Bank number (0-3)
 * @reg: Register address
 * @val: Pointer to store read value
 *
 * Select bank, read from register, and restore bank 0.
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
    int inv_icm20948_read_reg(struct inv_icm20948_state *st, u8 bank, u8 reg, u8 *val)
    {
        unsigned int regval;
        int ret;
        struct device *dev = regmap_get_device(st->map);
        unsigned int current_bank, read_val;

        /* 先读取当前Bank */
        ret = regmap_read(st->map, INV_ICM20948_REG_BANK_SEL, &current_bank);
        if (ret) {
            dev_err(dev, "读取当前Bank失败: %d\n", ret);
            return ret;
        }

        dev_dbg(dev, "当前Bank: 0x%02x, 目标Bank: 0x%02x\n",
                current_bank & INV_ICM20948_BANK_SEL_MASK, bank);

        if ((current_bank & INV_ICM20948_BANK_SEL_MASK) != bank) {
            /* 切换到目标Bank */
            ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, bank);
            if (ret) {
                dev_err(dev, "切换到Bank 0x%02x失败: %d\n", bank, ret);
                return ret;
            }

            /* 使用msleep替代usleep_range，确保更长的延迟 */
            msleep(10);

            /* 验证Bank切换成功 */
            ret = regmap_read(st->map, INV_ICM20948_REG_BANK_SEL, &read_val);
            if (ret) {
                dev_err(dev, "验证Bank切换失败: %d\n", ret);
                return ret;
            }

            if ((read_val & INV_ICM20948_BANK_SEL_MASK) != bank) {
                dev_err(dev, "Bank切换验证失败，期望: 0x%02x, 实际: 0x%02x\n",
                    bank, read_val & INV_ICM20948_BANK_SEL_MASK);
                return -EIO;
            }
        }

        /* 读取寄存器值 */
        ret = regmap_read(st->map, reg, &regval);
        if (ret) {
            dev_err(dev, "读取寄存器0x%02x失败: %d\n", reg, ret);
            /* 尝试恢复Bank 0 */
            regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
            msleep(10);
            return ret;
        }

        *val = (u8)regval;

        /* 恢复到Bank 0 */
        if (bank != INV_ICM20948_BANK_0) {
            ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
            if (ret) {
                dev_err(dev, "恢复到Bank 0失败: %d\n", ret);
                return ret;
            }
            msleep(10);
        }

        return 0;
    }


/**
 * inv_icm20948_set_gyro_fs - Set gyroscope full-scale range
 * @st: Driver state
 * @fs: Full-scale range index
 *
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_set_gyro_fs(struct inv_icm20948_state *st, int fs)
{
	u8 val;
	int ret;
	struct device *dev = regmap_get_device(st->map);
	unsigned int regval;

	if (fs >= INV_ICM20948_GYRO_FS_NB) {
		dev_err(dev, "陀螺仪量程设置值无效: %d\n", fs);
		return -EINVAL;
	}

	dev_info(dev, "设置陀螺仪满量程范围: %d\n", fs);

	/* 直接切换到Bank 2 */
	dev_info(dev, "切换到寄存器组2\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_2);
	if (ret) {
		dev_err(dev, "无法切换到寄存器组2: %d\n", ret);
		return ret;
	}

	/* 等待寄存器组切换完成 */
	usleep_range(2000, 3000);

	/* 读取当前配置 */
	ret = regmap_read(st->map, INV_ICM20948_REG_GYRO_CONFIG_1, &regval);
	if (ret) {
		dev_err(dev, "读取GYRO_CONFIG_1失败: %d\n", ret);
		/* 恢复到Bank 0 */
		regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
		return ret;
	}
	val = (u8)regval;
	dev_info(dev, "GYRO_CONFIG_1当前值: 0x%02x\n", val);

	/* 清除并设置量程位 */
	val &= ~INV_ICM20948_GYRO_FSR_MASK;
	val |= (fs << INV_ICM20948_GYRO_FSR_SHIFT) & INV_ICM20948_GYRO_FSR_MASK;
	dev_info(dev, "新的GYRO_CONFIG_1值: 0x%02x\n", val);

	/* 写入新的配置 */
	ret = regmap_write(st->map, INV_ICM20948_REG_GYRO_CONFIG_1, val);
	if (ret) {
		dev_err(dev, "写入GYRO_CONFIG_1失败: %d\n", ret);
		/* 恢复到Bank 0 */
		regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
		return ret;
	}

	/* 等待写入完成 */
	usleep_range(1000, 2000);

	/* 恢复到Bank 0 */
	dev_info(dev, "恢复到寄存器组0\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
	if (ret) {
		dev_err(dev, "无法恢复到寄存器组0: %d\n", ret);
		return ret;
	}

	/* 等待寄存器组切换完成 */
	usleep_range(1000, 2000);

	dev_info(dev, "陀螺仪满量程设置完成\n");
	return 0;
}

/**
 * inv_icm20948_set_accel_fs - Set accelerometer full-scale range
 * @st: Driver state
 * @fs: Full-scale range index
 *
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_set_accel_fs(struct inv_icm20948_state *st, int fs)
{
	u8 val;
	int ret;
	struct device *dev = regmap_get_device(st->map);
	unsigned int regval;

	if (fs >= INV_ICM20948_ACCEL_FS_NB) {
		dev_err(dev, "加速度计量程设置值无效: %d\n", fs);
		return -EINVAL;
	}

	dev_info(dev, "设置加速度计满量程范围: %d\n", fs);

	/* 直接切换到Bank 2 */
	dev_info(dev, "切换到寄存器组2\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_2);
	if (ret) {
		dev_err(dev, "无法切换到寄存器组2: %d\n", ret);
		return ret;
	}

	/* 等待寄存器组切换完成 */
	usleep_range(2000, 3000);

	/* 读取当前配置 */
	ret = regmap_read(st->map, INV_ICM20948_REG_ACCEL_CONFIG, &regval);
	if (ret) {
		dev_err(dev, "读取ACCEL_CONFIG失败: %d\n", ret);
		/* 恢复到Bank 0 */
		regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
		return ret;
	}
	val = (u8)regval;
	dev_info(dev, "ACCEL_CONFIG当前值: 0x%02x\n", val);

	/* 清除并设置量程位 */
	val &= ~INV_ICM20948_ACCEL_FSR_MASK;
	val |= (fs << INV_ICM20948_ACCEL_FSR_SHIFT) & INV_ICM20948_ACCEL_FSR_MASK;
	dev_info(dev, "新的ACCEL_CONFIG值: 0x%02x\n", val);

	/* 写入新的配置 */
	ret = regmap_write(st->map, INV_ICM20948_REG_ACCEL_CONFIG, val);
	if (ret) {
		dev_err(dev, "写入ACCEL_CONFIG失败: %d\n", ret);
		/* 恢复到Bank 0 */
		regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
		return ret;
	}

	/* 等待写入完成 */
	usleep_range(1000, 2000);

	/* 恢复到Bank 0 */
	dev_info(dev, "恢复到寄存器组0\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
	if (ret) {
		dev_err(dev, "无法恢复到寄存器组0: %d\n", ret);
		return ret;
	}

	/* 等待寄存器组切换完成 */
	usleep_range(1000, 2000);

	dev_info(dev, "加速度计满量程设置完成\n");
	return 0;
}

/**
 * inv_icm20948_set_sensor_rate - Set sensor sample rate
 * @st: Driver state
 * @type: Sensor type (0 for gyro, 1 for accel)
 * @rate: Desired rate in Hz
 *
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_set_sensor_rate(struct inv_icm20948_state *st, int type, int rate)
{
	u8 div;
	int ret;
	unsigned int regval;
	struct device *dev = regmap_get_device(st->map);
	u8 reg;

	dev_info(dev, "直接设置传感器采样率: type=%d, rate=%dHz\n", type, rate);

	/* 计算分频值 */
	div = 1125 / rate - 1;  /* 1125Hz内部采样率 */
	if (div > 255)
		div = 255;
	if (div < 0)
		div = 0;

	dev_info(dev, "计算的分频值: %d (0x%02x)\n", div, div);

	if (type == 0) {  /* 陀螺仪 */
		reg = INV_ICM20948_REG_GYRO_SMPLRT_DIV;
		dev_info(dev, "设置陀螺仪采样率，寄存器: 0x%02x\n", reg);
	} else if (type == 1) {  /* 加速度计 */
		reg = INV_ICM20948_REG_ACCEL_SMPLRT_DIV_1;
		dev_info(dev, "设置加速度计采样率，寄存器: 0x%02x\n", reg);
	} else {
		dev_err(dev, "未知的传感器类型: %d\n", type);
		return -EINVAL;
	}

	/* 验证通信 */
	/* 1. 切换到Bank 0，读取WHO_AM_I */
	dev_info(dev, "切换到Bank 0读取WHO_AM_I\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
	if (ret) {
		dev_err(dev, "切换到Bank 0失败: %d\n", ret);
		return ret;
	}
	msleep(30);

	ret = regmap_read(st->map, INV_ICM20948_REG_WHO_AM_I, &regval);
	if (ret) {
		dev_err(dev, "读取WHO_AM_I失败: %d\n", ret);
		return ret;
	}
	dev_info(dev, "WHO_AM_I值: 0x%02x (应为0xEA)\n", regval);

	/* 2. 切换到Bank 2 */
	dev_info(dev, "切换到Bank 2\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x20);
	if (ret) {
		dev_err(dev, "切换到Bank 2失败: %d\n", ret);
		return ret;
	}
	msleep(30);

	/* 3. 特别重要：正确配置FCHOICE和DLPF_CFG */
	if (type == 0) {  /* 陀螺仪 */
		ret = regmap_read(st->map, INV_ICM20948_REG_BANK_SEL, &regval);
		if (ret) {
			dev_err(dev, "读取BANK_SEL失败: %d\n", ret);
			return ret;
		}
		dev_info(dev, "当前 BANK_SEL: 0x%02x (期望 0x20)\n", regval);
		/* 读取现有的GYRO_CONFIG_1值 */
		ret = regmap_read(st->map, INV_ICM20948_REG_GYRO_CONFIG_1, &regval);
		if (ret) {
			dev_err(dev, "读取GYRO_CONFIG_1失败: %d\n", ret);
			regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
			return ret;
		}
		dev_info(dev, "当前GYRO_CONFIG_1值: 0x%02x\n", regval);

		/* 设置FCHOICE=1 (bit0=0)，这样SMPLRT_DIV才会生效 */
		regval &= ~INV_ICM20948_GYRO_FCHOICE_MASK; /* 清除FCHOICE位 */

		dev_info(dev, "写入GYRO_CONFIG_1: 0x%02x（保持FSR并设置FCHOICE=1）\n", regval);
		ret = regmap_write(st->map, INV_ICM20948_REG_GYRO_CONFIG_1, regval);
		if (ret) {
			dev_err(dev, "设置GYRO_CONFIG_1失败: %d\n", ret);
			regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
			return ret;
		}
		msleep(30);

		/* 配置GYRO_CONFIG_2，设置DLPF_CFG (应在1-6之间) */
		ret = regmap_read(st->map, INV_ICM20948_REG_GYRO_CONFIG_2, &regval);
		if (ret) {
			dev_err(dev, "读取GYRO_CONFIG_2失败: %d\n", ret);
			regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
			return ret;
		}
		dev_info(dev, "当前GYRO_CONFIG_2值: 0x%02x\n", regval);

		/* 设置DLPF_CFG为1-6之间的值 */
		regval &= ~INV_ICM20948_GYRO_LPF_MASK;
		regval |= 1 & INV_ICM20948_GYRO_LPF_MASK; /* 设置DLPF_CFG=1 */

		dev_info(dev, "写入GYRO_CONFIG_2: 0x%02x（设置DLPF_CFG=1）\n", regval);
		ret = regmap_write(st->map, INV_ICM20948_REG_GYRO_CONFIG_2, regval);
		if (ret) {
			dev_err(dev, "设置GYRO_CONFIG_2失败: %d\n", ret);
			regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
			return ret;
		}
		msleep(30);
	} else if (type == 1) {  /* 加速度计 */
		/* 读取现有的ACCEL_CONFIG值 */
		ret = regmap_read(st->map, INV_ICM20948_REG_ACCEL_CONFIG, &regval);
		if (ret) {
			dev_err(dev, "读取ACCEL_CONFIG失败: %d\n", ret);
			regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
			return ret;
		}
		dev_info(dev, "当前ACCEL_CONFIG值: 0x%02x\n", regval);

		/* 设置ACCEL_FCHOICE=1 (bit0=0)，这样SMPLRT_DIV才会生效 */
		regval &= ~INV_ICM20948_ACCEL_FCHOICE_MASK;

		dev_info(dev, "写入ACCEL_CONFIG: 0x%02x（保持FSR并设置FCHOICE=1）\n", regval);
		ret = regmap_write(st->map, INV_ICM20948_REG_ACCEL_CONFIG, regval);
		if (ret) {
			dev_err(dev, "设置ACCEL_CONFIG失败: %d\n", ret);
			regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
			return ret;
		}
		msleep(30);

		/* 配置ACCEL_CONFIG_2，设置ACCEL_DLPFCFG（应在1-6之间） */
		ret = regmap_read(st->map, INV_ICM20948_REG_ACCEL_CONFIG_2, &regval);
		if (ret) {
			dev_err(dev, "读取ACCEL_CONFIG_2失败: %d\n", ret);
			regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
			return ret;
		}
		dev_info(dev, "当前ACCEL_CONFIG_2值: 0x%02x\n", regval);

		/* 设置ACCEL_DLPFCFG为1-6之间的值 */
		regval &= ~INV_ICM20948_ACCEL_LPF_MASK;
		regval |= 1 & INV_ICM20948_ACCEL_LPF_MASK; /* 设置DLPFCFG=1 */

		dev_info(dev, "写入ACCEL_CONFIG_2: 0x%02x（设置DLPFCFG=1）\n", regval);
		ret = regmap_write(st->map, INV_ICM20948_REG_ACCEL_CONFIG_2, regval);
		if (ret) {
			dev_err(dev, "设置ACCEL_CONFIG_2失败: %d\n", ret);
			regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
			return ret;
		}
		msleep(30);
	}

	/* 4. 写入分频值 */
	dev_info(dev, "写入分频值0x%02x到寄存器0x%02x\n", div, reg);
	ret = regmap_write(st->map, reg, div);
	if (ret) {
		dev_err(dev, "写入分频值失败: %d\n", ret);
		regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
		return ret;
	}

	/* 更新状态结构体中的采样率字段 */
	if (type == 0) {
		st->conf.gyro.odr = div;
		dev_info(dev, "更新陀螺仪采样率字段: odr=%d (实际采样率: %dHz)\n", div, 1125/(div+1));
	} else if (type == 1) {
		st->conf.accel.odr = div;
		dev_info(dev, "更新加速度计采样率字段: odr=%d (实际采样率: %dHz)\n", div, 1125/(div+1));
	}

	msleep(30);

	/* 5. 读回分频值进行验证 */
	ret = regmap_read(st->map, reg, &regval);
	if (ret) {
		dev_err(dev, "读回分频值失败: %d\n", ret);
	} else {
		dev_info(dev, "读回的分频值: 0x%02x (预期: 0x%02x)\n", regval, div);
	}

	/* 6. 如果是加速度计，写入第二个寄存器 */
	if (type == 1) {
		dev_info(dev, "写入加速度计分频低字节0x00到寄存器0x%02x\n",
			INV_ICM20948_REG_ACCEL_SMPLRT_DIV_2);
		ret = regmap_write(st->map, INV_ICM20948_REG_ACCEL_SMPLRT_DIV_2, 0x00);
		if (ret) {
			dev_err(dev, "写入加速度计低字节分频值失败: %d\n", ret);
		} else {
			dev_info(dev, "写入加速度计低字节分频值成功\n");
		}
		msleep(30);
	}

	/* 7. 恢复到Bank 0 */
	dev_info(dev, "恢复到Bank 0\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, 0x00);
	if (ret) {
		dev_err(dev, "恢复到Bank 0失败: %d\n", ret);
		return ret;
	}
	msleep(30);

	dev_info(dev, "传感器采样率设置完成\n");
	return 0;
}

/**
 * inv_icm20948_setup_mag - Setup magnetometer I2C master
 * @st: Driver state
 *
 * Configure I2C master to communicate with AK09916 magnetometer.
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_setup_mag(struct inv_icm20948_state *st)
{
	int ret;
	u8 val;

	/* Configure I2C master clock frequency */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_MST_CTRL,
				    INV_ICM20948_I2C_MST_CLK_400KHZ);
	if (ret)
		return ret;

	/* Configure slave 0 - read from magnetometer */
	/* Set slave address with read bit */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_ADDR,
				    INV_ICM20948_MAG_I2C_ADDR | INV_ICM20948_I2C_SLV0_RNW);
	if (ret)
		return ret;

	/* Set register to read from - magnetometer data */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_REG,
				    INV_ICM20948_REG_MAG_ST1);
	if (ret)
		return ret;

	/* Enable reading from magnetometer, read 8 bytes */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_CTRL,
				    INV_ICM20948_I2C_SLV0_EN | 0x08);
	if (ret)
		return ret;

	/* Put magnetometer in continuous mode 4 (100Hz) */
	/* Set slave address for write */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_ADDR,
				    INV_ICM20948_MAG_I2C_ADDR);
	if (ret)
		return ret;

	/* Set register to write to */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_REG,
				    INV_ICM20948_REG_MAG_CNTL2);
	if (ret)
		return ret;

	/* Set value to write */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_DO,
				    INV_ICM20948_MAG_MODE_CONT4);
	if (ret)
		return ret;

	/* Enable write */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_CTRL,
				    INV_ICM20948_I2C_SLV0_EN | 0x01);
	if (ret)
		return ret;

	/* Wait for write to complete */
	msleep(10);

	/* Set back to read mode */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_ADDR,
				    INV_ICM20948_MAG_I2C_ADDR | INV_ICM20948_I2C_SLV0_RNW);
	if (ret)
		return ret;

	/* Set register to read from */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_REG,
				    INV_ICM20948_REG_MAG_ST1);
	if (ret)
		return ret;

	/* Enable reading, read 8 bytes */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_CTRL,
				    INV_ICM20948_I2C_SLV0_EN | 0x08);
	if (ret)
		return ret;

	st->mag_initialized = true;

	return 0;
}

/**
 * inv_icm20948_configure_fifo - Configure FIFO
 * @st: Driver state
 * @enable: Enable or disable FIFO
 *
 * Configure and enable/disable FIFO.
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_configure_fifo(struct inv_icm20948_state *st, bool enable)
{
	int ret;

	if (!enable) {
		/* Disable FIFO */
		ret = regmap_update_bits(st->map, INV_ICM20948_REG_USER_CTRL,
					INV_ICM20948_BIT_FIFO_EN, 0);
		return ret;
	}

	/* Reset FIFO */
	ret = regmap_update_bits(st->map, INV_ICM20948_REG_USER_CTRL,
				INV_ICM20948_BIT_FIFO_RST,
				INV_ICM20948_BIT_FIFO_RST);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->map, INV_ICM20948_REG_USER_CTRL,
				INV_ICM20948_BIT_FIFO_RST, 0);
	if (ret)
		return ret;

	/* Configure FIFO to receive accel, gyro, and possibly mag data */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_0,
				    INV_ICM20948_REG_FIFO_EN_2,
				    INV_ICM20948_FIFO_GYRO_EN_ALL |
				    INV_ICM20948_BIT_ACCEL_FIFO_EN);
	if (ret)
		return ret;

	if (st->mag_initialized) {
		ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_0,
					    INV_ICM20948_REG_FIFO_EN_1,
					    INV_ICM20948_BIT_SLV_0_FIFO_EN);
		if (ret)
			return ret;
	}

	/* Enable FIFO */
	ret = regmap_update_bits(st->map, INV_ICM20948_REG_USER_CTRL,
				INV_ICM20948_BIT_FIFO_EN,
				INV_ICM20948_BIT_FIFO_EN);
	if (ret)
		return ret;

	st->use_fifo = true;

	return 0;
}

int inv_icm20948_set_power_mode(struct inv_icm20948_state *st,
				   enum inv_icm20948_sensor_mode mode)
{
	int ret;
	unsigned int val, read_val;
	struct device *dev = regmap_get_device(st->map);

	/* 确保在Bank 0中操作 */
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
	if (ret) {
		dev_err(dev, "无法设置寄存器组0: %d\n", ret);
		return ret;
	}

	/* 添加延迟以确保寄存器组切换完成 */
	usleep_range(1000, 2000);

	switch (mode) {
	case INV_ICM20948_SENSOR_MODE_OFF:
		val = INV_ICM20948_BIT_SLEEP;
		dev_info(dev, "设置电源模式: 睡眠模式\n");
		break;
	case INV_ICM20948_SENSOR_MODE_STANDBY:
		val = 0;
		dev_info(dev, "设置电源模式: 待机模式\n");
		break;
	case INV_ICM20948_SENSOR_MODE_LOW_POWER:
		val = INV_ICM20948_BIT_LP_EN;
		dev_info(dev, "设置电源模式: 低功耗模式\n");
		break;
	case INV_ICM20948_SENSOR_MODE_LOW_NOISE:
		val = 0;
		dev_info(dev, "设置电源模式: 低噪声模式\n");
		break;
	default:
		dev_err(dev, "无效的电源模式: %d\n", mode);
		return -EINVAL;
	}

	/* 读取当前值 */
	ret = regmap_read(st->map, INV_ICM20948_REG_PWR_MGMT_1, &read_val);
	if (ret) {
		dev_err(dev, "读取PWR_MGMT_1失败: %d\n", ret);
		return ret;
	}
	dev_info(dev, "PWR_MGMT_1当前值: 0x%02x\n", read_val);

	/* 保留时钟源设置，仅修改电源管理位 */
	read_val &= INV_ICM20948_BIT_CLKSEL_MASK;
	val |= read_val;

	/* 直接写入寄存器值 */
	ret = regmap_write(st->map, INV_ICM20948_REG_PWR_MGMT_1, val);
	if (ret) {
		dev_err(dev, "设置电源模式失败: %d\n", ret);
		return ret;
	}

	/* 验证设置 */
	ret = regmap_read(st->map, INV_ICM20948_REG_PWR_MGMT_1, &read_val);
	if (ret) {
		dev_err(dev, "验证PWR_MGMT_1失败: %d\n", ret);
		return ret;
	}
	dev_info(dev, "PWR_MGMT_1设置后的值: 0x%02x\n", read_val);

	/* 等待电源模式稳定 */
	msleep(100);

	return 0;
}

int inv_icm20948_set_clock_source(struct inv_icm20948_state *st)
{
	int ret;
	struct device *dev = regmap_get_device(st->map);
	unsigned int val;

	/* 确保在Bank 0中操作 */
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
	if (ret) {
		dev_err(dev, "无法设置寄存器组0: %d\n", ret);
		return ret;
	}

	/* 添加延迟以确保寄存器组切换完成 */
	usleep_range(1000, 2000);

	/* 读取当前值 */
	ret = regmap_read(st->map, INV_ICM20948_REG_PWR_MGMT_1, &val);
	if (ret) {
		dev_err(dev, "读取PWR_MGMT_1失败: %d\n", ret);
		return ret;
	}
	dev_info(dev, "PWR_MGMT_1当前值: 0x%02x\n", val);

	/* 清除当前时钟位并设置为内部振荡器 */
	val &= ~INV_ICM20948_BIT_CLKSEL_MASK; /* 清除时钟选择位 */

	/* 直接写入整个寄存器 */
	ret = regmap_write(st->map, INV_ICM20948_REG_PWR_MGMT_1, val);
	if (ret) {
		dev_err(dev, "设置内部时钟源失败: %d\n", ret);
		return ret;
	}

	/* 验证设置 */
	ret = regmap_read(st->map, INV_ICM20948_REG_PWR_MGMT_1, &val);
	if (ret) {
		dev_err(dev, "验证PWR_MGMT_1失败: %d\n", ret);
		return ret;
	}
	dev_info(dev, "PWR_MGMT_1设置后的值: 0x%02x\n", val);

	/* 添加延迟以确保时钟源已稳定 */
	msleep(20);

	return 0;
}

/* 工作队列处理函数 - 在非中断上下文中处理中断 */
static void inv_icm20948_irq_work_fn(struct work_struct *work)
{
    struct inv_icm20948_state *st = container_of(work, struct inv_icm20948_state, irq_work);
    unsigned int status, status_1;
    int ret;
    struct device *dev = regmap_get_device(st->map);

    /* 确保在Bank 0中操作 */
    ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
    if (ret) {
        dev_err(dev, "工作队列中切换到Bank 0失败: %d\n", ret);
        return;
    }
    msleep(10);  // 充分等待切换完成

    /* 读取中断状态寄存器 */
    ret = regmap_read(st->map, INV_ICM20948_REG_INT_STATUS, &status);
    if (ret) {
        dev_err(dev, "工作队列中读取INT_STATUS失败: %d\n", ret);
        return;
    }

    /* 读取INT_STATUS_1寄存器 - 数据就绪中断在此寄存器中 */
    ret = regmap_read(st->map, INV_ICM20948_REG_INT_STATUS_1, &status_1);
    if (ret) {
        dev_err(dev, "工作队列中读取INT_STATUS_1失败: %d\n", ret);
        return;
    }

    dev_info(dev, "工作队列处理中断: INT_STATUS=0x%02x, INT_STATUS_1=0x%02x\n", status, status_1);

    /* 检查数据就绪中断标志 */
    if (status_1 & INV_ICM20948_BIT_RAW_DATA_RDY_INT) {
        dev_info(dev, "工作队列中检测到数据就绪中断\n");
        if (st->trig) {
            iio_trigger_poll(st->trig);
        }
    }
}

/* Handle the IRQ */
static irqreturn_t inv_icm20948_irq_handler(int irq, void *private)
{
    struct inv_icm20948_state *st = private;
    
    /* 只需将处理工作提交到工作队列，不在中断上下文中执行I2C操作 */
    schedule_work(&st->irq_work);
    
    return IRQ_HANDLED;
}

int inv_icm20948_check_magn(struct inv_icm20948_state *st)
{
	u8 val1, val2;
	unsigned int regval1, regval2;
	int ret;
    struct device *dev = regmap_get_device(st->map);

	/* Check if magnetometer is present by reading its ID registers */
	/* First we need to configure I2C master */
	ret = regmap_update_bits(st->map, INV_ICM20948_REG_USER_CTRL,
				INV_ICM20948_BIT_I2C_MST_EN,
				INV_ICM20948_BIT_I2C_MST_EN);
	if (ret)
		return ret;
    dev_info(dev, "准备写入I2C_MST_CTRL, 切换Bank到0x30\n");

	/* Configure I2C master clock frequency */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_MST_CTRL,
				    INV_ICM20948_I2C_MST_CLK_400KHZ);
	if (ret)
		return ret;
    dev_err(dev, "6666666666%d\n");

	/* Set slave address with read bit */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_ADDR,
				    INV_ICM20948_MAG_I2C_ADDR | INV_ICM20948_I2C_SLV0_RNW);
	if (ret)
		return ret;

	/* Set register to read from - WIA1 */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_REG,
				    INV_ICM20948_REG_MAG_WIA1);
	if (ret)
		return ret;

	/* Enable reading, read 2 bytes (WIA1 and WIA2) */
	ret = inv_icm20948_write_reg(st, INV_ICM20948_BANK_3,
				    INV_ICM20948_REG_I2C_SLV0_CTRL,
				    INV_ICM20948_I2C_SLV0_EN | 0x02);
	if (ret)
		return ret;

	/* Wait for I2C transaction to complete */
	msleep(20);

	/* Read the values */
	ret = regmap_read(st->map, INV_ICM20948_REG_EXT_SLV_SENS_DATA_00, &regval1);
	if (ret)
		return ret;

	ret = regmap_read(st->map, INV_ICM20948_REG_EXT_SLV_SENS_DATA_01, &regval2);
	if (ret)
		return ret;

	val1 = (u8)regval1;
	val2 = (u8)regval2;

	if (val1 != INV_ICM20948_MAG_WIA1_VAL || val2 != INV_ICM20948_MAG_WIA2_VAL) {
		dev_info(regmap_get_device(st->map),
			"Magnetometer not detected or unsupported (0x%02x, 0x%02x)\n",
			val1, val2);
		return -ENODEV;
	}

	return 0;
}

/**
 * inv_icm20948_clear_interrupts - Clear all interrupt flags
 * @st: Driver state
 *
 * Clears all interrupt flags by reading the INT_STATUS register.
 * Must be called with @st->lock held.
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm20948_clear_interrupts(struct inv_icm20948_state *st)
{
	unsigned int status, status_1, status_2, status_3;
	int ret;
	struct device *dev = regmap_get_device(st->map);

	/* 确保在Bank 0中操作 */
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
	if (ret) {
		dev_err(dev, "无法设置寄存器组0: %d\n", ret);
		return ret;
	}
	msleep(10);

	/* 读取所有中断状态寄存器以清除中断标志 */
	ret = regmap_read(st->map, INV_ICM20948_REG_INT_STATUS, &status);
	if (ret) {
		dev_err(dev, "读取INT_STATUS失败: %d\n", ret);
		return ret;
	}
	
	ret = regmap_read(st->map, INV_ICM20948_REG_INT_STATUS_1, &status_1);
	if (ret) {
		dev_err(dev, "读取INT_STATUS_1失败: %d\n", ret);
		return ret;
	}
	
	ret = regmap_read(st->map, INV_ICM20948_REG_INT_STATUS_2, &status_2);
	if (ret) {
		dev_err(dev, "读取INT_STATUS_2失败: %d\n", ret);
		return ret;
	}
	
	ret = regmap_read(st->map, INV_ICM20948_REG_INT_STATUS_3, &status_3);
	if (ret) {
		dev_err(dev, "读取INT_STATUS_3失败: %d\n", ret);
		return ret;
	}

	dev_info(dev, "清除中断状态: INT_STATUS=0x%02x, INT_STATUS_1=0x%02x, INT_STATUS_2=0x%02x, INT_STATUS_3=0x%02x\n", 
		 status, status_1, status_2, status_3);
	return 0;
}

static int inv_icm20948_init_device(struct inv_icm20948_state *st)
{
	unsigned int val;
	int ret;
	struct device *dev = regmap_get_device(st->map);

	dev_info(dev, "Initializing ICM20948 device\n");

	/* 显式设置寄存器组 0，确保初始化正确 */
	dev_info(dev, "Setting initial register bank 0\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
	if (ret) {
		dev_err(dev, "Failed to set register bank 0 directly: %d\n", ret);
		return ret;
	}

	/* 额外等待以确保寄存器组选择生效 */
	usleep_range(1000, 2000);

	/* 手动读取WHO_AM_I寄存器以验证通信 */
	ret = regmap_read(st->map, INV_ICM20948_REG_WHO_AM_I, &val);
	if (ret) {
		dev_err(dev, "Failed to read WHO_AM_I register before reset: %d\n", ret);
		return ret;
	}
	dev_info(dev, "WHO_AM_I register before reset: 0x%02x\n", val);

	/* Reset device */
	dev_info(dev, "Resetting device\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_PWR_MGMT_1,
			  INV_ICM20948_BIT_H_RESET);
	if (ret) {
		dev_err(dev, "Failed to reset device: %d\n", ret);
		return ret;
	}

	/* Wait for reset to complete */
	dev_info(dev, "Waiting for reset to complete (100ms)\n");
	msleep(100);

	/* 重置后再次选择寄存器组 0，因为复位会恢复默认状态 */
	dev_info(dev, "Setting register bank 0 after reset\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
	if (ret) {
		dev_err(dev, "Failed to set register bank 0 directly after reset: %d\n", ret);
		return ret;
	}

	/* 额外等待以确保寄存器组选择生效 */
	usleep_range(1000, 2000);

	/* Verify device ID */
	dev_info(dev, "Verifying device ID\n");
	ret = regmap_read(st->map, INV_ICM20948_REG_WHO_AM_I, &val);
	if (ret) {
		dev_err(dev, "Failed to read WHO_AM_I register: %d\n", ret);
		return ret;
	}
	dev_info(dev, "WHO_AM_I register value: 0x%02x (expected: 0x%02x)\n",
		 val, INV_ICM20948_WHO_AM_I_VAL);

	if (val != INV_ICM20948_WHO_AM_I_VAL) {
		dev_err(dev, "Invalid WHO_AM_I value 0x%02x\n", val);
		return -ENODEV;
	}

	/* Set clock source */
	dev_info(dev, "Setting clock source\n");
	ret = inv_icm20948_set_clock_source(st);
	if (ret) {
		dev_err(dev, "Failed to set clock source\n");
		return ret;
	}

	/* Configure power management */
	ret = inv_icm20948_set_power_mode(st, INV_ICM20948_SENSOR_MODE_STANDBY);
	if (ret) {
		dev_err(dev, "设置电源模式失败: %d\n", ret);
		return ret;
	}

	/* 确保寄存器组0被选中 */
	ret = regmap_write(st->map, INV_ICM20948_REG_BANK_SEL, INV_ICM20948_BANK_0);
	if (ret) {
		dev_err(dev, "无法设置寄存器组0: %d\n", ret);
		return ret;
	}
	usleep_range(1000, 2000);

	/* 读取PWR_MGMT_2当前值 */
	ret = regmap_read(st->map, INV_ICM20948_REG_PWR_MGMT_2, &val);
	if (ret) {
		dev_err(dev, "读取PWR_MGMT_2失败: %d\n", ret);
		return ret;
	}
	dev_info(dev, "PWR_MGMT_2当前值: 0x%02x\n", val);

	/* 启用所有传感器 (禁用加速度计和陀螺仪的睡眠模式) */
	dev_info(dev, "启用所有传感器轴\n");
	ret = regmap_write(st->map, INV_ICM20948_REG_PWR_MGMT_2, 0x00);
	if (ret) {
		dev_err(dev, "启用传感器失败: %d\n", ret);
		return ret;
	}

	/* 验证设置 */
	ret = regmap_read(st->map, INV_ICM20948_REG_PWR_MGMT_2, &val);
	if (ret) {
		dev_err(dev, "验证PWR_MGMT_2失败: %d\n", ret);
		return ret;
	}
	dev_info(dev, "PWR_MGMT_2设置后的值: 0x%02x\n", val);

	/* Set default sampling rates */
	/* 使用直接寄存器访问方式设置采样率 */
	dev_info(dev, "使用增强的寄存器访问方式设置采样率\n");
	ret = inv_icm20948_set_sensor_rate(st, 0, 100);  /* 陀螺仪设置为100Hz */
	if (ret)
		return ret;

	ret = inv_icm20948_set_sensor_rate(st, 1, 100);  /* 加速度计设置为100Hz */
	if (ret)
		return ret;

	/* Set default full-scale ranges */
	ret = inv_icm20948_set_gyro_fs(st, INV_ICM20948_GYRO_FS_2000DPS);
	if (ret)
		return ret;

	ret = inv_icm20948_set_accel_fs(st, INV_ICM20948_ACCEL_FS_16G);
	if (ret)
		return ret;

	/* Enable temperature sensor */
	ret = regmap_update_bits(st->map, INV_ICM20948_REG_PWR_MGMT_1,
			       INV_ICM20948_BIT_TEMP_DIS, 0);
	if (ret)
		return ret;
	st->conf.temp_en = true;

	/* 配置中断引脚 - 使用脉冲模式（移除锁存模式）*/
	ret = regmap_write(st->map, INV_ICM20948_REG_INT_PIN_CFG,
			  INV_ICM20948_BIT_INT_OPEN_DRAIN);
	if (ret)
		return ret;

	/* 先设置INT_ENABLE (0x10) - 总中断使能 */
	ret = regmap_write(st->map, INV_ICM20948_REG_INT_ENABLE, 0x01);
	if (ret)
		return ret;

	/* 设置INT_ENABLE_1 (0x11) - 数据就绪中断使能 */
	ret = regmap_write(st->map, INV_ICM20948_REG_INT_ENABLE_1,
			  INV_ICM20948_BIT_RAW_DATA_RDY_EN);
	if (ret)
		return ret;

	/* 读取并打印中断相关寄存器值，便于调试 */
	unsigned int int_pin_cfg_val, int_enable_val, int_enable_1_val;
	ret = regmap_read(st->map, INV_ICM20948_REG_INT_PIN_CFG, &int_pin_cfg_val);
	if (ret)
		dev_err(dev, "读取INT_PIN_CFG失败: %d\n", ret);
	else
		dev_info(dev, "INT_PIN_CFG设置后值: 0x%02x\n", int_pin_cfg_val);

	ret = regmap_read(st->map, INV_ICM20948_REG_INT_ENABLE, &int_enable_val);
	if (ret)
		dev_err(dev, "读取INT_ENABLE失败: %d\n", ret);
	else
		dev_info(dev, "INT_ENABLE设置后值: 0x%02x\n", int_enable_val);

	ret = regmap_read(st->map, INV_ICM20948_REG_INT_ENABLE_1, &int_enable_1_val);
	if (ret)
		dev_err(dev, "读取INT_ENABLE_1失败: %d\n", ret);
	else
		dev_info(dev, "INT_ENABLE_1设置后值: 0x%02x\n", int_enable_1_val);

	/* 清除可能存在的中断标志 */
	ret = inv_icm20948_clear_interrupts(st);
	if (ret)
		return ret;

	/* Check for magnetometer */
	ret = inv_icm20948_check_magn(st);
	if (ret == 0) {
		/* Setup magnetometer */
		ret = inv_icm20948_setup_mag(st);
		if (ret)
			return ret;
	}

	return 0;
}

int inv_icm20948_core_probe(struct regmap *regmap, int irq,
			   int (*bus_setup)(struct inv_icm20948_state *))
{
	struct inv_icm20948_state *st;
	struct device *dev = regmap_get_device(regmap);
	int ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->map = regmap;
	st->name = "icm20948";

	mutex_init(&st->lock);
	
	/* 初始化工作队列 */
	INIT_WORK(&st->irq_work, inv_icm20948_irq_work_fn);
	
	dev_set_drvdata(dev, st);

	/* Setup bus specific configuration */
	if (bus_setup) {
		ret = bus_setup(st);
		if (ret)
			return ret;
	}

	/* Initialize device */
	ret = inv_icm20948_init_device(st);
	if (ret)
		return ret;

	/* Initialize debug support */
	ret = inv_icm20948_debug_init(st);
	if (ret) {
		dev_warn(dev, "Failed to initialize debug support\n");
		/* Non-fatal error, continue without debug */
	}

	/* Initialize sensors */
	st->indio_gyro = inv_icm20948_gyro_init(st);
	if (IS_ERR(st->indio_gyro))
		return PTR_ERR(st->indio_gyro);

	st->indio_accel = inv_icm20948_accel_init(st);
	if (IS_ERR(st->indio_accel))
		return PTR_ERR(st->indio_accel);

	if (st->mag_initialized) {
		st->indio_magn = inv_icm20948_magn_init(st);
		if (IS_ERR(st->indio_magn))
			st->indio_magn = NULL;
	}

	/* Initialize temperature sensor */
	if (st->conf.temp_en) {
		st->indio_temp = inv_icm20948_temp_init(st);
		if (IS_ERR(st->indio_temp))
			st->indio_temp = NULL;
	}

	/* Initialize IIO trigger */
	st->trig = devm_iio_trigger_alloc(dev, "%s-trigger", st->name);
	if (!st->trig)
		return -ENOMEM;

	st->trig->dev.parent = dev;
	st->trig->ops = &inv_icm20948_trigger_ops;
	iio_trigger_set_drvdata(st->trig, st);
	ret = devm_iio_trigger_register(dev, st->trig);
	if (ret) {
		dev_err(dev, "Failed to register IIO trigger\n");
		return ret;
	}
	
	/* 关联触发器到每个传感器设备 */
	st->indio_gyro->trig = st->trig;
	st->indio_accel->trig = st->trig;
	if (st->indio_magn)
		st->indio_magn->trig = st->trig;
	
	/* 设置触发器 - 在注册IIO设备之前调用 */
	if (irq > 0) {
		dev_info(dev, "设置传感器触发器\n");
		ret = inv_icm20948_setup_trigger(st);
		if (ret) {
			dev_err(dev, "设置触发器失败: %d\n", ret);
			return ret;
		}
		dev_info(dev, "触发器设置成功\n");
	}
	
	/* Register IIO devices - 在设置触发器后完成 */
	ret = devm_iio_device_register(dev, st->indio_gyro);
	if (ret) {
		dev_err(dev, "Failed to register gyro IIO device\n");
		return ret;
	}

	ret = devm_iio_device_register(dev, st->indio_accel);
	if (ret) {
		dev_err(dev, "Failed to register accel IIO device\n");
		return ret;
	}

	if (st->indio_magn) {
		ret = devm_iio_device_register(dev, st->indio_magn);
		if (ret) {
			dev_err(dev, "Failed to register magn IIO device\n");
			return ret;
		}
	}

	if (st->indio_temp) {
		ret = devm_iio_device_register(dev, st->indio_temp);
		if (ret) {
			dev_err(dev, "Failed to register temp IIO device\n");
			return ret;
		}
	}
	
	/* FIFO setup if needed - 在注册中断处理程序前完成 */
	if (irq > 0) {
		ret = inv_icm20948_configure_fifo(st, true);
		if (ret) {
			dev_err(dev, "Failed to configure FIFO\n");
			/* Non-fatal error, continue without FIFO */
			st->use_fifo = false;
		}
	}
	
	/* 最后，在所有初始化完成后注册中断处理程序 */
	if (irq > 0) {
		dev_info(dev, "注册中断处理程序，IRQ: %d\n", irq);
		ret = devm_request_irq(dev, irq, inv_icm20948_irq_handler,
				      IRQF_TRIGGER_RISING, "inv_icm20948", st);
		if (ret) {
			dev_err(dev, "Failed to request IRQ %d\n", irq);
			return ret;
		}
		dev_info(dev, "中断处理程序注册成功\n");
	}

	dev_info(dev, "ICM-20948 initialized\n");

	return 0;
}

EXPORT_SYMBOL_GPL(inv_icm20948_core_probe);

static int __maybe_unused inv_icm20948_resume(struct device *dev)
{
	struct inv_icm20948_state *st = dev_get_drvdata(dev);

	return inv_icm20948_init_device(st);
}

static int __maybe_unused inv_icm20948_suspend(struct device *dev)
{
	struct inv_icm20948_state *st = dev_get_drvdata(dev);

	return inv_icm20948_set_power_mode(st, INV_ICM20948_SENSOR_MODE_OFF);
}

const struct dev_pm_ops inv_icm20948_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(inv_icm20948_suspend, inv_icm20948_resume)
};
EXPORT_SYMBOL_GPL(inv_icm20948_pm_ops);

MODULE_AUTHOR("Invensense, Inc.");
MODULE_DESCRIPTION("Invensense ICM-20948 driver");
MODULE_LICENSE("GPL v2");
