# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Invensense ICM-20948 IMU driver
#

# 主模块: inv_icm20948.ko
obj-$(CONFIG_INV_ICM20948) += inv_icm20948.o

# 主模块组件
inv_icm20948-objs := inv_icm20948_core.o inv_icm20948_buffer.o \
                     inv_icm20948_gyro.o inv_icm20948_accel.o 

# 可选组件
inv_icm20948-$(CONFIG_INV_ICM20948_MAGNETOMETER) += inv_icm20948_magn.o
inv_icm20948-$(CONFIG_INV_ICM20948_TEMPERATURE) += inv_icm20948_temp.o
inv_icm20948-$(CONFIG_INV_ICM20948_DEBUG) += inv_icm20948_debug.o

# I2C和SPI支持
inv_icm20948-$(CONFIG_INV_ICM20948_I2C) += inv_icm20948_i2c.o
inv_icm20948-$(CONFIG_INV_ICM20948_SPI) += inv_icm20948_spi.o