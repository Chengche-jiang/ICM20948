/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024 Invensense, Inc.
 */

#ifndef INV_ICM20948_H_
#define INV_ICM20948_H_

/* 删除 linux/bits.h 引用，并添加必要的宏定义 */
/* #include <linux/bits.h> - 不兼容Linux 4.9 */
#include <linux/bitfield.h>

/* 添加GENMASK定义，替代bits.h中的功能 */
#ifndef GENMASK
#define GENMASK(h, l) \
     (((~0UL) << (l)) & (~0UL >> (BITS_PER_LONG - 1 - (h))))
#endif

#ifndef GENMASK_ULL
#define GENMASK_ULL(h, l) \
     (((~0ULL) << (l)) & (~0ULL >> (BITS_PER_LONG_LONG - 1 - (h))))
#endif

/* 添加IIO_DMA_MINALIGN定义，在Linux 4.9中不存在 */
#ifndef IIO_DMA_MINALIGN
#define IIO_DMA_MINALIGN (8)
#endif

#include <linux/regmap.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/pm.h>
#include <linux/iio/iio.h>
#include "inv_sensors_timestamp.h"

/* Bank selection register, available in all banks */
#define INV_ICM20948_REG_BANK_SEL		0x7F
#define INV_ICM20948_BANK_SEL_MASK    GENMASK(5, 4)
#define INV_ICM20948_BANK_0			0x00
#define INV_ICM20948_BANK_1			0x10
#define INV_ICM20948_BANK_2			0x20
#define INV_ICM20948_BANK_3			0x30

/* Bank 0 registers */
#define INV_ICM20948_REG_WHO_AM_I		0x00
#define INV_ICM20948_WHO_AM_I_VAL		0xEA

#define INV_ICM20948_REG_USER_CTRL		0x03
#define INV_ICM20948_BIT_DMP_EN			BIT(7)
#define INV_ICM20948_BIT_FIFO_EN		BIT(6)
#define INV_ICM20948_BIT_I2C_MST_EN		BIT(5)
#define INV_ICM20948_BIT_I2C_IF_DIS		BIT(4)
#define INV_ICM20948_BIT_DMP_RST		BIT(3)
#define INV_ICM20948_BIT_FIFO_RST		BIT(2)
#define INV_ICM20948_BIT_I2C_MST_RST		BIT(1)
#define INV_ICM20948_BIT_SIG_COND_RST		BIT(0)

#define INV_ICM20948_REG_LP_CONFIG		0x05
#define INV_ICM20948_BIT_I2C_MST_CYCLE		BIT(6)
#define INV_ICM20948_BIT_ACCEL_CYCLE		BIT(5)
#define INV_ICM20948_BIT_GYRO_CYCLE		BIT(4)

#define INV_ICM20948_REG_PWR_MGMT_1		0x06
#define INV_ICM20948_BIT_H_RESET		BIT(7)
#define INV_ICM20948_BIT_SLEEP			BIT(6)
#define INV_ICM20948_BIT_LP_EN			BIT(5)
#define INV_ICM20948_BIT_TEMP_DIS		BIT(3)
#define INV_ICM20948_BIT_CLKSEL_MASK		GENMASK(2, 0)
#define INV_ICM20948_CLKSEL_AUTO		0x01

#define INV_ICM20948_REG_PWR_MGMT_2		0x07
#define INV_ICM20948_BIT_PWR_ACCEL_MASK		GENMASK(2, 0)
#define INV_ICM20948_BIT_PWR_ACCEL_OFF		0x07
#define INV_ICM20948_BIT_PWR_GYRO_MASK		GENMASK(5, 3)
#define INV_ICM20948_BIT_PWR_GYRO_OFF		0x07

#define INV_ICM20948_REG_INT_PIN_CFG		0x0F
#define INV_ICM20948_BIT_INT_LATCH_EN		BIT(5)
#define INV_ICM20948_BIT_INT_OPEN_DRAIN		BIT(4)
#define INV_ICM20948_BIT_INT_ACTIVE_LOW		BIT(7)
#define INV_ICM20948_BIT_BYPASS_EN		BIT(1)

#define INV_ICM20948_REG_INT_ENABLE		0x10
#define INV_ICM20948_BIT_RAW_DATA_0_RDY_EN	BIT(0)

#define INV_ICM20948_REG_INT_ENABLE_1		0x11
#define INV_ICM20948_BIT_RAW_DATA_RDY_EN	BIT(0)  /* 数据就绪中断使能位 */
#define INV_ICM20948_REG_INT_ENABLE_2		0x12
#define INV_ICM20948_REG_INT_ENABLE_3		0x13

#define INV_ICM20948_REG_INT_STATUS		0x19
#define INV_ICM20948_BIT_RAW_DATA_0_RDY_INT	BIT(0)

#define INV_ICM20948_REG_INT_STATUS_1		0x1A
#define INV_ICM20948_BIT_RAW_DATA_RDY_INT	BIT(0)  /* 数据就绪中断状态位 */
#define INV_ICM20948_REG_INT_STATUS_2		0x1B
#define INV_ICM20948_REG_INT_STATUS_3		0x1C

#define INV_ICM20948_REG_ACCEL_XOUT_H		0x2D
#define INV_ICM20948_REG_ACCEL_XOUT_L		0x2E
#define INV_ICM20948_REG_ACCEL_YOUT_H		0x2F
#define INV_ICM20948_REG_ACCEL_YOUT_L		0x30
#define INV_ICM20948_REG_ACCEL_ZOUT_H		0x31
#define INV_ICM20948_REG_ACCEL_ZOUT_L		0x32

#define INV_ICM20948_REG_GYRO_XOUT_H		0x33
#define INV_ICM20948_REG_GYRO_XOUT_L		0x34
#define INV_ICM20948_REG_GYRO_YOUT_H		0x35
#define INV_ICM20948_REG_GYRO_YOUT_L		0x36
#define INV_ICM20948_REG_GYRO_ZOUT_H		0x37
#define INV_ICM20948_REG_GYRO_ZOUT_L		0x38

#define INV_ICM20948_REG_TEMP_OUT_H		0x39
#define INV_ICM20948_REG_TEMP_OUT_L		0x3A

#define INV_ICM20948_REG_EXT_SLV_SENS_DATA_00	0x3B
#define INV_ICM20948_REG_EXT_SLV_SENS_DATA_01	0x3C
#define INV_ICM20948_REG_EXT_SLV_SENS_DATA_02	0x3D
#define INV_ICM20948_REG_EXT_SLV_SENS_DATA_03	0x3E
#define INV_ICM20948_REG_EXT_SLV_SENS_DATA_04	0x3F
#define INV_ICM20948_REG_EXT_SLV_SENS_DATA_05	0x40
#define INV_ICM20948_REG_EXT_SLV_SENS_DATA_06	0x41

#define INV_ICM20948_REG_FIFO_EN_1		0x66
#define INV_ICM20948_BIT_SLV_0_FIFO_EN		BIT(0)
#define INV_ICM20948_BIT_SLV_1_FIFO_EN		BIT(1)
#define INV_ICM20948_BIT_SLV_2_FIFO_EN		BIT(2)
#define INV_ICM20948_BIT_SLV_3_FIFO_EN		BIT(3)

#define INV_ICM20948_REG_FIFO_EN_2		0x67
#define INV_ICM20948_BIT_TEMP_FIFO_EN		BIT(0)
#define INV_ICM20948_BIT_GYRO_X_FIFO_EN		BIT(1)
#define INV_ICM20948_BIT_GYRO_Y_FIFO_EN		BIT(2)
#define INV_ICM20948_BIT_GYRO_Z_FIFO_EN		BIT(3)
#define INV_ICM20948_BIT_ACCEL_FIFO_EN		BIT(4)
#define INV_ICM20948_FIFO_GYRO_EN_ALL		(INV_ICM20948_BIT_GYRO_X_FIFO_EN | \
						 INV_ICM20948_BIT_GYRO_Y_FIFO_EN | \
						 INV_ICM20948_BIT_GYRO_Z_FIFO_EN)

#define INV_ICM20948_REG_FIFO_RST		0x68
#define INV_ICM20948_REG_FIFO_COUNT_H		0x70
#define INV_ICM20948_REG_FIFO_COUNT_L		0x71
#define INV_ICM20948_REG_FIFO_R_W		0x72

/* Bank 2 registers */
#define INV_ICM20948_REG_GYRO_SMPLRT_DIV	0x00
#define INV_ICM20948_REG_GYRO_CONFIG_1		0x01
#define INV_ICM20948_GYRO_FSR_MASK		GENMASK(2, 1)
#define INV_ICM20948_GYRO_FSR_SHIFT		1
#define INV_ICM20948_GYRO_FCHOICE_MASK		BIT(0)
#define INV_ICM20948_GYRO_FCHOICE_SHIFT		0

#define INV_ICM20948_REG_GYRO_CONFIG_2		0x02
#define INV_ICM20948_GYRO_LPF_MASK		GENMASK(2, 0)

#define INV_ICM20948_REG_ACCEL_SMPLRT_DIV_1	0x10
#define INV_ICM20948_REG_ACCEL_SMPLRT_DIV_2	0x11

#define INV_ICM20948_REG_ACCEL_CONFIG		0x14
#define INV_ICM20948_ACCEL_FSR_MASK		GENMASK(2, 1)
#define INV_ICM20948_ACCEL_FSR_SHIFT		1
#define INV_ICM20948_ACCEL_FCHOICE_MASK		BIT(0)
#define INV_ICM20948_ACCEL_FCHOICE_SHIFT	0

#define INV_ICM20948_REG_ACCEL_CONFIG_2		0x15
#define INV_ICM20948_ACCEL_LPF_MASK		GENMASK(2, 0)

/* Bank 3 registers - I2C master */
#define INV_ICM20948_REG_I2C_MST_CTRL		0x01
#define INV_ICM20948_I2C_MST_CLK_MASK		GENMASK(3, 0)
#define INV_ICM20948_I2C_MST_CLK_400KHZ		0x07

#define INV_ICM20948_REG_I2C_SLV0_ADDR		0x03
#define INV_ICM20948_I2C_SLV0_RNW		BIT(7)

#define INV_ICM20948_REG_I2C_SLV0_REG		0x04
#define INV_ICM20948_REG_I2C_SLV0_CTRL		0x05
#define INV_ICM20948_I2C_SLV0_EN		BIT(7)
#define INV_ICM20948_I2C_SLV0_LENG_MASK		GENMASK(3, 0)

#define INV_ICM20948_REG_I2C_SLV0_DO		0x06

/* AK09916 magnetometer registers and values */
#define INV_ICM20948_MAG_I2C_ADDR		0x0C

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

/* Sensor modes */
enum inv_icm20948_sensor_mode {
	INV_ICM20948_SENSOR_MODE_OFF,
	INV_ICM20948_SENSOR_MODE_STANDBY,
	INV_ICM20948_SENSOR_MODE_LOW_POWER,
	INV_ICM20948_SENSOR_MODE_LOW_NOISE,
	INV_ICM20948_SENSOR_MODE_NB,
};

/* Gyroscope full scale values */
enum inv_icm20948_gyro_fs {
	INV_ICM20948_GYRO_FS_250DPS,  /* 00 */
	INV_ICM20948_GYRO_FS_500DPS,  /* 01 */
	INV_ICM20948_GYRO_FS_1000DPS, /* 10 */
	INV_ICM20948_GYRO_FS_2000DPS, /* 11 */
	INV_ICM20948_GYRO_FS_NB,
};

/* Accelerometer full scale values */
enum inv_icm20948_accel_fs {
	INV_ICM20948_ACCEL_FS_2G,  /* 00 */
	INV_ICM20948_ACCEL_FS_4G,  /* 01 */
	INV_ICM20948_ACCEL_FS_8G,  /* 10 */
	INV_ICM20948_ACCEL_FS_16G, /* 11 */
	INV_ICM20948_ACCEL_FS_NB,
};

/* Magnetometer full scale values */
enum inv_icm20948_magn_fs {
	INV_ICM20948_MAGN_FS_4900UT,
	INV_ICM20948_MAGN_FS_NB,
};

/* ODR values */
enum inv_icm20948_odr {
	INV_ICM20948_ODR_1100HZ,
	INV_ICM20948_ODR_1000HZ,
	INV_ICM20948_ODR_500HZ,
	INV_ICM20948_ODR_200HZ,
	INV_ICM20948_ODR_100HZ,
	INV_ICM20948_ODR_50HZ,
	INV_ICM20948_ODR_25HZ,
	INV_ICM20948_ODR_12_5HZ,
	INV_ICM20948_ODR_NB,
};

struct inv_icm20948_sensor_conf {
	int mode;
	int fs;
	int odr;
};

#define INV_ICM20948_SENSOR_CONF_INIT   {-1, -1, -1}

struct inv_icm20948_conf {
	struct inv_icm20948_sensor_conf gyro;
	struct inv_icm20948_sensor_conf accel;
	struct inv_icm20948_sensor_conf magn;
	bool temp_en;
};

struct inv_icm20948_state {
	struct mutex lock;
	const char *name;
	struct regmap *map;
	struct regulator *vdd_supply;
	struct regulator *vddio_supply;
	struct iio_mount_matrix orientation;
	struct inv_icm20948_conf conf;
	struct iio_dev *indio_gyro;
	struct iio_dev *indio_accel;
	struct iio_dev *indio_magn;
	struct iio_dev *indio_temp;
	struct iio_trigger *trig;
	uint8_t buffer[8] __aligned(IIO_DMA_MINALIGN);
	struct {
		int64_t gyro;
		int64_t accel;
		int64_t magn;
	} timestamp;
	bool use_fifo;
	bool mag_initialized;
	struct work_struct irq_work; /* 用于将中断处理移出中断上下文 */
};

struct inv_icm20948_sensor_state {
	const int *scales;
	size_t scales_len;
	struct inv_sensors_timestamp ts;
};

/* Function declarations */
extern const struct regmap_config inv_icm20948_regmap_config;
extern const struct regmap_config inv_icm20948_spi_regmap_config;
extern const struct dev_pm_ops inv_icm20948_pm_ops;

/* Debug support functions */
#ifdef CONFIG_INV_ICM20948_DEBUG
int inv_icm20948_debug_init(struct inv_icm20948_state *st);
void inv_icm20948_debug_cleanup(struct inv_icm20948_state *st);
#else
static inline int inv_icm20948_debug_init(struct inv_icm20948_state *st) { return 0; }
static inline void inv_icm20948_debug_cleanup(struct inv_icm20948_state *st) { }
#endif

const struct iio_mount_matrix *
inv_icm20948_get_mount_matrix(const struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan);

int inv_icm20948_core_probe(struct regmap *regmap, int irq,
			   int (*bus_setup)(struct inv_icm20948_state *));

int inv_icm20948_set_bank(struct inv_icm20948_state *st, u8 bank);
int inv_icm20948_write_reg(struct inv_icm20948_state *st, u8 bank, u8 reg, u8 val);
int inv_icm20948_read_reg(struct inv_icm20948_state *st, u8 bank, u8 reg, u8 *val);

int inv_icm20948_set_gyro_fs(struct inv_icm20948_state *st, int fs);
int inv_icm20948_set_accel_fs(struct inv_icm20948_state *st, int fs);

struct iio_dev *inv_icm20948_gyro_init(struct inv_icm20948_state *st);
struct iio_dev *inv_icm20948_accel_init(struct inv_icm20948_state *st);
struct iio_dev *inv_icm20948_magn_init(struct inv_icm20948_state *st);
struct iio_dev *inv_icm20948_temp_init(struct inv_icm20948_state *st);

/* Buffer management functions */
int inv_icm20948_setup_trigger(struct inv_icm20948_state *st);
int inv_icm20948_configure_fifo(struct inv_icm20948_state *st, bool enable);
void inv_icm20948_cleanup_buffer(struct inv_icm20948_state *st);

int inv_icm20948_read_fifo(struct inv_icm20948_state *st);
int inv_icm20948_parse_fifo(struct inv_icm20948_state *st, uint8_t *data, int len);

/* Magnetometer setup */
int inv_icm20948_setup_mag(struct inv_icm20948_state *st);
int inv_icm20948_check_magn(struct inv_icm20948_state *st);

/* Power management */
int inv_icm20948_set_power_mode(struct inv_icm20948_state *st, enum inv_icm20948_sensor_mode mode);
int inv_icm20948_set_sensor_rate(struct inv_icm20948_state *st, int sensor, int rate);
int inv_icm20948_set_clock_source(struct inv_icm20948_state *st);
int inv_icm20948_clear_interrupts(struct inv_icm20948_state *st);

#endif /* INV_ICM20948_H_ */