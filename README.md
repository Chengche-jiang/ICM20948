# Invensense ICM-20948 Linux Driver

This directory contains a Linux kernel driver for the Invensense ICM-20948 9-axis IMU (Inertial Measurement Unit) sensor.

## Hardware Description

The ICM-20948 is a 9-axis motion tracking device that combines:
- 3-axis accelerometer (±2g to ±16g)
- 3-axis gyroscope (±250dps to ±2000dps)
- 3-axis magnetometer (AK09916 magnetometer, ±4900μT)
- Digital Motion Processor (DMP)
- Temperature sensor

The sensor supports both I2C (up to 400kHz) and SPI (up to 7MHz) communication interfaces.

## Driver Features

This driver provides:
- Support for accelerometer, gyroscope, magnetometer, and temperature sensors
- I2C and SPI communication interfaces
- IIO subsystem integration
- Configurable sampling rates
- Configurable full-scale ranges
- FIFO support
- Trigger buffer support
- Power management (low-power and low-noise modes)

## Building

The driver is integrated into the Linux kernel's Industrial I/O (IIO) subsystem. To build and use the driver:

1. Configure the kernel with the following options:
   ```
   CONFIG_IIO=y
   CONFIG_INV_ICM20948=m  # For building as a module
   CONFIG_INV_ICM20948_I2C=m  # For I2C interface
   CONFIG_INV_ICM20948_SPI=m  # For SPI interface
   ```

2. Build the kernel or modules:
   ```
   make -j $(nproc) modules M=drivers/iio/imu
   ```

3. Install the modules:
   ```
   sudo make modules_install M=drivers/iio/imu
   ```

## Device Tree Configuration

### I2C Example

```dts
&i2c1 {
    status = "okay";
    #address-cells = <1>;
    #size-cells = <0>;

    icm20948@68 {
        compatible = "invensense,icm20948";
        reg = <0x68>;
        interrupt-parent = <&gpio2>;
        interrupts = <20 IRQ_TYPE_EDGE_RISING>;
        
        /* Optional properties */
        vdd-supply = <&vdd_3v3>;
        vddio-supply = <&vdd_1v8>;
        mount-matrix = "0, 1, 0, 1, 0, 0, 0, 0, -1";
    };
};
```

### SPI Example

```dts
&spi0 {
    status = "okay";
    #address-cells = <1>;
    #size-cells = <0>;

    icm20948@0 {
        compatible = "invensense,icm20948";
        reg = <0>;
        spi-max-frequency = <7000000>; /* 7MHz max SPI clock */
        interrupt-parent = <&gpio2>;
        interrupts = <20 IRQ_TYPE_EDGE_RISING>;
        
        /* Optional properties */
        vdd-supply = <&vdd_3v3>;
        vddio-supply = <&vdd_1v8>;
        mount-matrix = "1, 0, 0, 0, 1, 0, 0, 0, 1";
    };
};
```

### Optional Device Tree Properties

The following optional properties can be used:
- `magnetometer-disable`: Disable magnetometer
- `accel-fifo-enable`: Enable accelerometer FIFO at startup
- `gyro-fifo-enable`: Enable gyroscope FIFO at startup
- `magnetometer-fifo-enable`: Enable magnetometer FIFO at startup
- `accel-rate-hz`: Set initial accelerometer sampling rate in Hz
- `gyro-rate-hz`: Set initial gyroscope sampling rate in Hz
- `magnetometer-rate-hz`: Set initial magnetometer sampling rate in Hz

## Usage

### Accessing Sensor Data

Sensor data can be accessed through the IIO subsystem in userspace. For example, to read raw accelerometer data:

```bash
# Read accelerometer raw X axis value
cat /sys/bus/iio/devices/iio:device*/in_accel_x_raw

# Read gyroscope raw Y axis value
cat /sys/bus/iio/devices/iio:device*/in_anglvel_y_raw

# Read magnetometer raw Z axis value
cat /sys/bus/iio/devices/iio:device*/in_magn_z_raw

# Read temperature
cat /sys/bus/iio/devices/iio:device*/in_temp_raw
```

### Setting Sensor Parameters

```bash
# Set accelerometer scale to 4g
echo 4 > /sys/bus/iio/devices/iio:device*/in_accel_scale

# Set gyroscope scale to 1000dps
echo 1000 > /sys/bus/iio/devices/iio:device*/in_anglvel_scale

# Set sampling frequency to 100Hz
echo 100 > /sys/bus/iio/devices/iio:device*/in_accel_sampling_frequency
echo 100 > /sys/bus/iio/devices/iio:device*/in_anglvel_sampling_frequency
```

### Using IIO Buffered Capture

```bash
# Enable the buffer
echo 1 > /sys/bus/iio/devices/iio:device*/buffer/enable

# Set buffer length
echo 128 > /sys/bus/iio/devices/iio:device*/buffer/length

# Enable channels
echo 1 > /sys/bus/iio/devices/iio:device*/scan_elements/in_accel_x_en
echo 1 > /sys/bus/iio/devices/iio:device*/scan_elements/in_accel_y_en
echo 1 > /sys/bus/iio/devices/iio:device*/scan_elements/in_accel_z_en

# Read data from the buffer
cat /dev/iio:device* > accel_data.bin
```

## Debugging

If you encounter issues, check:
- Kernel logs: `dmesg | grep -i icm20948`
- IIO device info: `ls -l /sys/bus/iio/devices/`
- IIO triggers: `ls -l /sys/bus/iio/devices/trigger*`

## License

This driver is released under GPL-2.0 license.

## Copyright

Copyright (C) 2024 Invensense, Inc. 