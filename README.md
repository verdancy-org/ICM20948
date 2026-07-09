# ICM20948

TDK InvenSense ICM-20948 I2C 9-axis IMU sensor module for XRobot.

This module initializes the ICM-20948 IMU and its internal AK09916 magnetometer
over I2C, samples accelerometer / gyroscope / magnetometer data from the
data-ready interrupt, publishes gyro, accel, and magnetometer topics, and stores
gyroscope zero-offset calibration in `LibXR::Database`.

The I2C and INT names are constructor arguments, so projects may use other
hardware aliases if needed.

## Required Hardware

- `icm20948_i2c`
- `icm20948_int`
- `ramfs`
- `database`

## Constructor Arguments

- `data_rate`: default `ICM20948::DataRate::RATE_225HZ`
- `accl_range`: default `ICM20948::AcclRange::RANGE_2G`
- `gyro_range`: default `ICM20948::GyroRange::DPS_2000`
- `mag_mode`: default `ICM20948::MagMode::CONTINUOUS_100HZ`
- `rotation`: sensor-frame to application-frame quaternion
- `gyro_topic_name`: default `"icm20948_gyro"`
- `accl_topic_name`: default `"icm20948_accl"`
- `magn_topic_name`: default `"icm20948_magn"`
- `task_stack_depth`: default `2048`
- `i2c_name`: default `"icm20948_i2c"`
- `int_pin_name`: default `"icm20948_int"`

## Published Topics

- `gyro_topic_name`: `Eigen::Matrix<float, 3, 1>`, rad/s
- `accl_topic_name`: `Eigen::Matrix<float, 3, 1>`, m/s^2
- `magn_topic_name`: `Eigen::Matrix<float, 3, 1>`, uT

## Shell Commands

The module registers `bin/icm20948` in `RamFS`.

- `bin/icm20948` or `bin/icm20948 status`: print chip ID, init state, temperature, and latest sensor data
- `bin/icm20948 list_offset`: print current gyroscope calibration offset
- `bin/icm20948 cali`: collect still gyroscope data and save offset to `database`

## XRobot Configuration Example

```yaml
- id: imu
  name: ICM20948
  constructor_args:
    data_rate: ICM20948::DataRate::RATE_225HZ
    accl_range: ICM20948::AcclRange::RANGE_2G
    gyro_range: ICM20948::GyroRange::DPS_2000
    mag_mode: ICM20948::MagMode::CONTINUOUS_100HZ
    rotation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0
    gyro_topic_name: "icm20948_gyro"
    accl_topic_name: "icm20948_accl"
    magn_topic_name: "icm20948_magn"
    task_stack_depth: 2048
    i2c_name: "icm20948_i2c"
    int_pin_name: "icm20948_int"
```
