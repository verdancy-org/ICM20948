# ICM20948

I2C 版 `ICM20948` LibXR 模块，风格对齐 `xrobot-org/BMI088` / `xrobot-org/ICM20948`。

模块功能：
- 初始化 ICM-20948 主 IMU 与内置 AK09916 磁力计
- 后台线程采样并发布陀螺仪、加速度计、磁力计 Topic
- 通过 `database` 保存陀螺仪零偏
- 通过 `ramfs/bin/icm20948` 提供状态与标定命令

## Required Hardware

- `i2c1` 或等价别名
- `IMU_INT`
- `ramfs`
- `database`

## Constructor Arguments

- `data_rate`
- `accl_range`
- `gyro_range`
- `mag_mode`
- `rotation`
- `gyro_topic_name`
- `accl_topic_name`
- `magn_topic_name`
- `task_stack_depth`
- `i2c_name`
- `int_pin_name`

## Published Topics

- `gyro_topic_name`: `Eigen::Matrix<float, 3, 1>`，单位 `rad/s`
- `accl_topic_name`: `Eigen::Matrix<float, 3, 1>`，单位 `m/s^2`
- `magn_topic_name`: `Eigen::Matrix<float, 3, 1>`，单位 `uT`

## Shell Commands

- `icm20948`
- `icm20948 status`
- `icm20948 list_offset`
- `icm20948 cali`
