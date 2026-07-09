#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: XRobot Module for TDK InvenSense ICM-20948 9-axis IMU sensor
constructor_args:
  - data_rate: ICM20948::DataRate::RATE_225HZ
  - accl_range: ICM20948::AcclRange::RANGE_2G
  - gyro_range: ICM20948::GyroRange::DPS_2000
  - mag_mode: ICM20948::MagMode::CONTINUOUS_100HZ
  - rotation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0
  - gyro_topic_name: "icm20948_gyro"
  - accl_topic_name: "icm20948_accl"
  - magn_topic_name: "icm20948_magn"
  - task_stack_depth: 2048
  - i2c_name: "icm20948_i2c"
  - int_pin_name: "icm20948_int"
template_args: []
required_hardware: icm20948_i2c icm20948_int ramfs database
depends: []
=== END MANIFEST === */
// clang-format on

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

#include "app_framework.hpp"
#include "database.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "message.hpp"
#include "ramfs.hpp"
#include "thread.hpp"
#include "transform.hpp"

class ICM20948 : public LibXR::Application {
 public:
  static constexpr float M_DEG2RAD_MULT = 0.01745329251f;
  static constexpr float STANDARD_GRAVITY = 9.80665f;

  enum class DataRate : uint8_t {
    RATE_1125HZ = 0,
    RATE_225HZ = 4,
    RATE_112HZ = 9,
    RATE_56HZ = 19,
    RATE_28HZ = 39,
  };

  enum class GyroRange : uint8_t {
    DPS_250 = 0,
    DPS_500 = 1,
    DPS_1000 = 2,
    DPS_2000 = 3,
  };

  enum class AcclRange : uint8_t {
    RANGE_2G = 0,
    RANGE_4G = 1,
    RANGE_8G = 2,
    RANGE_16G = 3,
  };

  enum class MagMode : uint8_t {
    POWER_DOWN = 0x00,
    SINGLE = 0x01,
    CONTINUOUS_10HZ = 0x02,
    CONTINUOUS_20HZ = 0x04,
    CONTINUOUS_50HZ = 0x06,
    CONTINUOUS_100HZ = 0x08,
  };

  ICM20948(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
           DataRate data_rate, AcclRange accl_range, GyroRange gyro_range,
           MagMode mag_mode, LibXR::Quaternion<float>&& rotation,
           const char* gyro_topic_name, const char* accl_topic_name,
           const char* magn_topic_name, size_t task_stack_depth,
           const char* i2c_name = "icm20948_i2c",
           const char* int_pin_name = "icm20948_int")
      : data_rate_(data_rate),
        accl_range_(accl_range),
        gyro_range_(gyro_range),
        mag_mode_(mag_mode),
        topic_gyro_(LibXR::Topic::CreateTopic<Eigen::Matrix<float, 3, 1>>(gyro_topic_name)),
        topic_accl_(LibXR::Topic::CreateTopic<Eigen::Matrix<float, 3, 1>>(accl_topic_name)),
        topic_magn_(LibXR::Topic::CreateTopic<Eigen::Matrix<float, 3, 1>>(magn_topic_name)),
        int_(hw.template FindOrExit<LibXR::GPIO>({int_pin_name})),
        i2c_(hw.template FindOrExit<LibXR::I2C>({i2c_name})),
        rotation_(std::move(rotation)),
        cmd_file_(LibXR::RamFS::CreateCommand("icm20948", CommandFunc, this)),
        gyro_offset_key_(*hw.template FindOrExit<LibXR::Database>({"database"}),
                         "icm20948_gyro_offset",
                         Eigen::Matrix<float, 3, 1>(0.0f, 0.0f, 0.0f)) {
    app.Register(*this);
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->bin_.Add(cmd_file_);

    int_->DisableInterrupt();
    int_->SetConfig(
        {LibXR::GPIO::Direction::RISING_INTERRUPT, LibXR::GPIO::Pull::NONE});

    auto int_cb = LibXR::GPIO::Callback::Create(
        [](bool in_isr, ICM20948* self) {
          self->sample_timestamp_ = LibXR::Topic::NowTimestamp();
          self->new_data_.PostFromCallback(in_isr);
        },
        this);
    int_->RegisterCallback(int_cb);

    while (!Init()) {
      LibXR::Thread::Sleep(100);
    }

    thread_.Create(this, ThreadFunc, "icm20948_thread", task_stack_depth,
                   LibXR::Thread::Priority::REALTIME);
  }

  void OnMonitor() override {}

 private:
  static constexpr uint16_t ICM20948_ADDR = 0x68;
  static constexpr uint16_t AK09916_ADDR = 0x0C;
  static constexpr uint8_t WHO_AM_I_VALUE = 0xEA;
  static constexpr uint8_t AK09916_WHO_AM_I_VALUE = 0x09;

  static constexpr uint8_t REG_BANK_SEL = 0x7F;
  static constexpr uint8_t BANK0 = 0x00;
  static constexpr uint8_t BANK2 = 0x20;

  static constexpr uint8_t WHO_AM_I = 0x00;
  static constexpr uint8_t USER_CTRL = 0x03;
  static constexpr uint8_t PWR_MGMT_1 = 0x06;
  static constexpr uint8_t PWR_MGMT_2 = 0x07;
  static constexpr uint8_t INT_PIN_CFG = 0x0F;
  static constexpr uint8_t INT_ENABLE_1 = 0x11;
  static constexpr uint8_t INT_STATUS_1 = 0x1A;
  static constexpr uint8_t ACCEL_XOUT_H = 0x2D;
  static constexpr uint8_t TEMP_OUT_H = 0x39;

  static constexpr uint8_t GYRO_SMPLRT_DIV = 0x00;
  static constexpr uint8_t GYRO_CONFIG_1 = 0x01;
  static constexpr uint8_t ACCEL_SMPLRT_DIV_2 = 0x11;
  static constexpr uint8_t ACCEL_CONFIG = 0x14;

  static constexpr uint8_t AK09916_WIA = 0x01;
  static constexpr uint8_t AK09916_HXL = 0x11;
  static constexpr uint8_t AK09916_CNTL2 = 0x31;

  static constexpr uint8_t IMU_READ_LEN = 12;
  static constexpr uint8_t MAG_READ_LEN = 8;

  static int CommandFunc(ICM20948* self, int argc, char** argv) {
    if (argc == 1 || (argc == 2 && std::strcmp(argv[1], "status") == 0)) {
      WriteStatus(self);
      return 0;
    }

    if (argc == 2 && std::strcmp(argv[1], "list_offset") == 0) {
      WriteOffset(self);
      return 0;
    }

    if (argc == 2 && std::strcmp(argv[1], "cali") == 0) {
      return Calibrate(self);
    }

    LibXR::STDIO::Printf<
        "usage: icm20948 [status|list_offset|cali]\r\n">();
    return -1;
  }

  static void WriteStatus(ICM20948* self) {
    LibXR::STDIO::Printf<
        "who=%02X init=%d temp=%.3f mag=%d sample_ms=%u\r\n">(
        self->who_am_i_, self->init_ok_ ? 1 : 0, self->temperature_,
        self->mag_enabled_ ? 1 : 0,
        static_cast<uint32_t>(static_cast<uint64_t>(self->sample_timestamp_) / 1000ULL));
  }

  static void WriteOffset(ICM20948* self) {
    LibXR::STDIO::Printf<"offset x=%.6f y=%.6f z=%.6f\r\n">(
        self->gyro_offset_key_.data_.x(), self->gyro_offset_key_.data_.y(),
        self->gyro_offset_key_.data_.z());
  }

  static int Calibrate(ICM20948* self) {
    self->gyro_cali_sum_ = Eigen::Matrix<std::int64_t, 3, 1>(0, 0, 0);
    self->cali_counter_ = 0;
    self->in_cali_ = true;

    LibXR::STDIO::Printf<"icm20948 cali start, keep sensor still\r\n">();
    LibXR::Thread::Sleep(3000);
    self->in_cali_ = false;

    if (self->cali_counter_ == 0) {
      LibXR::STDIO::Printf<"icm20948 cali failed\r\n">();
      return -1;
    }

    const float scale = self->GetGyroScale() /
                        static_cast<float>(self->cali_counter_);
    self->gyro_offset_key_.data_.x() = self->gyro_cali_sum_.x() * scale;
    self->gyro_offset_key_.data_.y() = self->gyro_cali_sum_.y() * scale;
    self->gyro_offset_key_.data_.z() = self->gyro_cali_sum_.z() * scale;

    if (self->gyro_offset_key_.Save() != LibXR::ErrorCode::OK) {
      LibXR::STDIO::Printf<"icm20948 cali save failed\r\n">();
      return -1;
    }

    WriteOffset(self);
    return 0;
  }

  bool Init() {
    init_ok_ = false;

    if (!SelectBank(BANK0)) {
      return false;
    }

    if (!ReadRegister(WHO_AM_I, who_am_i_)) {
      return false;
    }
    if (who_am_i_ != WHO_AM_I_VALUE) {
      return false;
    }

    if (!WriteRegister(PWR_MGMT_1, 0x80)) {
      return false;
    }
    LibXR::Thread::Sleep(100);

    if (!SelectBank(BANK0) || !WriteRegister(USER_CTRL, 0x00) ||
        !WriteRegister(PWR_MGMT_1, 0x01) ||
        !WriteRegister(PWR_MGMT_2, 0x00)) {
      return false;
    }

    if (!SelectBank(BANK2) ||
        !WriteRegister(GYRO_SMPLRT_DIV, static_cast<uint8_t>(data_rate_)) ||
        !WriteRegister(GYRO_CONFIG_1,
                       static_cast<uint8_t>((static_cast<uint8_t>(gyro_range_)
                                             << 1U) |
                                            (3U << 3U) | 0x01U)) ||
        !WriteRegister(ACCEL_SMPLRT_DIV_2, static_cast<uint8_t>(data_rate_)) ||
        !WriteRegister(ACCEL_CONFIG,
                       static_cast<uint8_t>((static_cast<uint8_t>(accl_range_)
                                             << 1U) |
                                            (5U << 3U) | 0x01U))) {
      return false;
    }

    if (!SelectBank(BANK0) || !WriteRegister(INT_PIN_CFG, 0x02) ||
        !WriteRegister(INT_ENABLE_1, 0x01)) {
      return false;
    }

    mag_enabled_ = InitMagnetometer();
    int_->EnableInterrupt();
    sample_timestamp_ = LibXR::Topic::NowTimestamp();
    init_ok_ = true;
    return true;
  }

  bool InitMagnetometer() {
    uint8_t who = 0;
    if (!ReadMagRegister(AK09916_WIA, who)) {
      return false;
    }
    if (who != AK09916_WHO_AM_I_VALUE) {
      return false;
    }
    return WriteMagRegister(AK09916_CNTL2, static_cast<uint8_t>(mag_mode_));
  }

  static void ThreadFunc(ICM20948* self) {
    while (true) {
      if (self->new_data_.Wait(self->GetWaitTimeoutMs()) != LibXR::ErrorCode::OK) {
        self->sample_timestamp_ = LibXR::Topic::NowTimestamp();
      }
      self->ReadSample();
    }
  }

  uint32_t GetWaitTimeoutMs() const {
    switch (data_rate_) {
      case DataRate::RATE_1125HZ:
        return 5;
      case DataRate::RATE_225HZ:
        return 10;
      case DataRate::RATE_112HZ:
        return 20;
      case DataRate::RATE_56HZ:
        return 40;
      case DataRate::RATE_28HZ:
        return 80;
      default:
        return 20;
    }
  }

  bool SelectBank(uint8_t bank) { return WriteRegister(REG_BANK_SEL, bank); }

  bool WriteRegister(uint8_t reg, uint8_t value) {
    LibXR::WriteOperation op(sem_i2c_, 50);
    return i2c_->MemWrite(ICM20948_ADDR, reg, {&value, 1}, op) == LibXR::ErrorCode::OK;
  }

  bool ReadRegister(uint8_t reg, uint8_t& value) {
    LibXR::ReadOperation op(sem_i2c_, 50);
    return i2c_->MemRead(ICM20948_ADDR, reg, {&value, 1}, op) == LibXR::ErrorCode::OK;
  }

  bool ReadRegisters(uint8_t reg, uint8_t* buffer, size_t size) {
    LibXR::ReadOperation op(sem_i2c_, 50);
    return i2c_->MemRead(ICM20948_ADDR, reg, {buffer, size}, op) ==
           LibXR::ErrorCode::OK;
  }

  bool WriteMagRegister(uint8_t reg, uint8_t value) {
    LibXR::WriteOperation op(sem_i2c_, 50);
    return i2c_->MemWrite(AK09916_ADDR, reg, {&value, 1}, op) ==
           LibXR::ErrorCode::OK;
  }

  bool ReadMagRegister(uint8_t reg, uint8_t& value) {
    LibXR::ReadOperation op(sem_i2c_, 50);
    return i2c_->MemRead(AK09916_ADDR, reg, {&value, 1}, op) ==
           LibXR::ErrorCode::OK;
  }

  bool ReadMagRegisters(uint8_t reg, uint8_t* buffer, size_t size) {
    LibXR::ReadOperation op(sem_i2c_, 50);
    return i2c_->MemRead(AK09916_ADDR, reg, {buffer, size}, op) ==
           LibXR::ErrorCode::OK;
  }

  void ReadSample() {
    if (!init_ok_) {
      return;
    }

    uint8_t imu_buffer[IMU_READ_LEN] = {};
    uint8_t temp_buffer[2] = {};
    uint8_t mag_buffer[MAG_READ_LEN] = {};
    uint8_t status = 0;

    (void)ReadRegister(INT_STATUS_1, status);

    if (!ReadRegisters(ACCEL_XOUT_H, imu_buffer, sizeof(imu_buffer)) ||
        !ReadRegisters(TEMP_OUT_H, temp_buffer, sizeof(temp_buffer))) {
      return;
    }

    ParseImu(imu_buffer, temp_buffer);
    if (mag_enabled_ && ReadMagRegisters(AK09916_HXL, mag_buffer, sizeof(mag_buffer))) {
      ParseMag(mag_buffer);
    }

    topic_accl_.Publish(accl_data_, sample_timestamp_);
    topic_gyro_.Publish(gyro_data_, sample_timestamp_);
    if (mag_enabled_) {
      topic_magn_.Publish(magn_data_, sample_timestamp_);
    }
  }

  void ParseImu(const uint8_t* imu_buffer, const uint8_t* temp_buffer) {
    std::array<int16_t, 3> accel_raw = {};
    std::array<int16_t, 3> gyro_raw = {};
    std::array<float, 3> accel_si = {};
    std::array<float, 3> gyro_si = {};

    for (size_t i = 0; i < 3; ++i) {
      accel_raw[i] = static_cast<int16_t>(
          static_cast<uint16_t>(imu_buffer[i * 2] << 8U) | imu_buffer[i * 2 + 1]);
      gyro_raw[i] = static_cast<int16_t>(
          static_cast<uint16_t>(imu_buffer[i * 2 + 6] << 8U) | imu_buffer[i * 2 + 7]);

      accel_si[i] = static_cast<float>(accel_raw[i]) * GetAccelScale();
      gyro_si[i] = static_cast<float>(gyro_raw[i]) * GetGyroScale();
    }

    if (in_cali_) {
      gyro_cali_sum_.x() += gyro_raw[0];
      gyro_cali_sum_.y() += gyro_raw[1];
      gyro_cali_sum_.z() += gyro_raw[2];
      ++cali_counter_;
    }

    const int16_t temp_raw = static_cast<int16_t>(
        static_cast<uint16_t>(temp_buffer[0] << 8U) | temp_buffer[1]);
    temperature_ = static_cast<float>(temp_raw) / 333.87f + 21.0f;

    accl_data_ = rotation_ * Eigen::Matrix<float, 3, 1>(
                                 accel_si[0], accel_si[1], accel_si[2]);
    gyro_data_ =
        rotation_ * Eigen::Matrix<float, 3, 1>(
                        gyro_si[0] - gyro_offset_key_.data_.x(),
                        gyro_si[1] - gyro_offset_key_.data_.y(),
                        gyro_si[2] - gyro_offset_key_.data_.z());
  }

  void ParseMag(const uint8_t* mag_buffer) {
    if (((mag_buffer[7] >> 3U) & 0x01U) != 0U) {
      return;
    }

    const int16_t raw_x = static_cast<int16_t>(
        static_cast<uint16_t>(mag_buffer[1] << 8U) | mag_buffer[0]);
    const int16_t raw_y = static_cast<int16_t>(
        static_cast<uint16_t>(mag_buffer[3] << 8U) | mag_buffer[2]);
    const int16_t raw_z = static_cast<int16_t>(
        static_cast<uint16_t>(mag_buffer[5] << 8U) | mag_buffer[4]);

    constexpr float MAG_SCALE_UT = 0.1495361328125f;
    magn_data_ = rotation_ * Eigen::Matrix<float, 3, 1>(
                                 static_cast<float>(raw_x) * MAG_SCALE_UT,
                                 static_cast<float>(raw_y) * MAG_SCALE_UT,
                                 static_cast<float>(raw_z) * MAG_SCALE_UT);
  }

  float GetAccelScale() const {
    switch (accl_range_) {
      case AcclRange::RANGE_2G:
        return 2.0f * STANDARD_GRAVITY / 32768.0f;
      case AcclRange::RANGE_4G:
        return 4.0f * STANDARD_GRAVITY / 32768.0f;
      case AcclRange::RANGE_8G:
        return 8.0f * STANDARD_GRAVITY / 32768.0f;
      case AcclRange::RANGE_16G:
        return 16.0f * STANDARD_GRAVITY / 32768.0f;
      default:
        return 0.0f;
    }
  }

  float GetGyroScale() const {
    float range_dps = 0.0f;
    switch (gyro_range_) {
      case GyroRange::DPS_250:
        range_dps = 250.0f;
        break;
      case GyroRange::DPS_500:
        range_dps = 500.0f;
        break;
      case GyroRange::DPS_1000:
        range_dps = 1000.0f;
        break;
      case GyroRange::DPS_2000:
        range_dps = 2000.0f;
        break;
      default:
        break;
    }
    return range_dps / 32768.0f * M_DEG2RAD_MULT;
  }

  DataRate data_rate_;
  AcclRange accl_range_;
  GyroRange gyro_range_;
  MagMode mag_mode_;

  float temperature_ = 0.0f;
  uint8_t who_am_i_ = 0;
  bool init_ok_ = false;
  bool mag_enabled_ = false;
  bool in_cali_ = false;
  uint32_t cali_counter_ = 0;

  LibXR::MicrosecondTimestamp sample_timestamp_ = 0;

  Eigen::Matrix<std::int64_t, 3, 1> gyro_cali_sum_;
  Eigen::Matrix<float, 3, 1> gyro_data_, accl_data_, magn_data_;

  LibXR::Topic topic_gyro_, topic_accl_, topic_magn_;
  LibXR::GPIO* int_;
  LibXR::I2C* i2c_;
  LibXR::Quaternion<float> rotation_;

  LibXR::Semaphore sem_i2c_, new_data_;
  LibXR::Thread thread_;
  LibXR::RamFS::File cmd_file_;
  LibXR::Database::Key<Eigen::Matrix<float, 3, 1>> gyro_offset_key_;
};
