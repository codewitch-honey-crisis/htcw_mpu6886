#pragma once
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#include <Wire.h>
namespace arduino {
#else
#ifdef ESP_PLATFORM
#include <inttypes.h>
#include <driver/i2c.h>
namespace esp_idf {
#else
#error "This library requires Arduino or an ESP32"
#endif
#endif


enum struct mpu6886_acc_scale {
    scale_2g = 0,
    scale_4g,
    scale_8g,
    scale_16g
};

enum struct mpu6886_gyro_scale {
    scale_250dps=0,
    scale_500dps,
    scale_1000dps,
    scale_2000dps
};

class mpu6886 {
    constexpr static const int8_t address = 0x68;
    constexpr static const uint8_t op_whoami = 0x75;
    constexpr static const uint8_t op_accel_intel_ctrl = 0x69;
    constexpr static const uint8_t op_smplrt_div = 0x19;
    constexpr static const uint8_t op_int_pin_cfg = 0x37;
    constexpr static const uint8_t op_int_enable = 0x38;
    constexpr static const uint8_t op_accel_xout_h = 0x3B;
    constexpr static const uint8_t op_accel_xout_l = 0x3C;
    constexpr static const uint8_t op_accel_yout_h = 0x3D;
    constexpr static const uint8_t op_accel_yout_l = 0x3E;
    constexpr static const uint8_t op_accel_zout_h = 0x3F;
    constexpr static const uint8_t op_accel_zout_l = 0x40;

    constexpr static const uint8_t op_temp_out_h = 0x41;
    constexpr static const uint8_t op_temp_out_l = 0x42;

    constexpr static const uint8_t op_gyro_xout_h = 0x43;
    constexpr static const uint8_t op_gyro_xout_l = 0x44;
    constexpr static const uint8_t op_gyro_yout_h = 0x45;
    constexpr static const uint8_t op_gyro_yout_l = 0x46;
    constexpr static const uint8_t op_gyro_zout_h = 0x47;
    constexpr static const uint8_t op_gyro_zout_l = 0x48;

    constexpr static const uint8_t op_user_ctrl = 0x6A;
    constexpr static const uint8_t op_pwr_mgmt_1 = 0x6B;
    constexpr static const uint8_t op_pwr_mgmt_2 = 0x6C;
    constexpr static const uint8_t op_config = 0x1A;
    constexpr static const uint8_t op_gyro_config = 0x1B;
    constexpr static const uint8_t op_accel_config = 0x1C;
    constexpr static const uint8_t op_accel_config2 = 0x1D;
    constexpr static const uint8_t op_fifo_en = 0x23;

    constexpr static const uint8_t op_fifo_enable = 0x23;
    constexpr static const uint8_t op_fifo_count = 0x72;
    constexpr static const uint8_t op_fifo_r_w = 0x74;
    constexpr static const uint8_t op_gyro_offset = 0x13;

    constexpr static const double rta = 57.324841;
    constexpr static const double atr = 0.0174533;
    constexpr static const double gyro_gr =	0.0010653;
#ifdef ARDUINO
    TwoWire& m_i2c;
#else
    i2c_port_t m_i2c;
#endif
    bool m_initialized;
    mpu6886_acc_scale m_acc_scale;
    mpu6886_gyro_scale m_gyro_scale;
    float m_ares;
    float m_gres;
    float m_imuId;
    void adc_reset();
    void update_gres();
    void update_ares();
    void acc_adc(int16_t* out_x,int16_t* out_y,int16_t* out_z);
    void gyro_adc(int16_t* out_x,int16_t*out_y,int16_t* out_z);
    int16_t temp_adc();
    mpu6886(const mpu6886& rhs)=delete;
    mpu6886& operator=(const mpu6886& rhs)=delete;
    void do_move(mpu6886& rhs);
   public:
    
    inline mpu6886(
#ifdef ARDUINO
        TwoWire& i2c = Wire
#else
        i2c_port_t i2c = I2C_NUM_0
#endif
        ) : m_i2c(i2c), m_initialized(false) {
    }
    mpu6886(mpu6886&& rhs);
    mpu6886& operator=(mpu6886&& rhs);
    inline bool initialized() const { return m_initialized; }
    bool initialize();
    inline mpu6886_acc_scale acc_scale() const { return m_acc_scale; }
    void acc_scale(mpu6886_acc_scale value);
    inline mpu6886_gyro_scale gyro_scale() const { return m_gyro_scale; }
    void gyro_offset(float x, float y, float z);
    void gyro_scale(mpu6886_gyro_scale value);
    void acc(float* out_x, float* out_y, float* out_z);
    void gyro(float* out_x, float* out_y, float* out_z);
    void ahrs(float* out_pitch, float* out_yaw, float* out_roll);
    float temp();
};
}  // namespace arduino