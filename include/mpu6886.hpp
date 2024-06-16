/*
 Note: The MPU6886 is an I2C sensor and uses the Arduino Wire library.
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or
 a 3.3 V Teensy 3.1. We have disabled the internal pull-ups used by the Wire
 library in the Wire.h/twi.c utility file. We are also using the 400 kHz fast
 I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
#ifndef _MPU6886_H_
#define _MPU6886_H_
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#include <Wire.h>
#else
#include <esp_timer.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <esp_idf_version.h>
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include <driver/i2c_master.h>
#else
#include <driver/i2c.h>
#endif
#endif


//#define G (9.8)
#define RtA     57.324841
#define AtR     0.0174533
#define Gyro_Gr 0.0010653
    enum struct mpu6886_scale {
        g2=0,
        g4,
        g8,
        g16
    };
    enum struct mpu6886_dps {
        
        dps250=0,
        dps500,
        dps1000,
        dps2000
    };
class mpu6886 {
   
   public:
    mpu6886(
 #ifdef ARDUINO
        TwoWire& i2c = Wire
#else
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        i2c_master_bus_handle_t i2c
#else
        i2c_port_t i2c = I2C_NUM_0
#endif
#endif       
    );
    bool initialize();
    bool initialized() const;
    void acc_raw_xyz(int16_t* ax, int16_t* ay, int16_t* az);
    void gyro_raw_xyz(int16_t* gx, int16_t* gy, int16_t* gz);
    void temp_raw(int16_t* t);
    void calibrate(int count = 256, uint32_t delay_ms=0);

    void acc_xyz(float* ax, float* ay, float* az);
    void gyro_xyz(float* gx, float* gy, float* gz);
    void temp(float* t);

    mpu6886_dps gyro_dps() const;
    void gyro_dps(mpu6886_dps value);
    mpu6886_scale acc_scale() const;
    void acc_scale(mpu6886_scale value);

    void ahrs(float* pitch, float* roll, float* yaw);
    void attitude(double* pitch, double* roll);

   public:
    
   private:
#ifdef ARDUINO
    TwoWire& m_i2c;
#else
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    i2c_master_bus_handle_t m_i2c_bus;
    i2c_master_dev_handle_t m_i2c;
#else
    constexpr static const uint8_t ACK_CHECK_EN = 0x1;
    constexpr static const uint8_t ACK_CHECK_DIS = 0x0;
    constexpr static const uint8_t ACK_VAL = 0x0;
    constexpr static const uint8_t NACK_VAL = 0x1;
    i2c_port_t m_i2c;
#endif
#endif
    bool m_initialized;
    mpu6886_dps Gyscale = mpu6886_dps::dps2000;
    mpu6886_scale Acscale = mpu6886_scale::g8;
    float aRes, gRes;
    float m_gox,m_goy,m_goz;
    float _last_theta = 0;
    float _last_phi   = 0;
    float _alpha      = 0.5;
    volatile float twoKp;  
    volatile float twoKi;
    volatile float
        q0 = 1.0,
        q1 = 0.0, q2 = 0.0,
        q3 = 0.0;  // quaternion of sensor frame relative to auxiliary frame
    volatile float integralFBx = 0.0f, integralFBy = 0.0f,
                integralFBz = 0.0f;  // integral error terms scaled by Ki
   private:
    void read_reg(uint8_t start_Addr,
                         uint8_t number_Bytes, uint8_t* read_Buffer);
    void write_reg(uint8_t start_Addr,
                          uint8_t number_Bytes, const uint8_t* write_Buffer);
    void update_gres();
    void update_ares();

    void mahony_ahrs_update(float gx, float gy, float gz, float ax, float ay,
                          float az, float mx, float my, float mz);
    void mahony_ahrs_update_imu(float gx, float gy, float gz, float ax, float ay,
                             float az, float *pitch, float *roll, float *yaw);
    static float inv_sqrt(float x);
#ifndef ARDUINO
    static void delay(uint32_t ms);
#endif
};
#endif