#include "mpu6886.hpp"
#ifdef ARDUINO
#define I2C_TYPE TwoWire&
#define mpu6886_delay(x) delay(x)
#else
#define I2C_TYPE i2c_port_t
#include <math.h>
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1
#define RAD_TO_DEG (57.295779513082320876798154814105f)
#define DEG_TO_RAD (0.017453292519943295769236907684886f)
static void mpu6886_delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
#endif
/*
MIT License

Copyright (c) 2022 honey the codewitch

Portions copyright (C) 2017 M5 Stack

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#define sampleFreq 25.0f        // sample frequency in Hz
#define twoKpDef (2.0f * 1.0f)  // 2 * proportional gain
#define twoKiDef (2.0f * 0.0f)  // 2 * integral gain
//#define twoKiDef	(0.0f * 0.0f)

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float mpu6886_twoKp = twoKpDef;                                            // 2 * proportional gain (Kp)
volatile float mpu6886_twoKi = twoKiDef;                                            // 2 * integral gain (Ki)
volatile float mpu6886_q0 = 1.0, mpu6886_q1 = 0.0, mpu6886_q2 = 0.0, mpu6886_q3 = 0.0;                      // quaternion of sensor frame relative to auxiliary frame
volatile float mpu6886_integralFBx = 0.0f, mpu6886_integralFBy = 0.0f, mpu6886_integralFBz = 0.0f;  // integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Function declarations

// float mpu6886_invSqrt(float x);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
float mpu6886_invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
#pragma GCC diagnostic pop
//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void mpu6886_mahony_ahrs_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        // MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = mpu6886_q0 * mpu6886_q0;
        q0q1 = mpu6886_q0 * mpu6886_q1;
        q0q2 = mpu6886_q0 * mpu6886_q2;
        q0q3 = mpu6886_q0 * mpu6886_q3;
        q1q1 = mpu6886_q1 * mpu6886_q1;
        q1q2 = mpu6886_q1 * mpu6886_q2;
        q1q3 = mpu6886_q1 * mpu6886_q3;
        q2q2 = mpu6886_q2 * mpu6886_q2;
        q2q3 = mpu6886_q2 * mpu6886_q3;
        q3q3 = mpu6886_q3 * mpu6886_q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if (mpu6886_twoKi > 0.0f) {
            mpu6886_integralFBx = mpu6886_integralFBx+ (mpu6886_twoKi * halfex * (1.0f / sampleFreq));  // integral error scaled by Ki
            mpu6886_integralFBy = mpu6886_integralFBy + (mpu6886_twoKi * halfey * (1.0f / sampleFreq));
            mpu6886_integralFBz = mpu6886_integralFBz + (mpu6886_twoKi * halfez * (1.0f / sampleFreq));
            gx += mpu6886_integralFBx;  // apply integral feedback
            gy += mpu6886_integralFBy;
            gz += mpu6886_integralFBz;
        } else {
            mpu6886_integralFBx = 0.0f;  // prevent integral windup
            mpu6886_integralFBy = 0.0f;
            mpu6886_integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += mpu6886_twoKp * halfex;
        gy += mpu6886_twoKp * halfey;
        gz += mpu6886_twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq));  // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = mpu6886_q0;
    qb = mpu6886_q1;
    qc = mpu6886_q2;
    mpu6886_q0 = mpu6886_q0+(-qb * gx - qc * gy - mpu6886_q3 * gz);
    mpu6886_q1 = mpu6886_q1+(qa * gx + qc * gz - mpu6886_q3 * gy);
    mpu6886_q2 = mpu6886_q2+(qa * gy - qb * gz + mpu6886_q3 * gx);
    mpu6886_q3 = mpu6886_q3+(qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = sqrt(mpu6886_q0 * mpu6886_q0 + mpu6886_q1 * mpu6886_q1 + mpu6886_q2 * mpu6886_q2 + mpu6886_q3 * mpu6886_q3);
    mpu6886_q0 = mpu6886_q0*recipNorm;
    mpu6886_q1 = mpu6886_q1*recipNorm;
    mpu6886_q2 = mpu6886_q2* recipNorm;
    mpu6886_q3 = mpu6886_q3*recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void mpu6886_mahony_ahrs_update_imu(float gx, float gy, float gz, float ax, float ay, float az, float* pitch, float* roll, float* yaw) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = mpu6886_invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = mpu6886_q1 * mpu6886_q3 - mpu6886_q0 * mpu6886_q2;
        halfvy = mpu6886_q0 * mpu6886_q1 + mpu6886_q2 * mpu6886_q3;
        halfvz = mpu6886_q0 * mpu6886_q0 - 0.5f + mpu6886_q3 * mpu6886_q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (mpu6886_twoKi > 0.0f) {
            mpu6886_integralFBx = mpu6886_integralFBx+(mpu6886_twoKi * halfex * (1.0f / sampleFreq));  // integral error scaled by Ki
            mpu6886_integralFBy = mpu6886_integralFBy+(mpu6886_twoKi * halfey * (1.0f / sampleFreq));
            mpu6886_integralFBz = mpu6886_integralFBz+(mpu6886_twoKi * halfez * (1.0f / sampleFreq));
            gx += mpu6886_integralFBx;  // apply integral feedback
            gy += mpu6886_integralFBy;
            gz += mpu6886_integralFBz;
        } else {
            mpu6886_integralFBx = 0.0f;  // prevent integral windup
            mpu6886_integralFBy = 0.0f;
            mpu6886_integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += mpu6886_twoKp * halfex;
        gy += mpu6886_twoKp * halfey;
        gz += mpu6886_twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq));  // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = mpu6886_q0;
    qb = mpu6886_q1;
    qc = mpu6886_q2;
    mpu6886_q0 = mpu6886_q0+(-qb * gx - qc * gy - mpu6886_q3 * gz);
    mpu6886_q1 = mpu6886_q1+(qa * gx + qc * gz - mpu6886_q3 * gy);
    mpu6886_q2 = mpu6886_q2+(qa * gy - qb * gz + mpu6886_q3 * gx);
    mpu6886_q3 = mpu6886_q3+(qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = mpu6886_invSqrt(mpu6886_q0 * mpu6886_q0 + mpu6886_q1 * mpu6886_q1 + mpu6886_q2 * mpu6886_q2 + mpu6886_q3 * mpu6886_q3);
    mpu6886_q0 = mpu6886_q0*recipNorm;
    mpu6886_q1 = mpu6886_q1*recipNorm;
    mpu6886_q2 = mpu6886_q2*recipNorm;
    mpu6886_q3 = mpu6886_q3*recipNorm;

    if (pitch != nullptr) {
        *pitch = asin(-2 * mpu6886_q1 * mpu6886_q3 + 2 * mpu6886_q0 * mpu6886_q2);  // pitch
        *pitch *= RAD_TO_DEG;
    }
    if (roll != nullptr) {
        *roll = atan2(2 * mpu6886_q2 * mpu6886_q3 + 2 * mpu6886_q0 * mpu6886_q1, -2 * mpu6886_q1 * mpu6886_q1 - 2 * mpu6886_q2 * mpu6886_q2 + 1);  // roll
        *roll *= RAD_TO_DEG;
    }
    if (yaw != nullptr) {
        *yaw = atan2(2 * (mpu6886_q1 * mpu6886_q2 + mpu6886_q0 * mpu6886_q3), mpu6886_q0 * mpu6886_q0 + mpu6886_q1 * mpu6886_q1 - mpu6886_q2 * mpu6886_q2 - mpu6886_q3 * mpu6886_q3);  // yaw
        *yaw *= RAD_TO_DEG;
        // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
        // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
        // - http://www.ngdc.noaa.gov/geomag-web/#declination
        *yaw -= 8.5;
    }
}

bool mpu6886_i2c_read_bytes(I2C_TYPE w, uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t count) {
#if ARDUINO
    w.beginTransmission(address);
    w.write(reg);
    uint8_t i = 0;
    if (w.endTransmission(false) == 0 && w.requestFrom(address, (uint8_t)count)) {
        while (w.available()) {
            buffer[i++] = w.read();
        }
        return true;
    }
    return false;
#else
    return (ESP_OK==i2c_master_write_read_device(w,address,&reg,1,buffer,count,pdMS_TO_TICKS(1000)));
#endif
}
bool mpu6886_i2c_write_bytes(I2C_TYPE w, uint8_t address, uint8_t reg, uint8_t* data, uint8_t length) {
#ifdef ARDUINO
    w.beginTransmission(address);
    w.write(reg);
    for (int i = 0; i < length; i++) {
        w.write(*(data + i));
    }
    return (w.endTransmission() == 0);  // Send the Tx buffer
#else
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write(cmd,data,length,ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t res = i2c_master_cmd_begin(w, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return res==ESP_OK;
#endif
}
#ifdef ARDUINO
namespace arduino {
#else
namespace esp_idf {
#endif
void mpu6886::do_move(mpu6886& rhs) {
    m_acc_scale=rhs.m_acc_scale;
    m_ares = rhs.m_ares;
    m_gres=rhs.m_gres;
    m_gyro_scale = rhs.m_gyro_scale;
    m_i2c = rhs.m_i2c;
    m_imuId = rhs.m_imuId;
    m_initialized = rhs.m_initialized;
}
mpu6886::mpu6886(mpu6886&& rhs) : m_i2c(rhs.m_i2c) {
    do_move(rhs);
}
mpu6886& mpu6886::operator=(mpu6886&& rhs) {
    do_move(rhs);
    return *this;
}
bool mpu6886::initialize() {
    if (!m_initialized) {
        m_initialized = true;
        unsigned char tempdata[1];
        unsigned char regdata;

        m_gyro_scale = mpu6886_gyro_scale::scale_2000dps;
        m_acc_scale = mpu6886_acc_scale::scale_8g;
#ifdef ARDUINO
        m_i2c.begin();
#endif
        mpu6886_i2c_read_bytes(m_i2c, address, op_whoami, tempdata, 1);
        m_imuId = tempdata[0];
        
        mpu6886_delay(1);

        regdata = 0x00;
        mpu6886_i2c_write_bytes(m_i2c, address, op_pwr_mgmt_1, &regdata, 1);
        mpu6886_delay(10);

        regdata = (0x01 << 7);
        mpu6886_i2c_write_bytes(m_i2c, address, op_pwr_mgmt_1, &regdata, 1);
        mpu6886_delay(10);

        regdata = (0x01 << 0);
        mpu6886_i2c_write_bytes(m_i2c, address, op_pwr_mgmt_1, &regdata, 1);
        mpu6886_delay(10);

        // +- 8g
        regdata = 0x10;
        mpu6886_i2c_write_bytes(m_i2c, address, op_accel_config, &regdata, 1);
        mpu6886_delay(1);

        // +- 2000 dps
        regdata = 0x18;
        mpu6886_i2c_write_bytes(m_i2c, address, op_gyro_config, &regdata, 1);
        mpu6886_delay(1);

        // 1khz output
        regdata = 0x01;
        mpu6886_i2c_write_bytes(m_i2c, address, op_config, &regdata, 1);
        mpu6886_delay(1);

        // 2 div, FIFO 500hz out
        regdata = 0x01;
        mpu6886_i2c_write_bytes(m_i2c, address, op_smplrt_div, &regdata, 1);
        mpu6886_delay(1);

        regdata = 0x00;
        mpu6886_i2c_write_bytes(m_i2c, address, op_int_enable, &regdata, 1);
        mpu6886_delay(1);

        regdata = 0x00;
        mpu6886_i2c_write_bytes(m_i2c, address, op_accel_config2, &regdata, 1);
        mpu6886_delay(1);

        regdata = 0x00;
        mpu6886_i2c_write_bytes(m_i2c, address, op_user_ctrl, &regdata, 1);
        mpu6886_delay(1);

        regdata = 0x00;
        mpu6886_i2c_write_bytes(m_i2c, address, op_fifo_en, &regdata, 1);
        mpu6886_delay(1);

        regdata = 0x22;
        mpu6886_i2c_write_bytes(m_i2c, address, op_int_pin_cfg, &regdata, 1);
        mpu6886_delay(1);

        regdata = 0x01;
        mpu6886_i2c_write_bytes(m_i2c, address, op_int_enable, &regdata, 1);

        mpu6886_delay(10);
        m_initialized = true;
        gyro_scale(m_gyro_scale);
        acc_scale(m_acc_scale);
    }
    return m_initialized;
}

// Possible gyro scales (and their register bit settings)
void mpu6886::update_gres() {
    switch (m_gyro_scale) {
        case mpu6886_gyro_scale::scale_250dps:
            m_gres = 250.0 / 32768.0;
            break;
        case mpu6886_gyro_scale::scale_500dps:
            m_gres = 500.0 / 32768.0;
            break;
        case mpu6886_gyro_scale::scale_1000dps:
            m_gres = 1000.0 / 32768.0;
            break;
        case mpu6886_gyro_scale::scale_2000dps:
            m_gres = 2000.0 / 32768.0;
            break;
    }
}

// Possible accelerometer scales (and their register bit settings) are:
// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
void mpu6886::update_ares() {
    switch (m_acc_scale) {
        case mpu6886_acc_scale::scale_2g:
            m_ares = 2.0 / 32768.0;
            break;
        case mpu6886_acc_scale::scale_4g:
            m_ares = 4.0 / 32768.0;
            break;
        case mpu6886_acc_scale::scale_8g:
            m_ares = 8.0 / 32768.0;
            break;
        case mpu6886_acc_scale::scale_16g:
            m_ares = 16.0 / 32768.0;
            break;
    }
}
void mpu6886::acc_adc(int16_t* ax, int16_t* ay, int16_t* az) {
    initialize();
    uint8_t buf[6];
    mpu6886_i2c_read_bytes(m_i2c, address, op_accel_xout_h, buf, 6);
    *ax = ((int16_t)buf[0] << 8) | buf[1];
    *ay = ((int16_t)buf[2] << 8) | buf[3];
    *az = ((int16_t)buf[4] << 8) | buf[5];
}

void mpu6886::gyro_adc(int16_t* gx, int16_t* gy, int16_t* gz) {
    initialize();
    uint8_t buf[6];
    mpu6886_i2c_read_bytes(m_i2c, address, op_gyro_xout_h, buf, 6);
    *gx = ((uint16_t)buf[0] << 8) | buf[1];
    *gy = ((uint16_t)buf[2] << 8) | buf[3];
    *gz = ((uint16_t)buf[4] << 8) | buf[5];
}

int16_t mpu6886::temp_adc() {
    initialize();
    uint8_t buf[2];
    mpu6886_i2c_read_bytes(m_i2c, address, op_temp_out_h, buf, 2);
    return ((uint16_t)buf[0] << 8) | buf[1];
}

void mpu6886::gyro_scale(mpu6886_gyro_scale value) {
    initialize();
    unsigned char regdata;
    regdata = ((uint8_t)value << 3);
    mpu6886_i2c_write_bytes(m_i2c, address, op_gyro_config, &regdata, 1);
    mpu6886_delay(10);
    m_gyro_scale = value;
    update_gres();
}
void mpu6886::gyro_offset(float x, float y, float z) {
    initialize();
  uint8_t buf_out[6];
  int16_t xx = x / m_gres;
  int16_t yy = y / m_gres;
  int16_t zz = z / m_gres;
  buf_out[0] = xx >> 8;
  buf_out[1] = xx & 0xff;
  buf_out[2] = yy >> 8;
  buf_out[3] = yy & 0xff;
  buf_out[4] = zz >> 8;
  buf_out[5] = zz & 0xff;
  mpu6886_i2c_write_bytes(m_i2c,address, op_gyro_offset, buf_out, 6);
}
void mpu6886::acc_scale(mpu6886_acc_scale scale) {
    initialize();
    unsigned char regdata;
    regdata = ((uint8_t)scale << 3);
    mpu6886_i2c_write_bytes(m_i2c, address, op_accel_config, &regdata, 1);
    mpu6886_delay(10);
    m_acc_scale = scale;
    update_ares();
}
void mpu6886::acc(float* ax, float* ay, float* az) {
    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    acc_adc(&accX, &accY, &accZ);
    if(ax!=nullptr) {
        *ax = (float)accX * m_ares;
    }
    if(ay!=nullptr) {
        *ay = (float)accY * m_ares;
    }
    if(az!=nullptr) {
        *az = (float)accZ * m_ares;
    }
}

void mpu6886::gyro(float* gx, float* gy, float* gz) {
    int16_t gyroX = 0;
    int16_t gyroY = 0;
    int16_t gyroZ = 0;
    gyro_adc(&gyroX, &gyroY, &gyroZ);
    if(gx!=nullptr) {
        *gx = (float)gyroX * m_gres;
    }
    if(gy!=nullptr) {
        *gy = (float)gyroY * m_gres;
    }
    if(gz!=nullptr) {
        *gz = (float)gyroZ * m_gres;
    }
}

float mpu6886::temp() {
    return (float)temp_adc() / 326.8 + 25.0;
}
void mpu6886::ahrs(float *pitch, float *roll, float *yaw) {

  float accX = 0; 
  float accY = 0;
  float accZ = 0;

  float gyroX = 0;
  float gyroY = 0;
  float gyroZ = 0;


  gyro(&gyroX, &gyroY, &gyroZ);
  acc(&accX, &accY, &accZ);
  mpu6886_mahony_ahrs_update_imu(gyroX * DEG_TO_RAD, gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, accX, accY, accZ, pitch, roll, yaw);
}

}  // namespace arduino