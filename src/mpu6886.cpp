#include "mpu6886.hpp"
#include <math.h>
#ifndef ARDUINO
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif


#define MPU6886_ADDRESS          0x68
#define MPU6886_WHOAMI           0x75
#define MPU6886_ACCEL_INTEL_CTRL 0x69
#define MPU6886_SMPLRT_DIV       0x19
#define MPU6886_INT_PIN_CFG      0x37
#define MPU6886_INT_ENABLE       0x38
#define MPU6886_ACCEL_XOUT_H     0x3B
#define MPU6886_ACCEL_XOUT_L     0x3C
#define MPU6886_ACCEL_YOUT_H     0x3D
#define MPU6886_ACCEL_YOUT_L     0x3E
#define MPU6886_ACCEL_ZOUT_H     0x3F
#define MPU6886_ACCEL_ZOUT_L     0x40

#define MPU6886_TEMP_OUT_H 0x41
#define MPU6886_TEMP_OUT_L 0x42

#define MPU6886_GYRO_XOUT_H 0x43
#define MPU6886_GYRO_XOUT_L 0x44
#define MPU6886_GYRO_YOUT_H 0x45
#define MPU6886_GYRO_YOUT_L 0x46
#define MPU6886_GYRO_ZOUT_H 0x47
#define MPU6886_GYRO_ZOUT_L 0x48

#define MPU6886_USER_CTRL     0x6A
#define MPU6886_PWR_MGMT_1    0x6B
#define MPU6886_PWR_MGMT_2    0x6C
#define MPU6886_CONFIG        0x1A
#define MPU6886_GYRO_CONFIG   0x1B
#define MPU6886_ACCEL_CONFIG  0x1C
#define MPU6886_ACCEL_CONFIG2 0x1D
#define MPU6886_FIFO_EN       0x23

#define sampleFreq 25.0f          // sample frequency in Hz
#define twoKpDef   (2.0f * 1.0f)  // 2 * proportional gain
#define twoKiDef   (2.0f * 0.0f)  // 2 * integral gain
#ifndef ARDUINO
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif

mpu6886::mpu6886(
#ifdef ARDUINO
        TwoWire& i2c
#else
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        i2c_master_bus_handle_t i2c
#else
        i2c_port_t i2c = I2C_NUM_0
#endif
#endif
) :
#if defined(ARDUINO) || ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        m_i2c(i2c),
#else
        m_i2c_bus(i2c),
        m_i2c(nullptr),
#endif
    m_initialized(false)
{

}

void mpu6886::read_reg(uint8_t start_Addr,
                              uint8_t number_Bytes, uint8_t* read_Buffer) {
#ifdef ARDUINO
    m_i2c.beginTransmission(MPU6886_ADDRESS);
    m_i2c.write(start_Addr);
    m_i2c.endTransmission(false);
    uint8_t i = 0;
    m_i2c.requestFrom((uint8_t)MPU6886_ADDRESS,(uint8_t)number_Bytes);

    //! Put read results in the Rx buffer
    while (m_i2c.available()) {
        read_Buffer[i++] = m_i2c.read();
    }
#else
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    i2c_master_transmit_receive(m_i2c,&start_Addr,1,read_Buffer,number_Bytes,1000);
#else
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6886_ADDRESS << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, start_Addr, I2C_MASTER_ACK);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6886_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &result, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(m_i2c, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);    
#endif
#endif
}

void mpu6886::write_reg(uint8_t start_Addr,
                               uint8_t number_Bytes, const uint8_t* write_Buffer) {

#ifdef ARDUINO
    m_i2c.beginTransmission(MPU6886_ADDRESS);
    m_i2c.write(start_Addr);
    m_i2c.write(write_Buffer,number_Bytes);
    m_i2c.endTransmission();
#else
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    i2c_master_transmit(m_i2c,&start_Addr,1,1000);
    i2c_master_transmit(m_i2c,write_Buffer,number_Bytes,1000);
#else
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6886_ADDRESS << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, start_Addr, ACK_CHECK_EN);
    i2c_master_write(cmd, write_Buffer,number_Bytes, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(m_i2c, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
#endif
#endif
}

bool mpu6886::initialize() {
    if(!m_initialized) {
        unsigned char tempdata[1];
        unsigned char regdata;

#ifdef ARDUINO
        m_i2c.begin();
#else
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        i2c_device_config_t dev_cfg;
        memset(&dev_cfg,0,sizeof(dev_cfg));
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address = MPU6886_ADDRESS;
        dev_cfg.scl_speed_hz = 100*1000;
        if(ESP_OK!=i2c_master_bus_add_device(m_i2c_bus, &dev_cfg, &m_i2c)) {
            return false;
        }
#endif
#endif

        read_reg( MPU6886_WHOAMI, 1, tempdata);
        if (tempdata[0] != 0x19) { 
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
            i2c_master_bus_rm_device(m_i2c);
#endif
            return false;
        }
        delay(1);

        regdata = 0x00;
        write_reg( MPU6886_PWR_MGMT_1, 1, &regdata);
        delay(10);

        regdata = (0x01 << 7);
        write_reg( MPU6886_PWR_MGMT_1, 1, &regdata);
        delay(10);

        regdata = (0x01 << 0);
        write_reg( MPU6886_PWR_MGMT_1, 1, &regdata);
        delay(10);

        regdata = 0x10;
        write_reg( MPU6886_ACCEL_CONFIG, 1, &regdata);
        delay(1);

        regdata = 0x18;
        write_reg( MPU6886_GYRO_CONFIG, 1, &regdata);
        delay(1);

        regdata = 0x01;
        write_reg( MPU6886_CONFIG, 1, &regdata);
        delay(1);

        regdata = 0x05;
        write_reg( MPU6886_SMPLRT_DIV, 1, &regdata);
        delay(1);

        regdata = 0x00;
        write_reg( MPU6886_INT_ENABLE, 1, &regdata);
        delay(1);

        regdata = 0x00;
        write_reg( MPU6886_ACCEL_CONFIG2, 1, &regdata);
        delay(1);

        regdata = 0x00;
        write_reg( MPU6886_USER_CTRL, 1, &regdata);
        delay(1);

        regdata = 0x00;
        write_reg( MPU6886_FIFO_EN, 1, &regdata);
        delay(1);

        regdata = 0x22;
        write_reg( MPU6886_INT_PIN_CFG, 1, &regdata);
        delay(1);

        regdata = 0x01;
        write_reg( MPU6886_INT_ENABLE, 1, &regdata);

        delay(100);
        update_gres();
        update_ares();
        m_initialized = true;
    }
    return m_initialized;
}

void mpu6886::acc_raw_xyz(int16_t* ax, int16_t* ay, int16_t* az) {
    uint8_t buf[6];
    read_reg( MPU6886_ACCEL_XOUT_H, 6, buf);

    *ax = ((int16_t)buf[0] << 8) | buf[1];
    *ay = ((int16_t)buf[2] << 8) | buf[3];
    *az = ((int16_t)buf[4] << 8) | buf[5];
}
void mpu6886::gyro_raw_xyz(int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t buf[6];
    read_reg( MPU6886_GYRO_XOUT_H, 6, buf);

    *gx = ((uint16_t)buf[0] << 8) | buf[1];
    *gy = ((uint16_t)buf[2] << 8) | buf[3];
    *gz = ((uint16_t)buf[4] << 8) | buf[5];
}

void mpu6886::temp_raw(int16_t* t) {
    uint8_t buf[2];
    read_reg( MPU6886_TEMP_OUT_H, 2, buf);

    *t = ((uint16_t)buf[0] << 8) | buf[1];
}

//!俯仰，航向，横滚：pitch，yaw，roll，指三维空间中飞行器的旋转状态。
void mpu6886::ahrs(float* pitch, float* roll, float* yaw) {
    float accX = 0;
    float accY = 0;
    float accZ = 0;

    float gyroX = 0;
    float gyroY = 0;
    float gyroZ = 0;

    gyro_xyz(&gyroX, &gyroY, &gyroZ);
    acc_xyz(&accX, &accY, &accZ);

    mahony_ahrs_update_imu(gyroX * DEG_TO_RAD, gyroY * DEG_TO_RAD,
                        gyroZ * DEG_TO_RAD, accX, accY, accZ, pitch, roll, yaw);
}

void mpu6886::attitude(double* pitch, double* roll) {
    float accX = 0;
    float accY = 0;
    float accZ = 0;

    float gyroX = 0;
    float gyroY = 0;
    float gyroZ = 0;

    gyro_xyz(&gyroX, &gyroY, &gyroZ);
    acc_xyz(&accX, &accY, &accZ);

    if ((accX < 1) && (accX > -1)) {
        *pitch = asin(-accX) * 57.295;
    }
    if (accZ != 0) {
        *roll = atan(accY / accZ) * 57.295;
    }

    (*pitch) = _alpha * (*pitch) + (1 - _alpha) * _last_theta;
    (*roll)  = _alpha * (*roll) + (1 - _alpha) * _last_phi;
}

void mpu6886::update_gres() {
    switch (Gyscale) {
            // Possible gyro scales (and their register bit settings) are:
        case mpu6886_dps::dps250:
            gRes = 250.0 / 32768.0;
            break;
        case mpu6886_dps::dps500:
            gRes = 500.0 / 32768.0;
            break;
        case mpu6886_dps::dps1000:
            gRes = 1000.0 / 32768.0;
            break;
        case mpu6886_dps::dps2000:
            gRes = 2000.0 / 32768.0;
            break;
    }
}

void mpu6886::update_ares() {
    switch (Acscale) {
            // Possible accelerometer scales (and their register bit settings)
            // are: 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). Here's a
            // bit of an algorith to calculate DPS/(ADC tick) based on that
            // 2-bit value:
        case mpu6886_scale::g2:
            aRes = 2.0 / 32768.0;
            break;
        case mpu6886_scale::g4:
            aRes = 4.0 / 32768.0;
            break;
        case mpu6886_scale::g8:
            aRes = 8.0 / 32768.0;
            break;
        case mpu6886_scale::g16:
            aRes = 16.0 / 32768.0;
            break;
    }
}
mpu6886_dps mpu6886::gyro_dps() const {
    return Gyscale;
}
void mpu6886::gyro_dps(mpu6886_dps value) {
    // return IIC_Write_Byte(MPU_GYRO_CFG_REG,scale<<3);//设置陀螺仪满量程范围
    unsigned char regdata;
    regdata = (((int)value) << 3);
    write_reg( MPU6886_GYRO_CONFIG, 1, &regdata);
    delay(10);

    Gyscale = value;
    update_gres();
}
mpu6886_scale mpu6886::acc_scale() const {
    return Acscale;
}
void mpu6886::acc_scale(mpu6886_scale value) {
    unsigned char regdata;
    regdata = (((int)value) << 3);
    write_reg( MPU6886_ACCEL_CONFIG, 1, &regdata);
    delay(10);

    Acscale = value;
    update_ares();
}

void mpu6886::acc_xyz(float* ax, float* ay, float* az) {
    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    acc_raw_xyz(&accX, &accY, &accZ);

    *ax = (float)accX * aRes;
    *ay = (float)accY * aRes;
    *az = (float)accZ * aRes;
}

void mpu6886::gyro_xyz(float* gx, float* gy, float* gz) {
    int16_t gyroX = 0;
    int16_t gyroY = 0;
    int16_t gyroZ = 0;
    gyro_raw_xyz(&gyroX, &gyroY, &gyroZ);

    *gx = (float)gyroX * gRes;
    *gy = (float)gyroY * gRes;
    *gz = (float)gyroZ * gRes;
}

void mpu6886::temp(float* t) {
    int16_t temp = 0;
    temp_raw(&temp);

    *t = (float)temp / 326.8 + 25.0;
}

void mpu6886::mahony_ahrs_update(float gx, float gy, float gz, float ax, float ay,
                      float az, float mx, float my, float mz) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in
    // magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        // MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // Compute feedback only if accelerometer measurement valid (avoids NaN in
    // accelerometer normalisation)
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
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) +
                     mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) +
                     mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) +
                     mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and
        // measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex *
                           (1.0f / sampleFreq);  // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx;  // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;  // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq));  // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void mpu6886::mahony_ahrs_update_imu(float gx, float gy, float gz, float ax, float ay,
                         float az, float *pitch, float *roll, float *yaw) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in
    // accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic
        // flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured
        // direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex *
                           (1.0f / sampleFreq);  // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx;  // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;  // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq));  // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2);  // pitch
    *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1,
                  -2 * q1 * q1 - 2 * q2 * q2 + 1);  // roll
    *yaw   = atan2(2 * (q1 * q2 + q0 * q3),
                 q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);  // yaw

    *pitch *= RAD_TO_DEG;
    *yaw *= RAD_TO_DEG;
    // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
    // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    *yaw -= 8.5;
    *roll *= RAD_TO_DEG;

    /// Serial.printf("%f    %f    %f \r\n",  pitch, roll, yaw);
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float mpu6886::inv_sqrt(float x) {
    float halfx = 0.5f * x;
    float y  =0;
    y  = x;
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
    long i = *(long *)&y;
    i      = 0x5f3759df - (i >> 1);
    y      = *(float *)&i;
#pragma GCC diagnostic warning "-Wstrict-aliasing"
    y = y * (1.5f - (halfx * y * y));
    return y;
}
#ifndef ARDUINO
void mpu6886::delay(uint32_t ms) { 
    vTaskDelay(pdMS_TO_TICKS(ms)); 
}
#endif