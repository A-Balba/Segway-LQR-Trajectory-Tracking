
#include <MPU6050.h>          
#include <Arduino.h>
#include <Wire.h>
#include <avr/io.h>
#include <util/delay.h>
#include <HardwareSerial.h>

#include <interfaces/mpu.h>

#define I2C_BAUD_RATE 400000 // 400 kHz

#define GYRO_SENSITIVITY 131.0
#define ACCEL_SENSITIVITY 16384.0

const double GS = (1 / GYRO_SENSITIVITY) * DEG_TO_RAD; 
    // converts to deg/sec by multiplying with resolution, then to rad/sec

MPU6050 mpu;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int16_t gx_mean = 0;
int16_t ay_mean = 0;
int16_t az_mean = 0;

double prev_comp_angle = 0;

void mpu_init() {
    // Init I2C for sensors
    Wire.begin();
    Wire.setClock(I2C_BAUD_RATE);

    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1);
    }

    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    // mpu.setDHPFMode(MPU6050_DLPF_BW_42);
    mpu.setDLPFMode(MPU6050_DLPF_BW_20);

    // Sample rate = 1kHz / (1 + divider)
    mpu.setRate(1);   // 1kHz / (9+1) = 100 Hz output
}


void mpu_calibrate() {

    const uint16_t sample_count = 100;

    // int16_t ax_log[sample_count];
    int16_t ay_log[sample_count];
    int16_t az_log[sample_count];
    int16_t gx_log[sample_count];
    // int16_t gy_log[sample_count];
    // int16_t gz_log[sample_count];

    for (uint16_t i = 0; i < sample_count; i++) {
        delay(20);
        mpu_get_raw_measurements();
        // ax_log[i] = ax;
        ay_log[i] = ay;
        az_log[i] = az;
        gx_log[i] = gx;
        // gy_log[i] = gy;
        // gz_log[i] = gz;
    }

    // calculate mean

    long sum_ay = 0, sum_az = 0, sum_gx = 0;

    for (uint16_t i = 0; i < sample_count; i++) {
        // sum_ax += ax_log[i];
        sum_ay += ay_log[i];
        sum_az += az_log[i];
        sum_gx += gx_log[i];
        // sum_gy += gy_log[i];
        // sum_gz += gz_log[i];
    }

    // float mean_ax = sum_ax / (float)sample_count;
    float mean_ay = (float)sum_ay / (float)sample_count;
    float mean_az = (float)sum_az / (float)sample_count;
    float mean_gx = (float)sum_gx / (float)sample_count;
    // float mean_gy = sum_gy / (float)sample_count;
    // float mean_gz = sum_gz / (float)sample_count;

    // Serial.print("Mean ax: "); Serial.println(mean_ax);
    // Serial.print("Mean ay: "); Serial.println(mean_ay);
    // Serial.print("Mean az: "); Serial.println(mean_az);
    // Serial.print("Mean gx: "); Serial.println(mean_gx);
    // Serial.print("Mean gy: "); Serial.println(mean_gy);
    // Serial.print("Mean gz: "); Serial.println(mean_gz);

    gx_mean = mean_gx;
    ay_mean = mean_ay;
    az_mean = mean_az;
}

void mpu_get_raw_measurements() {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

double mpu_calculate_angular_velocity() {

    double gx_double = (double)gx;

    double gx_bias_corrected = gx_double - (double)gx_mean; // bias correction

    double alpha_dot_radpsec = gx_bias_corrected * GS; // conversion from raw data to radians/sec

    return alpha_dot_radpsec;
}

double mpu_calculate_angle(double alpha_dot_radpsec, double Ts) {

    // Accelerometer measurement
    double az_bias_corrected =  az - az_mean; // bias correction
    double ay_bias_corrected =  ay - (ACCEL_SENSITIVITY + ay_mean); // bias correction

    double alpha_rad = atan2(-az_bias_corrected, -ay_bias_corrected); 
        // minus signs as in Simulink
        // atan2 respects the quadrant

    // Complementary Filter
    double alpha_accelerometer = -alpha_rad;
    double alpha_gyro = prev_comp_angle + (alpha_dot_radpsec * Ts); // Euler forward integration
    double comp_angle = alpha_accelerometer * 0.02 + alpha_gyro * 0.98;

    prev_comp_angle = comp_angle;

    return comp_angle;
}

