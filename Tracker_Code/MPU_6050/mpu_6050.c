#include "mpu_6050.h"  /* Include the header (not strictly necessary here) */
#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <math.h>

static const uint8_t ADDR = 0x68;

static const uint8_t PWR_MGMT_1 = 0x6B;
static const uint8_t WHO_AM_I = 0x75;
static const uint8_t ACCEL_X_H = 0x3B;
static const uint8_t GYRO_X_H = 0x43;


int MPU_6050_init(i2c_inst_t *i2c){
    // reset MPU6050 + sleep
    MPU_6050_reset(i2c);
    sleep_ms(1000);

    //read in MPU6050 address, to check that I2C communication is working
    uint8_t addr = MPU_6050_get_address(i2c);

    if (addr == 52){
        return 1;
    }

    return 0;

    
}

void MPU_6050_reset(i2c_inst_t *i2c){
    
    // set PWE_MGMT_1 to 0x80
    uint8_t pwr_reg_val = 0x00;
    uint8_t buff[2] = {PWR_MGMT_1, pwr_reg_val};
    i2c_write_blocking(i2c, ADDR, buff, 2, false);

}

uint8_t MPU_6050_get_address(i2c_inst_t *i2c)    /* Function definition */
{
    uint8_t addr = 0;
    i2c_write_blocking(i2c, ADDR, &WHO_AM_I, 1, true);
    uint8_t num_bytes_received = i2c_read_blocking(i2c, ADDR, &addr, 1, false);
    return addr>>1;
}


void MPU_6050_update_pitch_info(i2c_inst_t *i2c, MPU6050_dataframe *dataframe)
{
    uint8_t buffer[6];
    int16_t accel[3];
    int16_t gyro[3];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    i2c_write_blocking(i2c, ADDR, &ACCEL_X_H, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c, ADDR, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

   
    float AccX=(float)accel[0]/16384; 
    float AccY=(float)accel[1]/16384;
    float AccZ=(float)accel[2]/16384;
    
    // pitch angle
    dataframe->angle = atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180); 



    // Start reading gyro registers from register 0x3B for 6 bytes
    i2c_write_blocking(i2c, ADDR, &GYRO_X_H, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c, ADDR, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

   
    float GyroX=(float)gyro[0]/131; 
    float GyroY=(float)gyro[1]/131;
    float GyroZ=(float)gyro[2]/131;

    // pitch_rate
    dataframe->rate = GyroY;


}


void MPU_6050_kalman_1d(MPU6050_dataframe *dataframe){

  // Predict Kalman state
  float KalmanState = dataframe->KalmanAngle+0.004*dataframe->rate;

  // Calculate uncertainty of prediction
  float KalmanUncertainty = dataframe->KalmanUncertaintyAngle + 0.004 * 0.004 * 4 * 4;


  // calculate Kalman Gain
  float KalmanGain = KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);

  // Update predicted state and uncertainty
  dataframe->KalmanAngle = KalmanState+KalmanGain * (dataframe->angle-KalmanState);
  dataframe->KalmanUncertaintyAngle=(1-KalmanGain) * KalmanUncertainty;

}

void MPU_6050_calibrate_pitch(i2c_inst_t *i2c, MPU6050_dataframe *dataframe){
    dataframe->rate_calibration = 0;
    for (int i = 0; i<2000; i++){
        MPU_6050_update_pitch_info(i2c,dataframe);
        dataframe->rate_calibration += dataframe->rate;
    }
    dataframe->rate_calibration /= 2000.0;
}