#ifndef MPU_6050_H_   /* Include guard */
#define MPU_6050_H_

#include "pico/stdlib.h"
#include "hardware/i2c.h"

typedef struct {
    float angle;
    float rate;

    float rate_calibration;

    float KalmanAngle;
    float KalmanUncertaintyAngle;
  

} MPU6050_dataframe;


int MPU_6050_init(i2c_inst_t *i2c);
void MPU_6050_reset(i2c_inst_t *i2c);
uint8_t MPU_6050_get_address(i2c_inst_t *i2c);
void MPU_6050_update_pitch_info(i2c_inst_t *i2c, MPU6050_dataframe *dataframe);
void MPU_6050_calibrate_pitch(i2c_inst_t *i2c, MPU6050_dataframe *dataframe);
void MPU_6050_kalman_1d(MPU6050_dataframe *dataframe); 


#endif // FOO_H_