#ifndef HMC_5883L_H_   /* Include guard */
#define HMC_5883L_H_

#include "pico/stdlib.h"
#include "hardware/i2c.h"

typedef struct {
    uint8_t data_rate;
    uint8_t data_avg;
    uint8_t gain_setting;
    uint8_t mode_setting;

    int16_t heading;
} HMC5883L_dataframe;


int HMC5883L_init(i2c_inst_t *i2c);
uint8_t HMC5883L_get_id(i2c_inst_t *i2c);
void HMC5883L_update_measurement(i2c_inst_t *i2c, HMC5883L_dataframe *dataframe);
void HMC5883L_config(i2c_inst_t *i2c, HMC5883L_dataframe *dataframe);


#endif // FOO_H_