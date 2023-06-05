#include "hmc5883l.h"  /* Include the header (not strictly necessary here) */
#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <math.h>


static const uint8_t ADDR = 0x1E;
static const uint8_t IRA = 0xA;
static const uint8_t DO_X_H = 0x03;
static const uint8_t CONF_REG_A = 0x00;
static const uint8_t CONF_REG_B = 0x01;
static const uint8_t MODE_REG = 0x02;


int HMC5883L_init(i2c_inst_t *i2c){
    
    //read identification register A in order to check if the HMC5883L is working
    uint8_t id = HMC5883L_get_id(i2c);

    if (id == 72){
        return 1;
    }
    return 0;
}
uint8_t HMC5883L_get_id(i2c_inst_t *i2c){
    uint8_t id = 0;
    i2c_write_blocking(i2c, ADDR, &IRA, 1, true);
    uint8_t num_bytes_received = i2c_read_blocking(i2c, ADDR, &id, 1, false);
    return id;

}
void HMC5883L_update_measurement(i2c_inst_t *i2c, HMC5883L_dataframe *dataframe){
    // read out heading z register
    uint8_t buff[6];
    i2c_write_blocking(i2c, ADDR, &DO_X_H, 1, true);
    uint8_t num_bytes_received = i2c_read_blocking(i2c, ADDR, buff, 6, false);
    int16_t magnetic_x = (buff[0] << 8) | buff[1];
    int16_t magnetic_y = (buff[4] << 8) | buff[5];

    float heading = atan2(magnetic_y, magnetic_x);
    //float declinationAngle = 0;
    //heading += declinationAngle;
    if (heading < 0) {
        heading += 2 * 3.141592;
    }
    if (heading > 2 * 3.141592) {
        heading -= 2 * 3.141592;
    }
    dataframe->heading = heading * 180 / 3.141592;
    
}

void HMC5883L_config(i2c_inst_t *i2c, HMC5883L_dataframe *dataframe){
    // Configuration register A
    uint8_t buff[2] = {0,0};
    buff[0] = CONF_REG_A;
    buff[1] |= (dataframe->data_avg) << 5;
    buff[1] |= (dataframe->data_rate) << 2;
    i2c_write_blocking(i2c, ADDR, buff, 2, false);

    // Configueration register B
    buff[0] = CONF_REG_B;
    buff[1] = (dataframe->gain_setting) << 5;
    i2c_write_blocking(i2c, ADDR, buff, 2, false);

    // Mode register
    buff[0] = MODE_REG;
    buff[1] = dataframe->mode_setting;
    i2c_write_blocking(i2c, ADDR, buff, 2, false);


}