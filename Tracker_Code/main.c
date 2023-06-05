
#include <stdio.h>
#include "mpu_6050.h"  /* Include the header here, to obtain the function declaration */
#include "hmc5883l.h"
#include "pico/stdlib.h"
#include <stdlib.h>
#include "hardware/i2c.h"


// receive non_blocking chars from input
int read_serial(char* buffer, int length){
    int c;
    for (int i = 0; i<length; i++){
        c = getchar_timeout_us(1);
        if (c >= 0){
            buffer[i] = (char)c;
        } else {
            return 2;
        }	
    }
    return 1;
}

float calculate_pitch_offset(MPU6050_dataframe* frame, i2c_inst_t*i2c){
    
    // run the Kalman filter for some iterrations to get an equilibrium
    float elev_offset = 0;
    // calculate elev_offset
    for (int i = 0; i<4000; i++){
        uint64_t start = time_us_64();
        MPU_6050_update_pitch_info(i2c,frame);
        // subtract callibration value
        frame->rate -= frame->rate_calibration;
        // update Kalman angle
        MPU_6050_kalman_1d(frame);
        while(time_us_64() - start < 4000);
    }
    
    // take measurements and zero out elevation
    for (int i = 0; i<100; i++){
        uint64_t start = time_us_64();
        MPU_6050_update_pitch_info(i2c,frame);
        // subtract callibration value
        frame->rate -= frame->rate_calibration;
        // update Kalman angle
        MPU_6050_kalman_1d(frame);
        elev_offset += frame->KalmanAngle;
        printf("elev_offset = %f\n", elev_offset);
        // wait for next measurement
        while(time_us_64() - start < 4000);
    }
    return elev_offset/100.0;

}





int main(void)
{   

    // Pins
    const uint sda_pin = 16;
    const uint scl_pin = 17;
    const uint led_pin = 25;
    const uint AZ_R = 21 , AZ_L = 20 , EL_U = 18 , EL_D = 19; // AZ/EL Pins
    const uint led_red = 14, led_green = 15;


    // I2C handles
    i2c_inst_t *i2c = i2c0;
    //Initialize I2C port at 400 kHz
    i2c_init(i2c, 400 * 1000);

    // Initialize I2C pins
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    // Data frames
    MPU6050_dataframe frame;
    HMC5883L_dataframe compass_frame;

    // Initialize chosen serial port
    stdio_init_all();

    // Initialize LED pins
    gpio_init(led_pin);
    gpio_init(led_green);
    gpio_init(led_red);
    gpio_set_dir(led_pin, GPIO_OUT);
    gpio_set_dir(led_green, GPIO_OUT);
    gpio_set_dir(led_red, GPIO_OUT);

    // Initialize MOSFET pins
    gpio_init(AZ_L);
    gpio_set_dir(AZ_L, GPIO_OUT);
    gpio_init(AZ_R);
    gpio_set_dir(AZ_R, GPIO_OUT);
    gpio_init(EL_D);
    gpio_set_dir(EL_D, GPIO_OUT);
    gpio_init(EL_U);
    gpio_set_dir(EL_U, GPIO_OUT);

    // Initialize Sensors
    int status_mpu6050 = MPU_6050_init(i2c);
    int status_hmc5883l = HMC5883L_init(i2c);
    
    if (status_mpu6050 == 1 && status_hmc5883l == 1){
        printf("Sensors sucessfully detected!\n");
        gpio_put(led_pin, 1);
    } else {
        printf("Sensors not detected!\n");
    }



    sleep_ms(1000);

    // calibrate gyro 
    MPU_6050_calibrate_pitch(i2c, &frame);

    // setup compass
    compass_frame.data_rate = 5;
    compass_frame.data_avg = 3;
    compass_frame.gain_setting = 1;
    compass_frame.mode_setting = 0;
    HMC5883L_config(i2c, &compass_frame);
    

    float elev_offset = 0;
    
    // calculate offset for pitch (optional)
    // elev_offset = calculate_pitch_offset(&frame, i2c);

    
    // loop for tracker
    char buffer[8] = {0};

    int heading_gr = 0;
    int elevation_gr = 0;

    bool heading_motor_on = false;
    bool elevation_motor_on = false;

    int error = 6;

    
    // example reception
    // # AZ: 180°, EL: 45° -> 18000450
    char c = 0;

    while(true){
        // make loop time measurement
        uint64_t start = time_us_64();

        // check, if new heading/elevation arrive from serial
        int status = read_serial(buffer, 8);

        if (status == 1){
            //printf("%.8s\n", buffer);
            heading_gr = 100 * (buffer[0] - '0') + 10 * (buffer[1] - '0') + buffer[2] - '0';
            elevation_gr = 100 * (buffer[4] - '0') + 10 * (buffer[5] - '0') + buffer[6] - '0';
        }

        // update measurement
        MPU_6050_update_pitch_info(i2c,&frame);
        // subtract callibration value
        frame.rate -= frame.rate_calibration;
        // update Kalman angle
        MPU_6050_kalman_1d(&frame);
        // HC5883L heading measurement
        HMC5883L_update_measurement(i2c, &compass_frame);


        // compare to heading/elevation from sensors
        // if error is larger than a predefined value --> rotate
        int elev_error = (int)frame.KalmanAngle - elevation_gr  - elev_offset ;

        if (!elevation_motor_on){
            if (elev_error > error){
                //printf("elev_rotate down\n");
                gpio_put(EL_D, 1);
                gpio_put(led_red, 1);

                elevation_motor_on = true;
            } else if (elev_error < -error){
                //printf("elev_rotate up\n");
                elevation_motor_on = true;
                gpio_put(EL_U, 1);
                gpio_put(led_red, 1);

            } 
        } else {
            if (abs(elev_error) < error/2){
                //printf("elev_rotate stop\n");
                elevation_motor_on = false;
                gpio_put(EL_U, 0);
                gpio_put(EL_D, 0);
                gpio_put(led_red, 0);

            }
        }

        
        int heading_error = heading_gr - compass_frame.heading;
        if (abs(heading_error) > 180){
            if (compass_frame.heading > heading_gr){
                heading_error = heading_error + 360;
            } else {
                heading_error = -360 + heading_error;
            }
        }


        if (!heading_motor_on){
            if (heading_error > error){
                //printf("head_rotate clockwise\n");
                gpio_put(AZ_R, 1);
                gpio_put(led_green, 1);
                heading_motor_on = true;
            } else if (heading_error < -error){
                //printf("head_rotate anticlockwise\n");
                gpio_put(AZ_L, 1);
                gpio_put(led_green, 1);
                heading_motor_on = true;
            } 
        } else {
            if (abs(heading_error) < error/2){
                //printf("head_rotate stop\n");
                gpio_put(AZ_L, 0);
                gpio_put(AZ_R, 0);
                gpio_put(led_green, 0);
                heading_motor_on = false;
            }
        }

        printf("pitch_angle = %f, heading = %d\n", frame.KalmanAngle - (int)elev_offset, compass_frame.heading);

        // wait for next measurement
        while(time_us_64() - start < 4000);

    }
    
}