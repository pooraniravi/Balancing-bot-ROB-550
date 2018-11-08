/*******************************************************************************
* measure_motors.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the parameters for your motors
*
*******************************************************************************/
#include <stdio.h>
#include <rc/mpu.h>
#define I2C_BUS 2

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(int argc, char** argv) {
    printf("\nThis program will generate a new gyro calibration file\n");
    printf("keep your board very still for this procedure.\n");
    printf("Press any key to continue\n");
    getchar();
    printf("Starting calibration routine\n");
    rc_mpu_config_t config = rc_mpu_default_config();
    config.i2c_bus = I2C_BUS;
    if(rc_mpu_calibrate_gyro_routine(config)<0){
        printf("Failed to complete gyro calibration\n");
        return -1;
    }
    printf("\ngyro calibration file written\n");
    printf("run rc_test_mpu to check performance\n");

    printf("\nThis program will generate a new accelerometer calibration file\n");
    printf("Press any key to continue\n");
    getchar();
    printf("Starting calibration routine\n");
    if(rc_mpu_calibrate_accel_routine(config)<0){
        printf("Failed to complete accelerometer calibration\n");
        return -1;
    }
    printf("\nacceleometer calibration file written\n");
    printf("run rc_test_mpu to check performance\n");

    return 0;
}