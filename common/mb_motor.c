/*******************************************************************************
* mb_motors.c
*
* Control up to 2 DC motor drivers
*
*******************************************************************************/
#include <stdio.h>
#include <rc/motor.h>
#include <rc/model.h>
#include <rc/gpio.h>
#include <rc/pwm.h>
#include <rc/adc.h>
#include "mb_motor.h"
#include "mb_defs.h"

// preposessor macros
#define unlikely(x) __builtin_expect (!!(x), 0)

// global initialized flag
static int init_flag = 0;
static int dirA_chip[CHANNELS];
static int dirA_pin[CHANNELS];
static int dirB_chip[CHANNELS];
static int dirB_pin[CHANNELS];
static int pwmss[CHANNELS];
static int pwmch[CHANNELS];

/*******************************************************************************
* int mb_motor_init()
* 
* initialize mb_motor with default frequency
*******************************************************************************/
int mb_motor_init(){
    
    return mb_motor_init_freq(MB_MOTOR_DEFAULT_PWM_FREQ);
}

/*******************************************************************************
* int mb_motor_init_freq()
* 
* set up pwm channels, gpio assignments and make sure motors are left off.
*******************************************************************************/
int mb_motor_init_freq(int pwm_freq_hz){
    if (rc_motor_init_freq(pwm_freq_hz)) {
        return -1;
    }
    rc_motor_brake(LEFT_MOTOR);
    rc_motor_brake(RIGHT_MOTOR);

    // TODO setup pwm channels? seems like subsystem 1,2 are for controlling motor drivers
    dirA_chip[RIGHT_MOTOR-1] = MDIR1_CHIP;
    dirA_pin[RIGHT_MOTOR-1] = MDIR1_PIN;

    dirA_chip[LEFT_MOTOR-1] = MDIR2_CHIP;
    dirA_pin[LEFT_MOTOR-1] = MDIR2_PIN;

    init_flag = 1;
    return 0;
}

/*******************************************************************************
* mb_motor_cleanup()
* 
*******************************************************************************/
int mb_motor_cleanup(){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying cleanup before motors have been initialized\n");
        return -1;
    }

    return 0;
}

/*******************************************************************************
* mb_motor_brake()
* 
* allows setting the brake function on the motor drivers
* returns 0 on success, -1 on failure
*******************************************************************************/
int mb_motor_brake(int brake_en){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to enable brake before motors have been initialized\n");
        return -1;
    }

   return 0;
}

/*******************************************************************************
* int mb_disable_motors()
* 
* disables PWM output signals
* returns 0 on success
*******************************************************************************/
int mb_motor_disable(){
    
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to disable motors before motors have been initialized\n");
        return -1;
    }
    
    return 0;
}


/*******************************************************************************
* int mb_motor_set(int motor, double duty)
* 
* set a motor direction and power
* motor is from 1 to 2, duty is from -1.0 to +1.0
* uses the defines in mb_defs.h
* returns 0 on success
*******************************************************************************/
int mb_motor_set(int motor, double duty){
    if(unlikely(motor<0 || motor>CHANNELS)){
        fprintf(stderr,"ERROR in rc_motor_set, motor argument must be between 0 & %d\n", CHANNELS);
        return -1;
    }
    if(unlikely(!init_flag)){
        fprintf(stderr,"ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }
    if	(duty > 1.0) {
      duty = 1.0;
    }
    else if	(duty <-1.0) {
        duty =-1.0;
    }
    return 0;
}

/*******************************************************************************
* int mb_motor_set_all(double duty)
* 
* applies the same duty cycle argument to both motors
*******************************************************************************/
int mb_motor_set_all(double duty){

    if(unlikely(!init_flag)){
        printf("ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }
    
    return 0;
}


/*******************************************************************************
* int mb_motor_read_current(int motor)
* 
* returns the measured current in A
*******************************************************************************/
double mb_motor_read_current(int motor){
    //DRV8801 driver board CS pin puts out 500mV/A
    return 0.0;
}