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
static int chip[CHANNELS];
static int dir_pin[CHANNELS];
static int pwmss[CHANNELS];
static int pwmch[CHANNELS];
static int brake_mode = 1;
static int polarity[CHANNELS];

/*******************************************************************************
* int mb_motor_init()
* 
* initialize mb_motor with default frequency
*******************************************************************************/
int mb_motor_init() {

    return mb_motor_init_freq(MB_MOTOR_DEFAULT_PWM_FREQ);
}

/*******************************************************************************
* int mb_motor_init_freq()
* 
* set up pwm channels, gpio assignments and make sure motors are left off.
*******************************************************************************/
int mb_motor_init_freq(int pwm_freq_hz) {
    if (rc_motor_init_freq(pwm_freq_hz)) {
        return -1;
    }

    // right motor is subsystem 1A
    chip[RIGHT_MOTOR - 1] = MDIR1_CHIP;
    dir_pin[RIGHT_MOTOR - 1] = MDIR1_PIN;
    pwmss[RIGHT_MOTOR - 1] = 1;
    pwmch[RIGHT_MOTOR - 1] = 'A';
    polarity[RIGHT_MOTOR - 1] = MOT_1_POL;

    // left motor is subsystem 1B
    chip[LEFT_MOTOR - 1] = MDIR2_CHIP;
    dir_pin[LEFT_MOTOR - 1] = MDIR2_PIN;
    pwmss[LEFT_MOTOR - 1] = 1;
    pwmch[LEFT_MOTOR - 1] = 'B';
    polarity[LEFT_MOTOR - 1] = MOT_2_POL;

    // set up PWM
    if (unlikely(rc_pwm_init(1, pwm_freq_hz)) {
        fprintf(stderr, "ERROR: failed to initialize pwm subsystem 1 in motor init\n");
        return -1;
    }

    // set up GPIO
    for (int c = 0; c < CHANNELS; ++c) {
        if (unlikely(rc_gpio_init(chip[c], dir_pin[c], GPIOHANDLE_REQUEST_OUTPUT))) {
            fprintf(stderr, "ERROR: failed to set up gpio %d,%d in motor init\n", chip[c], dir_pin[c]);
            return -1;
        }
    }

    // by default motor break is pulled high which is the behaviour we want
    brake_mode = 1;
    // TODO set gpio and pwm values to something predictable

    init_flag = 1;
    return 0;
}

/*******************************************************************************
* mb_motor_cleanup()
* 
*******************************************************************************/
int mb_motor_cleanup() {

    if (unlikely(!init_flag)) {
        fprintf(stderr, "ERROR: trying cleanup before motors have been initialized\n");
        return -1;
    }

    rc_pwm_cleanup(1);
    for (int c = 0; c < CHANNELS; ++c) {
        rc_gpio_cleanup(chip[c], dir_pin[c]);
    }

    return 0;
}

/*******************************************************************************
* mb_motor_brake()
* 
* allows setting the brake function on the motor drivers
* returns 0 on success, -1 on failure
*******************************************************************************/
int mb_motor_brake(int brake_en) {

    if (unlikely(!init_flag)) {
        fprintf(stderr, "ERROR: trying to enable brake before motors have been initialized\n");
        return -1;
    }


    if (unlikely(rc_gpio_set_value(chip[0], MOT_BRAKE_EN, brake_en))) {
        printf("ERROR: in motor_brake, failed to write to gpio pin %d,%d\n", chip[0], MOT_BRAKE_EN);
        return -1;
    }

    // just to keep track; not sure if necessary?
    brake_mode = brake_en;
    return 0;
}

/*******************************************************************************
* int mb_disable_motors()
* 
* disables PWM output signals
* returns 0 on success
*******************************************************************************/
int mb_motor_disable() {

    if (unlikely(!init_flag)) {
        fprintf(stderr, "ERROR: trying to disable motors before motors have been initialized\n");
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
int mb_motor_set(int motor, double duty) {
    // sanity checks
    if (unlikely(motor < 0 || motor > CHANNELS)) {
        fprintf(stderr, "ERROR in motor_set, motor argument must be between 0 & %d\n", CHANNELS);
        return -1;
    }
    if (unlikely(!init_flag)) {
        fprintf(stderr, "ERROR: trying to motor_set before they have been initialized\n");
        return -1;
    }
    if (duty > 1.0) {
        duty = 1.0;
    } else if (duty < -1.0) {
        duty = -1.0;
    }

    // index of motor is 1 less than the enum we assign to it (for some reason... why not start at 0)?
    int m = motor - 1;
    // correct by polarity
    duty *= polarity[m];

    // set DIR value (1 for forward, 0 for reverse)
    int forward = 1;
    // duty given to pwm should always be positive
    if (duty < 0) {
        forward = 0;
        duty *= -1;
    }

    if (unlikely(rc_gpio_set_value(chip[m], dir_pin[m], forward))) {
        printf("ERROR: in motor_set, failed to write to gpio pin %d,%d\n", chip[m], dir_pin[m]);
        return -1;
    }
    if (unlikely(rc_pwm_set_duty(pwmss[m], pwmch[m], duty))) {
        printf("ERROR: in motor_set, failed to write to pwm %d%c\n", pwmss[m], pwmch[m]);
        return -1;
    }
    return 0;
}

/*******************************************************************************
* int mb_motor_set_all(double duty)
* 
* applies the same duty cycle argument to both motors
*******************************************************************************/
int mb_motor_set_all(double duty) {

    if (unlikely(!init_flag)) {
        printf("ERROR: trying to rc_set_motor_all before they have been initialized\n");
        return -1;
    }

    for (int m = 0; m < CHANNELS; ++m) {
        if (unlikely(mb_motor_set(m+1, duty))) {
            printf("ERROR: motor set all failed\n");
            return -1;
        }
    }

    return 0;
}


/*******************************************************************************
* int mb_motor_read_current(int motor)
* 
* returns the measured current in A
*******************************************************************************/
double mb_motor_read_current(int motor) {
    //DRV8801 driver board CS pin puts out 500mV/A
    return 0.0;
}