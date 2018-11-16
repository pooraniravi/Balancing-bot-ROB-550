/*******************************************************************************
* mb_defs.h
*
*   defines for your bot
*   You will need to fill this in based on the data sheets, schematics, etc. 
*      and your specific configuration...
* 
*******************************************************************************/
#ifndef MB_DEFS_H
#define MB_DEFS_H

# define PI		3.14159265358979323846

// 1 is RIGHT 2 is LEFT
#define DEFAULT_PWM_FREQ        25000 // period of motor drive pwm
#define LEFT_MOTOR              2     // id of left motor
#define RIGHT_MOTOR             1     // id of right motor
#define CHANNELS                2     // id of 0 indicates all motors
#define MDIR1_CHIP              1
#define MDIR1_PIN               28  //gpio1.28  P9.12
#define MDIR2_CHIP              1
#define MDIR2_PIN               16  //gpio1.16  P9.15
#define MOT_BRAKE_EN            20    // gpio0.20  P9.41
#define MOT_1_POL               -1    // polarity of motor 1
#define MOT_2_POL               1    // polarity of motor 2
#define ENC_1_POL               1    // polarity of encoder 1
#define ENC_2_POL               -1    // polarity of encoder 2
#define MOT_1_CS                0    // analog in of motor 1 current sense
#define MOT_2_CS                1    // analog in of motor 2 current sense
#define GEAR_RATIO              20.4  // gear ratio of motor
#define ENCODER_RES             48.0  // encoder counts per motor shaft revolution
#define ENCODER_TICKS_TO_ROT    1/(GEAR_RATIO * ENCODER_RES)
#define WHEEL_DIAMETER          0.08 // diameter of wheel in meters
#define WHEEL_BASE              0.2  // wheel separation distance in meters
#define FWD_VEL_SENSITIVITY     0.1   // sensitivity of RC control for moving
#define TURN_VEL_SENSITIVITY    0.1   // sensitivity of RC control for turning
#define SAMPLE_RATE_HZ          100   // main filter and control loop speed
#define DT                      0.01  // 1/sample_rate
#define PRINTF_HZ               10    // rate of print loop
#define RC_CTL_HZ               25    // rate of RC data update

#endif
