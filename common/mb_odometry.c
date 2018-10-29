/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality 
*
*******************************************************************************/

#include "../balancebot/balancebot.h"
#include "../common/mb_defs.h"
#include <math.h>

// TODO: determine left and right wheel diameter and distinguish them here
const double TICKS_TO_M_L = ENCODER_TICKS_TO_ROT * M_PI * WHEEL_DIAMETER;
const double TICKS_TO_M_R = ENCODER_TICKS_TO_ROT * M_PI * WHEEL_DIAMETER;

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float theta){
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->psi = theta;
}

void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
    // Differential drive odometry with psi in [-pi, pi]
    // displacements
    const float dSL = mb_state->left_encoder * TICKS_TO_M_L;
    const float dSR = mb_state->right_encoder * TICKS_TO_M_R;
    // d ~ dS (straight line approximation for travelling along curve)
    const float d = (dSL + dSR) / 2;
    // change in angle/heading
    const float dtheta = (dSR - dSL) / WHEEL_BASE;

    const float dx = d * cos(mb_odometry->psi + dtheta/2);
    const float dy = d * sin(mb_odometry->psi + dtheta/2);

    // apply update
    mb_odometry->x += dx;
    mb_odometry->y += dy;
    mb_odometry->psi = mb_clamp_radians(mb_odometry->psi + dtheta);
}


float mb_clamp_radians(float angle){
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

void resetEncoders() {
    rc_encoder_eqep_write(1, 0);
    rc_encoder_eqep_write(2, 0);
}