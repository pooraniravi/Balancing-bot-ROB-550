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
const float TICKS_TO_M_L = ENCODER_TICKS_TO_ROT * M_PI * WHEEL_DIAMETER;
const float TICKS_TO_M_R = ENCODER_TICKS_TO_ROT * M_PI * WHEEL_DIAMETER;
const double TICKS_TO_RAD = ENCODER_TICKS_TO_ROT * 2 * M_PI;

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float theta){
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->psi = theta;
}

double now(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1e6;
}

void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state, double dt){
    // Differential drive odometry with psi in [-pi, pi]
    // displacements
    const float dSL = mb_state->left_encoder * TICKS_TO_M_L;
    const float dSR = mb_state->right_encoder * TICKS_TO_M_R;

    // average dphi not over dt yet
    const double dphi = (double)(mb_state->right_encoder - mb_state->left_encoder) / 2 * TICKS_TO_RAD;

    // d ~ dS (straight line approximation for travelling along curve)
    const float d = (dSL + dSR) / 2;
    // change in angle/heading
    const float dpsi = (dSR - dSL) / WHEEL_BASE;


    const float dx = d * cos(mb_odometry->psi + dpsi/2);
    const float dy = d * sin(mb_odometry->psi + dpsi/2);

    // apply update to odometry and state
    mb_state->phiDot = dphi / dt;
    mb_state->phi += dphi;

    mb_odometry->x += dx;
    mb_odometry->y += dy;
    mb_odometry->psi = mb_clamp_radians(mb_odometry->psi + dpsi);
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