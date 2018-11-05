/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry functionality 
*
*******************************************************************************/

#include "../balancebot/balancebot.h"
#include "../common/mb_defs.h"
#include <rc/encoder_eqep.h>
#include <math.h>
#include <rc/math.h>
#include <rc/time.h>
#include <sys/time.h>

// TODO: determine left and right wheel diameter and distinguish them here
const double TICKS_TO_M_L = ENCODER_TICKS_TO_ROT * PI * WHEEL_DIAMETER;
const double TICKS_TO_M_R = ENCODER_TICKS_TO_ROT * PI * WHEEL_DIAMETER;
const double TICKS_TO_RAD = ENCODER_TICKS_TO_ROT * 2 * PI;

void mb_odometry_init(mb_odometry_t* mb_odometry, float x, float y, float theta){
    mb_odometry->x = x;
    mb_odometry->y = y;
    mb_odometry->dHeading = theta;
}

double now(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1e6;
}

void mb_odometry_update(mb_odometry_t* mb_odometry, mb_state_t* mb_state, double dt){
    const int dL = mb_state->left_encoder * ENC_2_POL;
    const int dR = mb_state->right_encoder * ENC_1_POL;

    // Differential drive odometry with dHeading in [-pi, pi]
    // displacements
    const double dSL = dL * TICKS_TO_M_L;
    const double dSR = dR * TICKS_TO_M_R;

    // average dphi not over dt yet
    const double dphi = (double)(dR + dL) / 2 * TICKS_TO_RAD;

    // d ~ dS (straight line approximation for travelling along curve)
    const double d = (dSL + dSR) / 2;
    // change in angle/heading
    mb_odometry->dHeading = (dSR - dSL) / WHEEL_BASE;


    const double dx = d * cos(mb_state->heading + mb_odometry->dHeading/2);
    const double dy = d * sin(mb_state->heading + mb_odometry->dHeading/2);

    // apply update to odometry and state
    mb_state->phiDot = dphi / dt;
    mb_state->phi += dphi;
    mb_state->vL = dSL / dt;
    mb_state->vR = dSR / dt;

    mb_odometry->x += dx;
    mb_odometry->y += dy;
}


float mb_clamp_radians(float angle){
    while (angle > PI) {
        angle -= 2 * PI;
    }
    while (angle < -PI) {
        angle += 2 * PI;
    }
    return angle;
}

double angularDiff(double a, double b) {
    double diff = b - a;
    diff = (diff + PI)
}

void resetEncoders() {
    rc_encoder_eqep_write(1, 0);
    rc_encoder_eqep_write(2, 0);
}