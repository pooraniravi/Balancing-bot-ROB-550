#include "mb_controller.h"
#include "mb_defs.h"
#include <math.h>
#include <rc/math.h>
#include <rc/math/filter.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

/*******************************************************************************
 * int mb_controller_init()
 *
 * this initializes the controllers from the configuration file
 * you can use this as is or modify it if you want a different format
 *
 * return 0 on success
 *
 *******************************************************************************/

// PID controllers
rc_filter_t thetaController = RC_FILTER_INITIALIZER;
rc_filter_t phiController = RC_FILTER_INITIALIZER;
rc_filter_t lController = RC_FILTER_INITIALIZER;
rc_filter_t rController = RC_FILTER_INITIALIZER;

// LQR controller
rc_matrix_t K = RC_MATRIX_INITIALIZER;
rc_vector_t x = RC_VECTOR_INITIALIZER;
rc_vector_t u = RC_VECTOR_INITIALIZER;


int mb_controller_init() {
    rc_matrix_zeros(&K, 1, 4);
    rc_vector_zeros(&x, 4);
    rc_vector_zeros(&u, 1);


    if (mb_controller_load_config() != 0) {
        return -1;
    }

    return 0;
}

/*******************************************************************************
 * int mb_controller_load_config()
 *
 * this provides a basic configuration load routine
 * you can use this as is or modify it if you want a different format
 *
 * return 0 on success
 *
 *******************************************************************************/

int mb_controller_load_config() {
    FILE *file = fopen(PID_PATH, "r");
    if (file == NULL) {
        printf("Error opening %s\n", PID_PATH);
        return -1;
    }

    // max theta velocity in rad/s (max spin to allow for turning)
    double kpTheta, kiTheta, kdTheta, maxThetaVelocity;
    fscanf(file, "%lf %lf %lf %lf", &kpTheta, &kiTheta, &kdTheta, &maxThetaVelocity);
    // average of the two wheel diameters?
    maxThetaVelocity *= PI * WHEEL_DIAMETER;

    double kpPhi, kiPhi, kdPhi, maxTheta;
    fscanf(file, "%lf %lf %lf %lf", &kpPhi, &kiPhi, &kdPhi, &maxTheta);

    kpTheta *= maxThetaVelocity;
    kiTheta *= maxThetaVelocity;
    kdTheta *= maxThetaVelocity;
    if (rc_filter_pid(&thetaController, kpTheta, kiTheta, kdTheta, 4 * DT,
                      DT)) {
        fprintf(stderr,
                "ERROR in rc_balance, failed to make theta controller\n");
        return -1;
    }

    if (rc_filter_pid(&phiController, kpPhi, kiPhi, kdPhi, 4 * DT, DT)) {
        fprintf(stderr, "ERROR in rc_balance, failed to make phi controller\n");
        return -1;
    }

    double kp, ki, kd;
    fscanf(file, "%lf %lf %lf", &kp, &ki, &kd);
    if (rc_filter_pid(&lController, kp, ki, kd, 4 * DT, DT)) {
        fprintf(stderr, "ERROR in rc_balance, failed to make left controller\n");
        return -1;
    }
    fscanf(file, "%lf %lf %lf", &kp, &ki, &kd);
    if (rc_filter_pid(&rController, kp, ki, kd, 4 * DT, DT)) {
        fprintf(stderr, "ERROR in rc_balance, failed to make right controller\n");
        return -1;
    }


    rc_filter_enable_saturation(&thetaController, -maxThetaVelocity, maxThetaVelocity);
    rc_filter_enable_saturation(&phiController, -maxTheta, maxTheta);
    rc_filter_enable_saturation(&lController, -1, 1);
    rc_filter_enable_saturation(&rController, -1, 1);

    fclose(file);

    file = fopen(CFG_PATH, "r");
    if (file == NULL) {
        printf("Error opening %s\n", CFG_PATH);
        return -1;
    }

    fscanf(file, "%lf %lf %lf %lf", &K.d[0][0], &K.d[0][1], &K.d[0][2], &K.d[0][3]);
    rc_matrix_print(K);

    fclose(file);
    return 0;
}

/*******************************************************************************
 * int mb_controller_update()
 *
 *
 * take inputs from the global mb_state
 * write outputs to the global mb_state
 *
 * this should only be called in the imu call back function, no mutex needed
 *
 * return 0 on success
 *
 *******************************************************************************/

int mb_controller_update(mb_state_t *mb_state, Setpoint *sp) {
    // u = -Kx (configuration file holds negative K already)

    // uncomment below to use PID controller
    // phi controller negated because dphi will be neagtive when theta is positive
    sp->theta =
            rc_filter_march(&phiController, sp->phi - mb_state->phi);
    if (fabs(sp->theta) < 0.005) {
        sp->theta = 0;
    }
    // linear velocity
    const double desiredVelocity =
            rc_filter_march(&thetaController, sp->theta - mb_state->theta);

    // uncomment below to use LQR controller
//    x.d[0] = mb_state->theta;
//    x.d[1] = mb_state->thetaDot;
//    x.d[2] = mb_state->phi - sp->phi;
//    x.d[3] = mb_state->phiDot;
//    rc_matrix_times_col_vec(K,x,&u);
//    const double desiredVelocity = u.d[0];

    const double l = rc_filter_march(&lController, desiredVelocity - mb_state->vL);
    const double r = rc_filter_march(&rController, desiredVelocity - mb_state->vR);
    mb_state->left_cmd = l;
    mb_state->right_cmd = r;

    return 0;
}

/*******************************************************************************
 * int mb_controller_cleanup()
 *
 * TODO: Free all resources associated with your controller
 *
 * return 0 on success
 *
 *******************************************************************************/

int mb_controller_cleanup() {
    rc_filter_free(&thetaController);
    rc_filter_free(&phiController);
    rc_filter_free(&lController);
    rc_filter_free(&rController);
    rc_matrix_free(&K);
    rc_vector_free(&x);
    rc_vector_free(&u);

    return 0;
}