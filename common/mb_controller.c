#include "mb_controller.h"
#include "mb_defs.h"
#include "mb_odometry.h"
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
rc_filter_t headingController = RC_FILTER_INITIALIZER;

// LQR controller
rc_matrix_t K = RC_MATRIX_INITIALIZER;
rc_vector_t x = RC_VECTOR_INITIALIZER;
rc_vector_t u = RC_VECTOR_INITIALIZER;

double maxPhiControlStep;

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
double maxTheta;
int mb_controller_load_config() {
    FILE *file = fopen(PID_PATH, "r");
    if (file == NULL) {
        printf("Error opening %s\n", PID_PATH);
        return -1;
    }

    // max theta velocity in rad/s (max spin to allow for turning)
    double kpTheta, kiTheta, kdTheta;
    double kpPhi, kiPhi, kdPhi, riseTime;

    fscanf(file, "%lf %lf %lf %lf", &kpTheta, &kiTheta, &kdTheta, &riseTime);


    if (rc_filter_pid(&thetaController, kpTheta, kiTheta, kdTheta, 0.0278,
                      DT)) {
        fprintf(stderr,
                "ERROR in rc_balance, failed to make theta controller\n");
        return -1;
    }

    fscanf(file, "%lf %lf %lf %lf %lf", &kpPhi, &kiPhi, &kdPhi, &maxTheta, &riseTime);

    if (rc_filter_pid(&phiController, kpPhi, kiPhi, kdPhi, riseTime, DT)) {
        fprintf(stderr, "ERROR in rc_balance, failed to make phi controller\n");
        return -1;
    }

    double kp;
    // just a P controller
    fscanf(file, "%lf %lf", &kp, &maxPhiControlStep);
    if (rc_filter_pid(&headingController, kp, 0, 0, 4 * DT, DT)) {
        fprintf(stderr, "ERROR in rc_balance, failed to make heading controller\n");
        return -1;
    }


    rc_filter_enable_saturation(&thetaController, -1, 1);
    rc_filter_enable_saturation(&headingController, -0.5, 0.5);
    rc_filter_enable_saturation(&phiController, -maxTheta, maxTheta);

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

double punch(const double ctrl) {
    const double epsilon = 1e-5;
    const double punchPWM = 0.05;
    if (ctrl < -epsilon) {
        return ctrl - punchPWM;
    } else if (ctrl > epsilon) {
        return ctrl + punchPWM;
    }
    return ctrl;
}

int mb_controller_update(mb_state_t *mb_state, Setpoint *sp) {
    // u = -Kx (configuration file holds negative K already)

    // uncomment below to use PID controller
    // phi controller negated because dphi will be neagtive when theta is positive
    sp->theta =
            rc_filter_march(&phiController, sp->phi - mb_state->phi);
    if (fabs(sp->theta) < 0.00000001) {
        sp->theta = 0;
    }
    // linear velocity
    double sharedVelocity =
            rc_filter_march(&thetaController, sp->theta - mb_state->theta);


    // uncomment below to use LQR controller
//    x.d[0] = mb_state->theta;
//    x.d[1] = mb_state->thetaDot;
//    x.d[2] = mb_state->theta + mb_state->phi - sp->phi;
//    x.d[3] = mb_state->phiDot;
//    rc_matrix_times_col_vec(K, x, &u);
//    const double sharedVelocity = u.d[0];

    // steering controller
    // difference in velocity to allow turning
    double diffVelocity = rc_filter_march(&headingController, mb_clamp_radians(sp->heading - mb_state->heading));

    // velocity controller is too slow (need to integrate) to balance
    mb_state->left_cmd = punch(sharedVelocity - diffVelocity);
    mb_state->right_cmd = punch(sharedVelocity + diffVelocity);

    return 0;
}

void reset_phi_controller() {
    rc_filter_reset(&phiController);
    rc_filter_enable_saturation(&phiController, -maxTheta, maxTheta);
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
    rc_filter_free(&headingController);
    rc_matrix_free(&K);
    rc_vector_free(&x);
    rc_vector_free(&u);

    return 0;
}