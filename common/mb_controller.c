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

rc_filter_t thetaController = RC_FILTER_INITIALIZER;
rc_filter_t phiController = RC_FILTER_INITIALIZER;

int mb_controller_init() {
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
    FILE* file = fopen(PID_PATH, "r");
    if (file == NULL) {
        printf("Error opening %s\n", PID_PATH);
        return -1;
    }

    double kpTheta, kiTheta, kdTheta;
    fscanf(file, "%lf %lf %lf", &kpTheta, &kiTheta, &kdTheta);
    double kpPhi, kiPhi, kdPhi, maxTheta;
    fscanf(file, "%lf %lf %lf", &kpPhi, &kiPhi, &kdPhi, &maxTheta);

    if (rc_filter_pid(&thetaController, kpTheta, kiTheta, kdTheta, 4 * DT,
                      DT)) {
        fprintf(stderr,
                "ERROR in rc_balance, failed to make theta controller\n");
        return -1;
    }

    if (rc_filter_pid(&thetaController, kpPhi, kiPhi, kdPhi, 4 * DT, DT)) {
        fprintf(stderr, "ERROR in rc_balance, failed to make phi controller\n");
        return -1;
    }

    rc_filter_enable_saturation(&thetaController, -1, 1);
    rc_filter_enable_saturation(&phiController, -maxTheta, maxTheta);

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

int mb_controller_update(mb_state_t* mb_state, Setpoint sp) {
    // u = -Kx (configuration file holds negative K already)

    const double desiredTheta =
        rc_filter_march(&phiController, sp.phi - mb_state->phi);
    const double u =
        rc_filter_march(&thetaController, desiredTheta - mb_state->theta);
    mb_state->left_cmd = u;
    mb_state->right_cmd = u;

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
    return 0;
}