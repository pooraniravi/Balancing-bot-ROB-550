#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <rc/math.h>
#include <rc/math/filter.h>
#include "mb_controller.h"
#include "mb_defs.h"

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
    FILE *file = fopen(PID_PATH, "r");
    if (file == NULL) {
        printf("Error opening %s\n", CFG_PATH);
        return -1;
    }

    double kpTheta, kiTheta, kdTheta;
    fscanf(file, "%lf %lf %lf", &kpTheta, &kiTheta, &kdTheta);

    if(rc_filter_pid(&thetaController, kpTheta, kiTheta, kdTheta, 4*DT, DT)){
        fprintf(stderr,"ERROR in rc_balance, failed to make theta controller\n");
        return -1;
    }
    rc_filter_enable_saturation(&thetaController, -1, 1);


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

int mb_controller_update(mb_state_t *mb_state) {
    // u = -Kx (configuration file holds negative K already)
    // inner theta controller

    const double u = rc_filter_march(&thetaController, 0 - mb_state->theta);
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
    return 0;
}