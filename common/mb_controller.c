#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <rc/math.h>
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

rc_matrix_t K = RC_MATRIX_INITIALIZER;
rc_vector_t x = RC_VECTOR_INITIALIZER;
rc_vector_t u = RC_VECTOR_INITIALIZER;

int mb_controller_init() {
    rc_matrix_zeros(&K, 1, 4);
    rc_vector_zeros(&x, 4);
    rc_vector_zeros(&u, 1);

    mb_controller_load_config();
    /* TODO initialize your controllers here*/

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
    FILE *file = fopen(CFG_PATH, "r");
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

int mb_controller_update(mb_state_t *mb_state) {
    // u = -Kx (configuration file holds negative K already)
    // TODO: add in reference state so u = rN - Kx
    x.d[0] = mb_state->theta;
    x.d[1] = mb_state->thetaDot;
    x.d[2] = mb_state->phi;
    x.d[3] = mb_state->phiDot;

    rc_matrix_times_col_vec(K,x,&u);
    mb_state->left_cmd = u.d[0];
    mb_state->right_cmd = u.d[0];

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
    rc_matrix_free(&K);
    rc_vector_free(&x);
    rc_vector_free(&u);
    return 0;
}