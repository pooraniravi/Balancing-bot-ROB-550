/*******************************************************************************
* measure_motors.c
*
* Use this template to write data to a file for analysis in Python or Matlab
* to determine the parameters for your motors
*
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>    // for mkdir and chmod
#include <sys/types.h>    // for mkdir and chmod
#include <rc/start_stop.h>
#include <rc/cpu.h>
#include <rc/encoder_eqep.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <sys/time.h>
#include "../common/mb_motor.h"
#include "../common/mb_defs.h"
#include "../common/mb_odometry.h"

#define ROUNDS_TO_MEASURE 1000

FILE *f1;

int64_t us_now(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

/**
 * Determine K and R through running in steady state (di/dt = 0)
 * Remember to measure voltage across the motor with a multimeter
 * @param motor
 * @param duty
 */
void measureMotor(int motor, double duty) {
    double times[ROUNDS_TO_MEASURE];
    int ticks[ROUNDS_TO_MEASURE];
    double currents[ROUNDS_TO_MEASURE];

    // let motor run at no load
    mb_motor_set(motor, duty);
    // wait for it to enter steady state
    rc_nanosleep(1e9);

//    printf("Assuming in steady state now\n");
    resetEncoders();

    int64_t startTime = us_now();

    int64_t now = startTime;
    int i = 0;
    while (now - startTime < 5e6) {
        now = us_now();
        times[i] = now - startTime;
        ticks[i] = rc_encoder_eqep_read(motor);
        currents[i] = mb_motor_read_current(motor);

        ++i;
        rc_nanosleep(5e6);
    }

    for (int j = 0; j < i; ++j) {
        printf("%f, %d, %f\n", times[j], ticks[j], currents[j]);
    }
    mb_motor_set(motor, 0);
}

void measureInertia(int motor, double duty) {
    double times[ROUNDS_TO_MEASURE];
    int ticks[ROUNDS_TO_MEASURE];
    double currents[ROUNDS_TO_MEASURE];

    // let motor run at no load
    mb_motor_set(motor, duty);
    // wait for it to enter steady state
    rc_nanosleep(1e9);

//    printf("Turning off motor\n");
    resetEncoders();

    // TODO should we break or not? take a look at current and then decide
    if (mb_motor_brake(0) != 0) {
        fprintf(stderr, "Motor can't modify brake");
        return;
    };
    // turn off motor
    mb_motor_set(motor, 0);

    int64_t startTime = us_now();

    int64_t now = startTime;
    int i = 0;
    while (now - startTime < 2 * 1e6) {
        now = us_now();
        times[i] = now - startTime;
        ticks[i] = rc_encoder_eqep_read(motor);
        currents[i] = mb_motor_read_current(motor);

        ++i;
        rc_nanosleep(1e7);
    }

    for (int j = 0; j < i; ++j) {
        printf("%f, %d, %f\n", times[j], ticks[j], currents[j]);
    }
    mb_motor_set(motor, 0);
//    printf("Out of loop\n");
}

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(int argc, char** argv) {

    // make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if (rc_kill_existing_process(2.0) < -2) return -1;

    // start signal handler so we can exit cleanly
    if (rc_enable_signal_handler() == -1) {
        fprintf(stderr, "ERROR: failed to start signal handler\n");
        return -1;
    }

    if (rc_cpu_set_governor(RC_GOV_PERFORMANCE) < 0) {
        fprintf(stderr, "Failed to set governor to PERFORMANCE\n");
        return -1;
    }

    // initialize enocders
    if (rc_encoder_eqep_init() == -1) {
        fprintf(stderr, "ERROR: failed to initialize eqep encoders\n");
        return -1;
    }

    // initialize adc
    if (rc_adc_init() == -1) {
        fprintf(stderr, "ERROR: failed to initialize adc\n");
        return -1;
    }

    if (mb_motor_init() < 0) {
        fprintf(stderr, "ERROR: failed to initialze mb_motors\n");
        return -1;
    }

    // make PID file to indicate your project is running
    // due to the check made on the call to rc_kill_existing_process() above
    // we can be fairly confident there is no PID file already and we can
    // make our own safely.
    rc_make_pid_file();


    rc_set_state(RUNNING);

    if (argc != 3) {
        fprintf(stderr, "Specify motor (L,R) and duty (float)\n");
        return -1;
    }
    int motor;
    switch (argv[1][0]) {
        case 'r':
        case 'R':
            motor = RIGHT_MOTOR;
            break;
        case 'l':
        case 'L':
            motor = LEFT_MOTOR;
            break;
        default:
            fprintf(stderr, "Specify motor (L,R) and duty (float)\n");
            return -1;
    }

    double duty = atof(argv[2]);
//    measureMotor(motor, duty);
    measureInertia(motor, duty);

    // exit cleanly
    rc_encoder_eqep_cleanup();
    rc_remove_pid_file();   // remove pid file LAST
    return 0;
}