/*******************************************************************************
 * balancebot.c
 *
 * Main template code for the BalanceBot Project
 * based on rc_balance
 *
 *******************************************************************************/

#include <rc/adc.h>
#include <rc/bmp.h>
#include <rc/button.h>
#include <rc/cpu.h>
#include <rc/dsm.h>
#include <rc/encoder_eqep.h>
#include <rc/led.h>
#include <rc/mpu.h>
#include <rc/pthread.h>
#include <rc/servo.h>
#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/math/filter.h>

#include "balancebot.h"

#include <sys/types.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include "../lcmtypes/balancebot_msg_t.h"
#include "../lcmtypes/pose_xyt_t.h"
#include "../lcmtypes/balancebot_gate_t.h"
#include "../optitrack/common/serial.h"

// threshold on the difference between gyro and odometry incremental heading
// TODO determine heading threshold for gyrodometry by recording difference in
// dHeading from odometry and gyro
const double GYRODOMETRY_THRESHOLD_RAD = 0.1;
Setpoint setpoint;
rc_filter_t thetaDotFilter = RC_FILTER_INITIALIZER;
const int baudRate = 57600;
const char port[] = "/dev/ttyO5";
const int num_gates = 4;
uint64_t last_utime = 0;
int bytes_avail = 0;
int err_counter = 0;
const char headByte = 0x1B;
const char tailByte = 0xFF;
int fd;
int dataCount = 0;

void getData(balancebot_msg_t *BBmsg) {
    char *ptr;
    int packetLength = balancebot_msg_t_encoded_size(BBmsg) + 2;
    char *dataPacket = (char *) malloc(packetLength);
    ptr = dataPacket;
    while (read(fd, ptr, 1) > 0) {
        // if the first Byte is wrong keep looking
        if ((ptr == dataPacket) && (*ptr != headByte)) {
            continue;
        }
        ptr++;
        // Once we have all of the Bytes check to make sure first and last are good
        if ((ptr - dataPacket) == packetLength) {
            if ((dataPacket[0] != headByte) || (dataPacket[packetLength - 1] != tailByte)) {
                err_counter += 1;
            } else {
                //packet is good, decode it into BBmsg
                int status = balancebot_msg_t_decode(dataPacket, 1, packetLength - 2, BBmsg);
                if (status < 0) {
                    fprintf(stderr, "error %d decoding balancebot_msg_t!!!\n", status);;
                }
                // if we have less than a full message in the serial buffer
                // we are done, we'll get the next one next time
                ioctl(fd, FIONREAD, &bytes_avail);
                if (bytes_avail < packetLength) {
                    break;
                }
            }
            //keep reading until buffer is almost empty
            ptr = dataPacket;
        }
    }
    ++dataCount;
}

void printData(balancebot_msg_t BBmsg) {
    if (BBmsg.utime < 1000000) printf("    %"PRId64"|", BBmsg.utime);
    else if (BBmsg.utime < 10000000) printf("   %"PRId64"|", BBmsg.utime);
    else if (BBmsg.utime < 100000000) printf("  %"PRId64"|", BBmsg.utime);
    else if (BBmsg.utime < 1000000000)printf(" %"PRId64"|", BBmsg.utime);
    else printf("%"PRId64"|", BBmsg.utime);

    if (bytes_avail < 10) printf("  %d|", bytes_avail);
    else if (bytes_avail < 100) printf(" %d|", bytes_avail);
    else printf("%d|", bytes_avail);

    if (err_counter < 10) printf("  %d|", err_counter);
    else if (err_counter < 100) printf(" %d|", err_counter);
    else printf("%d|", err_counter);

    printf("  %3.2f |", 1.0E6 / (BBmsg.utime - last_utime));
    printf("%+7.6f|", BBmsg.pose.x);
    printf("%+7.6f|", BBmsg.pose.y);
    printf("%+7.6f|", BBmsg.pose.theta);
    printf("  %d   |", BBmsg.num_gates);
    if (BBmsg.num_gates > 0) {
        printf("%+7.6f|", (BBmsg.gates[0].left_post[0] + BBmsg.gates[0].right_post[0]) / 2.0);
        printf("%+7.6f|", (BBmsg.gates[0].left_post[1] + BBmsg.gates[0].right_post[1]) / 2.0);
    }
    printf("         \r");
    fflush(stdout);

    last_utime = BBmsg.utime;
}

//construct message for storage
balancebot_msg_t BBmsg;

void *motion_capture_receive_message_loop(void *ptr) {
    //open serial port non-blocking
    fd = serial_open(port, baudRate, 0);

    if (fd == -1) {
        printf("Failed to open Serial Port: %s", port);
        return NULL;
    }

    pose_xyt_t BBpose;
    balancebot_gate_t BBgates[num_gates];
    BBmsg.pose = BBpose;
    BBmsg.num_gates = num_gates;
    BBmsg.gates = BBgates;
    int packetLength = balancebot_msg_t_encoded_size(&BBmsg) + 2;
    //printf("packetLength: %d\n", packetLength);

    // Print Header
    //printf("\n");
    //printf("   Time   |");
    //printf("Buf|");
    //printf("Err|");
    //printf("  Rate  |");
    //printf("    X    |");
    //printf("    Y    |");
    //printf("    θ    |");
    //printf("#GATES|");
    //printf("   G1X   |");
    //printf("   G1Y   |");
    //printf("\n");

    while (1) {
        //check bytes in serial buffer
        //printf("here");
        ioctl(fd, FIONREAD, &bytes_avail);
        //printf("bytes: %d\n",bytes_avail);
        if (bytes_avail >= packetLength) {
            getData(&BBmsg);
        }
        rc_nanosleep(100E3);
    }
    serial_close(fd);
    return NULL;
}

/*******************************************************************************
 * int main()
 *
 *******************************************************************************/
int main() {
    // make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if (rc_kill_existing_process(2.0) < -2)
        return -1;

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

    if (rc_dsm_init() == -1) {
        fprintf(stderr, "failed to start initialize DSM\n");
        return -1;
    }

    // make PID file to indicate your project is running
    // due to the check made on the call to rc_kill_existing_process() above
    // we can be fairly confident there is no PID file already and we can
    // make our own safely.
    rc_make_pid_file();

    // start printf_thread if running from a terminal
    // if it was started as a background process then don't bother
    printf("starting print thread... \n");
    pthread_t printf_thread;
    rc_pthread_create(&printf_thread, printf_loop, (void *) NULL, SCHED_OTHER, 0);

    // start control thread
    printf("starting setpoint thread... \n");
    pthread_t setpoint_control_thread;
    rc_pthread_create(&setpoint_control_thread, setpoint_control_loop,
                      (void *) NULL, SCHED_FIFO, 50);

    // TODO: start motion capture message recieve thread
    printf("starting motion capture message receive thread... \n");
//    pthread_t motion_capture_receive_message_thread;
//    rc_pthread_create(&motion_capture_receive_message_thread, motion_capture_receive_message_loop,
//                      (void *) NULL, SCHED_FIFO, 50);

    // set up IMU configuration
    printf("initializing imu... \n");
    // set up mpu configuration
    rc_mpu_config_t mpu_config = rc_mpu_default_config();
    mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
    mpu_config.orient = ORIENTATION_Z_DOWN;
    mpu_config.dmp_fetch_accel_gyro = 1;

    // now set up the imu for dmp interrupt operation
    if (rc_mpu_initialize_dmp(&mpu_data, mpu_config)) {
        printf("rc_mpu_initialize_failed\n");
        return -1;
    }

    // rc_nanosleep(5E9); // wait for imu to stabilize

    // initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);
    pthread_mutex_init(&setpoint_mutex, NULL);

    // attach controller function to IMU interrupt
    printf("initializing controller...\n");
    if (mb_controller_init() == -1) {
        printf("controller initialization failed\n");
        return -1;
    }

    printf("initializing motors...\n");
    if (mb_motor_init() == -1) {
        printf("motor initialization failed\n");
        return -1;
    }

    printf("resetting encoders...\n");
    rc_filter_butterworth_lowpass(&thetaDotFilter, 2, DT, 50 * 2 * PI);
    rc_encoder_eqep_write(1, 0);
    rc_encoder_eqep_write(2, 0);

    printf("initializing state...\n");
    mb_state.t = now();
    mb_state.phi = 0;
    mb_state.heading = 0;

    printf("initializing setpoint...\n");
    // drive 2 rad forward
    setpoint.phi = 0;
    setpoint.heading = 0;

    printf("initializing odometry...\n");
    mb_odometry_init(&mb_odometry, 0.0, 0.0, 0.0);

    printf("attaching imu interupt...\n");
    rc_mpu_set_dmp_callback(&balancebot_controller);

    printf("we are running!!!...\n");
    // done initializing so set state to RUNNING
    rc_set_state(RUNNING);

    // Keep looping until state changes to EXITING
    while (rc_get_state() != EXITING) {

        // TODO have a calibration routine that we can comment out

        // all the balancing is handled in the imu interupt function
        // other functions are handled in other threads
        // there is no need to do anything here but sleep
        // always sleep at some point
        rc_nanosleep(1E9);
    }

    // exit cleanly
    rc_mpu_power_off();
    mb_motor_cleanup();
    mb_controller_cleanup();
    rc_led_cleanup();
    rc_encoder_eqep_cleanup();
    rc_remove_pid_file(); // remove pid file LAST
    return 0;
}

/*******************************************************************************
 * void balancebot_controller()
 *
 * discrete-time balance controller operated off IMU interrupt
 * Called at SAMPLE_RATE_HZ
 *
 * TODO: You must implement this function to keep the balancebot balanced
 *
 *
 *******************************************************************************/
void balancebot_controller() {

    // lock state mutex
    pthread_mutex_lock(&state_mutex);
    // Read IMU
    const double t = now();
    const double dt = t - mb_state.t;

    // correct for theta such that it's 0 when the robot is standing upright
    // this is the phi in
    // http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling
    const double theta = mb_clamp_radians(mpu_data.dmp_TaitBryan[TB_PITCH_X]);
    const double heading = mpu_data.dmp_TaitBryan[TB_YAW_Z];

    mb_state.t = t;
    mb_state.dt = dt;
//    mb_state.thetaDot = (theta - mb_state.theta) / dt;
    mb_state.thetaDot = rc_filter_march(&thetaDotFilter, mpu_data.gyro[0] * DEG_TO_RAD);
    mb_state.theta = theta;
    // Read encoders
    mb_state.left_encoder = rc_encoder_eqep_read(LEFT_MOTOR);
    mb_state.right_encoder = rc_encoder_eqep_read(RIGHT_MOTOR);

    resetEncoders();

    // Update odometry
    mb_odometry_update(&mb_odometry, &mb_state, dt);
    const double dOdometryHeading = mb_odometry.dHeading;
    const double dGyroHeading = mb_state.gyroHeading - heading;
    mb_state.gyroHeading = heading;

    // gyrodometry
    if (fabs(dGyroHeading - dOdometryHeading) > GYRODOMETRY_THRESHOLD_RAD) {
        mb_state.heading += dGyroHeading;
    } else {
        mb_state.heading += dOdometryHeading;
    }
    mb_state.heading = mb_clamp_radians(mb_state.heading);

    // Calculate controller outputs
    mb_controller_update(&mb_state, &setpoint);

    if (!mb_setpoints.manual_ctl) {
        // send motor commands
        mb_motor_set(LEFT_MOTOR, mb_state.left_cmd);
        mb_motor_set(RIGHT_MOTOR, mb_state.right_cmd);
    } else if (mb_setpoints.manual_ctl) {
        // send motor commands
        mb_motor_set(LEFT_MOTOR, 0);
        mb_motor_set(RIGHT_MOTOR, 0);
    }

    // unlock state mutex
    pthread_mutex_unlock(&state_mutex);
}

/*******************************************************************************
 *  setpoint_control_loop()
 *
 *  sets current setpoints based on dsm radio data, odometry, and Optitrak
 *
 *
 *******************************************************************************/
const float MAX_HEADING_VEL = 0.3; // in radians
double lastCommandTime = 0;
const double minTimeBetweenCommands = 1;

void *setpoint_control_loop(void *ptr) {

    while (1) {

        // channel 3 controls body velocity (positive goes forward, negative goes backward)
        // channel 4 controls turning (positive goes left, negative goes right)
        // channel 5 controls mode (0 is manual, 0.5 is simple auto balance, 1 is planner auto?)
        if (rc_dsm_is_new_data()) {
            // You may should implement switching between manual and autonomous
            // mode using channel 5 of the DSM data.
            const float mode = rc_dsm_ch_normalized(5);
            if (mode < 0.3) {
                mb_setpoints.manual_ctl = 1;
            } else {
                mb_setpoints.manual_ctl = 0;
                // enable planner mode?
                if (mode > 0.7) {
                }
            };

            float vel = rc_dsm_ch_normalized(3);
            float heading = rc_dsm_ch_normalized(4);
            // use small dead zone to prevent slow drifts
            if (fabs(vel) > DSM_DEAD_ZONE) {
                const double commandTime = now();
                if (commandTime - lastCommandTime > minTimeBetweenCommands) {
                    setpoint.phi = mb_state.phi + vel * maxPhiControlStep;
                    lastCommandTime = commandTime;
                }
            }
            if (fabs(heading) < DSM_DEAD_ZONE) {
                heading = 0;
            }

            setpoint.heading = mb_state.heading + heading * MAX_HEADING_VEL;
        }
        rc_nanosleep(1E9 / RC_CTL_HZ);
    }
    return NULL;
}

/*******************************************************************************
 * printf_loop()
 *
 * prints diagnostics to console
 * this only gets started if executing from terminal
 *
 * TODO: Add other data to help you tune/debug your code
 *******************************************************************************/
void *printf_loop(void *ptr) {
    rc_state_t last_state, new_state; // keep track of last state
    while (rc_get_state() != EXITING) {
        new_state = rc_get_state();
        // check if this is the first time since being paused
        if (new_state == RUNNING && last_state != RUNNING) {
            printf("\nRUNNING: Hold upright to balance.\n");
            printf("                 SENSORS               |           "
                   "ODOMETRY          |");
            printf("\n");
            printf("    θ    |");
            printf("    φ    |");
            printf(" heading  |");
//            printf("  L Enc  |");
//            printf("  R Enc  |");
//            printf("    X    |");
//            printf("    Y    |");
//            printf("    ψ    |");
            printf("   θdot  |");
            printf("   φdot  |");
            printf("    φr   |");
            printf("    θr   |");
            printf(" headingr |");
            printf("    vl   |");
            printf("    vr   |");
            printf("    L    |");
            printf("    R    |");

            printf("\n");
        } else if (new_state == PAUSED && last_state != PAUSED) {
            printf("\nPAUSED\n");
        }
        last_state = new_state;

        if (new_state == RUNNING) {
            printf("\r");
            // Add Print stattements here, do not follow with /n
            pthread_mutex_lock(&state_mutex);
            printf("%7.3f  |", mb_state.theta);
            printf("%7.3f  |", mb_state.phi);
//            printf("%7d  |", mb_state.left_encoder);
//            printf("%7d  |", mb_state.right_encoder);
//            printf("%7.3f  |", mb_odometry.x);
//            printf("%7.3f  |", mb_odometry.y);
            printf("%7.3f  |", mb_state.heading);
            printf("%7.3f  |", mb_state.thetaDot);
            printf("%7.3f  |", mb_state.phiDot);
            printf("%7.3f  |", setpoint.phi);
            printf("%7.3f  |", setpoint.theta);
            printf("%7.3f  |", setpoint.heading);
            printf("%7.3f  |", mb_state.vL);
            printf("%7.3f  |", mb_state.vR);
            printf("%7.3f  |", mb_state.left_cmd);
            printf("%7.3f  |", mb_state.right_cmd);
//            printf("%7d  |", dataCount);
            //printData(BBmsg);
//            printf("%+7.6f|", BBmsg.pose.x);
//            printf("%+7.6f|", BBmsg.pose.y);
//            printf("%+7.6f|", BBmsg.pose.theta);
//            printf("  %d   |", BBmsg.num_gates);
//            if (BBmsg.num_gates > 0) {
//                printf("%+7.6f|", (BBmsg.gates[0].left_post[0] + BBmsg.gates[0].right_post[0]) / 2.0);
//                printf("%+7.6f|", (BBmsg.gates[0].left_post[1] + BBmsg.gates[0].right_post[1]) / 2.0);
//            }
            printf("\r");
            pthread_mutex_unlock(&state_mutex);
            fflush(stdout);
        }
        rc_nanosleep(1E9 / PRINTF_HZ);
    }
    return NULL;
}
