#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    int     left_encoder;      // left encoder counts since last reading
    int     right_encoder;     // right encoder counts since last reading

    // LQR controller relevant state
    double   theta;             // body angle (rad)
    double   thetaDot;
    double   phi;               // average wheel angle (rad)
    double   phiDot;

    // heading controller state
    double gyroHeading;
    // gyrodometry blended heading
    double heading;

    // time to get dt
    double t;
    double dt;

    //outputs
    float   left_cmd;  //left wheel command [-1..1]
    float   right_cmd; //right wheel command [-1..1]

    //TODO: Add more variables to this state as needed
};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{

    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    int manual_ctl;
};

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{

    double x;        //x position from initialization in m
    double y;        //y position from initialization in m
    double dHeading; //incremental heading in rad
};

#endif