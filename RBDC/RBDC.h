/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_RBDC_H_
#define CATIE_SIXTRON_RBDC_H_

#include <math.h>
#include <stdint.h>

#include "mobile_base/mobile_base.h"
#include "odometry/odometry.h"
#include "pid/pid.h"

#ifndef M_PI
    #define M_PI (3.14159265358979323846)
#endif

#ifndef M_PI_2
    #define M_PI_2 (1.57079632679489661923)
#endif

namespace sixtron {

// Defines for direction, useful for 2 wheels robots only ?
#define RBDC_DIR_FORWARD (1)
#define RBDC_DIR_BACKWARD (-1)

#define RBDC_MAX_STATUS 7

typedef enum {
    two_wheels_robot,
    three_wheels_robot,
    four_wheels_robot
} RBDC_format;

typedef enum {
    absolute,
    relative
} RBDC_reference;

typedef enum {
    RBDC_standby = 0,
    RBDC_working = 1, // This should never happen, as we cover all the cases at each iteration.
    RBDC_done = 2, // 1.1.1 robot arrive on target
    RBDC_correct_final_angle = 3, // 1.1.2
    RBDC_moving = 4, // 1.2.1
    RBDC_moving_and_correct_angle = 5, // 1.2.2.1
    RBDC_correct_initial_angle = 6, // 1.2.2.2
} RBDC_status;

/*!
 *  \struct target_position
 *  RBDC target position
 */
typedef struct target_position target_position;

struct target_position {
    position pos;
    RBDC_reference ref = RBDC_reference::absolute;
    bool correct_final_theta = true;
    bool is_a_vector = false;
    bool absolute_angle = true;
    bool shortest_angle = true;
};

typedef struct trapeze trapeze;

struct trapeze {
    float V_max = 0.5f; //s
    float T_ramp = 10;//0.625f; //s
    float acc_max = V_max/T_ramp;
    float T_plat;
    float T_elapsed;

};


/*!
 *  \struct RBDC_param
 *  PID parameters structure
 */
typedef struct RBDC_params RBDC_params;

struct RBDC_params {
    RBDC_format rbdc_format = two_wheels_robot;
    PID_params pid_param_dv, pid_param_dteta, pid_param_dtan;
    float max_output_dtheta = 1.0f; // max command output, eg -1.0f to +1.0f
    float max_output_dv = 1.0f;
    float max_output_dtan = 1.0f;
    float final_theta_precision = 0.0f;
    float moving_theta_precision = 0.0f;
    float target_precision = 0.5f; // must be greater than dv_precision
    float dv_precision = 0.0f;
    float dv_reducing_coefficient = 0.80f; // coefficient, between 0.0f and 1.0f;
    float dt_seconds = 0.0f;
    bool can_go_backward = true;
};

class RBDC {

public:
    RBDC(Odometry *odometry, MobileBase *mobile_base, RBDC_params rbdc_parameters);

    void setTarget(float x, float y, RBDC_reference reference = RBDC_reference::absolute);
    void setTarget(
            float x, float y, float theta, RBDC_reference reference = RBDC_reference::absolute);
    void setTarget(position target_pos, RBDC_reference reference = RBDC_reference::absolute);
    void setTarget(target_position rbdc_target_pos);
    void setVector(float x, float y);
    void trapeze_init(float X, float Y, float Theta);
    void trapeze_calcul();

    void cancel(); // cancel current target.

    void pause(); // save current goal, wait for next start to continue
    void stop(); // cancel current target and put RBD in standby mode. Need start to wake up.
    void start(); // get out of standby mode.

    int getRunningDirection();

    void setAbsolutePosition(float x, float y, float theta);
    void setAbsolutePosition(position absolute_pos);

    RBDC_status update();

    target_position getTarget();

private:
    target_speeds _rbdc_cmds;
    bool _standby = false;
    int _running_direction;
    void updateMobileBase();
    void updateTargetFromVector();

    Odometry *_odometry;
    MobileBase *_mobile_base;

    RBDC_params _parameters;
    target_position _target_pos;
    trapeze _trapeze_x, _trapeze_y, _trapeze_theta;
    position _target_vector;
    PID _pid_dv, _pid_dtheta, _pid_dtan;
    PID_args _args_pid_dv, _args_pid_dtheta, _args_pid_dtan;

    float _arrived_theta = 0.0f;
    bool _dv_zone_reached = false;
    bool _first_move = true;
};

} // namespace sixtron

#endif // CATIE_SIXTRON_RBDC_H_
