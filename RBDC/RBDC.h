/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_RBDC_H_
#define CATIE_SIXTRON_RBDC_H_

#include <math.h>
#include <stdint.h>

#include "motor_base/motor_base.h"
#include "odometry/odometry.h"
#include "pid/pid.h"

namespace sixtron {

// Defines for direction, useful for 2 wheels robots only ?
#define RBDC_DIR_FORWARD (1)
#define RBDC_DIR_BACKWARD (-1)

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
    RBDC_done = 1, // 1.1.1 robot arrive on target
    RBDC_correct_final_angle = 2, // 1.1.2
    RBDC_moving = 3, // 1.2.1
    RBDC_moving_and_correct_angle = 4, // 1.2.2.1
    RBDC_correct_initial_angle = 5, // 1.2.2.2
} RBDC_status;

/*!
 *  \struct RBDC_param
 *  PID parameters structure
 */
typedef struct RBDC_params RBDC_params;

struct RBDC_params {
    RBDC_format rbdc_format = two_wheels_robot;
    PID_params pid_param_dv, pid_param_dteta;
    float max_output_dtheta = 1.0f; // max command output, eg -1.0f to +1.0f
    float max_output_dv = 1.0f;
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
    RBDC(Odometry *odometry, MotorBase *motor_base, RBDC_params rbdc_parameters);

    void setTarget(float x, float y, float theta, RBDC_reference reference = RBDC_reference::absolute);
    void setTarget(position target_pos, RBDC_reference reference = RBDC_reference::absolute);

    void cancel(); // cancel current target.

    void pause(); // save current goal, wait for next start to continue
    void stop(); // cancel current target and put RBD in standby mode. Need start to wake up.
    void start(); // get out of standby mode.

    void setAbsolutePosition(float x, float y, float theta);
    void setAbsolutePosition(position absolute_pos);

    RBDC_status update();

private:
    bool _standby = false;
    void updateMotorBase();

    Odometry *_odometry;
    MotorBase *_motor_base;

    RBDC_params _parameters;
    position _target_pos;
    PID _pid_dv, _pid_dtheta;
    PID_args _args_pid_dv, _args_pid_dtheta;
    bool _dv_zone_reached = false;
};

} // namespace sixtron

#endif // CATIE_SIXTRON_RBDC_H_
