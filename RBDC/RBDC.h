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
    RBDC_working = 0,
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
    float max_output = 1.0f; // max command output, eg -1.0f to +1.0f
    float theta_precision = 0.0f;
    float dv_precision = 0.0f;
    float dt_seconds = 0.0f;

    bool can_go_backward = true;
};

class RBDC {

public:
    RBDC(Odometry *odometry, MotorBase *motor_base, RBDC_params rbdc_parameters);

    void setTarget(position target_pos);

    RBDC_status update();

    //    RBDC_outputs getSpeeds();

private:
    Odometry *_odometry;
    MotorBase *_motor_base;

    RBDC_params _parameters;
    position _target_pos;
    PID _pid_dv, _pid_dtheta;
    PID_args _args_pid_dv, _args_pid_dtheta;
};

} // namespace sixtron

#endif // CATIE_SIXTRON_RBDC_H_
