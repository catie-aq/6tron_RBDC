/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_RBDC_H_
#define CATIE_SIXTRON_RBDC_H_

#include <stdint.h>
#include <pid.h>
#include <math.h>

namespace sixtron {

    //Defines for direction, useful for 2 wheels robots only ?
#define RBDC_DIR_FORWARD             (1)
#define RBDC_DIR_BACKWARD            (-1)

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

    typedef struct RBDC_outputs RBDC_outputs;
    struct RBDC_outputs {
        float cmd_vel = 0.0f;
        float cmd_rot = 0.0f;
        float cmd_tra = 0.0f; // for omnidirectional drive robots (more than 2 wheels)
    };


    typedef struct RBDC_position RBDC_position;
    struct RBDC_position {
        float x = 0.0f;
        float y = 0.0f;
        float theta = 0.0f;
    };

    class RBDC {

    public:
        RBDC(RBDC_params rbdc_parameters);

        void setTarget(RBDC_position target_pos);

        RBDC_status compute(RBDC_position curent_pos);

        RBDC_outputs getSpeeds();

        RBDC_params _parameters;

    private:

        RBDC_position _target_pos;
        PID _pid_dv, _pid_dtheta;
        PID_args _args_pid_dv, _args_pid_dtheta;

        float _target_angle,_delta_angle;
        int _compute_dv_off, _compute_XY_angle, _compute_last_angle, _compute_cap_dv;

    };

} // namespace sixtron

#endif // CATIE_SIXTRON_RBDC_H_
