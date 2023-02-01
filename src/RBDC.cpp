/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "RBDC/RBDC.h"

namespace sixtron {

RBDC::RBDC(RBDC_params rbdc_parameters):
        _parameters(rbdc_parameters),
        _pid_dv(rbdc_parameters.pid_param_dv, rbdc_parameters.dt_seconds),
        _pid_dtheta(rbdc_parameters.pid_param_dteta, rbdc_parameters.dt_seconds)
{

    _compute_XY_angle = 1;
    _compute_last_angle = 0;
    _compute_cap_dv = 0;

    _pid_dv.setLimit(sixtron::PID_limit::output_limit_HL, _parameters.max_output);
}

void RBDC::setTarget(RBDC_position target_pos)
{

    // Check and copy new targets values
    _target_pos = target_pos;
}

static inline float getDeltaFromTargetTHETA(float target_angle_deg, float current_angle)
{

    float delta = fmod((target_angle_deg - current_angle), float(2 * M_PI));

    if (delta > float(M_PI)) {
        delta -= float(2 * M_PI);
    } else if (delta < -float(M_PI)) {
        delta += float(2 * M_PI);
    }

    return delta;
}

RBDC_status RBDC::compute(RBDC_position current_pos)
{

    float e_x = _target_pos.x - current_pos.x;
    float e_y = _target_pos.y - current_pos.y;
    float error_dv = sqrtf((e_x * e_x) + (e_y * e_y));

    // 1 Q : Is robot inside the target zone ?
    if ((error_dv < _parameters.dv_precision) && (error_dv > -_parameters.dv_precision)) {
        // 1.1 A : Yes it is.

        // Be sure that dv is shutdown
        _args_pid_dv.output = 0.0f;
        // Compute the final angle
        float delta_angle = getDeltaFromTargetTHETA(_target_pos.theta, current_pos.theta);

        // 1.1 Q : Is target angle (or final angle) correct ?
        if (abs(delta_angle) < _parameters.theta_precision) {
            // 1.1.1 A : Yes it is. The robot base is in target position.
            _args_pid_dtheta.output
                    = 0.0f; // be sure to stop correcting dtheta, as precision is reached.
            return RBDC_status::RBDC_done;
        } else {
            // 1.1.2 A : No it is not.

            // then update pid theta
            _args_pid_dtheta.actual = 0.0f;
            _args_pid_dtheta.target = delta_angle;
            _pid_dtheta.compute(&_args_pid_dtheta);

            return RBDC_status::RBDC_correct_final_angle;
        }

    } else {
        // 1.2 A : No it isn't. The base has to move to the target position.

        // Compute the angle error based on target X/Y
        float target_angle
                = (atan2f((_target_pos.y - current_pos.y), (_target_pos.x - current_pos.x)));
        float delta_angle = getDeltaFromTargetTHETA(target_angle, current_pos.theta);

        float running_direction = RBDC_DIR_FORWARD;
        // Check if it is better to go backward or not. Update delta angle accordingly.
        if (_parameters.can_go_backward) {
            if (delta_angle > float(M_PI_2)) {
                delta_angle -= float(M_PI);
                running_direction = RBDC_DIR_BACKWARD;
            } else if (delta_angle < -float(M_PI_2)) {
                delta_angle += float(M_PI);
                running_direction = RBDC_DIR_BACKWARD;
            }
        }

        // update pid theta
        _args_pid_dtheta.actual = 0.0f;
        _args_pid_dtheta.target = delta_angle;
        _pid_dtheta.compute(&_args_pid_dtheta);

        // 1.2 Q : Is the base align with the target position (angle thinking) ?
        if (abs(delta_angle) < _parameters.theta_precision) {
            // 1.2.1 A : Yes it is.

            error_dv = running_direction * error_dv; // Add direction of moving

            // update pid dv
            _args_pid_dv.actual = 0.0f;
            _args_pid_dv.target = error_dv;
            _pid_dv.compute(&_args_pid_dv);

            return RBDC_status::RBDC_moving;

        } else {
            // 1.2.2 A : No it isn't. Angle must be corrected.

            // 1.2.2 Q : Is the robot already moving ?
            if (_args_pid_dv.output == 0.0f) {
                // 1.2.2.2 A : No it is not. Must be the first angle.
                return RBDC_status::RBDC_correct_initial_angle;
            } else {
                // 1.2.2.1 A : Yes it is. The Base is probably already moving to the target
                // position. But, the PID Theta can't keep up (maybe output PWM are already at max)
                // So we need to reduce the speed, in order for the angle to be corrected correctly.
                _args_pid_dv.output = _args_pid_dv.output * 0.80f; // reduce by 20%
                return RBDC_status::RBDC_moving_and_correct_angle;
            }
        }
    }
}

RBDC_outputs RBDC::getSpeeds()
{

    RBDC_outputs cmd_outputs;

    cmd_outputs.cmd_rot = _args_pid_dtheta.output;
    cmd_outputs.cmd_vel = _args_pid_dv.output;

    return cmd_outputs;
}

} // namespace sixtron
