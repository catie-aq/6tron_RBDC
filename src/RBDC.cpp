/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "RBDC/RBDC.h"

namespace sixtron {


    RBDC::RBDC(RBDC_params rbdc_parameters) : _parameters(rbdc_parameters),
                                              _pid_dv(rbdc_parameters.pid_param_dv, rbdc_parameters.dt_seconds),
                                              _pid_dtheta(rbdc_parameters.pid_param_dteta, rbdc_parameters.dt_seconds) {

        _compute_dv_off = 0;
        _compute_first_angle = 1;
        _compute_cap_dv = 0;

        _pid_dv.setLimit(sixtron::PID_limit::output_limit_HL, _parameters.max_output);
    }

    void RBDC::setTarget(RBDC_position target_pos) {

        // Check and copy new targets values
        _target_pos = target_pos;
    }


    static inline float getDeltaFromTargetTHETA(float target_angle_deg, float current_angle) {

        float delta = fmod((target_angle_deg - current_angle), float(2 * M_PI));

        if (delta > float(M_PI)) {
            delta -= float(2 * M_PI);
        } else if (delta < -float(M_PI)) {
            delta += float(2 * M_PI);
        }

        return delta;
    }

    RBDC_status RBDC::compute(RBDC_position current_pos) {

        float target_angle = (atan2f((_target_pos.y - current_pos.y), (_target_pos.x - current_pos.x)));
        float delta_angle = getDeltaFromTargetTHETA(target_angle, current_pos.theta);
        float running_direction = RBDC_DIR_FORWARD;


        // Check if it is better to go backward or not
        if (_parameters.can_go_backward) {
            if (delta_angle > float(M_PI_2)) {
                delta_angle -= float(M_PI);
                running_direction = RBDC_DIR_BACKWARD;
            } else if (delta_angle < -float(M_PI_2)) {
                delta_angle += float(M_PI);
                running_direction = RBDC_DIR_BACKWARD;
            }
        }

        // by default
        _compute_dv_off = 0;


        if (abs(delta_angle) > _parameters.theta_precision) {
            _compute_dv_off = 1;
            _compute_first_angle = 0;
        }


        // pid theta
        _args_pid_dtheta.actual = 0.0f;
        _args_pid_dtheta.target = delta_angle;
        _pid_dtheta.compute(&_args_pid_dtheta);

        if (!_compute_dv_off) {
            _compute_cap_dv = 1;// reset cap dv for next round, if angle is to be correct
            float e_x = _target_pos.x - current_pos.x;
            float e_y = _target_pos.y - current_pos.y;

            float error_dv = running_direction * sqrtf((e_x * e_x) + (e_y * e_y));

            if ((error_dv < _parameters.dv_precision) && (error_dv > -_parameters.dv_precision)) {
                _args_pid_dv.output = 0.0f;
                _compute_first_angle = 1;
                return RBDC_done;
            } else {
                // pid dv
                _args_pid_dv.actual = -error_dv;
                _args_pid_dv.target = 0.0f;
                _pid_dv.compute(&_args_pid_dv);
            }

        } else {
            if (_compute_first_angle) {
                _args_pid_dv.output = 0.0f;
            } else {
                if (_compute_cap_dv) {
                    _args_pid_dv.output = _args_pid_dv.output * 0.80f;
                    _compute_cap_dv = 0;
                }
            }

        }
        return RBDC_working;

    }

    RBDC_outputs RBDC::getSpeeds() {

        RBDC_outputs cmd_outputs;

        cmd_outputs.cmd_rot = _args_pid_dtheta.output;
        cmd_outputs.cmd_vel = _args_pid_dv.output;

        return cmd_outputs;
    }

} // namespace sixtron
