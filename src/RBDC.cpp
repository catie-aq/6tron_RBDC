/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#include "RBDC/RBDC.h"

namespace sixtron {

RBDC::RBDC(Odometry *odometry, MotorBase *motor_base, RBDC_params rbdc_parameters):
        _odometry(odometry),
        _motor_base(motor_base),
        _parameters(rbdc_parameters),
        _pid_dv(rbdc_parameters.pid_param_dv, rbdc_parameters.dt_seconds),
        _pid_dtheta(rbdc_parameters.pid_param_dteta, rbdc_parameters.dt_seconds)
{
    _pid_dv.setLimit(sixtron::PID_limit::output_limit_HL, _parameters.max_output_dv);
    _pid_dtheta.setLimit(sixtron::PID_limit::output_limit_HL, _parameters.max_output_dtheta);

    if (_parameters.dv_reducing_coefficient < 0.0f) {
        _parameters.dv_reducing_coefficient = 0.0f;
    } else if (_parameters.dv_reducing_coefficient > 1.0f) {
        _parameters.dv_reducing_coefficient = 1.0f;
    }

    if (_parameters.dv_precision > _parameters.target_precision) {
        _parameters.dv_precision = _parameters.target_precision;
    }

    _odometry->init();
    _motor_base->init();
}

void RBDC::setTarget(position target_pos)
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

RBDC_status RBDC::update()
{

    // ====== Get actual odometry ===========
    _odometry->update();

    // ======= Standby check ================

    if (_standby) {

        _args_pid_dv.output = 0.0f;
        _args_pid_dtheta.output = 0.0f;

        updateMotorBase();

        return RBDC_status::RBDC_standby;
    }

    // =========== Run RBDC =================
    RBDC_status rbdc_end_status;
    float e_x = _target_pos.x - _odometry->getX();
    float e_y = _target_pos.y - _odometry->getY();
    float error_dv = sqrtf((e_x * e_x) + (e_y * e_y));

    // 1 Q : Is robot inside the target zone ?
    if ((error_dv < _parameters.target_precision) && (error_dv > -_parameters.target_precision)) {
        // 1.1 A : Yes it is.

        // Check if robot is inside dv zone. Target zone must be greater than dv zone.
        if ((error_dv < _parameters.dv_precision) && (error_dv > -_parameters.dv_precision)) {
            _dv_zone_reached = true;
        }

        // Correct angle ONLY if inside target zone AND dv zone already reached
        if (_dv_zone_reached) {
            // Be sure that dv is shutdown
            _args_pid_dv.output = 0.0f;
            // Compute the final angle
            float delta_angle = getDeltaFromTargetTHETA(_target_pos.theta, _odometry->getTheta());

            // 1.1 Q : Is target angle (or final angle) correct ?
            if (abs(delta_angle) < _parameters.final_theta_precision) {
                // 1.1.1 A : Yes it is. The robot base is in target position.
                _args_pid_dtheta.output
                        = 0.0f; // be sure to stop correcting dtheta, as precision is reached.
                rbdc_end_status = RBDC_status::RBDC_done;
            } else {
                // 1.1.2 A : No it is not.

                // then update pid theta
                _args_pid_dtheta.actual = 0.0f;
                _args_pid_dtheta.target = delta_angle;
                _pid_dtheta.compute(&_args_pid_dtheta);

                rbdc_end_status = RBDC_status::RBDC_correct_final_angle;
            }
        }

    } else {
        _dv_zone_reached = false;
    }

    if (!_dv_zone_reached) {
        // 1.2 A : No it isn't. The base has to move to the target position.

        // Compute the angle error based on target X/Y
        float target_angle = (atan2f(
                (_target_pos.y - _odometry->getY()), (_target_pos.x - _odometry->getX())));
        float delta_angle = getDeltaFromTargetTHETA(target_angle, _odometry->getTheta());

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
        if (abs(delta_angle) < _parameters.moving_theta_precision) {
            // 1.2.1 A : Yes it is.

            error_dv = running_direction * error_dv; // Add direction of moving

            // update pid dv
            _args_pid_dv.actual = 0.0f;
            _args_pid_dv.target = error_dv;
            _pid_dv.compute(&_args_pid_dv);

            rbdc_end_status = RBDC_status::RBDC_moving;

        } else {
            // 1.2.2 A : No it isn't. Angle must be corrected.

            // 1.2.2 Q : Is the robot already moving ?
            if (_args_pid_dv.output == 0.0f) {
                // 1.2.2.2 A : No it is not. Must be the first angle.
                rbdc_end_status = RBDC_status::RBDC_correct_initial_angle;
            } else {
                // 1.2.2.1 A : Yes it is. The Base is probably already moving to the target
                // position. But, the PID Theta can't keep up (maybe output PWM are already at max)
                // So we need to reduce the speed, in order for the angle to be corrected correctly.

                _args_pid_dv.output = _args_pid_dv.output
                        * _parameters.dv_reducing_coefficient; // reduce by given coefficient,
                                                               // only one time

                rbdc_end_status = RBDC_status::RBDC_moving_and_correct_angle;
            }
        }
    }

    // ======== Update Motor Base ============
    updateMotorBase();

    return rbdc_end_status;
}

void RBDC::cancel()
{
    _target_pos.x = _odometry->getX();
    _target_pos.y = _odometry->getY();
    _target_pos.theta = _odometry->getTheta();
}

void RBDC::stop()
{
    if (!_standby) {
        cancel();
        _standby = true;
    }
}

void RBDC::start()
{
    if (_standby) {
        _standby = false;
    }
}

void RBDC::updateMotorBase()
{
    target_speeds rbdc_cmds;
    rbdc_cmds.cmd_rot = _args_pid_dtheta.output;
    rbdc_cmds.cmd_lin = _args_pid_dv.output;

    _motor_base->setTargetSpeeds(rbdc_cmds);
    _motor_base->update();
}

} // namespace sixtron
