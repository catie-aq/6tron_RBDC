/*
 * Copyright (c) 2022, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef CATIE_SIXTRON_RBDC_H_
#define CATIE_SIXTRON_RBDC_H_

#include <stdint.h>
#include <pid.h>

namespace sixtron {

    class RobotBaseDriveControl {

    public:
        RobotBaseDriveControl();

        void compute();

    };

} // namespace sixtron

#endif // CATIE_SIXTRON_RBDC_H_
