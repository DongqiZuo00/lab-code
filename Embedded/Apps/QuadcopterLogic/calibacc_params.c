/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file calibacc_params.c
 *
 * Parameters hiperlab use
 */


/**
 * Accelerometer calibration , scale x
 *
 * Correct meas errors.
 *
 * @group Accelerometer calibration
 * @unit m/s^2
 * @decimal 2
 * @min 0.5
 * @max 2.0
 * @increment 0.001
 */
PARAM_DEFINE_FLOAT(CALIBACC_KX, 1.0f);

/**
 * Accelerometer calibration , scale y
 *
 * Correct meas errors.
 *
 * @group Accelerometer calibration
 * @unit m/s^2
 * @decimal 2
 * @min 0.5
 * @max 2.0
 * @increment 0.001
 */
PARAM_DEFINE_FLOAT(CALIBACC_KY, 1.0f);

/**
 * Accelerometer calibration , scale z
 *
 * Correct meas errors.
 *
 * @group Accelerometer calibration
 * @unit m/s^2
 * @decimal 2
 * @min 0.5
 * @max 2.0
 * @increment 0.001
 */
PARAM_DEFINE_FLOAT(CALIBACC_KZ, 1.0f);

/**
 * Accelerometer calibration , bias x
 *
 * Correct meas errors.
 *
 * @group Accelerometer calibration
 * @unit m/s^2
 * @decimal 4
 * @min -10.0
 * @max  10.0
 * @increment 0.001
 */
PARAM_DEFINE_FLOAT(CALIBACC_BX, 0.0f);

/**
 * Accelerometer calibration , bias y
 *
 * Correct meas errors.
 *
 * @group Accelerometer calibration
 * @unit m/s^2
 * @decimal 4
 * @min -10.0
 * @max  10.0
 * @increment 0.001
 */
PARAM_DEFINE_FLOAT(CALIBACC_BY, 0.0f);

/**
 * Accelerometer calibration , bias z
 *
 * Correct meas errors.
 *
 * @group Accelerometer calibration
 * @unit m/s^2
 * @decimal 4
 * @min -10.0
 * @max  10.0
 * @increment 0.001
 */
PARAM_DEFINE_FLOAT(CALIBACC_BZ, 0.0f);

/**
 * Propeller coefficient correction , prop 0 (after mixer, CF2-definition)
 *
 * Correct unknown factor in conversion from
 * commanded to real forces
 *
 * @group Propeller correction
 * @unit
 * @decimal 4
 * @min 0.0
 * @max 2.0
 * @increment 0.001
 */
PARAM_DEFINE_FLOAT(PROPCORR_0, 1.0f);

/**
 * Propeller coefficient correction , prop 1 (after mixer, CF2-definition)
 *
 * Correct unknown factor in conversion from
 * commanded to real forces
 *
 * @group Propeller correction
 * @unit
 * @decimal 4
 * @min 0.0
 * @max 2.0
 * @increment 0.001
 */
PARAM_DEFINE_FLOAT(PROPCORR_1, 1.0f);

/**
 * Propeller coefficient correction , prop 2 (after mixer, CF2-definition)
 *
 * Correct unknown factor in conversion from
 * commanded to real forces
 *
 * @group Propeller correction
 * @unit
 * @decimal 4
 * @min 0.0
 * @max 2.0
 * @increment 0.001
 */
PARAM_DEFINE_FLOAT(PROPCORR_2, 1.0f);

/**
 * Propeller coefficient correction , prop 3 (after mixer, CF2-definition)
 *
 * Correct unknown factor in conversion from
 * commanded to real forces
 *
 * @group Propeller correction
 * @unit
 * @decimal 4
 * @min 0.0
 * @max 2.0
 * @increment 0.001
 */
PARAM_DEFINE_FLOAT(PROPCORR_3, 1.0f);


/**
 * Motor type
 *
 *
 */
PARAM_DEFINE_INT32(MOTOR_TYPE, 0);


/**
 * FC type
 *
 *
 */
PARAM_DEFINE_INT32(FC_TYPE, 0);


/**
 * Vehicle ID (unique identifier)
 *
 *
 */
PARAM_DEFINE_INT32(VEHICLE_ID, 0);
