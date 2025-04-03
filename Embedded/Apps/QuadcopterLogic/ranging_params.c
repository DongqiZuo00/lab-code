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
 * copied from file battery_params.c
 *
 * @author mwm
 */


/**
 * Ranging radio ID
 *
 *
 *
 * @group Ranging radio
 * @value 0 Unconfigured
 */
PARAM_DEFINE_INT32(RANGRAD_ID, 0);

/**
 * Ranging radio position, coordinate x
 *
 * Defines the position of the radio
 *
 * @group Ranging radio
 * @unit m
 * @decimal 2
 * @min -10000.0
 * @max 10000.0
 * @increment 0.001
 */
PARAM_DEFINE_FLOAT(RANGRAD_POS_X, 0.0f);

/**
 * Ranging radio position, coordinate y
 *
 * Defines the position of the radio
 *
 * @group Ranging radio
 * @unit m
 * @decimal 2
 * @min -10000.0
 * @max 10000.0
 * @increment 0.001
 */
PARAM_DEFINE_FLOAT(RANGRAD_POS_Y, 0.0f);

/**
 * Ranging radio position, coordinate z
 *
 * Defines the position of the radio
 *
 * @group Ranging radio
 * @unit m
 * @decimal 2
 * @min -10000.0
 * @max 10000.0
 * @increment 0.001
 */
PARAM_DEFINE_FLOAT(RANGRAD_POS_Z, 0.0f);

