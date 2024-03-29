/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/log.h>

#include "ActuatorEffectivenessDirect.hpp"

using namespace matrix;

ActuatorEffectivenessDirect::ActuatorEffectivenessDirect(ModuleParams *parent)
	: ModuleParams(parent)
{
	for (int i = 0; i < MAX_COUNT; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_DR_M%u_TRQ_X", i);
		_param_handles[i].torque[0] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_DR_M%u_TRQ_Y", i);
		_param_handles[i].torque[1] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_DR_M%u_TRQ_Z", i);
		_param_handles[i].torque[2] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_DR_M%u_FRC_X", i);
		_param_handles[i].force[0] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_DR_M%u_FRC_Y", i);
		_param_handles[i].force[1] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_DR_M%u_FRC_Z", i);
		_param_handles[i].force[2] = param_find(buffer);
	}

	_count_handle = param_find("CA_DR_M_COUNT");
	updateParams();
}

void ActuatorEffectivenessDirect::updateParams()
{
	ModuleParams::updateParams();

	int32_t count = 0;

	if (param_get(_count_handle, &count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	_count = count;

	for (int i = 0; i < _count; i++) {
		Vector3f &torque = _params[i].torque;
		Vector3f &force = _params[i].force;

		for (int n = 0; n < 3; ++n) {
			param_get(_param_handles[i].torque[n], &torque(n));
			param_get(_param_handles[i].force[n], &force(n));
		}
	}
}

bool ActuatorEffectivenessDirect::addActuators(Configuration &configuration)
{
	for (int i = 0; i < _count; i++) {
		configuration.addActuator(ActuatorType::MOTORS, _params[i].torque, _params[i].force);
	}
	return true;
}

uint32_t ActuatorEffectivenessDirect::getMotors() const
{
	uint32_t motors = 0;

	for (int i = 0; i < _count; ++i) {
		motors |= 1u << i;
	}

	return motors;
}

