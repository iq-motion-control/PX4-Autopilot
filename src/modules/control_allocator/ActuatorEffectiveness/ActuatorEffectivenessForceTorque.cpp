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

#include "ActuatorEffectivenessForceTorque.hpp"

using namespace matrix;

ActuatorEffectivenessForceTorque::ActuatorEffectivenessForceTorque(ModuleParams *parent)
	: ModuleParams(parent)
{
	for (int i = 0; i < NUM_FT_MAX; ++i) {
		// param_t torque_axis[3];
		// param_t position[3];
		// param_t axis[3];
		// param_t thrust_coef;
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_FT%u_TRQ_R", i);
		_param_handles[i].torque_axis[0] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_FT%u_TRQ_P", i);
		_param_handles[i].torque_axis[1] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_FT%u_TRQ_Y", i);
		_param_handles[i].torque_axis[2] = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_FT%u_PX", i);
		_param_handles[i].position[0] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_FT%u_PY", i);
		_param_handles[i].position[1] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_FT%u_PZ", i);
		_param_handles[i].position[2] = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_FT%u_AX", i);
		_param_handles[i].axis[0] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_FT%u_AY", i);
		_param_handles[i].axis[1] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_FT%u_AZ", i);
		_param_handles[i].axis[2] = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_FT%u_CT", i);
		_param_handles[i].thrust_coef = param_find(buffer);

	}

	_count_handle = param_find("CA_FT_COUNT");
	updateParams();
}

void ActuatorEffectivenessForceTorque::updateParams()
{
	ModuleParams::updateParams();

	int32_t count = 0;

	if (param_get(_count_handle, &count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	_count = count;

	for (int i = 0; i < _count; i++) {
		Vector3f &torque_axis = _geometry.rotors[i].torque_axis;
		Vector3f &position = _geometry.rotors[i].position;
		Vector3f &axis = _geometry.rotors[i].axis;

		for (int n = 0; n < 3; ++n) {
			param_get(_param_handles[i].torque_axis[n], &torque_axis(n));
			param_get(_param_handles[i].position[n], &position(n));
			param_get(_param_handles[i].axis[n], &axis(n));
		}

		param_get(_param_handles[i].thrust_coef, &_geometry.rotors[i].thrust_coef);
	}
}

bool ActuatorEffectivenessForceTorque::addActuators(Configuration &configuration)
{
	int num_actuators = computeEffectivenessMatrix(_geometry,
			    configuration.effectiveness_matrices[configuration.selected_matrix],
			    configuration.num_actuators_matrix[configuration.selected_matrix]);
	configuration.actuatorsAdded(ActuatorType::FT, num_actuators);

	return true;
}

int ActuatorEffectivenessForceTorque::computeEffectivenessMatrix(const Geometry &geometry,
		EffectivenessMatrix &effectiveness, int actuator_start_index)
{
	int num_actuators = 0;

	for (int i = 0; i < geometry.num_rotors; i++) {

		if (i + actuator_start_index >= NUM_ACTUATORS) {
			break;
		}

		++num_actuators;

		// Get rotor axis
		Vector3f axis = geometry.rotors[i].axis;

		// Normalize axis
		float axis_norm = axis.norm();

		if (axis_norm > FLT_EPSILON) {
			axis /= axis_norm;

		} else {
			// Bad axis definition, ignore this rotor
			continue;
		}

		// Get rotor position
		const Vector3f &position = geometry.rotors[i].position;

		// Get coefficients
		float ct = geometry.rotors[i].thrust_coef;

		if (fabsf(ct) < FLT_EPSILON) {
			continue;
		}

		// Compute thrust generated by this rotor
		const::Vector3f thrust = ct * axis;

		// Compute moment generated force of this rotor
		matrix::Vector3f moment = ct * position.cross(axis);

		// Get direct torque magnitude
		const Vector3f &torque = geometry.rotors[i].torque_axis;

		// Fill corresponding items in effectiveness matrix
		for (size_t j = 0; j < 3; j++) {
			effectiveness(j, i + actuator_start_index) = torque(j) + moment(j);
			effectiveness(j + 3, i + actuator_start_index) = thrust(j);
		}
	}

	return num_actuators;
}
