/****************************************************************************
 *
 *   Copyright (c) 2012-2024 PX4 Development Team. All rights reserved.
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
#include "vertiq_configuration_handler.hpp"

VertiqConfigurationHandler::VertiqConfigurationHandler(VertiqSerialInterface * ser, VertiqClientManager * client_manager) :
	_serial_interface(ser),
	_client_manager(client_manager)
{
}

void VertiqConfigurationHandler::InitConfigurationClients(uint8_t object_id)
{
	_object_id_now = object_id; //Make sure we store the initial object ID

	_prop_input_parser_client = new EscPropellerInputParserClient(object_id);
	_client_manager->AddNewClient(_prop_input_parser_client);

#ifdef CONFIG_USE_IFCI_CONFIGURATION
	_ifci_client = new IQUartFlightControllerInterfaceClient(object_id);
	_client_manager->AddNewClient(_ifci_client);
#endif //CONFIG_USE_IFCI_CONFIGURATION

#ifdef CONFIG_USE_PULSING_CONFIGURATION
	_voltage_superposition_client = new VoltageSuperPositionClient(object_id);
	_client_manager->AddNewClient(_voltage_superposition_client);

	_pulsing_rectangular_input_parser_client = new PulsingRectangularInputParserClient(object_id);
	_client_manager->AddNewClient(_pulsing_rectangular_input_parser_client);
#endif //CONFIG_USE_PULSING_CONFIGURATION

//USER ADDED
	_arming_handler = new ArmingHandlerClient(object_id);
	_client_manager->AddNewClient(_arming_handler);
}

void VertiqConfigurationHandler::InitClientEntryWrappers(){
	AddNewClientEntry<float, float>(param_find("MAX_VELOCITY"), &(_prop_input_parser_client->velocity_max_));
	AddNewClientEntry<float, float>(param_find("MAX_VOLTS"), &(_prop_input_parser_client->volts_max_));
	AddNewClientEntry<uint8_t, int32_t>(param_find("CONTROL_MODE"), &(_prop_input_parser_client->mode_));
	AddNewClientEntry<uint8_t, int32_t>(param_find("VERTIQ_MOTOR_DIR"), &(_prop_input_parser_client->sign_));
	AddNewClientEntry<uint8_t, int32_t>(param_find("VERTIQ_FC_DIR"), &(_prop_input_parser_client->flip_negative_));

#ifdef CONFIG_USE_IFCI_CONFIGURATION
	AddNewClientEntry<uint8_t, int32_t>(param_find("THROTTLE_CVI"), &(_ifci_client->throttle_cvi_));
#endif //CONFIG_USE_IFCI_CONFIGURATION

#ifdef CONFIG_USE_PULSING_CONFIGURATION
	AddNewClientEntry<uint8_t, int32_t> (param_find("PULSE_VOLT_MODE"),
			&(_pulsing_rectangular_input_parser_client->pulsing_voltage_mode_));
	AddNewClientEntry<uint8_t, int32_t>(param_find("X_CVI"), &(_ifci_client->x_cvi_));
	AddNewClientEntry<uint8_t, int32_t>(param_find("Y_CVI"), &(_ifci_client->y_cvi_));
	AddNewClientEntry<float, float>(param_find("ZERO_ANGLE"), &(_voltage_superposition_client->zero_angle_));
	AddNewClientEntry<float, float>(param_find("VELOCITY_CUTOFF"),
			&(_voltage_superposition_client->velocity_cutoff_));
	AddNewClientEntry<float, float>(param_find("TORQUE_OFF_ANGLE"),
			&(_voltage_superposition_client->propeller_torque_offset_angle_));
	AddNewClientEntry<float, float>(param_find("PULSE_VOLT_LIM"),
						&(_pulsing_rectangular_input_parser_client->pulsing_voltage_limit_));
#endif //CONFIG_USE_PULSING_CONFIGURATION

//USER ADDED
	AddNewClientEntry<float, float>(param_find("ARM_THROT_HIGH"), &(_arming_handler->arm_throttle_upper_limit_));
	AddNewClientEntry<float, float>(param_find("ARM_THROT_LOW"), &(_arming_handler->arm_throttle_lower_limit_));
}

void VertiqConfigurationHandler::UpdateClientsToNewObjId(uint8_t new_object_id)
{
	_object_id_now = new_object_id;

	DestroyAndRecreateClient<EscPropellerInputParserClient>(_prop_input_parser_client, new_object_id);

#ifdef CONFIG_USE_IFCI_CONFIGURATION
	DestroyAndRecreateClient<IQUartFlightControllerInterfaceClient>(_ifci_client, new_object_id);
#endif

#ifdef CONFIG_USE_PULSING_CONFIGURATION
	DestroyAndRecreateClient<VoltageSuperPositionClient>(_voltage_superposition_client, new_object_id);
	DestroyAndRecreateClient<PulsingRectangularInputParserClient>(_pulsing_rectangular_input_parser_client, new_object_id);
#endif

//USER ADDED
	DestroyAndRecreateClient<ArmingHandlerClient>(_arming_handler, new_object_id);
}

void VertiqConfigurationHandler::MarkConfigurationEntriesForRefresh()
{
	for (uint8_t i = 0; i < _added_configuration_entry_wrappers; i++) {
		_configuration_entry_wrappers[i]->SetNeedsInit();
	}
}

void VertiqConfigurationHandler::UpdateIquartConfigParams()
{
	for (uint8_t i = 0; i < _added_configuration_entry_wrappers; i++) {
		_configuration_entry_wrappers[i]->SendGet(_serial_interface);
		//Ensure that these get messages get out
		_serial_interface->process_serial_tx();
	}

	//Now go ahead and grab responses, and update everyone to be on the same page, but do it quickly.
	CoordinateIquartWithPx4Params();
}

void VertiqConfigurationHandler::CoordinateIquartWithPx4Params(hrt_abstime timeout)
{
	//Get the start time and the end time
	hrt_abstime time_now = hrt_absolute_time();
	hrt_abstime end_time = time_now + timeout;

	while (time_now < end_time) {
		for (uint8_t i = 0; i < _added_configuration_entry_wrappers; i++) {
			_configuration_entry_wrappers[i]->Update(_serial_interface);
		}

		//Update the time
		time_now = hrt_absolute_time();

		//Update our serial rx
		_client_manager->HandleClientCommunication();
	}
}

uint8_t VertiqConfigurationHandler::GetObjectIdNow(){
	return _object_id_now;
}
