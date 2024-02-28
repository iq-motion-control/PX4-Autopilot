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
#include "vertiq_client_manager.hpp"

VertiqClientManager::VertiqClientManager(VertiqSerialInterface *serial_interface) :
	_configuration_client_handler(serial_interface),
	_serial_interface(serial_interface),
	_broadcast_prop_motor_control(_kBroadcastID), //Initialize with a module ID of 63 for broadcasting
	_broadcast_arming_handler(_kBroadcastID)
{
	AddNewOperationalClient(&_broadcast_prop_motor_control);
	AddNewOperationalClient(&_broadcast_arming_handler);
}

void VertiqClientManager::Init(uint8_t object_id)
{
	_configuration_client_handler.InitConfigurationClients(object_id);
	_configuration_client_handler.InitClientEntryWrappers();
}

void VertiqClientManager::HandleClientCommunication()
{
	//Called periodically in the main loop to handle all communications not handled directly by
	//parameter setting
	//Update our serial tx before we take in the rx
	_serial_interface->process_serial_tx();

	//Update our serial rx
	_serial_interface->process_serial_rx_for_all(_operational_client_array, _operational_clients_in_use, _configuration_client_handler.GetClientArray(), _configuration_client_handler.GetNumberOfConfigurationClients());
}

void VertiqClientManager::SendSetForceArm()
{
	_broadcast_arming_handler.motor_armed_.set(*_serial_interface->get_iquart_interface(), 1);
}

void VertiqClientManager::SendSetForceDisarm()
{
	_broadcast_arming_handler.motor_armed_.set(*_serial_interface->get_iquart_interface(), 0);
}

void VertiqClientManager::SendSetCoast()
{
	_broadcast_prop_motor_control.ctrl_coast_.set(*_serial_interface->get_iquart_interface());
}

void VertiqClientManager::SendSetVelocitySetpoint(uint16_t velocity_setpoint)
{
	_broadcast_prop_motor_control.ctrl_velocity_.set(*_serial_interface->get_iquart_interface(), velocity_setpoint);
}

void VertiqClientManager::AddNewOperationalClient(ClientAbstract * client){
	if(_operational_clients_in_use < MAXIMUM_OPERATIONAL_CLIENTS){
		_operational_client_array[_operational_clients_in_use] = client;
		_operational_clients_in_use++;
	}else{
		PX4_INFO("Could not add this client. Maximum number exceeded");
	}
}

uint8_t VertiqClientManager::GetNumberOfOperationalClients(){
	return _operational_clients_in_use;
}
