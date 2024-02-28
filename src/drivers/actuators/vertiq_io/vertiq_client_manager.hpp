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
#ifndef VERTIQ_CLLIENT_MANAGER_HPP
#define VERTIQ_CLLIENT_MANAGER_HPP

#include <drivers/device/device.h>
#include <lib/led/led.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>

#include <parameters/param.h>

#include <px4_log.h>
#include <px4_platform_common/module.h>

#include "vertiq_serial_interface.hpp"

#include "entry_wrapper.hpp"
#include "vertiq_configuration_client_handler.hpp"

#include "iq-module-communication-cpp/inc/propeller_motor_control_client.hpp"
#include "iq-module-communication-cpp/inc/brushless_drive_client.hpp"
#include "iq-module-communication-cpp/inc/arming_handler_client.hpp"

#include "iq-module-communication-cpp/inc/esc_propeller_input_parser_client.hpp"
#include "iq-module-communication-cpp/inc/iquart_flight_controller_interface_client.hpp"

#ifdef CONFIG_USE_PULSING_CONFIGURATION
#include "iq-module-communication-cpp/inc/voltage_superposition_client.hpp"
#include "iq-module-communication-cpp/inc/pulsing_rectangular_input_parser_client.hpp"
#endif //CONFIG_USE_PULSING_CONFIGURATION

static const uint8_t _kBroadcastID = 63;

class VertiqClientManager
{
public:
	/**
	* @brief Construct a new VertiqClientManager object
	*
	* @param serial_interface A pointer to a VertiqSerialInterface object
	*/
	VertiqClientManager(VertiqSerialInterface *serial_interface);

	/**
	* @brief Initialize all of our clients with the object ID given by the PX4 parameter TARGET_MODULE_ID
	*/
	void Init(uint8_t object_id);

	/**
	* @brief Handle the IQUART interface. Make sure that we update TX and RX buffers
	*/
	void HandleClientCommunication();

	/**
	* @brief Adds a new client to our array of Operational Clients. Operational clients are those meant to hold an operational client such as those used
	* for direct motor control. Operational clients should have a constant module ID, and should be made only once
	*/
	void AddNewOperationalClient(ClientAbstract * client);

	/**
	* @brief Returns the number of clients added to our Operational Clients array
	*
	* @return The value _operational_clients_in_use
	*/
	uint8_t GetNumberOfOperationalClients();

	VertiqConfigurationClilentHandler _configuration_client_handler;

private:
	//We need a serial handler in order to talk over the serial port
	VertiqSerialInterface *_serial_interface;

////////////////////////////////////////////////////////////////////////
//Vertiq client information

/**
In order to communicate with connected Vertiq modules with the most possible flexibility, we have introduced two types of Clients (see comment above about
Vertiq clients and entries) to be used within PX4. We have designated "Operational Clients" as those whose object IDs (a.k.a target module IDs) are constant.
For example, when sending IFCI commands, we are always transmitting them using the object ID 63 in order to broadcast to all connected Vertiq modules. There is no
reason to change its object ID. In order to interact with specific client entries from specific modules, we have introduced "Configuration Clients." Configuration
clients are those whose object IDs are not constant, and which are created and destroyed as the Target Module ID parameter is updated. An example of a configuration
parameter is each module's Velocity Max entry. Since each module has its own version of a Velocity Max entry, designated by unique module IDs, we need a way to
dynamically change the client's object ID to reach the correct module. Our "Configuration Clients" are used to meet this end.

An example of a Vertiq client is documented here https://iqmotion.readthedocs.io/en/latest/modules/vertiq_2306_2200.html#propeller-motor-control. In this case,
Propeller Motor Control is the client, and its entries are specified in the message table (https://iqmotion.readthedocs.io/en/latest/modules/vertiq_2306_2200.html#id6).
You can find the C++ representation in ./src/drivers/actuators/vertiq_io/iq-module-communication-cpp/inc/propeller_motor_control_client.hpp
*/

	//Some constants to help us out
	static const uint8_t MAXIMUM_OPERATIONAL_CLIENTS = 20; //These are clients that are used for module control/telemetry. They have a static Module ID
	ClientAbstract *_operational_client_array[MAXIMUM_OPERATIONAL_CLIENTS];
	uint8_t _operational_clients_in_use = 0;
////////////////////////////////////////////////////////////////////////
};

#endif
