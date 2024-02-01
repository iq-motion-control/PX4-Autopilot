
#include "vertiq_client_manager.hpp"

VertiqClientManager::VertiqClientManager(VertiqSerialInterface * serial_interface) :
	_serial_interface(serial_interface),
	_broadcast_prop_motor_control(_kBroadcastID), //Initialize with a module ID of 63 for broadcasting
	_broadcast_arming_handler(_kBroadcastID)

{
	_client_array[0] = &_broadcast_prop_motor_control;
	_client_array[1] = &_broadcast_arming_handler;
}

void VertiqClientManager::Init(uint8_t object_id){
	#ifdef CONFIG_USE_IFCI_CONFIGURATION
		static IQUartFlightControllerInterfaceClient ifci = IQUartFlightControllerInterfaceClient(object_id);
		_ifci_client = &ifci;
		_client_array[_clients_in_use] = _ifci_client;
		_clients_in_use++;

		static EscPropellerInputParserClient prop_input_parser = EscPropellerInputParserClient(object_id);
		_prop_input_parser_client = &prop_input_parser;
		_client_array[_clients_in_use] = _prop_input_parser_client;
		_clients_in_use++;
	#endif


	_serial_interface->SetNumberOfClients(_clients_in_use);
}

void VertiqClientManager::HandleClientCommunication(){
	//Update our serial tx before we take in the RX
	_serial_interface->process_serial_tx();

	//Update our serial rx
	_serial_interface->process_serial_rx(&_motor_interface, _client_array);
}

IFCI * VertiqClientManager::GetMotorInterface(){
	return &_motor_interface;
}

uint8_t VertiqClientManager::GetNumberOfClients(){
	return _clients_in_use;
}

void VertiqClientManager::SendSetForceArm(){
	_broadcast_arming_handler.motor_armed_.set(*_serial_interface->get_iquart_interface(), 1);
}

void VertiqClientManager::SendSetForceDisarm(){
	_broadcast_arming_handler.motor_armed_.set(*_serial_interface->get_iquart_interface(), 0);
}

void VertiqClientManager::SendSetCoast(){
	_broadcast_prop_motor_control.ctrl_coast_.set(*_serial_interface->get_iquart_interface());
}

void VertiqClientManager::SendSetVelocitySetpoint(uint16_t velocity_setpoint){
	_broadcast_prop_motor_control.ctrl_velocity_.set(*_serial_interface->get_iquart_interface(), velocity_setpoint);
}

void VertiqClientManager::InitParameter(param_t parameter, bool * init_bool, char descriptor, EntryData * value){
	float float_value = value->float_data;
	uint32_t uint_value = value->uint_data;

	switch(descriptor){
		case 'f':
			param_set(parameter, &(float_value));
			*init_bool = false;
		break;

		case 'b':
			param_set(parameter, &(uint_value));
			*init_bool = false;
		break;

		default:
		return; //Don't go any further
		break;
	}
}

void VertiqClientManager::SendSetAndSave(ClientEntryAbstract * entry, char descriptor, EntryData * value){
	//Note that we have to use the brackets to make sure that the ClientEntry objects have a scope
	switch(descriptor){
		case 'f': {
			ClientEntry<float> * float_entry = (ClientEntry<float> *)(entry);
			float_entry->set(*_serial_interface->get_iquart_interface(), value->float_data);
			float_entry->save(*_serial_interface->get_iquart_interface());
		break;
		}
		case 'b':{
			ClientEntry<uint8_t> * byte_entry = (ClientEntry<uint8_t> *)(entry);
			byte_entry->set(*_serial_interface->get_iquart_interface(), value->uint_data);
			byte_entry->save(*_serial_interface->get_iquart_interface());
		break;
		}
		default:
		return; //Don't go any further
		break;
	}

	//Make sure our message gets out
	_serial_interface->process_serial_tx();
}

bool VertiqClientManager::FloatsAreClose(float val1, float val2, float tolerance){
	float diff = val1 - val2;
	return(abs(diff) < tolerance);
}

void VertiqClientManager::UpdateParameter(param_t parameter, bool * init_bool, char descriptor, EntryData * value, ClientEntryAbstract * entry){
	//Note that we have to use the brackets to make sure that the ClientEntry objects have a scope
	switch(descriptor){
		case 'f': {
			float module_float_value = 0;
			float px4_float_value = 0;
			ClientEntry<float> * float_entry = (ClientEntry<float> *)(entry);
			if(float_entry->IsFresh()){
				module_float_value = float_entry->get_reply();

				if(*init_bool){
					value->float_data = module_float_value;
					InitParameter(parameter, init_bool, descriptor, value);
				}else{
					param_get(parameter, &px4_float_value);

					if(!FloatsAreClose(px4_float_value, module_float_value)){
						value->float_data = px4_float_value;
						SendSetAndSave(float_entry, descriptor, value);
					}
				}
			}
		break;
		}
		case 'b':{
			uint32_t module_read_value = 0;
			int32_t px4_read_value = 0;
			ClientEntry<uint8_t> * byte_entry = (ClientEntry<uint8_t> *)(entry);
			if(byte_entry->IsFresh()){
				module_read_value = byte_entry->get_reply();
				if(*init_bool){
					value->uint_data = module_read_value;
					InitParameter(parameter, init_bool, 'b', value);
				}else{
					param_get(parameter, &px4_read_value);

					if((uint32_t)px4_read_value != module_read_value){
						value->uint_data = (uint32_t)px4_read_value;
						SendSetAndSave(byte_entry, 'b', value);
					}
				}
			}
		break;
		}
		default:
		return; //Don't go any further
		break;
	}
}

#ifdef CONFIG_USE_IFCI_CONFIGURATION

void VertiqClientManager::MarkIfciConfigsForRefresh(){
	_init_velocity_max = true;
	_init_volts_max = true;
	_init_mode = true;
	_init_throttle_cvi = true;
	_init_motor_dir = true;
	_init_fc_dir = true;
}

void VertiqClientManager::UpdateIfciConfigParams(){
	_prop_input_parser_client->velocity_max_.get(*_serial_interface->get_iquart_interface());
	_prop_input_parser_client->volts_max_.get(*_serial_interface->get_iquart_interface());
	_prop_input_parser_client->mode_.get(*_serial_interface->get_iquart_interface());
	_prop_input_parser_client->sign_.get(*_serial_interface->get_iquart_interface());
	_prop_input_parser_client->flip_negative_.get(*_serial_interface->get_iquart_interface());
	_ifci_client->throttle_cvi_.get(*_serial_interface->get_iquart_interface());

	//Ensure that these messages get out
	_serial_interface->process_serial_tx();

	CoordinateIquartWithPx4Params(100_ms);
}

void VertiqClientManager::CoordinateIquartWithPx4Params(hrt_abstime timeout){
	//Get the start time and the end time
	hrt_abstime time_now = hrt_absolute_time();
	hrt_abstime end_time = time_now + timeout;

	EntryData entry_values;

	while(time_now < end_time){
		// param_t parameter, bool * init_bool, char descriptor, EntryData value, ClientEntryAbstract * entry
		UpdateParameter(param_find("MAX_VELOCITY"), &_init_velocity_max, 'f', &entry_values, &(_prop_input_parser_client->velocity_max_));
		UpdateParameter(param_find("MAX_VOLTS"), &_init_volts_max, 'f', &entry_values, &(_prop_input_parser_client->volts_max_));
		UpdateParameter(param_find("CONTROL_MODE"), &_init_mode, 'b',  &entry_values, &(_prop_input_parser_client->mode_));
		UpdateParameter(param_find("VERTIQ_MOTOR_DIR"), &_init_motor_dir, 'b',  &entry_values, &(_prop_input_parser_client->sign_));
		UpdateParameter(param_find("VERTIQ_FC_DIR"), &_init_fc_dir, 'b',  &entry_values, &(_prop_input_parser_client->flip_negative_));
		UpdateParameter(param_find("THROTTLE_CVI"), &_init_throttle_cvi, 'b',  &entry_values, &(_ifci_client->throttle_cvi_));

		//Update
		time_now = hrt_absolute_time();
		//Update our serial rx
		_serial_interface->process_serial_rx(&_motor_interface, _client_array);
	}
}

#endif //CONFIG_USE_IFCI_CONFIGURATION
