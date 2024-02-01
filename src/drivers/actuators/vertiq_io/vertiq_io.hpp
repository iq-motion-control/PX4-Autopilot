#pragma once

#include <drivers/device/device.h>
#include <lib/led/led.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>

#include <px4_log.h>
#include <px4_platform_common/module.h>

#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/actuator_test.h>

#include "vertiq_telemetry_manager.hpp"
#include "vertiq_client_manager.hpp"
#include "vertiq_serial_interface.hpp"
#include "ifci.hpp"

#include "iq-module-communication-cpp/inc/propeller_motor_control_client.hpp"
#include "iq-module-communication-cpp/inc/brushless_drive_client.hpp"
#include "iq-module-communication-cpp/inc/arming_handler_client.hpp"

class VertiqIo : public ModuleBase<VertiqIo>, public OutputModuleInterface
{

public:

	/**
	* @brief Create a new VertiqIo object
	*/
	VertiqIo();

	/**
	* @brief destruct a VertiqIo object
	*/
	~VertiqIo() override;

	/**
	* @brief initialize the VertiqIo object. This will be called by the task_spawn function. Makes sure that the thread gets scheduled.
	*/
	bool init();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

	/** @see ModuleBase::run() */ //I do not think this actually comes from ModuleBase. it should come from scheduled work item
	void Run() override;

	/** @see OutputModuleInterface */
	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:
	static const uint8_t MAX_SUPPORTABLE_IFCI_CVS = 16;

	enum DISARM_BEHAVIORS {TRIGGER_MOTOR_DISARM, COAST_MOTOR, SEND_PREDEFINED_THROTTLE};
	enum ARM_BEHAVIORS {USE_MOTOR_ARMING, FORCE_ARMING};

	//Variables and functions necessary for properly configuring the serial interface
	//Determines whether or not we should initialize or re-initialize the serial connection
	static px4::atomic_bool _request_telemetry_init;

	MixingOutput _mixing_output{"VERTIQ_IO", MAX_SUPPORTABLE_IFCI_CVS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	//The name of the device we're connecting to. this will be something like /dev/ttyS3
	static char _telemetry_device[20];

	//Counters/timers to track our status
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": output update interval")};

	//We need a serial handler in order to talk over the serial port
	VertiqSerialInterface _serial_interface;

	//We need someone who can manage our clients
	VertiqClientManager _client_manager;

	//We need a telemetry handler
	VertiqTelemetryManager _telem_manager;

	IFCI * _motor_interface_ptr;

	//Store the number of control variables that we're using
	uint8_t _cvs_in_use = 0;

	//Store the telemetry bitmask for who we want to get telemetry from
	uint16_t _telem_bitmask = 0;

	//This is the variable we're actually going to use in the brodcast packed control message
	//We set and use it to _current_telemetry_target_module_id until we send the first
	//broadcast message with this as the tail byte. after that first transmission, set it
	//to an impossible module ID
	uint16_t _telemetry_request_id = 0;

	static const uint8_t _impossible_module_id = 255;

	bool _send_forced_arm = true;

	//We want to publish our ESC Status to anyone who will listen
	uORB::Publication<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};

	/**
	 * Check for parameter changes and update them if needed.
	 */
	void parameters_update();

	int i = 0;

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	//We need to bring in the parameters that we define in module.yaml in order to view them in the
	//control station, as well as to use them in the firmware
	DEFINE_PARAMETERS(
	(ParamInt<px4::params::VERTIQ_ENABLE>) _param_vertiq_enable,
	(ParamInt<px4::params::VERTIQ_BAUD>) _param_vertiq_baud,
	(ParamInt<px4::params::VERTIQ_NUM_CVS>) _param_vertiq_number_of_cvs,
	(ParamInt<px4::params::VERTIQ_TEL_MSK>) _param_vertiq_telemetry_mask,
	(ParamInt<px4::params::DISARM_THROTTLE>) _param_vertiq_disarm_throttle,
	(ParamInt<px4::params::DISARM_BEHAVE>) _param_vertiq_disarm_behavior,
	(ParamInt<px4::params::ARMING_BEHAVE>) _param_vertiq_arm_behavior,
	(ParamInt<px4::params::MODULE_ID>) _param_vertiq_module_id
	#ifdef CONFIG_USE_IFCI_CONFIGURATION
	,(ParamBool<px4::params::TRIGGER_READ>) _param_vertiq_trigger_read
	,(ParamInt<px4::params::THROTTLE_CVI>) _param_vertiq_throttle_cvi
	,(ParamInt<px4::params::CONTROL_MODE>) _param_vertiq_control_mode
	,(ParamFloat<px4::params::MAX_VELOCITY>) _param_vertiq_max_velo
	,(ParamFloat<px4::params::MAX_VOLTS>) _param_vertiq_max_volts
	,(ParamInt<px4::params::VERTIQ_MOTOR_DIR>) _param_vertiq_motor_direction
	,(ParamInt<px4::params::VERTIQ_FC_DIR>) _param_vertiq_fc_direction
	#endif
	// #ifdef CONFIG_USE_SYSTEM_CONTROL_CLIENT
	// ,(ParamInt<px4::params::OBJECT_ID>) _param_vertiq_sys_ctrl_id
	// ,(ParamInt<px4::params::DEVICE_ID>) _param_vertiq_dev_id
	// ,(ParamInt<px4::params::REV_ID>) _param_vertiq_rev_id
	// ,(ParamInt<px4::params::UID1>) _param_vertiq_uid1
	// ,(ParamInt<px4::params::UID2>) _param_vertiq_uid2
	// ,(ParamInt<px4::params::UID3>) _param_vertiq_uid3
	// ,(ParamInt<px4::params::MEM_SIZE>) _param_vertiq_mem_size
	// ,(ParamInt<px4::params::BUILD_YEAR>) _param_vertiq_build_year
	// ,(ParamInt<px4::params::BUILD_MONTH>) _param_vertiq_build_month
	// ,(ParamInt<px4::params::BUILD_DAY>) _param_vertiq_build_day
	// ,(ParamInt<px4::params::BUILD_HOUR>) _param_vertiq_build_hour
	// ,(ParamInt<px4::params::BUILD_MIN>) _param_vertiq_build_min
	// ,(ParamInt<px4::params::BUILD_SEC>) _param_vertiq_build_sec
	// ,(ParamInt<px4::params::MODULE_ID>) _param_vertiq_module_id_module_id
	// ,(ParamFloat<px4::params::MODULE_TIME>) _param_vertiq_module_time
	// ,(ParamInt<px4::params::FIRMWARE_VERSION>) _param_vertiq_firmware_version
	// ,(ParamInt<px4::params::HARDWARE_VERSION>) _param_vertiq_hardware_version
	// ,(ParamInt<px4::params::ELEC_VERSION>) _param_vertiq_electronics_version
	// ,(ParamInt<px4::params::FIRMWARE_VALID>) _param_vertiq_firmware_valid
	// ,(ParamInt<px4::params::APPS_PRESENT>) _param_vertiq_apps_present
	// ,(ParamInt<px4::params::BOOT_VERSION>) _param_vertiq_bootloader_version
	// ,(ParamInt<px4::params::UPGRADE_VERSION>) _param_vertiq_upgrade_version
	// ,(ParamInt<px4::params::SYS_CLOCK>) _param_vertiq_sys_clock
	// ,(ParamInt<px4::params::CTRL_FLAGS>) _param_vertiq_ctrl_flags
	// ,(ParamInt<px4::params::PCB_VERSION>) _param_vertiq_pcb_version
	// #endif
	)
};




