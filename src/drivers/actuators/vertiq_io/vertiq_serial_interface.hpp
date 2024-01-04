#pragma once

#include <px4_log.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

class VertiqSerialInterface {
	public:

		int init_serial(const char *uart_device);

		/**
		* set the Baudrate
		* @param baud
		* @return 0 on success, <0 on error
		*/
		int setBaudrate(unsigned baud);

		void deinit_serial();

		int updateSerial();

	private:
		static constexpr int FRAME_SIZE = 10;
		int _uart_fd{-1};

		#if ! defined(__PX4_QURT)
			struct termios		_orig_cfg;
			struct termios		_cfg;
		#endif
		int   _speed = -1;

};


