/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
#include "vertiq_serial_interface.hpp"

VertiqSerialInterface::VertiqSerialInterface()
{}

void VertiqSerialInterface::DeinitSerial()
{
	if (_uart_fd >= 0) {
#ifndef __PX4_QURT
		close(_uart_fd);
#endif
		_uart_fd = -1;
	}
}

int VertiqSerialInterface::InitSerial(const char *uart_device, unsigned baud)
{
	//Make sure we're starting clean
	DeinitSerial();
#ifdef __PX4_QURT
	_uart_fd = qurt_uart_open(uart_device, baud);
#else
	//Open up the port with read/write permissions and O_NOCTTY which is, "/* Required by POSIX */"
	_uart_fd = ::open(uart_device, O_RDWR | O_NOCTTY);
#endif
	if (_uart_fd < 0) {
		PX4_ERR("failed to open serial port: %s err: %d", uart_device, errno);
		return -errno;
	}

	PX4_INFO("Opened serial port successfully");

	//Now that we've opened the port, fully configure it for baud/bit num
	return ConfigureSerialPeripheral(baud);
}

int VertiqSerialInterface::ConfigureSerialPeripheral(unsigned baud)
{
#ifndef __PX4_QURT
	int speed;

	switch (baud) {
	case 9600:
		speed = B9600;
		break;

	case 19200:
		speed = B19200;
		break;

	case 38400:
		speed = B38400;
		break;

	case 57600:
		speed = B57600;
		break;

	case 115200:
		speed = B115200;
		break;

	case 230400:
		speed = B230400;
		break;

	case 460800:
		speed = B460800;
		break;

	case 500000:
		speed = B500000;
		break;

	case 921600:
		speed = B921600;
		break;

	case 1000000:
		speed = B1000000;
		break;

	default:
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_uart_fd, &uart_config);

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				 INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	uart_config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		return -errno;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		return -errno;
	}

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		return -errno;
	}
#endif
	_speed = baud;
	return 0;
}

bool VertiqSerialInterface::CheckForRx()
{
	if (_uart_fd < 0) {
		return -1;
	}
#ifndef __PX4_QURT
	// read from the uart. This must be non-blocking, so check first if there is data available
	_bytes_available = 0;
	int ret = ioctl(_uart_fd, FIONREAD, (unsigned long)&_bytes_available);

	if (ret != 0) {
		PX4_ERR("Reading error");
		return -1;
	}
#endif
#ifdef __PX4_QURT
#define ASYNC_UART_READ_WAIT_US 2000
	// The UART read on SLPI is via an asynchronous service so specify a timeout
	// for the return. The driver will poll periodically until the read comes in
	// so this may block for a while. However, it will timeout if no read comes in.
	_bytes_available = qurt_uart_read(_uart_fd, (char *) _rx_buf, sizeof(_rx_buf), ASYNC_UART_READ_WAIT_US);
#endif
	return _bytes_available > 0;
}

uint8_t *VertiqSerialInterface::ReadAndSetRxBytes()
{
	//Read the bytes available for us
#ifndef __PX4_QURT
	read(_uart_fd, _rx_buf, _bytes_available);
#endif
	//Put the data into our IQUART handler
	_iquart_interface.SetRxBytes(_rx_buf, _bytes_available);
	return _rx_buf;
}

void VertiqSerialInterface::ProcessSerialRx(ClientAbstract **client_array, uint8_t number_of_clients)
{
	//We have bytes
	if (CheckForRx()) {
		uint8_t *data_ptr = ReadAndSetRxBytes();

		//While we've got packets to look at, give the packet to each of the clients so that each
		//can decide what to do with it
		while (_iquart_interface.PeekPacket(&data_ptr, &_bytes_available) == 1) {

			for (uint8_t i = 0; i < number_of_clients; i++) {
				client_array[i]->ReadMsg(data_ptr, _bytes_available);
			}

			_iquart_interface.DropPacket();
		}

	}
}

void VertiqSerialInterface::ProcessSerialTx()
{
	//while there's stuff to write, write it
	//Clients are responsible for adding TX messages to the buffer through get/set/save commands
	while (_iquart_interface.GetTxBytes(_tx_buf, _bytes_available)) {
#ifdef __PX4_QURT
		qurt_uart_write(_uart_fd, (const char *) _tx_buf, _bytes_available);
#else
		write(_uart_fd, _tx_buf, _bytes_available);
#endif
	}
}

GenericInterface *VertiqSerialInterface::GetIquartInterface()
{
	return &_iquart_interface;
}
