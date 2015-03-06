/*
 * print.cpp
 *
 *  Created on: Mar 5, 2015
 *      Author: walmis
 */


#include <xpcc/architecture.hpp>

xpcc::IODeviceWrapper<xpcc::lpc11::Uart1> w;
xpcc::log::Logger xpcc::log::debug(w);

extern "C"
int printf(const char* fmt, ...) {
	va_list ap;
	va_start(ap, fmt);

	XPCC_LOG_DEBUG .vprintf(fmt, ap);

	va_end(ap);

	return 0;
}

extern "C" void dump(uint8_t* buf, uint8_t len) {
	XPCC_LOG_DEBUG .dump_buffer(buf, len);
}
