#include "PCSerial.h"

#ifndef SIMULATION

PCSerial::PCSerial(bool debug_in /*= false*/) : pc_usb(USBTX, USBRX), debug(debug_in) {}

PCSerial::~PCSerial(){}

void PCSerial::set_debug(){
	debug=true;
	pc_usb.printf("Debug enabled \n");
}

bool PCSerial::get_debug(){
	return debug;
}

char PCSerial::read(){
	if(pc_usb.readable()) return pc_usb.getc();
	return 0;
}

#endif
