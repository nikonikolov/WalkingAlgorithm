/* 

MBED USB Serial Port Wrapper: corresponds to the USB port on the mbed
===========================================================================================

FUNCTIONALITY:
	1. Used to provide unified debugging interface to all classes
	2. Able to switch debug mode on and off - note that printing can usually slow down operation a lot
		and should be switched off when not needed
	3. Provides the needed abstraction so simulation mode can run without MBED hardware; in this
		case simply redirects all print calls to STDOUT

-------------------------------------------------------------------------------------------

*/


#ifndef PCSERIAL_H
#define PCSERIAL_H

#include <string>
using std::string;

#ifndef SIMULATION

#include "mbed.h"

class PCSerial{

public:

	PCSerial(const bool& debug_in = false);
	~PCSerial();

	char read();
//	char write();

	inline void print_debug(const std::string& msg);

	void set_debug();
	bool get_debug();

private:

	mbed::Serial pc_usb;
	bool debug;
};


inline void PCSerial::print_debug(const std::string& msg){
	if(!debug) return;
	pc_usb.printf(msg.c_str());
}

#else

#include <iostream>

using std::cout;
using std::endl;

class PCSerial{

public:

	PCSerial(bool debug_in){}
	~PCSerial(){}

	inline void print_debug(const std::string& msg){
		cout<<msg<<endl;
	}

private:

};

#endif	//SIMULATION

#endif	//PC_SERIAL_H
