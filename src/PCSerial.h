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
