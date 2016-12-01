#include "Master.h"

#ifndef SIMULATION
Master::Master(PinName tx, PinName rx, int baud_in) :
	port(new mbed::Serial(tx, rx)), baud(baud_in), bit_period_(1000000.0/baud_in){}
#endif

bool Master::inputWalkForward(){
	static int out=0;
	if(out<7){
		out++;
		return true;
	} 
	else return false;
}


