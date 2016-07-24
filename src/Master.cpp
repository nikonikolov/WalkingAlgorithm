#include "Master.h"

#ifndef SIMULATION
Master::Master(PinName tx, PinName rx, int baud_in) :
	port(new mbed::Serial(tx, rx)), baud(baud_in), bitPeriod(1000000.0/baud_in){}
#endif

bool Master::InputWalkForward(){
	static int out=0;
	if(out<2){
		out++;
		return true;
	} 
	else return false;
}


