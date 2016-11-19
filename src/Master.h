/* 

Class used to model the Master microcontroller that issues commands to the mbed
===========================================================================================

CURRENT STATE: 
	1. Only some definitions and member data available. 
	2. Needs to be implemented and a corresponding class needs to be implemented in the Pixhawk autopilot

-------------------------------------------------------------------------------------------

*/


#ifndef MASTER_H
#define MASTER_H

#include "mbed.h"

class Master{
public:

#ifndef SIMULATION
	Master(PinName tx, PinName rx, int baud_in);	
#endif
	Master(){}

	bool inputWalkForward();

private:

#ifndef SIMULATION   
    mbed::Serial* port;
#endif
    int baud;
    double bit_period_;
};


#endif


