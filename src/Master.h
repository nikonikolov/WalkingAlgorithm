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

#ifndef SIMULATION
#include "mbed.h"
#endif

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

    int call=0;
    int steps1=20;
    int steps2=20;
    int steps3=20;
};


#endif


