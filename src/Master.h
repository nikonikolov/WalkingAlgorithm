#ifndef MASTER_H
#define MASTER_H

#include "include.h"

/*
 * Class used to model the Master microcontroller that issues commands to the mbed
 *
 * Current State: only some definitions and member data available. Needs to be implemented and a corresponding class needs to be implemented in the Pixhawk autopilot
 *
 */

class Master{
public:

#ifndef SIMULATION
	Master(PinName tx, PinName rx, int baud_in);	
#endif
	Master(){}

	bool InputWalkForward();

private:

#ifndef SIMULATION   
    mbed::Serial* port;
#endif
    int baud;
    double bitPeriod;
};


#endif


