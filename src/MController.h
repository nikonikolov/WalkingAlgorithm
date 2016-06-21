#ifndef MCONTROLLER_H
#define MCONTROLLER_H

#include "include.h"

class MController{
public:
	//MController(PinName tx, PinName rx, int baud_in);
	MController(){}

	bool InputWalkForward();

private:
    //mbed::Serial* port;
    int baud;
    double bitPeriod;
};


#endif


