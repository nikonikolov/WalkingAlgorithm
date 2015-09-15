#ifndef TRIPOD_H
#define TRIPOD_H

#include <XL320.h>
#include <AX12A.h>
#include <DNXServo.h>
#include <ServoJoint.h>
#include <Leg.h>

class Tripod{

public:
	Tripod (Leg& front_in, Leg& back_in, Leg middle_in);
	void BodyForward(int distance);
	void WriteAngles();

private:
	Leg *Legs[LEG_COUNT];

};

#define FRONTLEG 	0
#define MIDDLELEG 	1
#define BACKLEG 	2
#define LEG_COUNT	3

#endif //TRIPOD_H
