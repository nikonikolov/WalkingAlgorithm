#ifndef TRIPOD_H
#define TRIPOD_H

#include <XL320.h>
#include <AX12A.h>
#include <DNXServo.h>
#include <ServoJoint.h>
#include <Leg.h>

class Tripod{

public:
	Tripod (Leg& front_in, Leg& back_in, Leg& middle_in);
	void BodyForward(const double& distance);
	void BodyRotate(const double& angle);
	void WriteAngles();
	void Raise(const double& height);
	void Down(const double& height);

	void FlyQuadcopter();
	void FlyHexacopter();


private:
	Leg *Legs[LEG_COUNT];

};

#define FRONTLEG 	0
#define MIDDLELEG 	1
#define REARLEG 	2
#define LEG_COUNT	3

#endif //TRIPOD_H
