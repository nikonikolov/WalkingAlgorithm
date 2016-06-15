#ifndef SERVOJOINT_SIM_H
#define SERVOJOINT__SIM_H

#include "DNXServo.h"
#include "../src/wkq.h"

class ServoJoint{

public:

	ServoJoint(int ID_in, DNXServo* SerialObjPtrIn);

	int GetID() const;

    int SetID(int newID);
	int GetValue(int address);
    int SetBaud(int rate);
    int SetReturnLevel(int lvl);
    int SetLED(int colour); 
	int SetGoalPosition(int angle_in);
	int SetGoalPosition(double angle_in);
	int SetGoalVelocity(int velocity);
	int SetGoalTorque(int torque);
	int SetPunch(int punch);

	void operator=(const ServoJoint& obj_in);

private:

	int ID;
	string servo_name;
	double angle = 0;
};


#endif
