#ifndef SERVOJOINT_H
#define SERVOJOINT_H

#ifndef SIMULATION

#include "DnxSerialBase.h"



class ServoJoint{

public:

	ServoJoint(int ID_in, DnxSerialBase* SerialObjPtrIn);

	int GetID() const;

    int SetID(int newID);
	int GetValue(int address);
    int SetBaud(int rate);
    int SetReturnLevel(int lvl);
    int SetLED(int colour); 
	int SetGoalPosition(int angle);
	int SetGoalPosition(double angle);
	int SetGoalVelocity(int velocity);
	int SetGoalTorque(int torque);
	int SetPunch(int punch);

	void operator=(const ServoJoint& obj_in);

private:

	int ID;
	DnxSerialBase* SerialObjPtr;		// Pointer to the object (Knees/Hips/Arms/Wings) associated with the right serial port
};

#else

#include "include.h"
#include "wkq.h"

class ServoJoint{

public:

	// remember typedef GeomView DnxSerialBase - defined so that same interface is kept between simulation and reality
	ServoJoint(int ID_in, DnxSerialBase* robot_view_in);

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
	//GeomView* robot_view;
};

#endif 	// SIMUALTION


#endif // SERVOJOINT_H
