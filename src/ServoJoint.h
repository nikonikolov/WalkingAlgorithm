#ifndef SERVOJOINT_H
#define SERVOJOINT_H

#include <string>

#include "wkq.h"
using std::string;


#ifndef SIMULATION

#include "../libdnx/DnxHAL.h"

class ServoJoint{

public:

	ServoJoint(int ID_in, DnxHAL* serial_ptr_in);

	int getID() const;

    int setID(int newID);
	int getValue(int address);
    int setBaud(int rate);
    int setReturnLevel(int lvl);
    int setLED(int colour); 
	int setGoalPosition(int angle);
	int setGoalPosition(double angle);
	int setGoalVelocity(int velocity);
	int setGoalTorque(int torque);
	int setPunch(int punch);

	void operator=(const ServoJoint& obj_in);

private:

	int ID;
    string servo_name;
	DnxHAL* dnx_ptr;		// Pointer to the object (Knees/Hips/Arms/Wings) associated with the right serial port
};

#else

typedef int DnxHAL;
void wait(int time);

class ServoJoint{

public:

	// remember typedef GeomView DnxHAL - defined so that same interface is kept between simulation and reality
	ServoJoint(int ID_in, DnxHAL* robot_view_in);

	int getID() const;

    int setID(int newID);
	int getValue(int address);
    int setBaud(int rate);
    int setReturnLevel(int lvl);
    int setLED(int colour); 
	int setGoalPosition(int angle_in);
	int setGoalPosition(double angle_in);
	int setGoalVelocity(int velocity);
	int setGoalTorque(int torque);
	int setPunch(int punch);

	void operator=(const ServoJoint& obj_in);

private:

	int ID;
	string servo_name;
	double angle = 0;
	//GeomView* robot_view;
};

#endif 	// SIMUALTION


#endif // SERVOJOINT_H
