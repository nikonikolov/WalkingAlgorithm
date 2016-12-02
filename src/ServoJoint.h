#ifndef SERVOJOINT_H
#define SERVOJOINT_H

#include <string>
#include <unordered_map>
using std::string;
using std::unordered_map;

#include "wkq.h"

#ifndef SIMULATION
#include "../libdnx/DnxHAL.h"
#else
typedef int DnxHAL;
void wait(int time);
void wait_ms(int time);
#endif

class ServoJoint{

public:

	ServoJoint(int ID_in, unordered_map<int, DnxHAL*>& servo_map);

	int getID() const;

    int setID(int newID);
	int getValue(int address);
    int setBaud(int rate);
    int setReturnLevel(int lvl);
    int setLED(int colour); 
	int setGoalPosition(int angle, bool cash=false);
	int setGoalPosition(double angle, bool cash=false);
	int setGoalVelocity(int velocity);
	int setGoalTorque(int torque);
	int setPunch(int punch);
    int action(int arg);

	void operator=(const ServoJoint& obj_in);

private:

    int ID;
    string servo_name;
    bool debug_ = true;

#ifndef SIMULATION
	DnxHAL* dnx_ptr;		     // Pointer to the object associated with the right serial port
#else
    //double angle = 0;
    //GeomView* robot_view;
#endif

};

#endif
