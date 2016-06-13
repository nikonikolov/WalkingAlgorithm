#include <cstdlib>

#include "src/wkq.h"

#include "src/ServoJoint.h"
#include "src/Leg.h"
#include "src/Tripod.h"
#include "src/Robot.h"

#include "src/DNXServo.h"
#include "src/AX12A.h"
#include "src/XL320.h"
#include "src/include.h"

/* Global vars:
	PC pc - specifies connected pc for debugging purposes
*/

int main(){

	pc.set_debug();
	
	pc.print_debug("MAIN started\n");

	// Remember to set that to 1000000 - both for servos and for mbed
	int baud = 115200;
	double init_height = 10.0;

	//Serial deviceHipsKnees(p9, p10);
	// Instantiate Servo Objects
	AX12A HipsKnees(p9, p10, baud);
	//AX12A HipsKnees(p13, p14, baud);
	XL320 ArmsWings(p13, p14, baud);

	
	Robot* WkQuad;
	// Instantiate Robot
	try{
		WkQuad = new Robot(&HipsKnees, &ArmsWings, init_height, wkq::RS_standing);
	}
	catch(const string& msg){
		pc.print_debug(msg);
		exit(EXIT_FAILURE);
	}
	pc.print_debug("Robot Initialized\n");

	WkQuad->Stand();


// SINGLE SERVO TESTING

/*	int ID = 18;
	pc.print_debug("HipsKnees initialized\n");

	int val = HipsKnees.GetValue(ID, AX_RETURN_LEVEL);
	pc.print_debug("Return level is " + itos(val) + "\n");

	val = HipsKnees.GetValue(ID, AX_BAUD_RATE);
	pc.print_debug("baud rate read is " + itos(val) + "\n");
	HipsKnees.SetGoalPosition(AX_ID_Broadcast, 512);
	pc.print_debug("Goal Position set\n");
*/

/*
	int ID = 27;
	pc.print_debug("ArmsWings initialized\n");

	int val = ArmsWings.GetValue(ID, XL_RETURN_LEVEL);
	pc.print_debug("Return level is " + itos(val) + "\n");

	val = ArmsWings.GetValue(ID, 4);
	pc.print_debug("baud rate is " + itos(val) + "\n");

	ArmsWings.SetGoalPosition(ID, 3.14/4);
	ArmsWings.SetGoalPosition(XL_ID_Broadcast, 512);
	pc.print_debug("Goal Position set\n");

*/
/*
	for(int i=11; i<=22; i++){
		HipsKnees.SetGoalPosition(i, 512);
	}

	for(int i=23; i<=34; i++){
		ArmsWings.SetGoalPosition(i, 512);
	}
*/
	return 0;
}



