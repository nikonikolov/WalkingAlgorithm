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

#include "src/MController.h"

/* Global vars:
	PC pc - specifies connected pc for debugging purposes
*/

int main(){

	pc.set_debug();

	// PARAM_STEP defined in State_t.h. Meaning of each value defined in the same header
	const double robot_params[PARAM_STEP] = { 10.95, 2.65, 17.5, 30.0, 12.0, 2.25};
	
	pc.print_debug("MAIN started\n");

	// Remember to set that to 1000000 - both for servos and for mbed
	int baud = 115200;
	double init_height = 10.0;

	MController* pixhawk = new MController();

	pc.print_debug("Pixhawk initialized\n");

	// Instantiate Servo Objects
	AX12A HipsKnees(p9, p10, baud);
	XL320 ArmsWings(p13, p14, baud);

	
	Robot* WkQuad;
	// Instantiate Robot
	try{
		WkQuad = new Robot(pixhawk, &HipsKnees, &ArmsWings, init_height, robot_params, wkq::RS_standing);
	}
	catch(const string& msg){
		pc.print_debug(msg);
		exit(EXIT_FAILURE);
	}
	pc.print_debug("Robot Initialized\n");

	//WkQuad->Stand();

	pc.print_debug("Robot Standing\n");


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
		wait(1);
	}

*/
/*	wait(2);
	ArmsWings.SetGoalPosition(27, 512);
	

	int pr_pos = ArmsWings.GetValue(23, XL_PRESENT_POSITION);

	pc.print_debug("Present pos" + itos(pr_pos) + "\n");

	int pr_baud = ArmsWings.GetValue(23, XL_BAUD_RATE);

	pc.print_debug("Present baud" + itos(pr_baud) + "\n");
*/

	for(int i=23; i<=34; i++){
		ArmsWings.SetGoalPosition(i, 512);
		wait(0.2);
	}

	pc.print_debug("Servos set\n");

	return 0;
}



