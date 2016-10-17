#include <cstdlib>

#include "src/wkq.h"

#include "src/ServoJoint.h"
#include "src/Leg.h"
#include "src/Tripod.h"
#include "src/Robot.h"

#include "src/DnxSerialBase.h"
#include "src/AX12A_Serial.h"
#include "src/XL320_Serial.h"
#include "src/include.h"

#include "src/Master.h"

/*  
 * main() that runs on the mbed. To compile run $ make
 *
 *
 *	Global vars defined in other header files:
 *	PC pc - specifies connected pc for debugging purposes - initialized in include.h and include.cpp
 */

int main(){

	// Set the debug level so that debug messages are printed on the USB serial connection
	pc.set_debug();

	// PARAM_STEP defined in State_t.h. Meaning of each value defined in the same header
	const double robot_params[PARAM_STEP] = { 10.95, 2.65, 17.5, 30.0, 12.0, 2.25};
	
	pc.print_debug("MAIN started\n");

	int baud = 1000000; 		// No resistor between Rx and Tx
//	int baud = 115200; 			// resistor between Rx and Tx - can possibly work without one
	double init_height = 10.0;	// initial height for the robot

	Master* pixhawk = new Master();

	pc.print_debug("Pixhawk initialized\n");

	// Instantiate objects for communication with the servo motors 
	AX12A_Serial HipsKnees(p9, p10, baud);
	XL320_Serial Arms(p13, p14, baud);

	pc.print_debug("Communication ready\n");
	

	// Instantiate Robot
	Robot* WkQuad;
	try{
		WkQuad = new Robot(pixhawk, &HipsKnees, &Arms, init_height, robot_params, wkq::RS_STRAIGHT_QUAD);		// initialize to a ready for flight position
	//	WkQuad = new Robot(pixhawk, &HipsKnees, &Arms, init_height, robot_params, wkq::RS_STANDING);				// initialize to a standing position
	//	WkQuad = new Robot(pixhawk, &HipsKnees, &Arms, init_height, robot_params, wkq::RS_STANDING_QUAD);			
	//	WkQuad = new Robot(pixhawk, &HipsKnees, &Arms, init_height, robot_params, wkq::RS_DEFAULT);

	}
	catch(const string& msg){
		pc.print_debug(msg);
		exit(EXIT_FAILURE);
	}

	pc.print_debug("Robot Initialized\n");

	// Make the robot do something. Examples shown below. Check Robot.h for possible functions
	//WkQuad->Stand();							// make the robot stand on its legs
	//pc.print_debug("Robot Standing\n");		// make the robot walk
	//WkQuad->WalkForward(0.5);


	pc.print_debug("End of Program\n");
	return 0;
}



