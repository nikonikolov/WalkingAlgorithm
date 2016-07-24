#include <cstdlib>

#include "src/wkq.h"

#include "src/ServoJoint.h"
#include "src/Leg.h"
#include "src/Tripod.h"
#include "src/Robot.h"

#include "src/include.h"

#include "src/Master.h"

/*
 * main() for simulating the robot behaviour and debugging the algorithm without using the mbed
 * 
 * To compile run $ make bin/sim
 * 
 */


int main(){

	// PARAM_STEP defined in State_t.h. Meaning of each value defined in the same header
	const double robot_params[PARAM_STEP] = { 10.95, 2.65, 17.5, 30.0, 12.0, 2.25};
	
	pc.print_debug("MAIN started\n");

	int baud = 1000000;
	double init_height = 10.0;

	Master* pixhawk = new Master();

	pc.print_debug("Pixhawk initialized\n");
	
	Robot* WkQuad;
	// Instantiate Robot
	try{
		WkQuad = new Robot(pixhawk, NULL, NULL, init_height, robot_params, wkq::RS_standing);
	}
	catch(const string& msg){
		pc.print_debug(msg);
		exit(EXIT_FAILURE);
	}

	pc.print_debug("Robot Initialized\n");

	WkQuad->Stand();
	pc.print_debug("Robot Standing\n");
	WkQuad->StandQuad();
	pc.print_debug("Robot Standing as Quad\n");
	WkQuad->FlattenLegs();
	pc.print_debug("Legs Flattened\n");
	WkQuad->Default();
	pc.print_debug("Legs in Default position\n");
	WkQuad->Center();
	pc.print_debug("Legs Centered\n");
	WkQuad->WalkForward(0.7);
	pc.print_debug("Robot walked\n");

	// Further tests - lifting body after center to stand
	// Centering for arbitrary height


	return 0;
}



