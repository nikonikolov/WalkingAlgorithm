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
	
	pc.print_debug("MAIN STARTED\n");

	int baud = 1000000;
	double init_height = 10.0;

	Master* pixhawk = new Master();

	pc.print_debug("PIXHAWK INITIALIZED\n");
	
	Robot* wk_quad;
	// Instantiate Robot
	try{
		wk_quad = new Robot(pixhawk, NULL, NULL, init_height, robot_params, wkq::RS_FLAT_QUAD);
	}
	catch(const string& msg){
		pc.print_debug(msg);
		exit(EXIT_FAILURE);
	}

	pc.print_debug("ROBOT INITIALIZED\n");

	/*wk_quad->stand();
	pc.print_debug("Robot Standing\n");
	wk_quad->standQuad();
	pc.print_debug("Robot Standing as Quad\n");
	wk_quad->defaultPos();
	pc.print_debug("Legs in Default position\n");
	wk_quad->center();
	pc.print_debug("Legs Centered\n");
	*/
	
	//wk_quad->walkForward(0.7);
	//pc.print_debug("ROBOT WALKED\n");

	// Further tests - lifting body after center to stand
	// Centering for arbitrary height


	return 0;
}



