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

	pc.print_debug("MAIN STARTED\n");

	BodyParams robot_params;

	robot_params.DIST_CENTER = 10.95;
	robot_params.COXA = 2.65;
	//robot_params.FEMUR = 17.5;
	robot_params.FEMUR = 17.5-0.6;
	robot_params.TIBIA = 30;
	//robot_params.HIPKNEEMAXHDIST = 12.0;
	robot_params.KNEE_TO_MOTOR_DIST = 2.25;
	robot_params.compute_squares();


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



