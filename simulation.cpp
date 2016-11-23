#include <cstdlib>

#include "src/wkq.h"

#include "src/ServoJoint.h"
#include "src/Leg.h"
#include "src/Tripod.h"
#include "src/Robot.h"

#include "src/Master.h"

/*
 * main() for simulating the robot behaviour and debugging the algorithm without using the mbed
 * 
 * To compile run $ make bin/sim
 * 
 */


int main(){

	printf("MAIN STARTED\n");

	BodyParams robot_params;

#ifndef DOF3
	robot_params.DIST_CENTER = 10.95 + 2.15;
	robot_params.FEMUR = 17.1;
	robot_params.TIBIA = 15;
	robot_params.KNEE_TO_MOTOR_DIST = 2.6;
	robot_params.MIN_HEIGHT = robot_params.TIBIA*cos(wkq::radians(60));
	robot_params.MAX_HEIGHT = robot_params.TIBIA;
	robot_params.compute_squares();
#else 
	robot_params.DIST_CENTER = 10.95;
	robot_params.COXA = 2.65;
	robot_params.FEMUR = 17.1;
	robot_params.TIBIA = 30;
	robot_params.KNEE_TO_MOTOR_DIST = 2.6;

	// VERIFY THIS
	robot_params.MIN_HEIGHT = robot_params.TIBIA - robot_params.FEMUR*sin(wkq::radians(70));
	robot_params.MAX_HEIGHT = robot_params.FEMUR*sin(wkq::radians(70)) + robot_params.TIBIA;
	robot_params.compute_squares();
#endif	


	int baud = 1000000;
	double init_height = 25.0;

	Master* pixhawk = new Master();

	printf("PIXHAWK INITIALIZED\n");
	
	Robot* wk_quad;
	// Instantiate Robot
	try{
		//wk_quad = new Robot(pixhawk, NULL, NULL, init_height, robot_params, wkq::RS_FLAT_QUAD);
		wk_quad = new Robot(pixhawk, NULL, NULL, init_height, robot_params, wkq::RS_DEFAULT);
		wk_quad = new Robot(pixhawk, NULL, NULL, init_height, robot_params, wkq::RS_FLAT_QUAD);
	}
	catch(const string& msg){
		printf("%s\n\r", msg.c_str());
		exit(EXIT_FAILURE);
	}

	printf("ROBOT INITIALIZED\n");

	/*wk_quad->stand();
	printf("Robot Standing\n");
	wk_quad->standQuad();
	printf("Robot Standing as Quad\n");
	wk_quad->defaultPos();
	printf("Legs in Default position\n");
	wk_quad->center();
	printf("Legs Centered\n");
	*/
	
	//wk_quad->walkForward(0.7);
	//printf("ROBOT WALKED\n");

	// Further tests - lifting body after center to stand
	// Centering for arbitrary height


	return 0;
}



