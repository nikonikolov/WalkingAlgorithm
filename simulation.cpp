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

	printf("MAIN started\n\r");
	int baud, baud_xl320;
	double init_height;
	DnxHAL* dnx_hips_knees;
	DnxHAL* dnx_arms;
	Master* pixhawk;
	BodyParams robot_params;
	Robot* wk_quad;

#ifdef DOF3
	robot_params.DIST_CENTER 		= 10.95;
	robot_params.COXA 				= 2.65;
	robot_params.FEMUR 				= 17.1;
	robot_params.TIBIA 				= 30;
	robot_params.KNEE_TO_MOTOR_DIST = 2.6;
	robot_params.MIN_HEIGHT = robot_params.TIBIA - robot_params.FEMUR*sin(wkq::radians(70));
	robot_params.MAX_HEIGHT = robot_params.FEMUR*sin(wkq::radians(70)) + robot_params.TIBIA;
	robot_params.compute_squares();

	init_height 	= 15.0;	
	pixhawk 		= new Master();

	dnx_hips_knees 	= NULL;
	dnx_arms 		= NULL;

#else 
	robot_params.DIST_CENTER = 10.95 + 2.15;
	robot_params.FEMUR = 17.1;
	robot_params.TIBIA = 12 + 1.85;
	robot_params.KNEE_TO_MOTOR_DIST = 2.6;
	robot_params.MIN_HEIGHT = robot_params.TIBIA*cos(wkq::radians(60));
	robot_params.MAX_HEIGHT = robot_params.TIBIA;
	robot_params.compute_squares();

	init_height 	= robot_params.TIBIA;	
	pixhawk 		= new Master();

	dnx_hips_knees 	= NULL;
	dnx_arms 		= NULL;
#endif	

	printf("MAIN: All comms ready\n\r");
	
	wk_quad = new Robot(pixhawk, dnx_hips_knees, dnx_arms, init_height, robot_params, wkq::RS_DEFAULT);					
	//wk_quad = new Robot(pixhawk, dnx_hips_knees, dnx_arms, init_height, robot_params, wkq::RS_FLAT_QUAD);					
	//wk_quad = new Robot(pixhawk, dnx_hips_knees, dnx_arms, init_height, robot_params, wkq::RS_RECTANGULAR);					

	printf("MAIN: Robot Initialized\n\r");

	wk_quad->makeMovement(wkq::RM_HEXAPOD_GAIT, 1.0);
	//wk_quad->makeMovement(wkq::RM_ROTATION_HEXAPOD, 1.0);


	return 0;
}



