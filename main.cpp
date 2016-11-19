#include <cstdlib>

#include "src/wkq.h"

#include "src/ServoJoint.h"
#include "src/Leg.h"
#include "src/Tripod.h"
#include "src/Robot.h"

#include "src/Master.h"

// ROS Functionality
//#include <ros.h>
//#include "wkq_msgs/RPC.h"


int main(int argc, char **argv){

	printf("MAIN started\n\r");

	BodyParams robot_params;
	robot_params.DIST_CENTER = 10.95;
	robot_params.COXA = 2.65;
	//robot_params.FEMUR = 17.5;
	robot_params.FEMUR = 17.5-0.6;
	robot_params.TIBIA = 30;
	robot_params.KNEE_TO_MOTOR_DIST = 2.25;
	robot_params.MIN_HEIGHT = robot_params.TIBIA - robot_params.FEMUR*sin(wkq::radians(30));
	robot_params.MAX_HEIGHT = robot_params.FEMUR*sin(wkq::radians(30)) + robot_params.TIBIA;
	robot_params.compute_squares();
	

	int baud = 1000000; 		
	double init_height = 20.0;	

	Master* pixhawk = new Master();

	printf("Pixhawk initialized\n\r");

	// Instantiate objects for communication with the servo motors 
	//SerialAX12 dnx_hips_knees(DnxHAL::Port_t(p9, p10), baud);
	//SerialXL320 dnx_arms(DnxHAL::Port_t(p13, p14), baud);
	printf("Communication ready\n\r");
	

	// Instantiate Robot
	Robot* wk_quad;
	try{
	//	wk_quad = new Robot(pixhawk, &HipsKnees, &Arms, init_height, robot_params, wkq::RS_STRAIGHT_QUAD);		
		wk_quad = new Robot(pixhawk, baud, init_height, robot_params, wkq::RS_FLAT_QUAD);					
	}
	catch(const string& msg){
		printf("%s\n\r", msg.c_str());
		exit(EXIT_FAILURE);
	}

	printf("Robot Initialized\n\r");

/*
	// INIT ROS
  	ros::init(argc, argv, "walking_server");
  	ros::NodeHandle this_node;      // defaults to USBTX and USBRX

  	ros::ServiceServer service_default  		= this_node.advertiseService("add_two_ints", wk_quad.Default);
  	ros::ServiceServer service_stand_quad 		= this_node.advertiseService("add_two_ints", wk_quad.standQuad);
  	ros::ServiceServer service_stand 			= this_node.advertiseService("add_two_ints", wk_quad.stand);
  	ros::ServiceServer service_straight_quad 	= this_node.advertiseService("add_two_ints", wk_quad.straightQuad);
 
  	ROS_INFO("Ready to add two ints.");
  	ros::spin();

	// Make the robot do something. Examples shown below. Check Robot.h for possible functions
	//wk_quad->Stand();							// make the robot stand on its legs
	//printf("Robot Standing\n\r");		// make the robot walk
	//wk_quad->WalkForward(0.5);

*/
	printf("End of Program\n\r");
	return 0;
}



