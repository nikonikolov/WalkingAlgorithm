
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
	int baud, baud_xl320;
	double init_height;
	DnxHAL* front_legs;
	DnxHAL* back_legs;
	Master* pixhawk;
	BodyParams robot_params;
	Robot* wk_quad;
	unordered_map<int, DnxHAL*> servo_map;

#ifdef DOF3
	robot_params.DIST_CENTER 		= 10.95;
	robot_params.COXA 				= 2.65;
	robot_params.FEMUR 				= 17.1;
	robot_params.TIBIA 				= 30;
	robot_params.KNEE_TO_MOTOR_DIST = 2.6;
	robot_params.MIN_HEIGHT = robot_params.TIBIA - robot_params.FEMUR*sin(wkq::radians(70));
	robot_params.MAX_HEIGHT = robot_params.FEMUR*sin(wkq::radians(70)) + robot_params.TIBIA;
	robot_params.compute_squares();

	baud 			= 10000000;
	init_height 	= 15.0;	
	pixhawk 		= new Master();

	front_legs 	= new SerialAX12(DnxHAL::Port_t(p9, p10), baud);
	back_legs 	= new SerialXL320(DnxHAL::Port_t(p13, p14), baud);

#else 
	robot_params.DIST_CENTER = 10.95 + 2.15;
	robot_params.FEMUR = 17.1;
	robot_params.TIBIA = 12 + 1.85;
	robot_params.KNEE_TO_MOTOR_DIST = 2.6;
	robot_params.MIN_HEIGHT = robot_params.TIBIA*cos(wkq::radians(60));
	robot_params.MAX_HEIGHT = robot_params.TIBIA;
	robot_params.compute_squares();

	//baud 			= 115200;
	baud 			= 1000000;
	init_height 	= robot_params.TIBIA;	
	pixhawk 		= new Master();

	front_legs 	= new SerialAX12(DnxHAL::Port_t(p9, p10), baud);
	back_legs 	= new SerialAX12(DnxHAL::Port_t(p13, p14), baud);
	
	servo_map[wkq::KNEE_LEFT_FRONT] 	= front_legs;
	servo_map[wkq::KNEE_LEFT_MIDDLE] 	= back_legs;
	servo_map[wkq::KNEE_LEFT_BACK] 		= back_legs;
	servo_map[wkq::KNEE_RIGHT_FRONT] 	= front_legs;
	servo_map[wkq::KNEE_RIGHT_MIDDLE] 	= front_legs;
	servo_map[wkq::KNEE_RIGHT_BACK] 	= back_legs;

	servo_map[wkq::HIP_LEFT_FRONT] 		= front_legs;
	servo_map[wkq::HIP_LEFT_MIDDLE] 	= back_legs;
	servo_map[wkq::HIP_LEFT_BACK] 		= back_legs;
	servo_map[wkq::HIP_RIGHT_FRONT]		= front_legs;
	servo_map[wkq::HIP_RIGHT_MIDDLE] 	= front_legs;
	servo_map[wkq::HIP_RIGHT_BACK] 		= back_legs;

#endif	

	printf("MAIN: All comms ready\n\r");

	wk_quad = new Robot(pixhawk, servo_map, init_height, robot_params, wkq::RS_DEFAULT);					
	//wk_quad = new Robot(pixhawk, front_legs, back_legs, init_height, robot_params, wkq::RS_FLAT_QUAD);					
	//wk_quad = new Robot(pixhawk, front_legs, back_legs, init_height, robot_params, wkq::RS_RECTANGULAR);					

	printf("MAIN: Robot Initialized\n\r");

	wait(10);
	//wk_quad->makeMovement(wkq::RM_HEXAPOD_GAIT, .4);
	wk_quad->makeMovement(wkq::RM_ROTATION_HEXAPOD, 1);



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



