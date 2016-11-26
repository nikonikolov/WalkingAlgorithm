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

	baud 			= 115200;
	baud_xl320 		= 1000000; 		
	init_height 	= 15.0;	
	pixhawk 		= new Master();

	dnx_hips_knees 	= new SerialAX12(DnxHAL::Port_t(p9, p10), baud);
	dnx_arms 		= new SerialXL320(DnxHAL::Port_t(p13, p14), baud_xl320);

#else 
	robot_params.DIST_CENTER = 10.95 + 2.15;
	robot_params.FEMUR = 17.1;
	robot_params.TIBIA = 12 + 1.85;
	robot_params.KNEE_TO_MOTOR_DIST = 2.6;
	robot_params.MIN_HEIGHT = robot_params.TIBIA*cos(wkq::radians(60));
	robot_params.MAX_HEIGHT = robot_params.TIBIA;
	robot_params.compute_squares();

	baud 			= 115200;
	init_height 	= robot_params.TIBIA;	
	pixhawk 		= new Master();

	dnx_hips_knees 	= new SerialAX12(DnxHAL::Port_t(p9, p10), baud);
	dnx_arms 		= dnx_hips_knees;
#endif	

	printf("MAIN: All comms ready\n\r");

	//	wk_quad = new Robot(pixhawk, baud, init_height, robot_params, wkq::RS_FLAT_QUAD);					
	//	wk_quad = new Robot(pixhawk, baud, init_height, robot_params, wkq::RS_DEFAULT);					
	
	//wk_quad = new Robot(pixhawk, dnx_hips_knees, dnx_arms, init_height, robot_params, wkq::RS_DEFAULT);					
	//wk_quad = new Robot(pixhawk, dnx_hips_knees, dnx_arms, init_height, robot_params, wkq::RS_FLAT_QUAD);					
	wk_quad = new Robot(pixhawk, dnx_hips_knees, dnx_arms, init_height, robot_params, wkq::RS_RECTANGULAR);					

	printf("MAIN: Robot Initialized\n\r");

	wait(10);
	//wk_quad->testSingleTripodStand();
	wk_quad->walkForwardRectangularGait(1.0);


	//wk_quad->testSingleTripodStand();
	//wk_quad->walkForward(0.5);

/*
	while(true){
		int torque = 0;
		torque += dnx_hips_knees.getValue(11, AX_PRESENT_LOAD);
		printf("SINGLE KNEE troque = %d\n\r", torque);
		torque += dnx_hips_knees.getValue(12, AX_PRESENT_LOAD);
		torque += dnx_hips_knees.getValue(13, AX_PRESENT_LOAD);
		torque += dnx_hips_knees.getValue(14, AX_PRESENT_LOAD);
		torque += dnx_hips_knees.getValue(15, AX_PRESENT_LOAD);
		torque += dnx_hips_knees.getValue(16, AX_PRESENT_LOAD);
		//int torque = dnx_hips_knees.getValue(wkq::KNEE_RIGHT_FRONT, AX_PRESENT_LOAD);
		printf("AVERAGE KNEE troque = %d\n\r", torque/6);
		
		torque = 0;
		torque = dnx_hips_knees.getValue(17, AX_PRESENT_LOAD);
		printf("SINLE HIP troque = %d\n\r", torque);
		torque = dnx_hips_knees.getValue(18, AX_PRESENT_LOAD);
		torque = dnx_hips_knees.getValue(19, AX_PRESENT_LOAD);
		torque = dnx_hips_knees.getValue(20, AX_PRESENT_LOAD);
		torque = dnx_hips_knees.getValue(21, AX_PRESENT_LOAD);
		torque = dnx_hips_knees.getValue(22, AX_PRESENT_LOAD);
		printf("AVERAGE HIP troque = %d\n\r", torque/6);
		wait(2);
	}

*/

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



