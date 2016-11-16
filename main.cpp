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

// ROS Functionality
//#include <ros.h>
//#include "wkq_msgs/RPC.h"


/*  
 *	Global vars defined in other header files:
 *	PC pc - specifies connected pc for debugging purposes - initialized in include.h and include.cpp
 */

int main(int argc, char **argv){

	pc.set_debug();

	const double P_DISTCENTER = 10.95;
	//const double P_COXA = 2.65;
	//const double P_FEMUR = 17.5;
	const double P_COXA = 2.65-1.6;
	const double P_FEMUR = 17.5-0.6;
	const double P_TIBIA = 30;
	const double P_HIPKNEEMAXHDIST = 12.0;
	const double P_KNEEMOTORDIST = 2.25;

	// PARAM_STEP defined in State_t.h. Meaning of each value defined in the same header
	//const double robot_params[PARAM_STEP] = { 10.95, 2.65, 17.5, 30.0, 12.0, 2.25};
	const double robot_params[PARAM_STEP] = { P_DISTCENTER, P_COXA, P_FEMUR, P_TIBIA, P_HIPKNEEMAXHDIST, P_KNEEMOTORDIST};
	
	pc.print_debug("MAIN started\n");

	int baud = 1000000; 		
	double init_height = 10.0;	

	Master* pixhawk = new Master();

	pc.print_debug("Pixhawk initialized\n");

	// Instantiate objects for communication with the servo motors 
	//AX12A_Serial HipsKnees(p9, p10, baud);
	//XL320_Serial Arms(p13, p14, baud);

	pc.print_debug("Communication ready\n");
	

	// Instantiate Robot
	Robot* wk_quad;
	try{
	//	wk_quad = new Robot(pixhawk, &HipsKnees, &Arms, init_height, robot_params, wkq::RS_STRAIGHT_QUAD);		
		wk_quad = new Robot(pixhawk, baud, init_height, robot_params, wkq::RS_FLAT_QUAD);					
	}
	catch(const string& msg){
		pc.print_debug(msg);
		exit(EXIT_FAILURE);
	}

	pc.print_debug("Robot Initialized\n");

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
	//pc.print_debug("Robot Standing\n");		// make the robot walk
	//wk_quad->WalkForward(0.5);

*/
	pc.print_debug("End of Program\n");
	return 0;
}



