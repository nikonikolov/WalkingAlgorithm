#include <cstdlib>

#include "src/wkq.h"

#include "src/ServoJoint.h"
#include "src/Leg.h"
#include "src/Tripod.h"
#include "src/Robot.h"

#include "src/DNXServo.h"
#include "src/AX12A.h"
#include "src/XL320.h"
#include "src/include.h"

#include "src/Master.h"

#include <ros.h>
#include <std_msgs/RosCmd.h>


/*  
 * main() that runs on the mbed. To compile run $ make
 *
 *
 *	Global vars defined in other header files:
 *	PC pc - specifies connected pc for debugging purposes - initialized in include.h and include.cpp
 */

// PARAM_STEP defined in State_t.h. Meaning of each value defined in the same header
const double robot_params[PARAM_STEP] = { 10.95, 2.65, 17.5, 30.0, 12.0, 2.25};
double init_height = 10.0;															// initial height for the robot

int baud = 1000000; 		// No resistor between Rx and Tx
//	int baud = 115200; 			// resistor between Rx and Tx - can possibly work without one
AX12A hips_knees(p9, p10, baud);
XL320 arms(p13, p14, baud);

Master* pixhawk = new Master();


int main(){

	// Set the debug level so that debug messages are printed on the USB serial connection
	pc.set_debug();

	
	//pc.print_debug("MAIN started\n");

	//pc.print_debug("Pixhawk initialized\n");

	// Instantiate objects for communication with the servo motors 

	//pc.print_debug("Communication ready\n");
	

	// Connect via ROS 

	// Initialize node handle
	ros::NodeHandle pc_control;

	ros::Subscriber<std_msgs::RobotCmd> pc_sub("control_walking", &get_command);

    nh.initNode();
    nh.subscribe(pc_sub);

    while (1) {
        nh.spinOnce();
        wait_ms(1);
    }


	// Instantiate Robot
	Robot* wk_quad;
	try{
		wk_quad = new Robot(pixhawk, &hips_knees, &arms, init_height, robot_params, wkq::RS_STANDING_FLAT_QUAD);		// initialize to a ready for flight position
	//	wk_quad = new Robot(pixhawk, &hips_knees, &arms, init_height, robot_params, wkq::RS_STANDING);				// initialize to a standing position
	//	wk_quad = new Robot(pixhawk, &hips_knees, &arms, init_height, robot_params, wkq::RS_STANDING_QUAD);			
	//	wk_quad = new Robot(pixhawk, &hips_knees, &arms, init_height, robot_params, wkq::RS_DEFAULT);

	}
	catch(const string& msg){
		pc.print_debug(msg);
		exit(EXIT_FAILURE);
	}

	pc.print_debug("Robot Initialized\n");

	// Make the robot do something. Examples shown below. Check Robot.h for possible functions
	//wk_quad->Stand();							// make the robot stand on its legs
	//pc.print_debug("Robot Standing\n");		// make the robot walk
	//wk_quad->WalkForward(0.5);


	pc.print_debug("End of Program\n");
	return 0;
}



void get_command(const std_msgs::RobotCmd& cmd_msg){
	if(cmd_msg.cmd=="init"){
		if(wk_quad != NULL) delete wk_quad;
		wkq::RobotState_t instate;
		if(msg.args.size()==0) instate = RS_DEFAULT;
		else{
			if 		(msg.args[0]=="standing") 			instate = RS_STANDING
			else if (msg.args[0]=="standing_quad") 		instate = RS_STANDING_QUAD
			else if (msg.args[0]=="standing_flat_quad") instate = RS_STANDING_FLAT_QUAD
			else	(msg.args[0]=="default") 			instate = RS_DEFAULT;
		}
		wk_quad = new Robot(pixhawk, &hips_knees, &arms, init_height, robot_params, instate);
	}
	
	if(wk_quad == NULL) wk_quad = new Robot(pixhawk, &hips_knees, &arms, init_height, robot_params, wkq::RS_DEFAULT);

	else if(cmd_msg.cmd=="default") wk_quad->Default()
	else if(cmd_msg.cmd=="center") wk_quad->Center()
	else if(cmd_msg.cmd=="stand") wk_quad->Stand()
	else if(cmd_msg.cmd=="stand_quad") wk_quad->StandQuad()
	else if(cmd_msg.cmd=="flatten_legs") wk_quad->FlattenLegs()

}


