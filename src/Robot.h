/* 

Robot Abstraction Class: Corresponds to a the Walking Quadcopter as a robot. Construced from 2 Tripods.
===========================================================================================

FUNCTIONALITY:
	1. Acts as a high level class that indirectly controls the robot and all its movements
	2. Responsible for coordinating the work of the two Tripods
	3. Designed to be used only as a high level object. Computations performed in lower levels
	4. NOT Responsible for providing looping functionality and termination of movements on a particular signal - 
		this has to be done in the main
	5. Responsible for keeping track of the current state and arrangement of the robot

-------------------------------------------------------------------------------------------

FRAMEWORK:
	1. ALWAYS THINK ABOUT THE ORDER THE FUNCTIONS NEED TO BE CALLED. ASK YOURSELF - IS THIS A TRIPOD-ONLY MOVEMENT
		OR IT IS RELATED TO THE OTHER TRIPOD AND TO THE ROBOT ITSELF
	2. Reset() 		- 	NOTE: function does not care about current state and just writes defaultPos values to all servos
						without accounting for stability - i.e. the robot will most probably fall down !!!
						Supposed to be used in extreme cases only

-------------------------------------------------------------------------------------------

*/


#ifndef ROBOT_H
#define ROBOT_H

#include "wkq.h"
#include "Tripod.h"
#include "Master.h"

using wkq::RobotState_t;
using wkq::RobotMovement_t;

#ifndef SIMULATION
#include "../libdnx/SerialAX12.h"
#include "../libdnx/SerialXL320.h"
#endif

#define TRIPOD_COUNT 	2
#define TRIPOD_LEFT 	0	
#define TRIPOD_RIGHT 	1


class Robot{

public:

	Robot(Master* pixhawk_in, unordered_map<int, DnxHAL*>& servo_map, double height_in, const BodyParams& robot_params, wkq::RobotState_t state_in = wkq::RS_DEFAULT); 
	~Robot();

	/* ------------------------------------ SET STATIC POSITIONS ----------------------------------- */

	void setState(wkq::RobotState_t state_in, bool wait_call=false);


	/* ------------------------------------ WALK RELATED FUNCTIONALITY ----------------------------------- */

	void makeMovement(RobotMovement_t movement, double coeff);

	void raiseBody(double hraise);

	/* ------------------------------------ TESTING FUNCTIONS ----------------------------------- */

	void test();
	void testSingleTripodStand();
	void singleStepForwardTest(double coeff);
	void quadSetup();			// set the legs so that Pixhawk calibration can be performed 

private:
	void changeState(wkq::RobotState_t state_in, void (Tripod::*tripod_action)(), bool wait_call=false);

	double calcMaxStepSize();
	double calcMaxRotationAngle();

	bool noState();				// check if the current state is meaningless for the walking configuration


	/* ------------------------------------ MEMBER DATA ----------------------------------- */

	Tripod Tripods[TRIPOD_COUNT];
	
	//DnxHAL* Arms;
	//DnxHAL* dnx_hips_knees;

	Master* pixhawk;

	double max_rotation_angle;
	double max_step_size;

	static const double wait_time_; 	// wait time between writing angles for the two tripods
	static const double ef_raise_;

	wkq::RobotState_t state;

	enum RPC_Fn_t{
		RPC_DEFAULT_POS 	= 1,
		RPC_CENTER 			= 2,
		RPC_STAND 			= 3,
		RPC_STAND_QUAD 		= 4,
		RPC_STRAIGHT_QUAD	= 5
	};

	bool debug_ = false;
};

#endif //ROBOT_H
