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
	2. Reset() 		- 	NOTE: function does not care about current state and just writes default values to all servos
						without accounting for stability - i.e. the robot will most probably fall down !!!
						Supposed to be used in extreme cases only

-------------------------------------------------------------------------------------------

*/


#ifndef ROBOT_H
#define ROBOT_H

#include "wkq.h"
#include "Tripod.h"
#include "include.h"
#include "Master.h"

using wkq::RobotState_t;

#define TRIPOD_COUNT 	2
#define TRIPOD_LEFT 	0	
#define TRIPOD_RIGHT 	1


class Robot{

public:
	Robot(Master* pixhawk_in, DNXServo* HipsKnees, DNXServo* ArmsWings, double height_in, 
		const double robot_params[], wkq::RobotState_t state_in = wkq::RS_default); 
	~Robot();

	/* ------------------------------------ STANDING POSITIONS ----------------------------------- */

	void Default();					// Reset all Leg parameters to their default values
	void Center();					// Reset all Legs to their central positions and keep current height
	void Stand();					// Set all Legs to a standing state where height = Tibia
	void StandQuad();				// Same as Stand() but arms configured as quad
	void FlattenLegs(wkq::RobotState_t state_in = wkq::RS_standing_flat_quad);	// Flatten the knee

	/* ------------------------------------ WALK RELATED FUNCTIONALITY ----------------------------------- */
	
	void WalkForward(double coeff);
	void Rotate(double angle);

	void RaiseBody(double hraise);

	/* ------------------------------------ TESTING FUNCTIONS ----------------------------------- */

	void Test();
	void SingleStepForwardTest(double coeff);
	void QuadSetup();			// set the legs so that Pixhawk calibration can be performed 

private:
	double CalcMaxStepSize();
	double CalcMaxRotationAngle();

	bool no_state();		// check if the current state is meaningless for the walking configuration


	/* ------------------------------------ MEMBER DATA ----------------------------------- */

	Tripod Tripods[TRIPOD_COUNT];
	
	Master* pixhawk;
	// Both MAXes always positive; Negative limit same number
	double max_rotation_angle;
	double max_step_size;

	const double wait_time = 0.5; 	// wait time between writing angles for the two tripods

	wkq::RobotState_t state;
};

#endif //ROBOT_H
