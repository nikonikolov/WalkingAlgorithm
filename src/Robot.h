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

FIXES TO BE DONE:
	1. Configuring as Quad/Hex - need to coordinate with robot and clarify the sequence order
	5. Introduce a way to vary angle and step size in functions - use a coeff<1 as input and multiply it to param

	6. Body for CalcMaxStepSize
	7. Body for CalcMaxRotationAngle
	8. Add function for modifying state out of the class and automatically making the movement
	9. Have a look at movement functions
	10. InputForward() - improvize some momentary solution

*/


#ifndef ROBOT_H
#define ROBOT_H

#include "wkq.h"
#include "Tripod.h"
#include "include.h"

using wkq::RobotState_t;

#define TRIPOD_COUNT 	2
#define TRIPOD_LEFT 	0	
#define TRIPOD_RIGHT 	1


class Robot{

public:
	Robot(DNXServo* HipsKnees, DNXServo* ArmsWings, const double& height_in, wkq::RobotState_t state_in = wkq::RS_default); 
	~Robot();

	/* ------------------------------------ STANDING POSITIONS ----------------------------------- */

	void Default();					// Reset all Leg parameters to their default values
	void Center();					// Reset all Legs to their central positions and keep current height
	void Stand();					// Set all Legs to a standing state where height = Tibia
	void StandQuad();				// Same as Stand() but arms configured as quad
	void FlattenLegs(wkq::RobotState_t state_in = wkq::RS_standing_flat_quad);	// Flatten the knee

	/* ------------------------------------ WALK RELATED FUNCTIONALITY ----------------------------------- */
	
	void WalkForward(const double& coeff);
	void Rotate(const double& angle);

	void RaiseBody(const double& hraise);


private:
	double CalcMaxStepSize();
	double CalcMaxRotationAngle();


	/* ------------------------------------ MEMBER DATA ----------------------------------- */

	Tripod Tripods[TRIPOD_COUNT];
	
	// Both MAXes always positive; Negative limit same number
	double max_rotation_angle;
	double max_step_size;

	wkq::RobotState_t state;
};

#endif //ROBOT_H
