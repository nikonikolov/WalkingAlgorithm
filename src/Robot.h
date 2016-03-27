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
	2. Initializing and keeping State
	3. Calculating max step size - needed so that you can pass proper parameters to functions
	4. Compute somehow max rotation angle
	5. Introduce a way to vary angle and step size in functions - use a coeff<1 as input and multiply it to param


*/


#ifndef ROBOT_H
#define ROBOT_H

#include "Tripod.h"

#define TRIPOD_COUNT 	2
#define TRIPOD_LEFT 	0	
#define TRIPOD_RIGHT 	1


class Robot{

public:
	Robot(DNXServo* Knees, DNXServo* Hips, DNXServo* ArmsWings, const double& height_in); 
	~Robot ();

	void Reset();			

	/* ------------------------------------ WALK RELATED FUNCTIONALITY ----------------------------------- */
	//void WalkForward(const double& coeff);
	//void Rotate(const double& angle);
	void LiftBodyUp(const double& hraise);

	/* ------------------------------------ FLIGHT RELATED FUNCTIONALITY ----------------------------------- */

	//void ConfigureQuadcopter();
	//void ConfigureHexacopter();

	//void Stand();
	//void Default();


private:
	Tripod Tripods[TRIPOD_COUNT];
	//double Height;
	//double MaxRotationAngle;
	
	double MaxStepSize;
	wkquad::robot_state_t state;
};

#endif //ROBOT_H
