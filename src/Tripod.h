/* 

Tripod Abstraction Class: Corresponds to a physical tripod composed of 3 opposite legs
===========================================================================================

FUNCTIONALITY:
	1. Acts as a private class that can be controlled via the Robot Class 
	2. Responsible for coordinating function calls among the three legs 
	3. RESPONSIBLE for invoking the Leg class functionality that write the calculated values to the motors - usually
		performed in each function after all Legs have calculated their new positions
	4. Does not currently store any data - no need for that so far
	5. RESPONSIBLE for determining whether an algorithmic function or CopyState() must be called for more efficient
		operation


-------------------------------------------------------------------------------------------

FRAMEWORK:
	1. ALWAYS THINK ABOUT THE ORDER THE FUNCTIONS NEED TO BE CALLED. ASK YOURSELF - IS THIS A TRIPOD-ONLY MOVEMENT
		OR IT IS RELATED TO THE OTHER TRIPOD AND TO THE ROBOT ITSELF
	2. ALWAYS THINK ABOUT WHAT COMPUTATIONS DOES THE CALLED FUNCTION PERFORM - IF THE RESULTING STATE OF EACH LEG
		IS THE SAME, DO COMPUTATIONS FOR FIRST LEG AND USE CopyState()

-------------------------------------------------------------------------------------------

FIXES TO BE DONE:
	1. Configuring as Quad/Hex - need to coordinate with robot and clarify the sequence order
	2. NOTE: If a leg throws that movement 

-------------------------------------------------------------------------------------------

SUGGESTED IMPROVEMENTS:
	1. Leg collission and other safety checks. Note this cannot be performed in the 
	2. Step size correction - this has to be coordinated with the Leg class as it holds the parameters of the robot
*/


#ifndef TRIPOD_H
#define TRIPOD_H

#include "../Leg/Leg.h"

#define FRONTLEG 	0
#define MIDDLELEG 	1
#define BACKLEG 	2
#define LEG_COUNT	3


class Tripod{

public:
	Tripod(const int& ID_front_knee, const int& ID_middle_knee, const int& ID_back_knee,
		DNXServo* Knees, DNXServo* Hips, DNXServo* ArmsWings, const double& height_in);

	~Tripod ();


	void Reset();									// Reset all Legs to their default states
	void Center();									// Reset all Legs to their central positions but keep current height

	/* ------------------------------------ WALK RELATED FUNCTIONALITY ----------------------------------- */
	
	void BodyForward(const double& distance);
	void BodyRotate(const double& angle);

	void LiftTripodUp(const double& height);
	void PutTripodDownForStepForward(const double& distance);
	void PutTripodStraightDown(const double& height);
	//void PutTripodStraightDown(); - Leg Function needed to be overloaded to make valid
	
	void LiftBodyUp(const double& hraise);

	/* ------------------------------------ FLIGHT RELATED FUNCTIONALITY ----------------------------------- */
	//void ConfigureQuadcopter();
	//void ConfigureHexacopter();


private:
	void WriteAngles();
	void WriteHipKneeAngles();

	Leg Legs[LEG_COUNT];
};

#endif //TRIPOD_H
