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

*/


#ifndef TRIPOD_H
#define TRIPOD_H

#include "Leg.h"

#define FRONTLEG 	0
#define MIDDLELEG 	1
#define BACKLEG 	2
#define LEG_COUNT	3


class Tripod{

public:
	Tripod(int ID_front_knee, int ID_middle_knee, int ID_back_knee,
			DnxSerialBase* HipsKnees, DnxSerialBase* Arms, double height_in, const double robot_params[]);

	~Tripod ();

	/* ------------------------------------ COPYING STATE ----------------------------------- */

	void CopyState(const Tripod& tripod_in);

	/* ------------------------------------ STANDING POSITIONS ----------------------------------- */

	void Default();					// Reset all Leg parameters to their default values
	void Center();					// Reset all Legs to their central positions and keep current height
	void Stand(bool meaningless_state = false);		// Set all Legs to a standing state where height = Tibia
	void StandQuad(bool meaningless_state = false);	// Same as Stand() but arms configured as quad
	void FlattenLegs();				// Flatten the knee

	double Standing();				// Determines if standing, and if not returns by how much should be lifted

	/* ------------------------------------ RAISE AND LOWER ----------------------------------- */

	void LiftUp(double height_up);
	void LowerDown(double height_down);
	void FinishStep();

	/* ------------------------------------ WALKING ALGORITHMS ----------------------------------- */
	
	void BodyForward(double step_size);
	void StepForward(double step_size);

	void BodyRotate(double angle);
	void StepRotate(double angle);
	
	void RaiseBody(double hraise);

	/* ------------------------------------ TESTING FUNCTIONS ----------------------------------- */

	void QuadSetup();

private:
	void WriteAngles();
	void WriteHipKneeAngles();

	Leg Legs[LEG_COUNT];

	static const double leg_lift;
};

#endif 


