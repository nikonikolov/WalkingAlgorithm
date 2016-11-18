/* 

Tripod Abstraction Class: Corresponds to a physical tripod composed of 3 opposite legs
===========================================================================================

FUNCTIONALITY:
	1. Acts as a private class that can be controlled via the Robot Class 
	2. Responsible for coordinating function calls among the three legs 
	3. RESPONSIBLE for invoking the Leg class functionality that write the calculated values to the motors - usually
		performed in each function after all Legs have calculated their new positions
	4. Does not currently store any data - no need for that so far
	5. RESPONSIBLE for determining whether an algorithmic function or copyState() must be called for more efficient
		operation


-------------------------------------------------------------------------------------------

FRAMEWORK:
	1. ALWAYS THINK ABOUT THE ORDER THE FUNCTIONS NEED TO BE CALLED. ASK YOURSELF - IS THIS A TRIPOD-ONLY MOVEMENT
		OR IT IS RELATED TO THE OTHER TRIPOD AND TO THE ROBOT ITSELF
	2. ALWAYS THINK ABOUT WHAT COMPUTATIONS DOES THE CALLED FUNCTION PERFORM - IF THE RESULTING STATE OF EACH LEG
		IS THE SAME, DO COMPUTATIONS FOR FIRST LEG AND USE copyState()

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
			DnxSerialBase* HipsKnees, DnxSerialBase* Arms, double height_in, BodyParams robot_params);

	~Tripod ();

	/* ------------------------------------ COPYING STATE ----------------------------------- */

	void copyState(const Tripod& tripod_in);

	/* ------------------------------------ STATIC POSITIONS ----------------------------------- */

	void defaultPos();					// Reset all Leg parameters to their defaultPos values
	void center();						// Reset all Legs to their central positions and keep current height
	void stand();						// Set all Legs to a standing state where height = Tibia
	void standQuad();					// Same as stand() but arms configured as quad
	void flatQuad();					// Straighten legs to fly as a quad

	double standing();					// Determines if standing, and if not returns by how much should be lifted

	/* ------------------------------------ WALKING MOVEMENTS ----------------------------------- */
	
	void bodyForward(double step_size);
	void stepForward(double step_size);

	void bodyRotate(double angle);
	void stepRotate(double angle);
	
	void liftUp(double height_up);
	void lowerDown(double height_down);
	void finishStep();

	void raiseBody(double hraise);

	/* ------------------------------------ TESTING FUNCTIONS ----------------------------------- */

	void quadSetup();

private:
	void makeMovement(void (Leg::*leg_action)(double), double arg, const string debug_msg=""); 		// Wrapper for calling a function from Leg that makes any moevement
	void setPosition(void (Leg::*leg_action)(), const string debug_msg=""); 						// Wrapper for calling a function from Leg that sets a static leg position

	void writeAngles();
	void writeHipKneeAngles();

	Leg Legs[LEG_COUNT];

	static const double leg_lift;
};

#endif 


