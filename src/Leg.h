/* 

Leg Abstraction Class: Corresponds to a physical leg. Constructed from 4 ServoJoints. 
===========================================================================================

Stores information about the parameters of the leg such as:

DISTCENTER 		Distance from robot's center to the center of ARM servo
COXA 			Distance from the center of ARM servo to the center of HIP servo
FEMUR 			Distance from the center of HIP servo to the center of KNEE servo
TIBIA 			Distance from the center of KNEE servo to END EFFECTOR
HIPTOEND		Direct distance from hip mount point to the point where the end effector touches ground
HEIGHT 			Vertical distance from HIP mount point to ground
ARMGTOEND		Horizontal distance from the center of ARM servo's ground point projection to the END EFFECTOR
ANGLEOFFSET 	Angle between Y-axis(Robot orientation) and servo orientation; always a positive quantity
HIPKNEEMAXHDIST Maximum horizontal distance between center of HIP and center of KNEE so that torque on KNEE servo is in range
KNEEMOTORDIST 	Horizontal distance between center of KNEE and center of MOTOR - needed for calculating Quadcopter ARM angle

-------------------------------------------------------------------------------------------

FUNCTIONALITY:
	1. Can be only controlled from the Tripod Class 
	2. Responsible for automatic leg adjustment during a movement, i.e. calculating the required angle for each ServoJoint
	3. Does not write calculated values to the motors: this action is invoked from the Tripod for coordination
	4. The class does computations for LEFT LEG only. Values for RIGHT LEG are computed at the time of writing as opposite to Left Leg
	5. ServoAgnles[] stores values to be written to servo. These would usually be different from values you need for computations
	6. Although ServoAngles[] is HARDLY used for computing state, be careful if you need to use it

-------------------------------------------------------------------------------------------

FRAMEWORK:
	1. Algorithms work only for LEFT LEG. Values for RIGHT LEG are computed at the time of writing as opposite to Left Leg

	2. StateVars[]		-	Use UpdateVar(). Function automatically updates square value for the variable
							and the rest of the variables by calling StateUpdate()
	3. UpdateVar() 		- 	When you use this to update HIPTOEND it is assumed the change is in ENDEFFECTOR
							rather than in the HEIGHT of the robot. If change is in the HEIGHT, the StateVars[HEIGHT]
							will not be updates
	4. UpdateAngles()	-	Function automatically called when UpdateVar() is called. Basing on StateVars[]
							function computes new angles for Hip and Knee
	5. Center()			-	Assumes Leg is already lifted up, otherwise robot will probably fall down
	6. ComputeVars() 	- 	Computes all Vars[] based on the current ServoAngles[] does not use Update(), but computes variables
							manually to ensure no call to UpdateAngles()

-------------------------------------------------------------------------------------------

*/

#ifndef LEG_H
#define LEG_H

#include "ServoJoint.h"
#include "State_t.h"
#include "Point.h"
#include "wkq.h"
#include "include.h"
#include <cmath>


class Leg{

public:
	
	Leg(const int& ID_knee, const int& ID_hip, const int& ID_arm, const int& ID_wing,
		DNXServo* HipsKnees, DNXServo* ArmsWings, const double& height_in);

	~Leg();

	/* ---------------------------------------- GETTER AND COPY ---------------------------------------- */

	double Get(const int& param_type, const int& idx) const;
	void CopyState(const Leg& LegIn);								// Copy the state of input Leg

	/* ---------------------------------------- STANDING POSITIONS ---------------------------------------- */

	inline void Default();					// Reset all Leg parameters to their default values
	inline void Center();					// Reset all Legs to their central positions and keep current height
	inline void Stand();					// Centralize all Legs for a standing state where height = Tibia
	inline void Flatten();					// Flatten the knee
	void StandQuad();						// Same as Stand() but arms configured as quad


	/* ---------------------------------------- RAISE AND LOWER ---------------------------------------- */

	void LiftUp(const double& height);				// Lift End Effector in the air
	void LowerDown(const double& height);			// Put End Effector straight down
	void FinishStep();								// Put End Effector down with ARM, HIP and KNEE centered


	/* ---------------------------------------- WALKING ALGORITHMS ---------------------------------------- */

	void IKBodyForward(const double& step_size);	// Change angles and state of Leg for a step forward
	void StepForward(const double& step_size);		// Put End Effector down by making a step forward. Leg must be already lifted
	
	void IKBodyRotate(const double& angle);			// Change angles and state of Leg for a rotation around central axis
	void StepRotate(const double& angle);			// Put End Effector down by making a rotation step. Leg must be already lifted
	
	void RaiseBody(const double& hraise);
	
	/* ---------------------------------------- WRITE TO SERVOS ---------------------------------------- */

	void WriteAngles();							// Write ServoAngles[] to physcial servos in order ARM, HIP, KNEE
	void WriteAllAngles();						// Write ServoAngles[] to physcial servos in order WING, ARM, HIP, KNEE
	void WriteJoint(const int& idx);			// Write only a single angle contained in ServoAngles[] to physcial servo


private:
	/* ============================================== MEMBER DATA ============================================== */

	State_t state;
	ServoJoint Joints[JOINT_COUNT];					// Stores servo objects corresponding to the physical servos

	double AngleOffset;								// Angle between Y-axis and servo orientation; always positive
	double LegRight;								// 1.0 - Leg is LEFT, -1.0 - Leg is RIGHT

};


inline void Leg::Default(){
	state.LegDefault();
}

inline void Leg::Center(){
	state.LegCenter();
}

inline void Leg::Stand(){
	state.LegStand();
}
inline void Leg::Flatten(){	
	state.LegFlatten();
}


#endif


