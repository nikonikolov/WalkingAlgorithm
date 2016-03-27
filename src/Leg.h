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

-------------------------------------------------------------------------------------------

FUNCTIONALITY:
	1. Acts as a private class that can be controlled via the Tripod Class 
	2. Responsible for automatic leg adjustment during a movement, i.e. calculating the required angle for each ServoJoint
	3. NOT RESPONSIBLE for writing the calculated values to the motors - provides such functionality but it is invoked
		by the Tripod class for coordination
	4. Angles that are currently written to the servos or are stored in the instance of the class ARE HARDLY used for
		calculating new angle values for a specific movement. Be careful when you use them
	5. NB: If you need current angles in the algorithm BE VERY CAREFUL when using ServoAngles[] - remember that
		it saves the value to be written to the servo and the class as a whole works with LEFT LEG ANGLES ONLY
		Ask yourself, which angle exactly do you want and which angle does ServoAngles[] store


-------------------------------------------------------------------------------------------

FRAMEWORK:
	1. Algorithm and servo values stored are for LEDT LEGS only. Values for RIGHT LEG are exactly the opposite and are
	converted at the time of writing to the servos

	2. ServoAngles[] 	-	Operate Directly - value stored is the offset of the servo from it's central position
	3. StateVars[]		-	Use UpdateStateVars(). Function automatically updates square value for the variable
							and the rest of the variables by calling UpdateState()
	4. DefaultVars[]	-	Operate Directly 
	5. DefaultAngles[]	-	Operate Directly 
	6. Params[] 		-	Operate Directly
	7. AngleLimits[]  	-	Operate Directly
	8. UpdateStateVars()- 	When you use this to update HIPTOEND it is assumed the change is in ENDEFFECTOR
							rather than in the HEIGHT of the robot. If change is in the HEIGHT, the StateVars[HEIGHT]
							will not be updates
	9. UpdateHipKnee()	-	Function automatically called when UpdateStateVars() is called. Basing on StateVars[]
							function computes new angles for Hip and Knee
	10. Center()		-	Assumes Leg is already lifted up, otherwise robot will probably fall down


-------------------------------------------------------------------------------------------

*/

#ifndef LEG_H
#define LEG_H

#include "ServoJoint.h"
#include "wkquad.h"
#include <cmath>



/* ====================================== Indices for Joints and ServoAngle arrays ====================================== */
#define JOINT_COUNT		4

#define KNEE			0
#define HIP				1
#define ARM				2
#define WING 			3


/* ====================================== Indices for AngleLimits arrays ====================================== */
#define KNEE_MIN		(2*KNEE)
#define KNEE_MAX		((2*KNEE)+1)
#define HIP_MIN			(2*HIP)
#define HIP_MAX			((2*HIP)+1)
#define ARM_MIN			(2*ARM)
#define ARM_MAX			((2*ARM)+1)
#define WING_MIN		(2*WING)
#define WING_MAX		((2*WING)+1)


/* ====================================== Indices for Param array ====================================== */
#define PARAM_STEP 			5						// Help-macro to get indices in the array with parameters
#define PARAM_COUNT			(2*PARAM_STEP-1)		// Size of the array holding the parameters

#define DISTCENTER 			0						// Distance from Robot's Center to Coxa (Arm Mount Point)
#define COXA 				1
#define FEMUR 				2
#define TIBIA 				3
#define HIPKNEEMAXHDIST 	4
#define DISTCENTER_SQ 		(DISTCENTER+PARAM_STEP)
#define COXA_SQ 			(COXA+PARAM_STEP)
#define FEMUR_SQ 			(FEMUR+PARAM_STEP)
#define TIBIA_SQ 			(TIBIA+PARAM_STEP)
#define HIPKNEEMAXHDIST_SQ 	(HIPKNEEMAXHDIST+PARAM_STEP)


/* ====================================== Indices for StateVars and DefaultVar array ====================================== */
#define VAR_STEP 		3						// Help-macro to get indices in the array with state variables
#define VAR_COUNT		(2*VAR_STEP-1)			// Size of the array holding the state variables

#define HIPTOEND		0						// Direct distance from hip mount point to the point where the end effector touches ground
#define HEIGHT 			1						// Vertical distance from hip mount point to ground
#define ARMGTOEND		2						// Distance from arm ground projection to end effector
#define HIPTOEND_SQ		(HIPTOEND+VAR_STEP)		
#define HEIGHT_SQ 		(HEIGHT+VAR_STEP)
#define ARMGTOEND_SQ	(ARMGTOEND+VAR_STEP)


/* ====================================== Indices for Getter Decoding ====================================== */
#define SERVO_ANGLE		0						
#define STATE_VAR		1						
#define DEFAULT_VAR		2						
#define DEFAULT_ANGLE	3						
#define PARAM			4						
#define ANGLE_LIMIT		5						


/* ====================================== Additional Macros ====================================== */
#define SQUARE 			2

const double PI = 3.14159265358979323846264338327950288419716939937510;


class Leg{

public:
	
	Leg(const int& ID_knee, const int& ID_hip, const int& ID_arm, const int& ID_wing,
		DNXServo* HipKnees, DNXServo* ArmsWings, const double& height_in);

	~Leg();


	/* ---------------------------------------- MANUAL LEG MANIPULATIONS ---------------------------------------- */
	
	void Reset();								// Reset Leg to its Default state			
	void Center();								// Centers Leg basing on its current height

	void CopyState(const Leg& LegIn);			// Copy the state of input Leg

	int Straighten();							// Straightens Leg - all servos set to 0 degrees
	void LiftLegUp(const double& height);		// Lift End Effector in the air
	void PutStraightDown(const double& height);	// Put End Effector straight down
	//void PutDownInDefault();					//
	void PutDownForStepForward(const double& dist);

	/* ---------------------------------------- END MANUAL LEG MANIPULATIONS ---------------------------------------- */


	/* ---------------------------------------- WALK RELATED MANIPULATIONS ---------------------------------------- */

	void IKForward(const double& dist);			// Change angles and state of Leg for a step forward
	void IKRotate(const double& angle);			// Change angles and state of Leg for a rotation around central axis
	
	/* ---------------------------------------- END WALK RELATED MANIPULATIONS ---------------------------------------- */


	/* ---------------------------------------- FLIGHT RELATED MANIPULATIONS ---------------------------------------- */

	// void LiftBodyUp(const double& hraise);
	
	// The following two functions assume that the robot is already configured to the proper height
	void ConfigureQuadcopter();					// Update arm angle for QUADCOPTER flight and write it to servo
	void ConfigureHexacopter();					// Update arm angle for HEXACOPTER flight and write it to servo

	/* ---------------------------------------- END FLIGHT RELATED MANIPULATIONS ---------------------------------------- */


	/* ---------------------------------------- WRITE TO SERVOS ---------------------------------------- */

	void WriteAngles();							// Write ServoAngles[] to physcial servos except for wing servo
	void WriteAllAngles();						// Write ServoAngles[] to physcial servos including all servos
	void WriteJoint(const int& idx);			// Write only a single angle contained in ServoAngles[] to physcial servo

												// Set leg position manually by providing arguments for each servo
	void SetLegPosition(const double& knee_pos, const double& hip_pos, const double& arm_pos, const double& wing_pos);
	void SetLegPosition(const double& knee_pos, const double& hip_pos, const double& arm_pos);
	int SetJointPosition(const int& JointIdx, const double& position);

	/* ---------------------------------------- END WRITE TO SERVOS ---------------------------------------- */


	/* ---------------------------------------- GETTER ---------------------------------------- */
	
	double GetLegParam(const int& idx) const;



private:

	void PrepareArmForTakeOff();					// Safely makes sure all servos are in their proper positions for take-off


	/* ------------------------------------ UPDATE LEG INSTANCE'S PARAMETERS AND STATE ----------------------------------- */

	void UpdateStateVars(const int& idx, const double& value);
	void UpdateStateVars(const int& idx, const double& value, const double& valueSQ);
	// Automatically invoked when some of the StateVars[] are changed in order to keep robot state consistent
	void UpdateState(const int& idx);				
	void UpdateHipKnee();							// Update KNEE and HIP angles for class' instance according to current state
	void ClearState();								// Clears StateVars[] - needed for flight-related actions

	void VerifyState();								// Verifies the current leg state is physically possible and accurate

	/* ------------------------------------ END UPDATE LEG INSTANCE'S PARAMETERS AND STATE ----------------------------------- */

	/* ------------------------------------ FIND ANGLE/VARIABLE VALUES FOR LEG INSTANCE ----------------------------------- */

	void InitializeState();								// Computes valid StateVars[] basing on ServoAngles[]
	void InitializeState(const double& height_in);		// Computes valid StateVars[] basing on height_in 

	double CenterHip();									// Compute median HIP angle basing on Params[] ONLY
	// Compute KNEE angle basing on HIP angle, Params[] and HEIGHT (if no input provided)
	double CenterKnee(const double& height_hip = StateVars[HEIGHT]);	

	/* ------------------------------------ END FIND ANGLE/VARIABLE VALUES FOR LEG INSTANCE ----------------------------------- */


	
	/* ---------------------------------------- MEMBER DATA ---------------------------------------- */

	ServoJoint Joints[JOINT_COUNT];					// Stores servo objects corresponding to the physical servos
	
	double ServoAngles[JOINT_COUNT];				// In radians: 0.0 - center, positive - CW, negative - CCW
	
	double StateVars[VAR_COUNT];					// Store current values of variables that determine state of the robot

	double AngleOffset;								// Angle between Y-axis and servo orientation; always positive

	bool LegRight;									// 1 - Leg is LEFT, -1 - Leg is RIGHT

	static double DefaultVars[VAR_COUNT];			// Store default values for StateVars[VAR_COUNT]

	static double DefaultAngles[JOINT_COUNT]; 		// Store default values for ServoAngles[VAR_COUNT]

	static const double Params[PARAM_COUNT];		// Store const parameters of the robot

	static const double AngleLimits[JOINT_COUNT*2];	// Store angle limits of the robot


	/* ---------------------------------------- END MEMBER DATA ---------------------------------------- */
};


#endif// LEG_H
