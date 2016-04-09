/* 

State Class is an abstraction representing the momentary state of a Leg. Its main purpose is to encapsulate the information 
so that a proper protocol is followed when the state of the leg is updated
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
	1. Can be only controlled from the Leg Class 
	2. Responsible only for properly updating the state when provided arguments from the Leg class. Actual algorithmic
	computations for movements are partially performed in Leg Class
	3. Does not write calculated values to the motors: this action is invoked from the Tripod Class for coordination between legs
	4. The class does computations for LEFT LEG only. Values for RIGHT LEG are computed at the time of writing as opposite to Left Leg
	5. ServoAngles[] stores values to be written to servo. These would usually be different from values you need for computations
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

#ifndef STATE_T_H
#define STATE_T_H

#include "wkq.h"
#include "include.h"
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
#define PARAM_STEP 			6						// Help-macro to get indices in the array with parameters
#define PARAM_COUNT			(2*PARAM_STEP)			// Size of the array holding the parameters

#define DISTCENTER 			0						// Distance from Robot's Center to Coxa (Arm Mount Point)
#define COXA 				1
#define FEMUR 				2
#define TIBIA 				3
#define HIPKNEEMAXHDIST 	4
#define KNEEMOTORDIST 		5
#define DISTCENTER_SQ 		(DISTCENTER+PARAM_STEP)
#define COXA_SQ 			(COXA+PARAM_STEP)
#define FEMUR_SQ 			(FEMUR+PARAM_STEP)
#define TIBIA_SQ 			(TIBIA+PARAM_STEP)
#define HIPKNEEMAXHDIST_SQ 	(HIPKNEEMAXHDIST+PARAM_STEP)
#define KNEEMOTORDIST_SQ 	(KNEEMOTORDIST+PARAM_STEP)


/* ====================================== Indices for StateVars and DefaultVar array ====================================== */
#define VAR_STEP 		3						// Help-macro to get indices in the array with state variables
#define VAR_COUNT		(2*VAR_STEP)			// Size of the array holding the state variables

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

class State_t{

public:
	
	State_t(const double& height_in);

	~State_t();

	/* ---------------------------------------- GETTER AND COPY ---------------------------------------- */

	double Get(const int& param_type, const int& idx) const;
	void operator=(const State_t& StateIn);


	/* ------------------------------------ LEG STANDING POSITIONS ----------------------------------- */

	void LegDefault();					// Reset all Leg variables to their default values
	void LegCenter();					// Reset all Legs to their central positions and keep current height
	void LegStand();					// Centralize all Legs for a standing state where height = Tibia
	void LegFlatten();					// Flatten the knee


	/* ------------------------------------ MAINTAINING LEG STATE ----------------------------------- */

	void UpdateVar(const int& idx, const double& value, const bool& update_state=true);
	void UpdateVar(const int& idx, const double& value, const double& valueSQ);
	void ComputeVars();										// Computes valid StateVars[] basing on ServoAngles[]

	void Clear();											// Clears StateVars[] - needed for flight-related actions
	//void StateVerify();									// Verifies the current leg state is physically possible and accurate

	/* ------------------------------------ FIND CENTRALIZED ANGLE VALUES ----------------------------------- */

	double CenterHip();										// Compute median HIP angle basing on Params[] ONLY
	double CenterKnee(const double& height_hip_in = 0.0);	// Compute KNEE basing on HIP, Params[] and HEIGHT


	/* ============================================== PUBLIC MEMBER DATA ============================================== */

	double ServoAngles[JOINT_COUNT];						// In radians: 0.0 - center, positive - CW, negative - CCW

	static const double Params[PARAM_COUNT];				// Store const parameters of the robot
	static const double AngleLimits[JOINT_COUNT*2];			// Store angle limits of the robot


private:

	void Update(const int& idx);				// Auto-invoked when any StateVars[] changes in order to keep leg state consistent
	void UpdateAngles();						// Update KNEE and HIP angles basing on the current HIPTOEND and HEIGHT


	/* ============================================== PRIVATE MEMBER DATA ============================================== */
	
	double Vars[VAR_COUNT];							// Store current values of variables that determine state of the robot

	static double DefaultVars[VAR_COUNT];			// Store default values for StateVars[VAR_COUNT]
	static double DefaultAngles[JOINT_COUNT]; 		// Store default values for ServoAngles[VAR_COUNT]

	static bool defaults_calculated;				// Needed to know whether to calculate defaults
};


#endif


