/* 

State Class is an abstraction representing the momentary state of a Leg. Its main purpose is to encapsulate the information 
so that a proper protocol is followed when the state of the leg is updated
===========================================================================================

Stores information about the parameters of the leg such as:

DIST_CENTER 		Distance from robot's center to the center of ARM servo
COXA 			Distance from the center of ARM servo to the center of HIP servo
FEMUR 			Distance from the center of HIP servo to the center of KNEE servo
TIBIA 			Distance from the center of KNEE servo to END EFFECTOR
hip_to_end		Direct distance from hip mount point to the point where the end effector touches ground
HEIGHT 			Vertical distance from HIP mount point to ground
arm_ground_to_ef		Horizontal distance from the center of ARM servo's ground point projection to the END EFFECTOR
ANGLEOFFSET 	Angle between Y-axis(Robot orientation) and servo orientation; always a positive quantity
HIPKNEEMAXHDIST Maximum horizontal distance between center of HIP and center of KNEE so that torque on KNEE servo is in range
KNEE_TO_MOTOR_DIST 	Horizontal distance between center of KNEE and center of MOTOR - needed for calculating Quadcopter ARM angle

-------------------------------------------------------------------------------------------

FUNCTIONALITY:
	1. Can be only controlled from the Leg Class 
	2. Responsible only for properly updating the state when provided arguments from the Leg class. Actual algorithmic
	computations for movements are partially performed in Leg Class
	3. Does not write calculated values to the motors: this action is invoked from the Tripod Class for coordination between legs
	4. The class does computations for LEFT LEG only. Values for RIGHT LEG are computed at the time of writing as opposite to Left Leg
	5. servo_angles[] stores values to be written to servo. These would usually be different from values you need for computations
	6. Although servo_angles[] is HARDLY used for computing state, be careful if you need to use it

-------------------------------------------------------------------------------------------

FRAMEWORK:
	1. Algorithms work only for LEFT LEG. Values for RIGHT LEG are computed at the time of writing as opposite to Left Leg

	2. vars[]		-	Use updateVar(). Function automatically updates square value for the variable
							and the rest of the variables by calling StateUpdate()
	3. updateVar() 		- 	When you use this to update hip_to_end it is assumed the change is in ENDEFFECTOR
							rather than in the HEIGHT of the robot. If change is in the HEIGHT, the vars.height
							will not be updates
	4. configureAngles()	-	Function automatically called when updateVar() is called. Basing on vars[]
							function computes new servo_angles for Hip and Knee
	5. center()			-	Assumes Leg is already lifted up, otherwise robot will probably fall down
	6. configureVars() 	- 	Used when state changes and new vars[] need to be computed. Computes all vars[] based on the current 
							servo_angles[] does not use Update(), but computes variables manually to ensure no call to configureAngles()
	7. configureEFVars() 	- 	Called only by centerAngles(). Computes hip_to_end, arm_ground_to_ef and ef_center (if needed) basing on 
							params[] and KNEE
	8. centerAngles() 	- 	Computed median HIP and KNEE basing on params[] and current HEIGHT or the input HEIGHT if provided.
							Calls configureEFVars automatically in order to keep state consistent
	9. setAngles() 		- 	Should be called only on complete state change because automatically calls configureVars() and this
							updates all vars, including ef_center

-------------------------------------------------------------------------------------------

*/

#ifndef STATE_T_H
#define STATE_T_H

#include "wkq.h"
#include <cmath>
#include "robot_types.h"
#include <stddef.h>


class State_t{

public:
	
	State_t(double height_in, const BodyParams& robot_params);
	~State_t();

	void operator=(const State_t& StateIn);


	/* ------------------------------------ LEG STATIC POSITIONS ----------------------------------- */

	void legDefaultPos();									// Reset all Leg variables to their defaultPos values
	void legCenter();										// Reset all Legs to their central positions and keep current height
	void legStand();										// Centralize all Legs for a standing state where height = Tibia
	void legFlatten();										// flatten the knee

	/* ------------------------------------ MAINTAINING LEG STATE ----------------------------------- */

	void updateVar(double* address, double value, bool update_state=true);
	void updateVar(double* address, double value, double* address_sq, double value_sq);

	void clear();											// Clears vars[] - needed for flight-related actions
	//void StateVerify();									// Verifies the current leg state is physically possible and accurate

	void centerAngles(double height =0.0); 					// Compute median HIP, KNEE based on params[] and HEIGHT/height. Calls configureEFVars()

#ifdef DOF3
	void setAngles(double knee, double hip, double arm);	// Calls configureVars()
#else
	void setAngles(double knee, double hip);				// Calls configureVars()
#endif

	/* ------------------------------------ PUBLIC MEMBER DATA ------------------------------------ */

	BodyParams 				params;							// Store const parameters of the robot
	LegAngles 				servo_angles;					// In radians: 0.0 - center, positive - CW, negative - CCW
	DynamicVars 			vars;							// Store current values of variables that determine state of the robot

	//static Leg_Joints 	angle_limits_max;				// Store max angle limits of the robot
	//static Leg_Joints 	angle_limits_min;				// Store min angle limits of the robot

private:

	/* ------------------------------------ MAINTAINING LEG STATE ----------------------------------- */

	void update(double* address);							// Auto-invoked when any vars[] changes in order to keep leg state consistent
	void configureAngles();									// Update KNEE and HIP servo_angles basing on the current hip_to_end and HEIGHT
	void configureEFVars(double height=0.0); 					// Compute hip_to_end, arm_ground_to_ef based on KNEE, HEIGHT/height; called by centerAngles()
	void configureVars();										// Computes valid vars[] basing on servo_angles[]


	/* ------------------------------------ PRIVATE MEMBER DATA ------------------------------------ */
	
	static LegAngles 		default_pos_angles;				// Store default values for vars.var_count
	static DynamicVars		default_pos_vars; 				// Store default values for servo_angles[VAR_COUNT]

	static bool 			default_pos_calculated;			// Needed to know whether to calculate defaultPoss
};


#endif


