#ifndef WKQUAD_NAMESPACE_H
#define WKQUAD_NAMESPACE_H

namespace wkquad{

	double radians(const double& degrees);
	double degrees(const double& radians);
	const double PI = 3.14159265358979323846264338327950288419716939937510;

	/* 	SERVO NAMES: Number corresponds to ID of the servo
	 	NOTE: Try to keep IDs the same. Changing them will cause bugs in Tripod constructor parameters 
	 	and you will also have to change the ID of the physical servo */

	// SERIAL TX-9 RX-10 - KNEES
	const int knee_left_front =			11;
	const int knee_left_middle =		12;
	const int knee_left_back =			13;
	const int knee_right_front =		14;
	const int knee_right_middle =		15;
	const int knee_right_back =			16;

	// SERIAL TX-9 RX-10 - HIPS
	const int hip_left_front =			17;
	const int hip_left_middle =			18;
	const int hip_left_back =			19;
	const int hip_right_front =			20;
	const int hip_right_middle =		21;
	const int hip_right_back =			22;

	// SERIAL TX-13 RX-14 - WINGS
	const int wing_left_front =			23;
	const int wing_left_middle =		24;
	const int wing_left_back =			25;
	const int wing_right_front =		26;
	const int wing_right_middle =		27;
	const int wing_right_back =			28;

	// SERIAL TX-13 RX-14 - ARMS
	const int arm_left_front =			29;
	const int arm_left_middle =			30;
	const int arm_left_back =			31;
	const int arm_right_front =			32;
	const int arm_right_middle =		33;
	const int arm_right_back =			34;

	const int knees_start = 			11;
	const int hips_start = 				17;
	const int wings_start = 			23;
	const int arms_start = 				29;

	// LIMITS FOR ROTATION ANGLES FOR SERVO
	const int knee_min =				0;
	const int knee_max =				1024;
	const int hip_min =					0;
	const int hip_max =					1024;
	const int wing_min =				0;
	const int wing_max =				1024;	
	const int arm_min =					0;
	const int arm_max =					1024;


	enum RobotState_t{
		RS_idle 		= 0,			// not doing anything
		RS_in_flight 	= 1,			// flying with legs fully flat
		RS_walking 		= 2,			// walking
		RS_landing 		= 3,			// currently landing - knees on 90 degrees for landing
		RS_error 		= 4,			// not sure what is happening
		RS_default 		= 5,			// all servos configured to default positions
		RS_hex			= 6,			// not completely defined yet
		RS_quad			= 7,			// not completely defined yet
		RS_standing		= 8,			// body height equal to knee height, all knees to 90 degrees, the rest centered
	}; 


	enum LegState_t{
		LS_knee_max_overflow 	= 0,
		LS_knee_min_overflow 	= 1,
		LS_hip_max_overflow 	= 2,
		LS_hip_min_overflow 	= 3,
		LS_arm_max_overflow 	= 4,
		LS_arm_min_overflow 	= 5,
		LS_wing_max_overflow 	= 6,
		LS_wing_min_overflow 	= 7,

	}; 

}


#endif

