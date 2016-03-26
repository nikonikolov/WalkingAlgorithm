#ifndef WALKING_QUAD_MACROS_H
#define WALKING_QUAD_MACROS_H


#define RADIANS(A)				((A)/180*PI)
#define DEGREES(A)				((A)/PI*180)
const double PI = 3.14159265358979323846264338327950288419716939937510;

// SERVO NAMES: Number corresponds to ID of the servo
/* NOTE: Try to keep IDs the same. Changing them will cause bugs in Leg and Tripod constructors. They can be repaired, though */

// SERIAL 2 - knees
#define KNEE_LEFT_FRONT			11
#define KNEE_LEFT_MIDDLE		12
#define KNEE_LEFT_BACK			13
#define KNEE_RIGHT_FRONT		14
#define KNEE_RIGHT_MIDDLE		15
#define KNEE_RIGHT_BACK			16

// SERIAL 3 -hips
#define HIP_LEFT_FRONT			17
#define HIP_LEFT_MIDDLE			18
#define HIP_LEFT_BACK			19
#define HIP_RIGHT_FRONT			20
#define HIP_RIGHT_MIDDLE		21
#define HIP_RIGHT_BACK			22

// SERIAL 1 - arms_wings
#define WING_LEFT_FRONT			23
#define WING_LEFT_MIDDLE		24
#define WING_LEFT_BACK			25
#define WING_RIGHT_FRONT		26
#define WING_RIGHT_MIDDLE		27
#define WING_RIGHT_BACK			28

// SERIAL 1 - arms_wings
#define ARM_LEFT_FRONT			29
#define ARM_LEFT_MIDDLE			30
#define ARM_LEFT_BACK			31
#define ARM_RIGHT_FRONT			32
#define ARM_RIGHT_MIDDLE		33
#define ARM_RIGHT_BACK			34

#define KNEES 					11
#define HIPS 					17
#define WINGS 					23
#define ARMS 					29

// LIMIT ROTATION ANGLES FOR SERVO
#define KNEE_MIN				0
#define KNEE_MAX				1024
#define HIP_MIN					0
#define HIP_MAX					1024
#define WING_MIN				0
#define WING_MAX				1024	
#define ARM_MIN					0
#define ARM_MAX					1024

#endif // WALKING_QUAD_MACROS_H


