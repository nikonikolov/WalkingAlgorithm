#ifndef WALK_H
#define WALK_H

#include <XL320.h>
#include <AX12A.h>
#include <DNXServo.h>
#include <ServoJoint.h>
#include <Leg.h>


// SERVO NAMES: Number corresponds to ID of the servo

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

// GLOBAL VARIABLES
extern	XL320 ArmsWings 				;
extern	XL320 Knees 					;
extern	AX12A Hips 						;

extern	ServoJoint Hip_Left_Front 		;
extern	ServoJoint Hip_Left_Middle 		; 
extern	ServoJoint Hip_Left_Back 	 	; 
extern	ServoJoint Hip_Right_Front 		; 
extern	ServoJoint Hip_Right_Front 		; 
extern	ServoJoint Hip_Rihgt_Front 		; 

extern	ServoJoint Knee_Left_Front 		;
extern	ServoJoint Knee_Left_Middle 	; 
extern	ServoJoint Knee_Left_Back 		; 
extern	ServoJoint Knee_Right_Front 	; 
extern	ServoJoint Knee_Right_Middle 	; 
extern	ServoJoint Knee_Right_Back 		; 

extern	ServoJoint Wing_Left_Front 		;
extern	ServoJoint Wing_Left_Middle 	; 
extern	ServoJoint Wing_Left_Back 		; 
extern	ServoJoint Wing_Right_Front 	; 
extern	ServoJoint Wing_Right_Middle 	; 
extern	ServoJoint Wing_Right_Back 		; 

extern	ServoJoint Arm_Left_Front 		;
extern	ServoJoint Arm_Left_Middle 		; 
extern	ServoJoint Arm_Left_Back 		; 
extern	ServoJoint Arm_Right_Middle 	; 
extern	ServoJoint Arm_Right_Front 		; 
extern	ServoJoint Arm_Right_Back 		;

extern	Leg Left_Front 					;
extern	Leg Left_Middle 				; 
extern	Leg Left_Back 					; 
extern	Leg Right_Front 				;
extern	Leg Right_Middle 				; 
extern	Leg Right_Back 					; 



int RobotFly();

int RobotStand();

int RobotSetLegs(const int& knees_angle, const int& hips_angle, const int& arms_angle, const int& wings_angle=512);

int SetBunchPosition(const int& start, const int& position);

//void ForwardThree(Leg& 1st, Leg& 2nd, Leg& 3rd, int distance/*=think what the default should be*/);

//void HardWalk();

DNXServo* lookUpServo(const int& ID);
int SetGoalPosition(const int& ID, const int& position);


#endif //WALK_H
