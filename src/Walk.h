#ifndef WALK_H
#define WALK_H

#include <XL320.h>
#include <AX12A.h>
#include <DNXServo.h>
#include <ServoJoint.h>
#include <Leg.h>



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
