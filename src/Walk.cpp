#include "Walk.h"


/*
ServoJoint Hip_Left_Front 		(HIP_LEFT_FRONT, 	&Hips);
ServoJoint Hip_Left_Middle 		(HIP_LEFT_MIDDLE, 	&Hips); 
ServoJoint Hip_Left_Back 		(HIP_LEFT_BACK, 	&Hips); 
ServoJoint Hip_Right_Front 		(HIP_RIGHT_FRONT, 	&Hips); 
ServoJoint Hip_Right_Middle 	(HIP_RIGHT_MIDDLE, 	&Hips); 
ServoJoint Hip_Right_Back 		(HIP_RIGHT_BACK, 	&Hips); 

ServoJoint Knee_Left_Front 		(KNEE_LEFT_FRONT, 	&Knees);
ServoJoint Knee_Left_Middle 	(KNEE_LEFT_MIDDLE, 	&Knees); 
ServoJoint Knee_Left_Back 		(KNEE_LEFT_BACK, 	&Knees); 
ServoJoint Knee_Right_Front 	(KNEE_RIGHT_FRONT, 	&Knees); 
ServoJoint Knee_Right_Middle 	(KNEE_RIGHT_MIDDLE, &Knees); 
ServoJoint Knee_Right_Back 		(KNEE_RIGHT_BACK, 	&Knees); 

ServoJoint Wing_Left_Front 		(WING_LEFT_FRONT, 	&ArmsWings);
ServoJoint Wing_Left_Middle 	(WING_LEFT_MIDDLE, 	&ArmsWings); 
ServoJoint Wing_Left_Back 		(WING_LEFT_BACK, 	&ArmsWings); 
ServoJoint Wing_Right_Front 	(WING_RIGHT_FRONT, 	&ArmsWings); 
ServoJoint Wing_Right_Middle 	(WING_RIGHT_MIDDLE, &ArmsWings); 
ServoJoint Wing_Right_Back 		(WING_RIGHT_BACK, 	&ArmsWings);

ServoJoint Arm_Left_Front 		(ARM_LEFT_FRONT, 	&ArmsWings);
ServoJoint Arm_Left_Middle 		(ARM_LEFT_MIDDLE, 	&ArmsWings); 
ServoJoint Arm_Left_Back 		(ARM_LEFT_BACK, 	&ArmsWings); 
ServoJoint Arm_Right_Front 		(ARM_RIGHT_FRONT, 	&ArmsWings); 
ServoJoint Arm_Right_Middle 	(ARM_RIGHT_MIDDLE, 	&ArmsWings); 
ServoJoint Arm_Right_Back 		(ARM_RIGHT_BACK, 	&ArmsWings);


Leg Left_Front 		(Knee_Left_Front, 	Hip_Left_Front, 	Arm_Left_Front, 	Wing_Left_Front);
Leg Left_Middle 	(Knee_Left_Middle, 	Hip_Left_Middle, 	Arm_Left_Middle, 	Wing_Left_Middle); 
Leg Left_Back 		(Knee_Left_Back, 	Hip_Left_Back, 		Arm_Left_Back, 		Wing_Left_Back); 
Leg Right_Front 	(Knee_Right_Front, 	Hip_Right_Front, 	Arm_Right_Front, 	Wing_Right_Front);
Leg Right_Middle 	(Knee_Right_Middle, Hip_Right_Middle, 	Arm_Right_Middle, 	Wing_Right_Middle); 
Leg Right_Back 		(Knee_Right_Back, 	Hip_Right_Back, 	Arm_Right_Back, 	Wing_Right_Back); 

*/
int RobotFly(){
	Knees.SetGoalPosition(ID_Broadcast, 512); 
	Hips.SetGoalPosition(ID_Broadcast, 512);
	ArmsWings.SetGoalPosition(ID_Broadcast, 512);
	return 0;
}

int RobotStand(){
	Knees.SetGoalPosition(ID_Broadcast, 819); // 90 degrees
	Hips.SetGoalPosition(ID_Broadcast, 512);
	ArmsWings.SetGoalPosition(ID_Broadcast, 512);
	return 0;	
}


int RobotSetLegs(const int& knees_angle, const int& hips_angle, const int& arms_angle, const int& wings_angle/*=512*/){
	
	if ( knees_angle<=KNEE_MIN || knees_angle>=KNEE_MAX || 
		 hips_angle<=HIP_MIN || hips_angle>=HIP_MAX ||
		 arms_angle<=ARM_MIN || arms_angle>=ARM_MAX ||
		 wings_angle<=WING_MIN || wings_angle>=WING_MAX  ) return -1;
	
	Knees.SetGoalPosition(ID_Broadcast, knees_angle);
	Hips.SetGoalPosition(ID_Broadcast, hips_angle);
	SetBunchPosition(ARMS, arms_angle);
	SetBunchPosition(WINGS, wings_angle);

	return 0;	
}

// Sets all servos from a group to the same position
int SetBunchPosition(const int& start, const int& position){
	for(int id=start; id<start+6; id++){
		ArmsWings.SetGoalPosition(id, position);
	}
}


// you should find what are the limits for each servo in terms of degree of freedom, not in terms of stability, 
// so that you can scale RC_input
//void ForwardThree(Leg& 1st, Leg& 2nd, Leg& 3rd, int distance/*=think what the default should be*/){}


//void HardWalk(){}


DNXServo* lookUpServo(const int& ID){
    if (ID>=11 && ID<=16) return &Knees;
    else if (ID>=17 && ID<=22)  return &Hips;
    else return &ArmsWings;
}

int SetGoalPosition(const int& ID, const int& position){
        DNXServo * servo_ptr = lookUpServo(ID);
        return servo_ptr->SetGoalPosition(ID,position);
}