#include "ServoJoint.h"

ServoJoint::ServoJoint(int ID_in, DNXServo* SerialObjPtrIn) : ID(ID_in){
	switch(ID){
		case knee_left_front	:
			servo_name = "knee_left_front";
			break;
		case knee_left_middle	:
			servo_name = "knee_left_middle";
			break;
		case knee_left_back	:
			servo_name = "knee_left_back";
			break;
		case knee_right_front	:
			servo_name = "knee_right_front";
			break;
		case knee_right_middle	:
			servo_name = "knee_right_middle";
			break;
		case knee_right_back	:
			servo_name = "knee_right_back";
			break;
		case hip_left_front	:
			servo_name = "hip_left_front";
			break;
		case hip_left_middle	:
			servo_name = "hip_left_middle";
			break;
		case hip_left_back	:
			servo_name = "hip_left_back";
			break;
		case hip_right_front	:
			servo_name = "hip_right_front";
			break;
		case hip_right_middle	:
			servo_name = "hip_right_middle";
			break;
		case hip_right_back	:
			servo_name = "hip_right_back";
			break;
		case wing_left_front	:
			servo_name = "wing_left_front";
			break;
		case wing_left_middle	:
			servo_name = "wing_left_middle";
			break;
		case wing_left_back	:
			servo_name = "wing_left_back";
			break;
		case wing_right_front	:
			servo_name = "wing_right_front";
			break;
		case wing_right_middle	:
			servo_name = "wing_right_middle";
			break;
		case wing_right_back	:
			servo_name = "wing_right_back";
			break;
		case arm_left_front	:
			servo_name = "arm_left_front";
			break;
		case arm_left_middle	:
			servo_name = "arm_left_middle";
			break;
		case arm_left_back	:
			servo_name = "arm_left_back";
			break;
		case arm_right_front	:
			servo_name = "arm_right_front";
			break;
		case arm_right_middle	:
			servo_name = "arm_right_middle";
			break;
		case arm_right_back	:
			servo_name = "arm_right_back";
			break;
		default:
			servo_name = "error in servo ID";
	}		
}

// return ID of the object, not of the physical servo
int ServoJoint::GetID() const {
	return ID;
}

int ServoJoint::SetID(int newID){
	ID = newID;
	return 0;
}
	
int ServoJoint::GetValue(int address){
	return 0;
}
    
int ServoJoint::SetBaud(int rate){
	return 0;
}
    
int ServoJoint::SetReturnLevel(int lvl){
	return 0; 
}

int ServoJoint::SetLED(int colour){
	return 0; 
}

int ServoJoint::SetGoalPosition(int angle_in){
	return 0;
}

int ServoJoint::SetGoalPosition(double angle_in){
	angle=angle_in;
	cout<<servo_name<<" set to "<<degrees(angle);
	return 0;
}

int ServoJoint::SetGoalVelocity(int velocity){
	return 0;
}

int ServoJoint::SetGoalTorque(int torque){
	return 0;
}

int ServoJoint::SetPunch(int punch){
	return 0;
}

void ServoJoint::operator=(const ServoJoint& obj_in){
	if(this != &obj_in){
		ID = obj_in.ID;
		servo_name = obj_in.servo_name;
	}
}
