#include "ServoJoint.h"

#ifndef SIMULATION

ServoJoint::ServoJoint(int ID_in, DNXServo* SerialObjPtrIn) : ID(ID_in), SerialObjPtr(SerialObjPtrIn) {
	SetReturnLevel(1);	
	//SetPunch(512);	
}

// return ID of the object, not of the physical servo
int ServoJoint::GetID() const {
	return ID;
}

int ServoJoint::SetID(int newID){
	return SerialObjPtr->SetID(ID, newID);
}
	
int ServoJoint::GetValue(int address){
	return SerialObjPtr->GetValue(ID, address);
}
    
int ServoJoint::SetBaud(int rate){
	return SerialObjPtr->SetBaud(ID, rate);
}
    
int ServoJoint::SetReturnLevel(int lvl){
	return SerialObjPtr->SetReturnLevel(ID, lvl);
}

int ServoJoint::SetLED(int colour){
	return SerialObjPtr->SetLED(ID, colour);
}

int ServoJoint::SetGoalPosition(int angle){
	return SerialObjPtr->SetGoalPosition(ID, angle);
}

int ServoJoint::SetGoalPosition(double angle){
	return SerialObjPtr->SetGoalPosition(ID, angle);
}

int ServoJoint::SetGoalVelocity(int velocity){
	return SerialObjPtr->SetGoalVelocity(ID, velocity);
}

int ServoJoint::SetGoalTorque(int torque){
	return SerialObjPtr->SetGoalTorque(ID, torque);
}

int ServoJoint::SetPunch(int punch){
	return SerialObjPtr->SetPunch(ID, punch);
}

void ServoJoint::operator=(const ServoJoint& obj_in){
	if(this != &obj_in){
		ID = obj_in.ID;
		SerialObjPtr = obj_in.SerialObjPtr;
	}
}


#else


ServoJoint::ServoJoint(int ID_in, DNXServo* robot_view_in) : ID(ID_in)/*, robot_view(robot_view_in)*/{
	switch(ID){
		case wkq::knee_left_front	:
			servo_name = "knee_left_front";
			break;
		case wkq::knee_left_middle	:
			servo_name = "knee_left_middle";
			break;
		case wkq::knee_left_back	:
			servo_name = "knee_left_back";
			break;
		case wkq::knee_right_front	:
			servo_name = "knee_right_front";
			break;
		case wkq::knee_right_middle	:
			servo_name = "knee_right_middle";
			break;
		case wkq::knee_right_back	:
			servo_name = "knee_right_back";
			break;
		case wkq::hip_left_front	:
			servo_name = "hip_left_front";
			break;
		case wkq::hip_left_middle	:
			servo_name = "hip_left_middle";
			break;
		case wkq::hip_left_back	:
			servo_name = "hip_left_back";
			break;
		case wkq::hip_right_front	:
			servo_name = "hip_right_front";
			break;
		case wkq::hip_right_middle	:
			servo_name = "hip_right_middle";
			break;
		case wkq::hip_right_back	:
			servo_name = "hip_right_back";
			break;
		case wkq::wing_left_front	:
			servo_name = "wing_left_front";
			break;
		case wkq::wing_left_middle	:
			servo_name = "wing_left_middle";
			break;
		case wkq::wing_left_back	:
			servo_name = "wing_left_back";
			break;
		case wkq::wing_right_front	:
			servo_name = "wing_right_front";
			break;
		case wkq::wing_right_middle	:
			servo_name = "wing_right_middle";
			break;
		case wkq::wing_right_back	:
			servo_name = "wing_right_back";
			break;
		case wkq::arm_left_front	:
			servo_name = "arm_left_front";
			break;
		case wkq::arm_left_middle	:
			servo_name = "arm_left_middle";
			break;
		case wkq::arm_left_back	:
			servo_name = "arm_left_back";
			break;
		case wkq::arm_right_front	:
			servo_name = "arm_right_front";
			break;
		case wkq::arm_right_middle	:
			servo_name = "arm_right_middle";
			break;
		case wkq::arm_right_back	:
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
	pc.print_debug(servo_name + " set to " + dtos(wkq::degrees(angle)) +"\n");
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


#endif	// SIMUALTION
