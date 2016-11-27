#include "ServoJoint.h"

#ifndef SIMULATION

ServoJoint::ServoJoint(int ID_in, DnxHAL* dnx_ptr_in) : ID(ID_in), dnx_ptr(dnx_ptr_in) {
	switch(ID){
		case wkq::KNEE_LEFT_FRONT	:
			servo_name = "knee_left_front";
			break;
		case wkq::KNEE_LEFT_MIDDLE	:
			servo_name = "knee_left_middle";
			break;
		case wkq::KNEE_LEFT_BACK	:
			servo_name = "knee_left_back";
			break;
		case wkq::KNEE_RIGHT_FRONT	:
			servo_name = "knee_right_front";
			break;
		case wkq::KNEE_RIGHT_MIDDLE	:
			servo_name = "knee_right_middle";
			break;
		case wkq::KNEE_RIGHT_BACK	:
			servo_name = "knee_right_back";
			break;
		case wkq::HIP_LEFT_FRONT	:
			servo_name = "hip_left_front";
			break;
		case wkq::HIP_LEFT_MIDDLE	:
			servo_name = "hip_left_middle";
			break;
		case wkq::HIP_LEFT_BACK	:
			servo_name = "hip_left_back";
			break;
		case wkq::HIP_RIGHT_FRONT	:
			servo_name = "hip_right_front";
			break;
		case wkq::HIP_RIGHT_MIDDLE	:
			servo_name = "hip_right_middle";
			break;
		case wkq::HIP_RIGHT_BACK	:
			servo_name = "hip_right_back";
			break;
		case wkq::WING_LEFT_FRONT	:
			servo_name = "wing_left_front";
			break;
		case wkq::WING_LEFT_MIDDLE	:
			servo_name = "wing_left_middle";
			break;
		case wkq::WING_LEFT_BACK	:
			servo_name = "wing_left_back";
			break;
		case wkq::WING_RIGHT_FRONT	:
			servo_name = "wing_right_front";
			break;
		case wkq::WING_RIGHT_MIDDLE	:
			servo_name = "wing_right_middle";
			break;
		case wkq::WING_RIGHT_BACK	:
			servo_name = "wing_right_back";
			break;
		case wkq::ARM_LEFT_FRONT	:
			servo_name = "arm_left_front";
			break;
		case wkq::ARM_LEFT_MIDDLE	:
			servo_name = "arm_left_middle";
			break;
		case wkq::ARM_LEFT_BACK	:
			servo_name = "arm_left_back";
			break;
		case wkq::ARM_RIGHT_FRONT	:
			servo_name = "arm_right_front";
			break;
		case wkq::ARM_RIGHT_MIDDLE	:
			servo_name = "arm_right_middle";
			break;
		case wkq::ARM_RIGHT_BACK	:
			servo_name = "arm_right_back";
			break;
		defaultPos:
			servo_name = "error in servo ID";
	}		
	setReturnLevel(1);	
	wait_ms(0.5);
	setGoalVelocity(100);	
	wait_ms(0.5);
	//setPunch(512);	
}

// return ID of the object, not of the physical servo
int ServoJoint::getID() const {
	return ID;
}

int ServoJoint::setID(int newID){
	return dnx_ptr->setID(ID, newID);
}
	
int ServoJoint::getValue(int address){
	return dnx_ptr->getValue(ID, address);
}
    
int ServoJoint::setBaud(int rate){
	return dnx_ptr->setBaud(ID, rate);
}
    
int ServoJoint::setReturnLevel(int lvl){
	return dnx_ptr->setReturnLevel(ID, lvl);
}

int ServoJoint::setLED(int colour){
	return dnx_ptr->setLED(ID, colour);
}

int ServoJoint::setGoalPosition(int angle){
	return dnx_ptr->setGoalPosition(ID, angle);
}

// angle is in radians
int ServoJoint::setGoalPosition(double angle){
	printf("%s set to %f\n\r", servo_name.c_str(), wkq::degrees(angle));
	return dnx_ptr->setGoalPosition(ID, angle);
}

int ServoJoint::setGoalVelocity(int velocity){
	return dnx_ptr->setGoalVelocity(ID, velocity);
}

int ServoJoint::setGoalTorque(int torque){
	return dnx_ptr->setGoalTorque(ID, torque);
}

int ServoJoint::setPunch(int punch){
	return dnx_ptr->setPunch(ID, punch);
}

void ServoJoint::operator=(const ServoJoint& obj_in){
	if(this != &obj_in){
		ID = obj_in.ID;
		dnx_ptr = obj_in.dnx_ptr;
	}
}


#else

void wait(int time){}

ServoJoint::ServoJoint(int ID_in, DnxHAL* robot_view_in) : ID(ID_in)/*, robot_view(robot_view_in)*/{
	switch(ID){
		case wkq::KNEE_LEFT_FRONT	:
			servo_name = "knee_left_front";
			break;
		case wkq::KNEE_LEFT_MIDDLE	:
			servo_name = "knee_left_middle";
			break;
		case wkq::KNEE_LEFT_BACK	:
			servo_name = "knee_left_back";
			break;
		case wkq::KNEE_RIGHT_FRONT	:
			servo_name = "knee_right_front";
			break;
		case wkq::KNEE_RIGHT_MIDDLE	:
			servo_name = "knee_right_middle";
			break;
		case wkq::KNEE_RIGHT_BACK	:
			servo_name = "knee_right_back";
			break;
		case wkq::HIP_LEFT_FRONT	:
			servo_name = "hip_left_front";
			break;
		case wkq::HIP_LEFT_MIDDLE	:
			servo_name = "hip_left_middle";
			break;
		case wkq::HIP_LEFT_BACK	:
			servo_name = "hip_left_back";
			break;
		case wkq::HIP_RIGHT_FRONT	:
			servo_name = "hip_right_front";
			break;
		case wkq::HIP_RIGHT_MIDDLE	:
			servo_name = "hip_right_middle";
			break;
		case wkq::HIP_RIGHT_BACK	:
			servo_name = "hip_right_back";
			break;
		case wkq::WING_LEFT_FRONT	:
			servo_name = "wing_left_front";
			break;
		case wkq::WING_LEFT_MIDDLE	:
			servo_name = "wing_left_middle";
			break;
		case wkq::WING_LEFT_BACK	:
			servo_name = "wing_left_back";
			break;
		case wkq::WING_RIGHT_FRONT	:
			servo_name = "wing_right_front";
			break;
		case wkq::WING_RIGHT_MIDDLE	:
			servo_name = "wing_right_middle";
			break;
		case wkq::WING_RIGHT_BACK	:
			servo_name = "wing_right_back";
			break;
		case wkq::ARM_LEFT_FRONT	:
			servo_name = "arm_left_front";
			break;
		case wkq::ARM_LEFT_MIDDLE	:
			servo_name = "arm_left_middle";
			break;
		case wkq::ARM_LEFT_BACK	:
			servo_name = "arm_left_back";
			break;
		case wkq::ARM_RIGHT_FRONT	:
			servo_name = "arm_right_front";
			break;
		case wkq::ARM_RIGHT_MIDDLE	:
			servo_name = "arm_right_middle";
			break;
		case wkq::ARM_RIGHT_BACK	:
			servo_name = "arm_right_back";
			break;
		defaultPos:
			servo_name = "error in servo ID";
	}		
}

// return ID of the object, not of the physical servo
int ServoJoint::getID() const {
	return ID;
}

int ServoJoint::setID(int newID){
	ID = newID;
	return 0;
}
	
int ServoJoint::getValue(int address){
	return 0;
}
    
int ServoJoint::setBaud(int rate){
	return 0;
}
    
int ServoJoint::setReturnLevel(int lvl){
	return 0; 
}

int ServoJoint::setLED(int colour){
	return 0; 
}

int ServoJoint::setGoalPosition(int angle_in){
	return 0;
}

int ServoJoint::setGoalPosition(double angle_in){
	angle=angle_in;
	printf("%s set to %f\n\r", servo_name.c_str(), wkq::degrees(angle));
	return 0;
}

int ServoJoint::setGoalVelocity(int velocity){
	return 0;
}

int ServoJoint::setGoalTorque(int torque){
	return 0;
}

int ServoJoint::setPunch(int punch){
	return 0;
}

void ServoJoint::operator=(const ServoJoint& obj_in){
	if(this != &obj_in){
		ID = obj_in.ID;
		servo_name = obj_in.servo_name;
	}
}


#endif	// SIMUALTION
