#include "ServoJoint.h"

#ifndef SIMULATION
ServoJoint::ServoJoint(int ID_in, unordered_map<int, DnxHAL*>& servo_map) : ID(ID_in), dnx_ptr(servo_map[ID_in]){
#else
ServoJoint::ServoJoint(int ID_in, unordered_map<int, DnxHAL*>& servo_map) : ID(ID_in){
#endif

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
		default:
			servo_name = "error in servo ID";
	}		
	
	setReturnLevel(1);	
	wait_ms(0.1);
	setGoalVelocity(100);	
	wait_ms(0.1);
}

// return ID of the object, not of the physical servo
int ServoJoint::getID() const {
	return ID;
}

int ServoJoint::setID(int newID){
#ifndef SIMUALTION
	return dnx_ptr->setID(ID, newID);
#else
	ID = newID;
	return 0;
#endif
}
	
int ServoJoint::getValue(int address){
#ifndef SIMUALTION
	return dnx_ptr->getValue(ID, address);
#else
	return 0;
#endif
}
    
int ServoJoint::setBaud(int rate){
#ifndef SIMUALTION
	return dnx_ptr->setBaud(ID, rate);
#else
	return 0;
#endif
}
    
int ServoJoint::setReturnLevel(int lvl){
#ifndef SIMUALTION
	return dnx_ptr->setReturnLevel(ID, lvl);
#else
	return 0;
#endif
}

int ServoJoint::setLED(int colour){
#ifndef SIMUALTION
	return dnx_ptr->setLED(ID, colour);
#else
	return 0;
#endif
}

int ServoJoint::setGoalPosition(int angle, bool cash/*=false*/){
#ifndef SIMUALTION
	return dnx_ptr->setGoalPosition(ID, angle, cash);
#else
	return 0;
#endif
}

// angle is in radians
int ServoJoint::setGoalPosition(double angle, bool cash/*=false*/){
	if(debug_) printf("%s set to %f\n\r", servo_name.c_str(), wkq::degrees(angle));
#ifndef SIMUALTION
	return dnx_ptr->setGoalPosition(ID, angle, cash);
#else
	return 0;
#endif
}

int ServoJoint::setGoalVelocity(int velocity){
#ifndef SIMUALTION
	return dnx_ptr->setGoalVelocity(ID, velocity);
#else
	return 0;
#endif
}

int ServoJoint::setGoalTorque(int torque){
#ifndef SIMUALTION
	return dnx_ptr->setGoalTorque(ID, torque);
#else
	return 0;
#endif
}

int ServoJoint::setPunch(int punch){
#ifndef SIMUALTION
	return dnx_ptr->setPunch(ID, punch);
#else
	return 0;
#endif
}

int ServoJoint::action(int arg){
#ifndef SIMUALTION
	return dnx_ptr->action(ID);
#else
	return 0;
#endif
}

void ServoJoint::operator=(const ServoJoint& obj_in){
	if(this != &obj_in){
		this->ID 			= obj_in.ID;
		this->servo_name 	= obj_in.servo_name;

		this->dnx_ptr 		= obj_in.dnx_ptr;

	}
}


#ifdef SIMUALTION
void wait(int time){}
void wait_ms(int time){}
#endif

