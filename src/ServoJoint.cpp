#include "ServoJoint.h"

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
