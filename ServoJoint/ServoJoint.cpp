#include "ServoJoint.h"

ServoJoint::ServoJoint(int ID_in, DNXServo* ChainPtrIn):
	ID(ID_in), ChainPtr(ChainPtr) {}


int ServoJoint::GetPort() const {
	return ChainPtr->getPort();
}

// return ID of the object, not of the physical servo
int ServoJoint::GetID() const {
	return ID;
}

int ServoJoint::SetID(int newID){
	return ChainPtr->SetID(ID, newID);
}
	
int ServoJoint::GetValue(int address){
	return ChainPtr->GetValue(ID, address);
}
    
int ServoJoint::SetBaud(int rate){
	return ChainPtr->SetBaud(ID, rate);
}
    
int ServoJoint::SetReturnLevel(int lvl){
	return ChainPtr->SetReturnLevel(ID, lvl);
}

int ServoJoint::SetLED(int colour){
	return ChainPtr->SetLED(ID, colour);
}

int ServoJoint::SetGoalPosition(int angle){
	return ChainPtr->SetGoalPosition(ID, angle);
}

int ServoJoint::SetGoalPosition(double angle){
	return ChainPtr->SetGoalPosition(ID, angle);
}

int ServoJoint::SetGoalVelocity(int velocity){
	return ChainPtr->SetGoalVelocity(ID, velocity);
}

int ServoJoint::SetGoalTorque(int torque){
	return ChainPtr->SetGoalTorque(ID, torque);
}

int ServoJoint::SetPunch(int punch){
	return ChainPtr->SetPunch(ID, punch);
}
