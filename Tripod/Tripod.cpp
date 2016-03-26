#include "Tripod.h"


Tripod::Tripod (const int& ID_front_knee, const int& ID_middle_knee, const int& ID_back_knee,
				DNXServo* Knees, DNXServo* Hips, DNXServo* ArmsWings, const double& height_in) :
	
	Legs[FRONTLEG](ID_front_knee, ID_front_knee+6, ID_front_knee+12, ID_front_knee+18, Knees, Hips, ArmsWings, height_in),
	Legs[MIDDLELEG](ID_middle_knee, ID_middle_knee+6, ID_middle_knee+12, ID_middle_knee+18, Knees, Hips, ArmsWings, height_in),
	Legs[BACKLEG](ID_back_knee, ID_back_knee+6, ID_back_knee+12, ID_back_knee+18, Knees, Hips, ArmsWings, height_in){}

Tripod::~Tripod(){}


void Tripod::Reset(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].Reset();
	}
	WriteAngles();
}


void Tripod::Center(){
	LiftTripodUp(5);
	for(int i=0; i<LEG_COUNT; i++){
		if(i==0) Legs[i].Center();
		else Legs[i].CopyState(Legs[0]);
		Legs[i].WriteJoint(ARM);
		Legs[i].WriteJoint(HIP);
		Legs[i].WriteJoint(KNEE);
	}
}


void Tripod::CopyTripodState(const Tripod& TripodIn){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].CopyState(TripodIn.Legs[i]);
	}
	WriteAngles();
}

/* ================================================= WALK RELATED FUNCTIONALITY ================================================= */

void Tripod::BodyForward (const double& distance){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].IKForward(distance);
	}
	WriteAngles();
}

void Tripod::BodyRotate(double angle){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].IKRotate(angle);
	}
	WriteAngles();
}

void Tripod::LiftTripodUp(const double& height){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].LiftLegUp(height);
	}
	WriteAngles();
}

void Tripod::PutTripodDownForStepForward(const double& distance){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].PutDownForStepForward(distance);
	}
	WriteAngles();
}

void Tripod::PutTripodStraightDown(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].PutStraightDown();
	}
	WriteAngles();
}

void Tripod::PutTripodStraightDown(const double& height){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].PutStraightDown(height);
	}
	WriteAngles();
}

/* ================================================= END WALK RELATED FUNCTIONALITY ================================================= */


/* ================================================= FLIGHT RELATED FUNCTIONALITY ================================================= */

void Tripod::LiftBodyUp(const double& hraise){
	Center();
	for(int i=0; i<LEG_COUNT; i++){
		if(i==0) Legs[i].LiftBodyUp(hraise);
		else Legs[i].CopyState(Legs[0]);
	}
	WriteHipKneeAngles();
}


void Tripod::ConfigureQuadcopter(){
	if(Param[HEIGHT]!=HEIGHT_STANDING) LiftTripodUp(HEIGHT_STANDING-Param[HEIGHT]);

}

void Tripod::ConfigureHexacopter();

/* ================================================= END FLIGHT RELATED FUNCTIONALITY ================================================= */

/* ================================================= PRIVATE FUNCTIONALITY ================================================= */

void Tripod::WriteHipKneeAngles(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].WriteJoint(HIP);
		Legs[i].WriteJoint(KNEE);
	}
}

void Tripod::WriteAngles(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].WriteAngles();
	}
}
