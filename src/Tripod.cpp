#include "Tripod.h"

const double Tripod::leg_lift = 5.0; 

Tripod::Tripod (const int& ID_front_knee, const int& ID_middle_knee, const int& ID_back_knee,
				DNXServo* HipsKnees, DNXServo* ArmsWings, const double& height_in) :
	Legs{	Leg(ID_front_knee, 	ID_front_knee+6, 	ID_front_knee+12, 	ID_front_knee+18, 	HipsKnees, ArmsWings, height_in),
			Leg(ID_middle_knee, ID_middle_knee+6, 	ID_middle_knee+12, 	ID_middle_knee+18, 	HipsKnees, ArmsWings, height_in),
			Leg(ID_back_knee, 	ID_back_knee+6, 	ID_back_knee+12, 	ID_back_knee+18, 	HipsKnees, ArmsWings, height_in)
		} {}

Tripod::~Tripod(){}

/* ================================================= STANDING POSITIONS ================================================= */

void Tripod::Default(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].Default();
	}
	WriteAllAngles();
}

void Tripod::Center(){
	/*LiftTripodUp(5);
	for(int i=0; i<LEG_COUNT; i++){
		if(i==0) Legs[i].Center();
		// All Legs have same defaults so save some computations
		else Legs[i].CopyState(Legs[0]);
		Legs[i].WriteAllAngles();
	}*/
}

void Tripod::Stand(){
	// Robot already has made sure that body was lifted if necessary. Do it leg by leg to ensure you don't overload servos
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].Raise(leg_lift);
		Legs[i].WriteAngles();
		Legs[i].Stand();
		Legs[i].WriteAllAngles();
	}
}

void Tripod::StandQuad(){
	// Robot already has made sure that body was lifted if necessary. Do it leg by leg to ensure you don't overload servos
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].Raise(leg_lift);
		Legs[i].WriteAllAngles();
		Legs[i].StandQuad();
		Legs[i].WriteAllAngles();
	}
}

void Tripod::FlattenLegs(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].Flatten();
	}
	WriteHipKneeAngles();
}

double Tripod::Standing(){
	double height = Legs[0].Get(STATE_VAR, HEIGHT);
	double height_stand = Legs[0].Get(PARAM, KNEE);
	if(!almost_equals(height, height_stand)) return 0.0;
	else return height_stand - height;
}
/*
void Tripod::CopyTripodState(const Tripod& TripodIn){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].CopyState(TripodIn.Legs[i]);
	}
	WriteAngles();
}
*/

/* ================================================= WALK RELATED FUNCTIONALITY ================================================= */
/*
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
		Legs[i].Raise(height);
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
/*
void Tripod::LiftBodyUp(const double& hraise){
	Center();
	for(int i=0; i<LEG_COUNT; i++){
		if(i==0) Legs[i].LiftBodyUp(hraise);
		else Legs[i].CopyState(Legs[0]);
	}
	WriteHipKneeAngles();
}

/*
void Tripod::ConfigureQuadcopter(){
	if(Param[HEIGHT]!=HEIGHT_STANDING) LiftTripodUp(HEIGHT_STANDING-Param[HEIGHT]);
}
*/

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

void Tripod::WriteAllAngles(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].WriteAllAngles();
	}
}
