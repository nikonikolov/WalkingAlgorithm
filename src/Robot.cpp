#include "Robot.h"


Robot::Robot(DNXServo* HipsKnees, DNXServo* ArmsWings, const double& height_in, wkq::RobotState_t state_in /*= wkq::RS_default*/) : 
	
	Tripods{
		Tripod(wkq::knee_left_front, wkq::knee_right_middle, wkq::knee_left_back, HipsKnees, ArmsWings, height_in),
		Tripod(wkq::knee_right_front, wkq::knee_left_middle, wkq::knee_right_back, HipsKnees, ArmsWings, height_in)
	}, 
	state(state_in){
	
	// Calculate max size for movements
	max_step_size = 		CalcMaxStepSize();
	max_rotation_angle = 	CalcMaxRotationAngle();
	
	// Initialize servos according to state
	if 		(state == wkq::RS_default) 				Default();
	else if (state == wkq::RS_standing) 			Stand();
	else if (state == wkq::RS_standing_quad) 		StandQuad();
	else if (state == wkq::RS_standing_flat_quad) 	{ StandQuad(); FlattenLegs(); }

	//else throw string("Robot cannot be initialized to non-standing position");
}


Robot::~Robot(){}


/* ================================================= STANDING POSITIONS ================================================= */

void Robot::Default(){
	state = wkq::RS_default;
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].Default();
	}
}

void Robot::Center(){
	state = wkq::RS_centered;
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].Center();
	}
}

void Robot::Stand(){
	state = wkq::RS_standing;
	RaiseBody(Tripods[0].Standing());
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].Stand();
	}
}

void Robot::StandQuad(){
	state = wkq::RS_standing_quad;
	Stand();
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].StandQuad();
	}
}

void Robot::FlattenLegs(){
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].FlattenLegs();
	}
}


/* ================================================= WALK RELATED FUNCTIONALITY ================================================= */

/*
void Robot::WalkForward (const double& coeff){
	
	Tripods[TRIPOD_LEFT].LiftTripodUp(5);
	Tripods[TRIPOD_RIGHT].BodyForward(step);
	Tripods[TRIPOD_LEFT].PutTripodDownForStepForward(step);
	Tripods[TRIPOD_RIGHT].LiftTripodUp(5);
}


void Robot::Rotate(double angle){
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].IKRotate(angle);
	}

	WriteAngles();
}


/*
void Robot::RaiseBody(const double& hraise){
	if(almost_equals(0.0, hraise)) return;
	// Note: Coppying tripod state not a good idea since angles might be different for arms...
	Tripods[TRIPOD_LEFT].RaiseBody(hraise);
	Tripods[TRIPOD_RIGHT].CopyTripodState(Tripods[TRIPOD_LEFT]);
}

void Robot::PutRobotDown(const double& height){
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].PutStraightDown(height);
	}

	WriteAngles();
}

void Robot::WriteAngles(){
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].WriteAngles(distance);
	}
}
*/

void Robot::RaiseBody(const double& hraise){}


/* ================================================= PRIVATE METHODS ================================================= */

double Robot::CalcMaxStepSize(){
	return 11.0;
}

double Robot::CalcMaxRotationAngle(){
	return (wkq::PI)/3;
}

