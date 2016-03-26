#include "Robot.h"


Robot::Robot (DNXServo* Knees, DNXServo* Hips, DNXServo* ArmsWings, const double& height_in) : 
Tripods[TRIPOD_LEFT](wkquad::knee_left_front, wkquad::knee_right_middle, wkquad::knee_left_back, Knees, Hips, ArmsWings, height_in),
Tripods[TRIPOD_RIGHT](wkquad::knee_right_front, wkquad::knee_left_middle, wkquad::knee_right_back, Knees, Hips, ArmsWings, height_in),
	Height(height_in), state(wkquad::state_default){
	WriteAngles();
}


Robot::~Robot(){}

void Robot::Reset(){
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].Reset();
	}
}


void Robot::WriteAngles(){
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].WriteAngles(distance);
	}
}

/* ================================================= WALK RELATED FUNCTIONALITY ================================================= */

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



void Robot::LiftBodyUp(const double& hraise){
	Tripods[TRIPOD_LEFT].LiftBodyUp(hraise);
	Tripods[TRIPOD_RIGHT].CopyTripodState(Tripods[TRIPOD_LEFT]);
}

void Robot::PutRobotDown(const double& height){
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].PutStraightDown(height);
	}

	WriteAngles();
}

/* ================================================= END WALK RELATED FUNCTIONALITY ================================================= */


/* ================================================= FLIGHT RELATED FUNCTIONALITY ================================================= */


void Robot::ConfigureQuadcopter(){

}

void Robot::ConfigureHexacopter(){

}
