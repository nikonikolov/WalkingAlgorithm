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


void Robot::WalkForward (const double& coeff){

	double step_size = coeff*max_step_size;
	double ef_raise = 5.0;

	bool continue_movement = true;
	int tripod_up = TRIPOD_LEFT, tripod_down = TRIPOD_RIGHT;
	while(continue_movement){
		Tripods[tripod_up].LiftUp(ef_raise);
		Tripods[tripod_down].BodyForward(step_size);

		// Read Input and find out whether movement should go on
		//continue_movement = InputWalkForward();

		// Movement goes on
		if(continue_movement){
			Tripods[tripod_up].StepForward(step_size);
			std::swap(tripod_up, tripod_down);
		}
		
		// Movement stops
		else{
			Tripods[tripod_up].FinishStep();
			Tripods[tripod_down].LiftUp(ef_raise);
			Tripods[tripod_down].FinishStep();
		}
	}
}


void Robot::Rotate(const double& angle){
	/*for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].BodyRotate(angle);
	}

	WriteAngles();
	*/
}

void Robot::RaiseBody(const double& hraise){
	if(almost_equals(0.0, hraise)) return;
	Tripods[TRIPOD_LEFT].RaiseBody(hraise);
	Tripods[TRIPOD_RIGHT].CopyState(Tripods[TRIPOD_LEFT]);
}



/*

void Robot::WriteAngles(){
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].WriteAngles(distance);
	}
}
*/



/* ================================================= PRIVATE METHODS ================================================= */

double Robot::CalcMaxStepSize(){
	return 11.0;
}

double Robot::CalcMaxRotationAngle(){
	return (wkq::PI)/3;
}

