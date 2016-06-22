#include "Robot.h"

Robot::Robot(MController* pixhawk_in, DNXServo* HipsKnees, DNXServo* ArmsWings, double height_in, 
		const double robot_params[], wkq::RobotState_t state_in /*= wkq::RS_default*/) :
	Tripods{
		Tripod(wkq::knee_left_front, wkq::knee_right_middle, wkq::knee_left_back, HipsKnees, ArmsWings, height_in, robot_params),
		Tripod(wkq::knee_right_front, wkq::knee_left_middle, wkq::knee_right_back, HipsKnees, ArmsWings, height_in, robot_params)
	}, 
	pixhawk(pixhawk_in), state(state_in){
	
	pc.print_debug("Robot start\n");
	
	// Calculate max size for movements
	max_step_size = 		CalcMaxStepSize();
	max_rotation_angle = 	CalcMaxRotationAngle();
	
	pc.print_debug("Robot calculating state\n");

	state = wkq::RS_standing_flat_quad;
	// Initialize servos according to state
	if 		(state_in == wkq::RS_default) 				Default();
	else if (state_in == wkq::RS_standing) 				Stand();
	else if (state_in == wkq::RS_standing_quad) 		StandQuad();
	else if (state_in == wkq::RS_standing_flat_quad) 	{ StandQuad(); FlattenLegs(); }
	else throw string("Robot cannot be initialized to non-standing position");

	pc.print_debug("Robot done\n");
}


Robot::~Robot(){}


/* ================================================= STANDING POSITIONS ================================================= */

void Robot::Default(){
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].Default();
	}
	state = wkq::RS_default;
}

void Robot::Center(){
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].Center();
	}
	state = wkq::RS_centered;
}

void Robot::Stand(){
	bool meaningless_state = true;
	if(state!=wkq::RS_standing_flat_quad){
		RaiseBody(Tripods[0].Standing());
		meaningless_state = false;
	}
	Tripods[TRIPOD_LEFT].Stand(meaningless_state);
	wait(0.2);
	Tripods[TRIPOD_RIGHT].Stand(meaningless_state);
	/*for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].Stand(meaningless_state);
	}*/
	state = wkq::RS_standing;
}

void Robot::StandQuad(){
	bool meaningless_state = true;
	if(state!=wkq::RS_standing_flat_quad){
		RaiseBody(Tripods[0].Standing());
		meaningless_state = false;
	}
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].StandQuad(meaningless_state);
	}
	state = wkq::RS_standing_quad;
}

// input specifies the overall state of the quad - otherwise we only know the legs are flat
void Robot::FlattenLegs(wkq::RobotState_t state_in /*= wkq::RS_standing_flat_quad*/){
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].FlattenLegs();
	}
	state = state_in;
}


/* ================================================= WALK RELATED FUNCTIONALITY ================================================= */


void Robot::WalkForward (double coeff){
	if(coeff<0.0) coeff=0.1;
	if(coeff>1.0) coeff=1.0;

	double step_size = coeff*max_step_size;
	double ef_raise = 5.0;

	bool continue_movement = true;
	int tripod_up = TRIPOD_LEFT, tripod_down = TRIPOD_RIGHT;

	pc.print_debug("Movement forward starts\n");
	
	// repeat until walkforward signal stops
	while(continue_movement){
		Tripods[tripod_up].LiftUp(ef_raise);
		pc.print_debug("First tripod lifted\n");
		Tripods[tripod_down].BodyForward(step_size);
		pc.print_debug("Second tripod moved body forward\n");

		// Read Input and find out whether movement should go on
		continue_movement = pixhawk->InputWalkForward();

		// Movement goes on
		if(continue_movement){
			Tripods[tripod_up].StepForward(step_size);
			pc.print_debug("First tripod put down legs for another step forward\n");
			std::swap(tripod_up, tripod_down);			// swap the roles of the Tripods
		}
		
		// Movement stops
		else{
			Tripods[tripod_up].FinishStep();
			pc.print_debug("First tripod finished step\n");
			Tripods[tripod_down].LiftUp(ef_raise);
			pc.print_debug("Second tripod lifted\n");
			Tripods[tripod_down].FinishStep();
			pc.print_debug("Second tripod finished step\n");
		}
	}
}


void Robot::Rotate(double angle){
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
	return 5.0;
}

double Robot::CalcMaxRotationAngle(){
	return (wkq::PI)/3;
}

