#include "Robot.h"

//const double Robot::wait_time = 0.5;

Robot::Robot(Master* pixhawk_in, DnxSerialBase* HipsKnees, DnxSerialBase* Arms, double height_in, 
		const double robot_params[], wkq::RobotState_t state_in /*= wkq::RS_DEFAULT*/) :
	Tripods{
		Tripod(wkq::knee_left_front, wkq::knee_right_middle, wkq::knee_left_back, HipsKnees, Arms, height_in, robot_params),
		Tripod(wkq::knee_right_front, wkq::knee_left_middle, wkq::knee_right_back, HipsKnees, Arms, height_in, robot_params)
	}, 
	pixhawk(pixhawk_in), state(state_in){
	
	pc.print_debug("Robot start\n");
	
	// Calculate max size for movements
	max_step_size = 		calcMaxStepSize();
	max_rotation_angle = 	calcMaxRotationAngle();
	
	pc.print_debug("Robot calculating state\n");

	state = wkq::RS_FLAT_QUAD;
	// Initialize servos according to state
	if 		(state_in == wkq::RS_DEFAULT) 				defaultPos();
	else if (state_in == wkq::RS_STANDING) 				stand();
	else if (state_in == wkq::RS_STANDING_QUAD) 		standQuad();
	else if (state_in == wkq::RS_FLAT_QUAD) 			flatQuad();

	pc.print_debug("Robot done\n");
}


Robot::Robot(Master* pixhawk_in, int baud_in, double height_in, const double robot_params[], wkq::RobotState_t state_in/* = wkq::RS_DEFAULT*/) : 
	Robot(pixhawk_in, new AX12A_Serial(p9, p10, baud_in), new XL320_Serial(p13, p14, baud_in), height_in, robot_params, state_in) {}


Robot::~Robot(){}


/* ================================================= standING POSITIONS ================================================= */

void Robot::defaultPos(){
	changeState(wkq::RS_DEFAULT, &Tripod::defaultPos);

	//for(int i=0; i<TRIPOD_COUNT; i++){
	//	Tripods[i].defaultPos();
	//}
	Tripods[TRIPOD_LEFT].defaultPos();
	wait(wait_time);
	Tripods[TRIPOD_RIGHT].defaultPos();
	state = wkq::RS_DEFAULT;
}

void Robot::center(){
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].center();
	}
	state = wkq::RS_CENTERED;
}

void Robot::stand(){
	bool meaningless_state = true;
	if(noState()){
		raiseBody(Tripods[0].standing());
		meaningless_state = false;
	}
	Tripods[TRIPOD_LEFT].stand(meaningless_state);
	wait(wait_time);
	Tripods[TRIPOD_RIGHT].stand(meaningless_state);
	/*for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].stand(meaningless_state);
	}*/
	state = wkq::RS_STANDING;
}

void Robot::standQuad(){
	bool meaningless_state = true;
	if(noState()){
		raiseBody(Tripods[0].standing());
		meaningless_state = false;
	}
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].standQuad(meaningless_state);
	}
	state = wkq::RS_STANDING_QUAD;
}

/*
// input specifies the overall state of the quad - otherwise we only know the legs are flat
void Robot::flattenLegs(wkq::RobotState_t state_in /*= wkq::RS_FLAT_QUAD*){
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].flattenLegs();
	}
	state = state_in;
}
*/

// input specifies the overall state of the quad - otherwise we only know the legs are flat
void Robot::flatQuad(){
	/*for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].flattenLegs();
	}
	state = wkq::RS_FLAT_QUAD;
	*/
}


void Robot::changeState(wkq::RobotState_t state_in, void (Tripod::*tripod_action)(), bool wait_call/*=false, bool check_state*/){
	for(int i=0; i<TRIPOD_COUNT; i++){
		(Tripods[i].*tripod_action)();
		if(wait_call) wait(wait_time);
	}

	state = state_in;
}

/* ================================================= WALK RELATED FUNCTIONALITY ================================================= */


void Robot::walkForward(double coeff){
	if(coeff<0.0) coeff=0.1;
	if(coeff>1.0) coeff=1.0;

	double step_size = coeff*max_step_size;
	double ef_raise = 5.0;

	bool continue_movement = true;
	int tripod_up = TRIPOD_LEFT, tripod_down = TRIPOD_RIGHT;

	//pc.print_debug("Movement forward starts\n");
	
	// repeat until walkForward signal stops
	while(continue_movement){
		Tripods[tripod_up].liftUp(ef_raise);
		//pc.print_debug("First tripod lifted\n");
		Tripods[tripod_down].bodyForward(step_size);
		//pc.print_debug("Second tripod moved body forward\n");

		// Read Input and find out whether movement should go on
		continue_movement = pixhawk->inputWalkForward();

		// Movement goes on
		if(continue_movement){
			Tripods[tripod_up].stepForward(step_size);
			//pc.print_debug("First tripod put down legs for another step forward\n");
			std::swap(tripod_up, tripod_down);			// swap the roles of the Tripods
		}
		
		// Movement stops
		else{
			Tripods[tripod_up].finishStep();
			//pc.print_debug("First tripod finished step\n");
			Tripods[tripod_down].liftUp(ef_raise);
			//pc.print_debug("Second tripod lifted\n");
			Tripods[tripod_down].finishStep();
			//pc.print_debug("Second tripod finished step\n");
		}
	}
}
	


void Robot::rotate(double angle){
	/*for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].bodyRotate(angle);
	}

	writeAngles();
	*/
}

void Robot::raiseBody(double hraise){
	if(almost_equals(0.0, hraise)) return;
	Tripods[TRIPOD_LEFT].raiseBody(hraise);
	Tripods[TRIPOD_RIGHT].copyState(Tripods[TRIPOD_LEFT]);
}



/*

void Robot::writeAngles(){
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].writeAngles(distance);
	}
}
*/

/* ================================================= TESTING METHODS ================================================= */


void Robot::test(){
	//Tripods[TRIPOD_LEFT].liftUp(ef_raise);
}

void Robot::singleStepForwardTest(double coeff){
	if(coeff<0.0) coeff=0.1;
	if(coeff>1.0) coeff=1.0;

	double step_size = coeff*max_step_size;
	double ef_raise = 5.0;

	bool continue_movement = true;
	int tripod_up = TRIPOD_LEFT, tripod_down = TRIPOD_RIGHT;
		
	wait(wait_time);
	Tripods[tripod_up].liftUp(ef_raise);
	wait(wait_time);
	pc.print_debug("First tripod lifted\n");
	Tripods[tripod_down].bodyForward(step_size);
	wait(wait_time);
	pc.print_debug("Second tripod moved body forward\n");

	Tripods[tripod_up].stepForward(step_size);
	pc.print_debug("First tripod put down legs for another step forward\n");
}


// set the legs so that Pixhawk calibration can be performed 
void Robot::quadSetup(){
	Tripods[TRIPOD_LEFT].quadSetup();
	wait(wait_time);
	Tripods[TRIPOD_RIGHT].quadSetup();

	state = wkq::RS_QUAD_SETUP;
}			


/* ================================================= ROS COMMUNICATION ================================================= */
/*
void Robot::decodeInstruction(wkq_msgs::RPC::Request &req, wkq_msgs::RPC::Request &res){

	switch(RPC_Fn_t(req.fn)){
		case RPC_DEFAULT_POS:
			break;
		case RPC_CENTER:
			break;
		case RPC_STAND:
			break;
		case RPC_STAND_QUAD:
			break;
		case RPC_STRAIGHT_QUAD:	
			break;
		default:
	}

}
*/

/* ================================================= PRIVATE METHODS ================================================= */

bool Robot::noState(){
	if(state==wkq::RS_FLAT_QUAD) return true;
	if(state==wkq::RS_QUAD_SETUP) return true;
	return false;
}

// Needs to be dynamically adjusted after tests
double Robot::calcMaxStepSize(){
	return 5.0;
}

// Needs to be dynamically adjusted after tests
double Robot::calcMaxRotationAngle(){
	return (wkq::PI)/3;
}

