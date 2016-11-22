#include "Robot.h"

//const double Robot::wait_time = 0.5;

Robot::Robot(Master* pixhawk_in, DnxHAL* dnx_hips_knees, DnxHAL* dnx_arms, double height_in, 
		const BodyParams& robot_params, wkq::RobotState_t state_in /*= wkq::RS_DEFAULT*/) :
	Tripods{
		Tripod(wkq::KNEE_LEFT_FRONT, wkq::KNEE_RIGHT_MIDDLE, wkq::KNEE_LEFT_BACK, dnx_hips_knees, dnx_arms, height_in, robot_params),
		Tripod(wkq::KNEE_RIGHT_FRONT, wkq::KNEE_LEFT_MIDDLE, wkq::KNEE_RIGHT_BACK, dnx_hips_knees, dnx_arms, height_in, robot_params)
	}, 
	pixhawk(pixhawk_in), state(state_in){
	
	printf("ROBOT start\n\r");
	
	// Calculate max size for movements
	max_step_size = 		calcMaxStepSize();
	max_rotation_angle = 	calcMaxRotationAngle();
	
	printf("ROBOT calculating state\n\r");

	state = wkq::RS_FLAT_QUAD;
	// Initialize servos according to state
	if 		(state_in == wkq::RS_DEFAULT) 				defaultPos();
	else if (state_in == wkq::RS_STANDING) 				stand();
	else if (state_in == wkq::RS_STANDING_QUAD) 		standQuad();
	else if (state_in == wkq::RS_FLAT_QUAD) 			flatQuad();

	printf("ROBOT done\n\r");
}

#ifndef SIMULATION
Robot::Robot(Master* pixhawk_in, int baud_in, double height_in, const BodyParams& robot_params, wkq::RobotState_t state_in/* = wkq::RS_DEFAULT*/) : 
	Robot(pixhawk_in, new SerialAX12(DnxHAL::Port_t(p9, p10), baud_in), new SerialXL320(DnxHAL::Port_t(p13, p14), baud_in), height_in, robot_params, state_in) {}
#endif

Robot::~Robot(){}


/* ================================================= STATIC POSITIONS ================================================= */

void Robot::defaultPos(){
	changeState(wkq::RS_DEFAULT, &Tripod::defaultPos, true);
}

void Robot::center(){
	changeState(wkq::RS_CENTERED, &Tripod::center, true);
}

void Robot::stand(){
	changeState(wkq::RS_STANDING, &Tripod::stand, true);
}

void Robot::standQuad(){
	changeState(wkq::RS_STANDING_QUAD, &Tripod::standQuad, true);
}

void Robot::flatQuad(){
	changeState(wkq::RS_FLAT_QUAD, &Tripod::flatQuad, true);
}

void Robot::changeState(wkq::RobotState_t state_in, void (Tripod::*tripod_action)(), bool wait_call/*=false*/){
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

	//printf("Movement forward starts\n\r");
	
	// repeat until walkForward signal stops
	while(continue_movement){
		Tripods[tripod_up].liftUp(ef_raise);
		wait(2);
		//printf("First tripod lifted\n\r");
		Tripods[tripod_down].bodyForward(step_size);
		//printf("Second tripod moved body forward\n\r");

		// Read Input and find out whether movement should go on
		continue_movement = pixhawk->inputWalkForward();

		wait(2);
		// Movement goes on
		if(continue_movement){
			Tripods[tripod_up].stepForward(step_size);
			//printf("First tripod put down legs for another step forward\n\r");
			std::swap(tripod_up, tripod_down);			// swap the roles of the Tripods
			wait(2);
		}
		
		// Movement stops
		else{
			Tripods[tripod_up].finishStep();
			//printf("First tripod finished step\n\r");
		wait(2);
			Tripods[tripod_down].liftUp(ef_raise);
			//printf("Second tripod lifted\n\r");
		wait(2);
			Tripods[tripod_down].finishStep();
			//printf("Second tripod finished step\n\r");
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
	if(wkq::compare_doubles(0.0, hraise)) return;
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

void Robot::testSingleTripodStand(){
	Tripods[TRIPOD_LEFT].liftUp(5);
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
	printf("First tripod lifted\n\r");
	Tripods[tripod_down].bodyForward(step_size);
	wait(wait_time);
	printf("Second tripod moved body forward\n\r");

	Tripods[tripod_up].stepForward(step_size);
	printf("First tripod put down legs for another step forward\n\r");
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

