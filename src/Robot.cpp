#include "Robot.h"

const double Robot::wait_time_ = 3;
const double Robot::ef_raise_ = 5;

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

	// init to meaningless state
	state = wkq::RS_FLAT_QUAD;
	setState(state_in);

	printf("ROBOT done\n\r");
}

/*
Robot::Robot(Master* pixhawk_in, int baud_in, double height_in, const BodyParams& robot_params, wkq::RobotState_t state_in/* = wkq::RS_DEFAULT*) : 
#ifndef SIMULATION
	Robot(pixhawk_in, new SerialAX12(DnxHAL::Port_t(p9, p10), baud_in), new SerialXL320(DnxHAL::Port_t(p13, p14), baud_in), height_in, robot_params, state_in) {}
#else
	Robot(pixhawk_in, NULL, NULL, height_in, robot_params, state_in) {}
#endif
*/
Robot::~Robot(){}


/* ================================================= STATIC POSITIONS ================================================= */

void Robot::setState(wkq::RobotState_t state_in, bool wait_call/*=false*/){
	for(int i=0; i<TRIPOD_COUNT; i++){
		Tripods[i].setPosition(state_in);
		if(wait_call) wait(wait_time_);
	}
	state = state_in;
}

/*
void Robot::setState(wkq::RobotState_t state_in, void (Tripod::*tripod_action)(), bool wait_call/*=false*){
	for(int i=0; i<TRIPOD_COUNT; i++){
		(Tripods[i].*tripod_action)();
		if(wait_call) wait(wait_time_);
	}
	state = state_in;
}
*/
/* ================================================= WALK RELATED FUNCTIONALITY ================================================= */


void Robot::walkForward(double coeff){
	if(coeff<0.0) coeff=0.1;
	if(coeff>1.0) coeff=1.0;

	double step_size = coeff*max_step_size;

	bool continue_movement = true;
	int tripod_up = TRIPOD_LEFT, tripod_down = TRIPOD_RIGHT;

	//printf("Movement forward starts\n\r");
	
	// repeat until walkForward signal stops
	while(continue_movement){
		Tripods[tripod_up].liftUp(ef_raise_);
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
			Tripods[tripod_down].liftUp(ef_raise_);
			//printf("Second tripod lifted\n\r");
		wait(2);
			Tripods[tripod_down].finishStep();
			//printf("Second tripod finished step\n\r");
		}
	}
}
	

void Robot::walkForwardRectangularGait(double coeff){
	if(coeff<0.0) coeff=0.1;
	if(coeff>1.0) coeff=1.0;

	double step_size = coeff*max_step_size;

	bool continue_movement = true;
	int tripod_up = TRIPOD_RIGHT, tripod_down = TRIPOD_LEFT;

	//printf("Movement forward starts\n\r");
	
	bool first = true;
	// repeat until walkForward signal stops
	while(continue_movement){
		// Read Input and find out whether movement should go on
		if(!first) continue_movement = pixhawk->inputWalkForward();

		Tripods[tripod_up].liftUp(ef_raise_);
		wait(wait_time_);
		//printf("First tripod lifted\n\r");
		if(first || !continue_movement) 	Tripods[tripod_down].bodyForwardRectangularGait(step_size);
		else 		Tripods[tripod_down].bodyForwardRectangularGait(2*step_size);
		//printf("Second tripod moved body forward\n\r");

		break;
		
		wait(wait_time_);
		// Movement goes on
		if(continue_movement){
			if(first){
				Tripods[tripod_up].stepForwardRectangularGait(step_size);
				first = false;		
			}
			else	Tripods[tripod_up].stepForwardRectangularGait(2*step_size);
			//printf("First tripod put down legs for another step forward\n\r");
			std::swap(tripod_up, tripod_down);			// swap the roles of the Tripods
			wait(wait_time_);
		}
		
		// Movement stops
		else{
			Tripods[tripod_up].finishStepRectangularGait();
			//printf("First tripod finished step\n\r");
			wait(wait_time_);
			Tripods[tripod_down].liftUp(ef_raise_);
			//printf("Second tripod lifted\n\r");
			wait(wait_time_);
			Tripods[tripod_down].finishStepRectangularGait();
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
	//Tripods[TRIPOD_LEFT].liftUp(ef_raise_);
}

void Robot::testSingleTripodStand(){
	Tripods[TRIPOD_LEFT].liftUp(ef_raise_);
}

void Robot::singleStepForwardTest(double coeff){
	if(coeff<0.0) coeff=0.1;
	if(coeff>1.0) coeff=1.0;

	double step_size = coeff*max_step_size;

	bool continue_movement = true;
	int tripod_up = TRIPOD_LEFT, tripod_down = TRIPOD_RIGHT;
		
	wait(wait_time_);
	Tripods[tripod_up].liftUp(ef_raise_);
	wait(wait_time_);
	printf("First tripod lifted\n\r");
	Tripods[tripod_down].bodyForward(step_size);
	wait(wait_time_);
	printf("Second tripod moved body forward\n\r");

	Tripods[tripod_up].stepForward(step_size);
	printf("First tripod put down legs for another step forward\n\r");
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
	return 3.0;
}

// Needs to be dynamically adjusted after tests
double Robot::calcMaxRotationAngle(){
	return (wkq::PI)/3;
}

