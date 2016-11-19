#include "Tripod.h"

const double Tripod::leg_lift = 5.0; 

Tripod::Tripod (int ID_front_knee, int ID_middle_knee, int ID_back_knee,
				DnxHAL* dnx_hips_knees, DnxHAL* dnx_arms, double height_in, const BodyParams& robot_params) :
	legs{	Leg(ID_front_knee, 	ID_front_knee+6, 	ID_front_knee+18, 	dnx_hips_knees, dnx_arms, height_in, robot_params),
			Leg(ID_middle_knee, ID_middle_knee+6, 	ID_middle_knee+18, 	dnx_hips_knees, dnx_arms, height_in, robot_params),
			Leg(ID_back_knee, 	ID_back_knee+6, 	ID_back_knee+18, 	dnx_hips_knees, dnx_arms, height_in, robot_params)
		} {}
Tripod::~Tripod(){}


/* ================================================= STATIC POSITIONS ================================================= */

void Tripod::defaultPos(){
	setPosition(&Leg::defaultPos);
}

void Tripod::center(){
	liftUp(leg_lift);
	for(int i=0; i<LEG_COUNT; i++){
		if(i==0) legs[i].center();
		// All legs have same center positions so save computations by copying states
		else legs[i].copyState(legs[0]);
		legs[i].writeAngles();
	}
}

void Tripod::stand(){
	setPosition(&Leg::stand);
}

void Tripod::standQuad(){
	setPosition(&Leg::standQuad);
}

void Tripod::flatQuad(){
	setPosition(&Leg::flatQuad);
}


/* ================================================= WALKING MOVEMENTS ================================================= */

void Tripod::bodyForward (double step_size){
	makeMovement(&Leg::IKBodyForward, step_size, "MOVING BODY FORWARD\n\r");
}

void Tripod::stepForward (double step_size){
	makeMovement(&Leg::stepForward, step_size, "TRIPOD STEPPING FORWARD\n\r");
}

void Tripod::bodyRotate(double angle){
	makeMovement(&Leg::IKBodyRotate, angle);
}

void Tripod::stepRotate(double angle){
	makeMovement(&Leg::stepRotate, angle);
}

void Tripod::liftUp(double height_up){
	makeMovement(&Leg::liftUp, height_up, "LIFTING TRIPOD\n\r");
}

void Tripod::lowerDown(double height_down){
	makeMovement(&Leg::lowerDown, height_down, "LOWERING TRIPOD\n\r");
}

void Tripod::finishStep(){
	setPosition(&Leg::finishStep, "TRIPOD FINISHING STEP\n\r");
}


/* ================================================= RAISE AND LOWER ================================================= */

void Tripod::raiseBody(double hraise){
	center();
	for(int i=0; i<LEG_COUNT; i++){
		if(i==0) 	legs[i].raiseBody(hraise);
		else 		legs[i].copyState(legs[0]);
	}
	writeAngles();
}

/*
double Tripod::standing(){
	double height = legs[0].get(STATE_VAR, HEIGHT);
	double height_stand = legs[0].get(PARAM, TIBIA);
	if(!wkq::compare_doubles(height, height_stand)) return 0.0;
	else return height_stand - height;
}
*/

void Tripod::copyState(const Tripod& tripod_in){
	for(int i=0; i<LEG_COUNT; i++){
		legs[i].copyState(tripod_in.legs[i]);
	}
	writeAngles();
}


/* ================================================= TESTING FUNCTIONS ================================================= */

void Tripod::quadSetup(){
	legs[0].standQuad();
	legs[1].quadSetup();
	legs[2].standQuad();
	writeAngles();
}


/* ================================================= PRIVATE FUNCTIONALITY ================================================= */


void Tripod::makeMovement(void (Leg::*leg_action)(double), double arg, const string debug_msg /*=""*/ ){
	if(debug_msg!="") printf("%s\n\r", debug_msg.c_str());
	
	for(int i=0; i<LEG_COUNT; i++){
		(legs[i].*leg_action)(arg);
	}
	writeAngles();
}

void Tripod::setPosition(void (Leg::*leg_action)(), const string debug_msg /*=""*/){
	if(debug_msg!="") printf("%s\n\r", debug_msg.c_str());

	for(int i=0; i<LEG_COUNT; i++){
		(legs[i].*leg_action)();
	}
	writeAngles();
}

void Tripod::writeAngles(){
	for(int i=0; i<LEG_COUNT; i++){
		legs[i].writeAngles();
	}
}


