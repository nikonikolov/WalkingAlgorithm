#include "Tripod.h"

const double Tripod::leg_lift = 5.0; 

Tripod::Tripod (int ID_front_knee, int ID_middle_knee, int ID_back_knee,
				DnxSerialBase* HipsKnees, DnxSerialBase* Arms, double height_in, BodyParams robot_params) :
	Legs{	Leg(ID_front_knee, 	ID_front_knee+6, 	ID_front_knee+18, 	HipsKnees, Arms, height_in, robot_params),
			Leg(ID_middle_knee, ID_middle_knee+6, 	ID_middle_knee+18, 	HipsKnees, Arms, height_in, robot_params),
			Leg(ID_back_knee, 	ID_back_knee+6, 	ID_back_knee+18, 	HipsKnees, Arms, height_in, robot_params)
		} {}
Tripod::~Tripod(){}


/* ================================================= STATIC POSITIONS ================================================= */

void Tripod::defaultPos(){
	setPosition(&Leg::defaultPos);
}

void Tripod::center(){
	liftUp(leg_lift);
	for(int i=0; i<LEG_COUNT; i++){
		if(i==0) Legs[i].center();
		// All Legs have same center positions so save computations by copying states
		else Legs[i].copyState(Legs[0]);
		Legs[i].writeAngles();
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
		if(i==0) 	Legs[i].raiseBody(hraise);
		else 		Legs[i].copyState(Legs[0]);
	}
	writeHipKneeAngles();
}

double Tripod::standing(){
	double height = Legs[0].get(STATE_VAR, HEIGHT);
	double height_stand = Legs[0].get(PARAM, TIBIA);
	if(!compare_doubles(height, height_stand)) return 0.0;
	else return height_stand - height;
}

void Tripod::copyState(const Tripod& tripod_in){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].copyState(tripod_in.Legs[i]);
	}
	writeAngles();
}


/* ================================================= TESTING FUNCTIONS ================================================= */

void Tripod::quadSetup(){
	Legs[0].standQuad();
	Legs[1].quadSetup();
	Legs[2].standQuad();
	writeAngles();
}


/* ================================================= PRIVATE FUNCTIONALITY ================================================= */


void Tripod::makeMovement(void (Leg::*leg_action)(double), double arg, const string debug_msg /*=""*/ ){
	if(debug_msg!="") pc.print_debug(debug_msg);
	
	for(int i=0; i<LEG_COUNT; i++){
		(Legs[i].*leg_action)(arg);
	}
	writeAngles();
}

void Tripod::setPosition(void (Leg::*leg_action)(), const string debug_msg /*=""*/){
	if(debug_msg!="") pc.print_debug(debug_msg);

	for(int i=0; i<LEG_COUNT; i++){
		(Legs[i].*leg_action)();
	}
	writeAngles();
}

void Tripod::writeHipKneeAngles(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].writeJoint(HIP);
		Legs[i].writeJoint(KNEE);
	}
}

void Tripod::writeAngles(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].writeAngles();
	}
}


