#include "Tripod.h"

const double Tripod::leg_lift = 10.0; 

#ifdef DOF3
Tripod::Tripod (int ID_front_knee, int ID_middle_knee, int ID_back_knee, unordered_map<int, DnxHAL*>& servo_map, double height_in, const BodyParams& robot_params) :
	legs{	Leg(ID_front_knee, 	ID_front_knee+6, 	ID_front_knee+18, 	servo_map, height_in, robot_params),
			Leg(ID_middle_knee, ID_middle_knee+6, 	ID_middle_knee+18, 	servo_map, height_in, robot_params),
			Leg(ID_back_knee, 	ID_back_knee+6, 	ID_back_knee+18, 	servo_map, height_in, robot_params)
		} {}
#else
Tripod::Tripod (int ID_front_knee, int ID_middle_knee, int ID_back_knee, unordered_map<int, DnxHAL*>& servo_map, double height_in, const BodyParams& robot_params) :
	legs{	Leg(ID_front_knee, 	ID_front_knee+6, 	servo_map, height_in, robot_params),
			Leg(ID_middle_knee, ID_middle_knee+6, 	servo_map, height_in, robot_params),
			Leg(ID_back_knee, 	ID_back_knee+6, 	servo_map, height_in, robot_params)
		} {}
#endif		
Tripod::~Tripod(){}


/* ================================================= STATIC POSITIONS ================================================= */

void Tripod::setPosition(wkq::RobotState_t robot_state){
	for(int i=0; i<LEG_COUNT; i++){
		legs[i].setPosition(robot_state);
	}
	writeAngles();
}


void Tripod::center(){
	liftUp(leg_lift);
	for(int i=0; i<LEG_COUNT; i++){
		if(i==0) legs[i].setPosition(wkq::RS_CENTERED);
		// All legs have same center positions so save computations by copying states
		else legs[i].copyState(legs[0]);
		legs[i].writeAngles();
	}
}

/* ================================================= WALKING MOVEMENTS ================================================= */

void Tripod::bodyForward(double step_size){
	makeMovement(&Leg::bodyForward, step_size, "Tripod: bodyForward");
}

void Tripod::stepForward(double step_size){
	makeMovement(&Leg::stepForward, step_size, "Tripod: stepForward");
}

void Tripod::bodyForwardRectangularGait(double step_size){
	makeMovement(&Leg::bodyForwardRectangularGait, step_size, "Tripod: bodyForwardRectangularGait");
}

void Tripod::stepForwardRectangularGait(double step_size){
	makeMovement(&Leg::stepForwardRectangularGait, step_size, "Tripod: stepForwardRectangularGait");
}

void Tripod::bodyRotate(double angle){
	makeMovement(&Leg::bodyRotate, angle, "Tripod: bodyRotate");
}

void Tripod::stepRotate(double angle){
	makeMovement(&Leg::stepRotate, angle, "Tripod: stepRotate");
}

void Tripod::liftUp(double height_up){
	makeMovement(&Leg::liftUp, height_up, "Tripod: liftUp");
}

void Tripod::lowerDown(double height_down){
	makeMovement(&Leg::lowerDown, height_down, "Tripod: lowerDown");
}

void Tripod::finishStep(){
	//setPosition(&Leg::finishStep, "Tripod: finishStep\n\r");
	setPosition(wkq::RS_DEFAULT);
}

void Tripod::finishStepRectangularGait(){
	setPosition(wkq::RS_RECTANGULAR);
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


void Tripod::copyState(const Tripod& tripod_in){
	for(int i=0; i<LEG_COUNT; i++){
		legs[i].copyState(tripod_in.legs[i]);
	}
	writeAngles();
}



/* ================================================= PRIVATE FUNCTIONALITY ================================================= */


void Tripod::makeMovement(void (Leg::*leg_action)(double), double arg, const string debug_msg /*=""*/ ){
	if(debug_msg!="" && debug_) printf("\n\r%s\n\r", debug_msg.c_str());
	
	for(int i=0; i<LEG_COUNT; i++){
		(legs[i].*leg_action)(arg);
	}
	writeAngles();
}

void Tripod::writeAngles(){
	for(int i=0; i<LEG_COUNT; i++){
		legs[i].writeAngles();
	}
}


