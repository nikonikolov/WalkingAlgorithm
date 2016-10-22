#include "Tripod.h"

const double Tripod::leg_lift = 5.0; 

Tripod::Tripod (int ID_front_knee, int ID_middle_knee, int ID_back_knee,
				DnxSerialBase* HipsKnees, DnxSerialBase* Arms, double height_in, const double robot_params[]) :
	Legs{	Leg(ID_front_knee, 	ID_front_knee+6, 	ID_front_knee+18, 	HipsKnees, Arms, height_in, robot_params),
			Leg(ID_middle_knee, ID_middle_knee+6, 	ID_middle_knee+18, 	HipsKnees, Arms, height_in, robot_params),
			Leg(ID_back_knee, 	ID_back_knee+6, 	ID_back_knee+18, 	HipsKnees, Arms, height_in, robot_params)
		} {}
Tripod::~Tripod(){}

/* ================================================= standING POSITIONS ================================================= */

void Tripod::defaultPos(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].defaultPos();
	}
	writeAngles();
}

void Tripod::center(){
	liftUp(leg_lift);
	for(int i=0; i<LEG_COUNT; i++){
		if(i==0) Legs[i].center();
		// All Legs have same defaultPoss so save some computations
		else Legs[i].copyState(Legs[0]);
		Legs[i].writeAngles();
	}
}

// input specifies whether current state has any standing meaning - if not, we should not keep backwards consistency as 
// invalid positions for the servos will be calculated on lifting the legs
void Tripod::stand(bool meaningless_state /*= false*/){
	// Robot already has made sure that body was lifted if necessary. Do it leg by leg to ensure you don't overload servos
	for(int i=0; i<LEG_COUNT; i++){
		if(!meaningless_state){
			Legs[i].liftUp(leg_lift);
			Legs[i].writeAngles();
		} 
		Legs[i].stand();
		Legs[i].writeAngles();
	}
}

void Tripod::standQuad(bool meaningless_state /*= false*/){
	// Robot already has made sure that body was lifted if necessary. Do it leg by leg to ensure you don't overload servos
	for(int i=0; i<LEG_COUNT; i++){
		if(!meaningless_state){
			Legs[i].liftUp(leg_lift);
			Legs[i].writeAngles();
		} 
		Legs[i].standQuad();
		Legs[i].writeAngles();
	}
}

void Tripod::flattenLegs(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].flatten();
	}
	writeHipKneeAngles();
}

void Tripod::flatQuad(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].standQuad();
		Legs[i].flatten();
	}
	writeAngles();
}


double Tripod::standing(){
	double height = Legs[0].get(STATE_VAR, HEIGHT);
	double height_stand = Legs[0].get(PARAM, TIBIA);
	if(!almost_equals(height, height_stand)) return 0.0;
	else return height_stand - height;
}

void Tripod::copyState(const Tripod& tripod_in){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].copyState(tripod_in.Legs[i]);
	}
	writeAngles();
}


/* ================================================= WALKING ALGORITHMS ================================================= */

void Tripod::bodyForward (double step_size){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].IKBodyForward(step_size);
	}
	writeAngles();
}

void Tripod::stepForward (double step_size){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].stepForward(step_size);
	}
	writeAngles();
}


void Tripod::bodyRotate(double angle){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].IKBodyRotate(angle);
	}
	writeAngles();
}

void Tripod::stepRotate(double angle){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].stepRotate(angle);
	}
	writeAngles();
}


void Tripod::raiseBody(double hraise){
	center();
	for(int i=0; i<LEG_COUNT; i++){
		if(i==0) 	Legs[i].raiseBody(hraise);
		else 		Legs[i].copyState(Legs[0]);
	}
	writeHipKneeAngles();
}


/* ================================================= RAISE AND LOWER ================================================= */


void Tripod::liftUp(double height_up){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].liftUp(height_up);
	}
	writeAngles();
}

void Tripod::lowerDown(double height_down){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].lowerDown(height_down);
	}
	writeAngles();
}

void Tripod::finishStep(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].finishStep();
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
