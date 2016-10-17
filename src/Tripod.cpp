#include "Tripod.h"

const double Tripod::leg_lift = 5.0; 

Tripod::Tripod (int ID_front_knee, int ID_middle_knee, int ID_back_knee,
				DnxSerialBase* HipsKnees, DnxSerialBase* Arms, double height_in, const double robot_params[]) :
	Legs{	Leg(ID_front_knee, 	ID_front_knee+6, 	ID_front_knee+18, 	HipsKnees, Arms, height_in, robot_params),
			Leg(ID_middle_knee, ID_middle_knee+6, 	ID_middle_knee+18, 	HipsKnees, Arms, height_in, robot_params),
			Leg(ID_back_knee, 	ID_back_knee+6, 	ID_back_knee+18, 	HipsKnees, Arms, height_in, robot_params)
		} {}
Tripod::~Tripod(){}

/* ================================================= STANDING POSITIONS ================================================= */

void Tripod::Default(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].Default();
	}
	WriteAngles();
}

void Tripod::Center(){
	LiftUp(leg_lift);
	for(int i=0; i<LEG_COUNT; i++){
		if(i==0) Legs[i].Center();
		// All Legs have same defaults so save some computations
		else Legs[i].CopyState(Legs[0]);
		Legs[i].WriteAngles();
	}
}

// input specifies whether current state has any standing meaning - if not, we should not keep backwards consistency as 
// invalid positions for the servos will be calculated on lifting the legs
void Tripod::Stand(bool meaningless_state /*= false*/){
	// Robot already has made sure that body was lifted if necessary. Do it leg by leg to ensure you don't overload servos
	for(int i=0; i<LEG_COUNT; i++){
		if(!meaningless_state){
			Legs[i].LiftUp(leg_lift);
			Legs[i].WriteAngles();
		} 
		Legs[i].Stand();
		Legs[i].WriteAngles();
	}
}

void Tripod::StandQuad(bool meaningless_state /*= false*/){
	// Robot already has made sure that body was lifted if necessary. Do it leg by leg to ensure you don't overload servos
	for(int i=0; i<LEG_COUNT; i++){
		if(!meaningless_state){
			Legs[i].LiftUp(leg_lift);
			Legs[i].WriteAngles();
		} 
		Legs[i].StandQuad();
		Legs[i].WriteAngles();
	}
}

void Tripod::FlattenLegs(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].Flatten();
	}
	WriteHipKneeAngles();
}

double Tripod::Standing(){
	double height = Legs[0].Get(STATE_VAR, HEIGHT);
	double height_stand = Legs[0].Get(PARAM, TIBIA);
	if(!almost_equals(height, height_stand)) return 0.0;
	else return height_stand - height;
}

void Tripod::CopyState(const Tripod& tripod_in){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].CopyState(tripod_in.Legs[i]);
	}
	WriteAngles();
}


/* ================================================= WALKING ALGORITHMS ================================================= */

void Tripod::BodyForward (double step_size){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].IKBodyForward(step_size);
	}
	WriteAngles();
}

void Tripod::StepForward (double step_size){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].StepForward(step_size);
	}
	WriteAngles();
}


void Tripod::BodyRotate(double angle){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].IKBodyRotate(angle);
	}
	WriteAngles();
}

void Tripod::StepRotate(double angle){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].StepRotate(angle);
	}
	WriteAngles();
}


void Tripod::RaiseBody(double hraise){
	Center();
	for(int i=0; i<LEG_COUNT; i++){
		if(i==0) 	Legs[i].RaiseBody(hraise);
		else 		Legs[i].CopyState(Legs[0]);
	}
	WriteHipKneeAngles();
}


/* ================================================= RAISE AND LOWER ================================================= */


void Tripod::LiftUp(double height_up){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].LiftUp(height_up);
	}
	WriteAngles();
}

void Tripod::LowerDown(double height_down){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].LowerDown(height_down);
	}
	WriteAngles();
}

void Tripod::FinishStep(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].FinishStep();
	}
	WriteAngles();
}

/* ================================================= TESTING FUNCTIONS ================================================= */

void Tripod::QuadSetup(){
	Legs[0].StandQuad();
	Legs[1].QuadSetup();
	Legs[2].StandQuad();
	WriteAngles();
}


/* ================================================= PRIVATE FUNCTIONALITY ================================================= */

void Tripod::WriteHipKneeAngles(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].WriteJoint(HIP);
		Legs[i].WriteJoint(KNEE);
	}
}

void Tripod::WriteAngles(){
	for(int i=0; i<LEG_COUNT; i++){
		Legs[i].WriteAngles();
	}
}
