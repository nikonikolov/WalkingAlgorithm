#include "Leg.h"

/* ===================================================== PUBLIC METHODS ===================================================== */

// Tripod constructor does not write to angles, so Leg constrcutor is only responsible for calculating the proper defaultPos
Leg::Leg 	(int ID_knee, int ID_hip, int ID_arm,
			DnxSerialBase* HipsKnees, DnxSerialBase* Arms, double height_in, const double robot_params[]) :
	
	state(height_in, robot_params), 
	// Instantiate Joints
	Joints	{ 	ServoJoint(ID_knee, HipsKnees), 
				ServoJoint(ID_hip, HipsKnees), 
				ServoJoint(ID_arm, Arms), 
			} {
	// Check if RIGHT or LEFT
	if(ID_knee>wkq::knee_left_back) 	LegRight=-1.0;
	else 								LegRight=1.0;

	// Calculate AngleOffset
	if(ID_knee == wkq::knee_left_front 	|| ID_knee == wkq::knee_right_front) 	AngleOffset = wkq::radians(30);
	if(ID_knee == wkq::knee_left_middle || ID_knee == wkq::knee_right_middle) 	AngleOffset = wkq::radians(30+60);
	if(ID_knee == wkq::knee_left_back 	|| ID_knee == wkq::knee_right_back) 	AngleOffset = wkq::radians(30+120);
}

Leg::~Leg(){}

/* ------------------------------------------------- standING POSITIONS ------------------------------------------------- */

void Leg::standQuad(){
	state.legStand();
	if(almost_equals(AngleOffset, wkq::PI/2)) return;

	double MotorAngle = 
			asin( state.Params[DISTcenter] * sin(wkq::PI/12)/(state.Params[COXA]+state.Params[FEMUR]-state.Params[KNEEMOTORDIST]) );
	// MotorAngle valid for a LEFT LEG servo facing downwards
	if 		(almost_equals(AngleOffset, wkq::PI/6)) state.ServoAngles[ARM] = MotorAngle + (wkq::PI/12);
	else 											state.ServoAngles[ARM] = -(MotorAngle + (wkq::PI/12));
}




/* ------------------------------------------------- RAISE AND LOWER ------------------------------------------------- */

// input equals height required for the end effector; Untested for all joints and negative input
void Leg::liftUp(double height){
	state.ServoAngles[KNEE] = wkq::PI - state.ServoAngles[KNEE] + acos(1 - pow(height,2) / state.Params[TIBIA_SQ]); 
}

// input equals current height of the end effector
void Leg::lowerDown(double height){
	state.ServoAngles[KNEE] = wkq::PI - state.ServoAngles[KNEE] - acos(1 - pow(height,2) / state.Params[TIBIA_SQ]); 
}

/* 	
	Put End Effector down with ARM, HIP and KNEE centered. This is equivalent to properly terminating a movement forward
	Note that End Effector is still kept on the same line as HIP and KNEE are centered for this position
*/
void Leg::finishStep(){
	state.ServoAngles[ARM] = 0.0;
	// center HIP and KNEE and Compute new state
	state.centerAngles();
}


/* ------------------------------------------------- WALKING ALGORITHMS ------------------------------------------------- */ 



// Valid calculations for negative input as well in the current form
void Leg::IKBodyForward(double step_size){

	double step_sizeSQ = pow(step_size,2);

	// Cosine Rule to find new HipToEndSQ
	double ArmGToEndNewSQ = state.get(STATE_VAR, ARMGTOEND_SQ) + step_sizeSQ - 
								2 * step_size * state.get(STATE_VAR, ARMGTOEND) * cos( AngleOffset + state.ServoAngles[ARM] ) ;

	double ArmGToEndNew = sqrt(ArmGToEndNewSQ);

	// Cosine Rule to find new Arm Servo Angle
	double ArmTmp = acos( (step_sizeSQ + ArmGToEndNewSQ - state.get(STATE_VAR, ARMGTOEND_SQ)) / ( 2 * step_size * ArmGToEndNew ) );
	// Convert to actual angle for the servo
	state.ServoAngles[ARM] = wkq::PI - AngleOffset - ArmTmp; 			// NB: Valid for a LEFT servo facing down

	state.updateVar(ARMGTOEND, ArmGToEndNew, ArmGToEndNewSQ);		// Automatically changes robot state in software
}


// Input is the currently used step size
void Leg::stepForward(double step_size){

	// Distance from Robot center to end effector for a fully centered robot at this height
	double ef_center = state.get(STATE_VAR, EFcenter);

	// Create Point representing the new END EFFECTOR position
	double x_EFNew, y_EFNew;
	if(almost_equals(AngleOffset, wkq::PI/2)){			// Middle Joint
		//x_EFNew = ef_center - step_size/sqrt(3); - change when you have prev and current step size as inputs
		x_EFNew = ef_center - step_size/sqrt(3);
		y_EFNew = 0.0;
	}	
	else{
		x_EFNew = ef_center/2.0 + step_size/sqrt(3);
		y_EFNew = sqrt(3)*x_EFNew; 						// Front Joint

		if(almost_equals(AngleOffset, wkq::radians(150.0))) 	y_EFNew -= sqrt(3)*ef_center; 			// Back Joint
	}		
	wkq::Point EFNew(x_EFNew, y_EFNew);

	// Create Point representing current HIP position
	double hip_arg = wkq::PI/2 -AngleOffset;
	wkq::Point Hip(state.Params[DISTcenter]*cos(hip_arg), state.Params[DISTcenter]*sin(hip_arg));

	// Create Point representing the new HIP position
	wkq::Point HipNew(Hip);
	HipNew.translate_y(step_size*2);

	// Calculate new ARMGTOEND
	double ArmGToEndNewSQ = Hip.dist_sq(EFNew);

	// Cosine Rule to find new ARM angle
	double ArgTriangle = 
			acos( (Hip.dist_sq(HipNew) +  Hip.dist_sq(EFNew) - HipNew.dist_sq(EFNew)) / (2*Hip.dist(HipNew)*Hip.dist_sq(EFNew)) );
	//state.ServoAngles[ARM] = wkq::PI - AngleOffset - ArgTriangle;
	state.ServoAngles[ARM] = ArgTriangle - AngleOffset ;		// Unconfirmed that valid for all joints

	state.updateVar(ARMGTOEND_SQ, ArmGToEndNewSQ);				// Automatically changes HIPTOEND, HIP and KNEE	
}



void Leg::IKBodyRotate(double angle){

	// Sine Rule to find rotation distance
	double RotDist = 2 * state.Params[DISTcenter] * sin (fabs(angle)/2);

	double RotDistSQ = pow(RotDist, 2);

	// Cosine Rule to find new ArmGToEndSQ
	double ArmGToEndNewSQ;
	if (angle>=0.0) ArmGToEndNewSQ = state.get(STATE_VAR, ARMGTOEND_SQ) + RotDistSQ - 
									2 * RotDist * state.get(STATE_VAR, ARMGTOEND) * cos( wkq::PI/2 + angle/2 - state.ServoAngles[ARM] );
	else ArmGToEndNewSQ = state.get(STATE_VAR, ARMGTOEND_SQ) + RotDistSQ - 
									2 * RotDist * state.get(STATE_VAR, ARMGTOEND) * cos( wkq::PI/2 - angle/2 + state.ServoAngles[ARM] );	

	double ArmGToEndNew = sqrt(ArmGToEndNewSQ);

	// Cosine Rule to find new Arm Servo Angle
	double ArmTmp = acos( (RotDistSQ + ArmGToEndNewSQ - state.get(STATE_VAR, ARMGTOEND_SQ)) / ( 2 * RotDist * ArmGToEndNew ) );
	// Convert to actual angle for the servo - valid for a LEFT LEG servo facing down
	if (angle>=0.0)	state.ServoAngles[ARM] = wkq::PI/2 - ArmTmp + angle/2;  // - (- wkq::PI/2 + state.ServoAngles[ARM] - angle/2)
	else 			state.ServoAngles[ARM] = -wkq::PI/2 + ArmTmp + angle/2;	// - (  wkq::PI/2 - state.ServoAngles[ARM] - angle/2)

	state.updateVar(ARMGTOEND, ArmGToEndNew, ArmGToEndNewSQ);					// Automatically changes robot state in software		
}


// Input is the currently used step size
void Leg::stepRotate(double step_size){
}


// input equals height to be raised by; negative values also work
void Leg::raiseBody(double hraise){

	state.updateVar(HEIGHT, (state.get(STATE_VAR, HEIGHT) + hraise));			// HIPTOEND, HIP and KNEE automatically get updated

/*
	if (StateVars[HIPTOEND] > (state.Params[TIBIA] + state.Params[FEMUR]) ){
		state.updateVar(HIPTOEND, (state.Params[TIBIA] + state.Params[FEMUR]) );
		UpdateHeight();
	}

	else if (StateVars[HIPTOEND] < (state.Params[TIBIA] - state.Params[FEMUR]) ){ 
		state.updateVar(HIPTOEND, 1.5 * (state.Params[TIBIA] - state.Params[FEMUR]) );	 // state.Params[TIBIA] - state.Params[FEMUR] is impossible position
		// Think of better way to calculate minimal position - use Hip limit angle
		UpdateHeight();
	}
*/
}

/* ------------------------------------------------- TESTING FUNCTIONS ------------------------------------------------- */

void Leg::quadSetup(){
	state.clear();
	state.ServoAngles[ARM] = 0;
	state.ServoAngles[HIP] = wkq::radians(50);
	state.ServoAngles[KNEE] = wkq::PI/2;
}


/* ------------------------------------------------- WRITING TO SERVOS ------------------------------------------------- */


// Write state.ServoAngles[] to physcial servos in order ARM, HIP, KNEE
void Leg::writeAngles(){
	Joints[ARM].SetGoalPosition(LegRight * state.ServoAngles[ARM]);
	Joints[HIP].SetGoalPosition(LegRight * state.ServoAngles[HIP]);
	Joints[KNEE].SetGoalPosition(LegRight * state.ServoAngles[KNEE]);
}

// WRITE only a single angle contained in state.ServoAngles[] TO PHYSCIAL SERVO
void Leg::writeJoint(int idx){
	Joints[idx].SetGoalPosition(LegRight * state.ServoAngles[idx]);
}


/* ------------------------------------------------- GETTER AND COPY ------------------------------------------------- */

double Leg::get(int param_type, int idx) const{
	if 		(param_type==SERVO_ANGLE) 	return state.ServoAngles[idx];
	else if (param_type==STATE_VAR) 	return state.get(STATE_VAR, idx);
	else if (param_type==defaultPos_VAR) 	return state.get(defaultPos_VAR, idx);
	else if (param_type==defaultPos_ANGLE) return state.get(defaultPos_ANGLE, idx);
	else if (param_type==PARAM) 		return state.Params[idx];
	else if (param_type==ANGLE_LIMIT) 	return state.AngleLimits[idx];
	else return 0.0;
}


void Leg::copyState(const Leg& LegIn){
	state = LegIn.state;
}

