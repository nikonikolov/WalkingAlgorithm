#include "Leg.h"

/* ===================================================== PUBLIC METHODS ===================================================== */

// Tripod constructor does not write to angles, so Leg constrcutor is only responsible for calculating the proper defaults
Leg::Leg 	(const int& ID_knee, const int& ID_hip, const int& ID_arm, const int& ID_wing,
			DNXServo* HipsKnees, DNXServo* ArmsWings, const double& height_in) :
	
	state(height_in), 
	// Instantiate Joints
	Joints	{ 	ServoJoint(ID_knee, HipsKnees), 
				ServoJoint(ID_hip, HipsKnees), 
				ServoJoint(ID_arm, ArmsWings), 
				ServoJoint(ID_wing, ArmsWings)
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

/* ------------------------------------------------- STANDING POSITIONS ------------------------------------------------- */

void Leg::StandQuad(){
	state.LegStand();

	double MotorAngle = 
			asin( (state.Params[COXA]+state.Params[FEMUR]-state.Params[KNEEMOTORDIST])/state.Params[DISTCENTER] * sin(wkq::PI/12) );
	// MotorAngle valid for a LEFT LEG servo facing downwards
	if 		(almost_equals(AngleOffset, wkq::PI/3)) state.ServoAngles[ARM] = -(MotorAngle + (wkq::PI/12));
	else if (almost_equals(AngleOffset, wkq::PI/2)) state.ServoAngles[ARM] = 0.0;
	else 											state.ServoAngles[ARM] = MotorAngle + (wkq::PI/12);
}




/* ------------------------------------------------- RAISE AND LOWER ------------------------------------------------- */

// input equals height required for the end effector; Untested for all joints and negative input
void Leg::LiftUp(const double& height){
	state.ServoAngles[KNEE] = wkq::PI - state.ServoAngles[KNEE] + acos(1 - pow(height,2) / state.Params[TIBIA_SQ]); 
}

// input equals current height of the end effector
void Leg::LowerDown(const double& height){
	state.ServoAngles[KNEE] = wkq::PI - state.ServoAngles[KNEE] - acos(1 - pow(height,2) / state.Params[TIBIA_SQ]); 
}

/* 	
	Put End Effector down with ARM, HIP and KNEE centered. This is equivalent to properly terminating a movement forward
	Note that End Effector is still kept on the same line as HIP and KNEE are centered for this position
*/
void Leg::FinishStep(){
	state.ServoAngles[ARM] = 0.0;
	// Center HIP and KNEE and Compute new state
	state.CenterAngles();
}


/* ------------------------------------------------- WALKING ALGORITHMS ------------------------------------------------- */ 



// Valid calculations for negative input as well in the current form
void Leg::IKBodyForward(const double& step_size){

	double step_sizeSQ = pow(step_size,2);

	// Cosine Rule to find new HipToEndSQ
	double ArmGToEndNewSQ = state.Get(STATE_VAR, ARMGTOEND_SQ) + step_sizeSQ - 
								2 * step_size * state.Get(STATE_VAR, ARMGTOEND) * cos( AngleOffset + state.ServoAngles[ARM] ) ;

	double ArmGToEndNew = sqrt(ArmGToEndNewSQ);

	// Cosine Rule to find new Arm Servo Angle
	double ArmTmp = acos( (step_sizeSQ + ArmGToEndNewSQ - state.Get(STATE_VAR, ARMGTOEND_SQ)) / ( 2 * step_size * ArmGToEndNew ) );
	// Convert to actual angle for the servo
	state.ServoAngles[ARM] = wkq::PI - AngleOffset - ArmTmp; 			// NB: Valid for a LEFT servo facing down

	state.UpdateVar(ARMGTOEND, ArmGToEndNew, ArmGToEndNewSQ);		// Automatically changes robot state in software
}


// Input is the currently used step size
void Leg::StepForward(const double& step_size){

	// Distance from Robot center to end effector for a fully centered robot at this height
	double ef_center = state.Get(STATE_VAR, EFCENTER);

	// Create Point representing the new END EFFECTOR position
	double x_EFNew, y_EFNew;
	if(almost_equals(AngleOffset, wkq::PI/2)) {			// Middle Joint
		x_EFNew = ef_center - step_size/sqrt(3);
		y_EFNew = 0.0;
	}	
	else{
		x_EFNew = ef_center/2.0 + step_size/sqrt(3);
		y_EFNew = sqrt(3)*x_EFNew; 						// Front Joint

		if(almost_equals(AngleOffset, wkq::radians(150.0))) 	y_EFNew -= 4/(3*sqrt(3)*ef_center); 	// Back Joint
	}		
	Point EFNew(x_EFNew, y_EFNew);

	// Create Point representing current HIP position
	double hip_arg = wkq::PI/2 -AngleOffset;
	Point Hip(state.Params[DISTCENTER]*cos(hip_arg), state.Params[DISTCENTER]*sin(hip_arg));

	// Create Point representing the new HIP position
	Point HipNew(Hip);
	HipNew.translate_y(step_size*2);

	// Calculate new ARMGTOEND
	double ArmGToEndNewSQ = Hip.dist_sq(EFNew);

	// Cosine Rule to find new ARM angle
	double ArgTriangle = 
			acos( (Hip.dist_sq(HipNew) +  Hip.dist_sq(EFNew) - HipNew.dist_sq(EFNew)) / (2*Hip.dist(HipNew)*Hip.dist_sq(EFNew)) );
	//state.ServoAngles[ARM] = wkq::PI - AngleOffset - ArgTriangle;
	state.ServoAngles[ARM] = ArgTriangle - AngleOffset ;		// Unconfirmed that valid for all joints

	state.UpdateVar(ARMGTOEND_SQ, ArmGToEndNewSQ);				// Automatically changes HIPTOEND, HIP and KNEE	
}



void Leg::IKBodyRotate(const double& angle){

	// Sine Rule to find rotation distance
	double RotDist = 2 * state.Params[DISTCENTER] * sin (std::abs(angle)/2);

	double RotDistSQ = pow(RotDist, 2);

	// Cosine Rule to find new ArmGToEndSQ
	double ArmGToEndNewSQ;
	if (angle>=0.0) ArmGToEndNewSQ = state.Get(STATE_VAR, ARMGTOEND_SQ) + RotDistSQ - 
									2 * RotDist * state.Get(STATE_VAR, ARMGTOEND) * cos( wkq::PI/2 + angle/2 - state.ServoAngles[ARM] );
	else ArmGToEndNewSQ = state.Get(STATE_VAR, ARMGTOEND_SQ) + RotDistSQ - 
									2 * RotDist * state.Get(STATE_VAR, ARMGTOEND) * cos( wkq::PI/2 - angle/2 + state.ServoAngles[ARM] );	

	double ArmGToEndNew = sqrt(ArmGToEndNewSQ);

	// Cosine Rule to find new Arm Servo Angle
	double ArmTmp = acos( (RotDistSQ + ArmGToEndNewSQ - state.Get(STATE_VAR, ARMGTOEND_SQ)) / ( 2 * RotDist * ArmGToEndNew ) );
	// Convert to actual angle for the servo - valid for a LEFT LEG servo facing down
	if (angle>=0.0)	state.ServoAngles[ARM] = wkq::PI/2 - ArmTmp + angle/2;  // - (- wkq::PI/2 + state.ServoAngles[ARM] - angle/2)
	else 			state.ServoAngles[ARM] = -wkq::PI/2 + ArmTmp + angle/2;	// - (  wkq::PI/2 - state.ServoAngles[ARM] - angle/2)

	state.UpdateVar(ARMGTOEND, ArmGToEndNew, ArmGToEndNewSQ);					// Automatically changes robot state in software		
}


// Input is the currently used step size
void Leg::StepRotate(const double& step_size){
}


// input equals height to be raised by; negative values also work
void Leg::RaiseBody(const double& hraise){

	state.UpdateVar(HEIGHT, (state.Get(STATE_VAR, HEIGHT) + hraise));			// HIPTOEND, HIP and KNEE automatically get updated

/*
	if (StateVars[HIPTOEND] > (state.Params[TIBIA] + state.Params[FEMUR]) ){
		state.UpdateVar(HIPTOEND, (state.Params[TIBIA] + state.Params[FEMUR]) );
		UpdateHeight();
	}

	else if (StateVars[HIPTOEND] < (state.Params[TIBIA] - state.Params[FEMUR]) ){ 
		state.UpdateVar(HIPTOEND, 1.5 * (state.Params[TIBIA] - state.Params[FEMUR]) );	 // state.Params[TIBIA] - state.Params[FEMUR] is impossible position
		// Think of better way to calculate minimal position - use Hip limit angle
		UpdateHeight();
	}
*/
}

/* ------------------------------------------------- WRITING TO SERVOS ------------------------------------------------- */


// Write state.ServoAngles[] to physcial servos in order ARM, HIP, KNEE
void Leg::WriteAngles(){
	Joints[ARM].SetGoalPosition(LegRight * state.ServoAngles[ARM]);
	Joints[HIP].SetGoalPosition(LegRight * state.ServoAngles[HIP]);
	Joints[KNEE].SetGoalPosition(LegRight * state.ServoAngles[KNEE]);
}

// Write state.ServoAngles[] to physcial servos in order WING, ARM, HIP, KNEE
void Leg::WriteAllAngles(){
	Joints[WING].SetGoalPosition(LegRight * state.ServoAngles[WING]);
	WriteAngles();
}

// WRITE only a single angle contained in state.ServoAngles[] TO PHYSCIAL SERVO
void Leg::WriteJoint(const int& idx){
	Joints[idx].SetGoalPosition(LegRight * state.ServoAngles[idx]);
}


/* ------------------------------------------------- GETTER AND COPY ------------------------------------------------- */

double Leg::Get(const int& param_type, const int& idx) const{
	if 		(param_type==SERVO_ANGLE) 	return state.ServoAngles[idx];
	else if (param_type==STATE_VAR) 	return state.Get(STATE_VAR, idx);
	else if (param_type==DEFAULT_VAR) 	return state.Get(DEFAULT_VAR, idx);
	else if (param_type==DEFAULT_ANGLE) return state.Get(DEFAULT_ANGLE, idx);
	else if (param_type==PARAM) 		return state.Params[idx];
	else if (param_type==ANGLE_LIMIT) 	return state.AngleLimits[idx];
	else return 0.0;
}


void Leg::CopyState(const Leg& LegIn){
	state = LegIn.state;
}

