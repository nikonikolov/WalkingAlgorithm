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

/* -------------------------------------------- STANDING POSITIONS -------------------------------------------- */

void Leg::StandQuad(){
	state.LegStand();

	double MotorAngle = asin( (state.Params[COXA]+state.Params[FEMUR]-state.Params[KNEEMOTORDIST])/state.Params[DISTCENTER] * sin(wkq::PI/12) );
	// MotorAngle valid for a LEFT LEG servo facing downwards
	if 		(almost_equals(AngleOffset, wkq::PI/3)) state.ServoAngles[ARM] = -(MotorAngle + (wkq::PI/12));
	else if (almost_equals(AngleOffset, wkq::PI/2)) state.ServoAngles[ARM] = 0.0;
	else 											state.ServoAngles[ARM] = MotorAngle + (wkq::PI/12);
}



void Leg::Raise(const double& height){}

/* -------------------------------------------- MANUAL LEG MANIPULATIONS -------------------------------------------- */

/*
// input equals height required for the end effector
void Leg::LiftLegUp(const double& height){
	state.ServoAngles[KNEE] = wkq::PI - state.ServoAngles[KNEE] + acos(1 -pow(height,2) / state.Params[TIBIA_SQ]); 
}

// input equals current height of the end effector
void Leg::PutStraightDown(const double& height){
	state.ServoAngles[KNEE] = wkq::PI - state.ServoAngles[KNEE] - acos(1 -pow(height,2) / state.Params[TIBIA_SQ]); 
}

/*void Leg::PutDownInDefault(){
	for(int i=0; i<JOINT_COUNT-1; i++){
		state.ServoAngles[i] = DefaultAngles[i];
	}

}

// Input is the currently used step size
// NB: Untested and not confirmed for negative input. Might need to invert the value for angle only
void Leg::PutDownForStepForward(const double& dist){
	double distSQ = pow(dist,2);

	// Cosine Rule to find new HipToEndSQ
	double ArmGToEndNewSQ = DefaultVars[ARMGTOEND_SQ] + distSQ - 
								2 * dist * DefaultVars[ARMGTOEND] * cos(wkq::PI - AngleOffset) ;

	// Cosine Rule to find new Arm Servo Angle
	double ArmTmp = acos( (distSQ + DefaultVars[ARMGTOEND_SQ] - ArmGToEndNewSQ) / ( 2 * dist * DefaultVars[ARMGTOEND] ) );
	// Convert to actual angle for the servo
	state.ServoAngles[ARM] = wkq::PI - AngleOffset - ArmTmp; 			// NB: Valid for a LEFT servo facing down

	state.UpdateVar(ARMGTOEND_SQ, ArmGToEndNewSQ);			// Automatically changes robot state in software	
}



/* -------------------------------------------- WALK RELATED MANIPULATIONS -------------------------------------------- 



// Valid calculations for negative input as well in the current form
void Leg::IKForward(const double& dist){

	double distSQ = pow(dist,2);

	// Cosine Rule to find new HipToEndSQ
	double ArmGToEndNewSQ = StateVars[ARMGTOEND_SQ] + distSQ - 
								2 * dist * StateVars[ARMGTOEND] * cos( AngleOffset + state.ServoAngles[ARM] ) ;

	double ArmGToEndNew = sqrt(ArmGToEndNewSQ);

	// Cosine Rule to find new Arm Servo Angle
	double ArmTmp = acos( (distSQ + ArmGToEndNewSQ - StateVars[ARMGTOEND_SQ]) / ( 2 * dist * ArmGToEndNew ) );
	// Convert to actual angle for the servo
	state.ServoAngles[ARM] = wkq::PI - AngleOffset - ArmTmp; 			// NB: Valid for a LEFT servo facing down

	state.UpdateVar(ARMGTOEND, ArmGToEndNew, ArmGToEndNewSQ);		// Automatically changes robot state in software
}


void Leg::IKRotate(const double& angle){

	// Sine Rule to find rotation distance
	double RotDist = 2 * state.Params[DISTCENTER] * sin (std::abs(angle)/2);

	double RotDistSQ = pow(RotDist, 2);

	// Cosine Rule to find new ArmGToEndSQ
	double ArmGToEndNewSQ;
	if (angle>=0.0) ArmGToEndNewSQ = StateVars[ARMGTOEND_SQ] + RotDistSQ - 
									2 * RotDist * StateVars[ARMGTOEND] * cos( wkq::PI/2 + angle/2 - state.ServoAngles[ARM] ) ;
	else ArmGToEndNewSQ = StateVars[ARMGTOEND_SQ] + RotDistSQ - 
									2 * RotDist * StateVars[ARMGTOEND] * cos( wkq::PI/2 - angle/2 + state.ServoAngles[ARM] ) ;	

	double ArmGToEndNew = sqrt(ArmGToEndNewSQ);

	// Cosine Rule to find new Arm Servo Angle
	double ArmTmp = acos( (RotDistSQ + ArmGToEndNewSQ - StateVars[ARMGTOEND_SQ]) / ( 2 * RotDist * ArmGToEndNew ) );
	// Convert to actual angle for the servo - valid for a LEFT LEG servo facing down
	if (angle>=0.0)	state.ServoAngles[ARM] = wkq::PI/2 - ArmTmp + angle/2;  // - (- wkq::PI/2 + state.ServoAngles[ARM] - angle/2)
	else 			state.ServoAngles[ARM] = -wkq::PI/2 + ArmTmp + angle/2;	// - (  wkq::PI/2 - state.ServoAngles[ARM] - angle/2)

	state.UpdateVar(ARMGTOEND, ArmGToEndNew, ArmGToEndNewSQ);					// Automatically changes robot state in software		
}


// input equals height to be raised by; negative values also work
void Leg::LiftBodyUp(const double& hraise){

	state.UpdateVar(HEIGHT, (StateVars[HEIGHT] + hraise));			// HIPTOEND automatically gets updated

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
//}



/* -------------------------------------------- WRITING TO SERVOS -------------------------------------------- */


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


/* -------------------------------------------- GETTER AND COPY -------------------------------------------- */

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

