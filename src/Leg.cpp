#include "Leg.h"


/* ************************************************* PUBLIC METHODS *************************************************** */

const double Leg::Params[PARAM_COUNT] = { 10.95, 2.65, 17.5, 30, 12
										pow(10.95,2), pow(2.65,2), pow(17.5,2), pow(30,2), pow(12,2)	};

// Values are valid for a left leg and correspond to values that are written to the servo
const double Leg::AngleLimits[JOINT_COUNT*2] = { 	wkquad::radians(0), wkquad::radians(150),				/* KNEE */ 
													wkquad::radians(-(90-20)), wkquad::radians(90-20),		/* HIP - JUST A GUESS */
													wkquad::radians(-(90-20)), wkquad::radians(90-20),		/* ARM - JUST A GUESS */
													wkquad::radians(-150), wkquad::radians(150)				/* WING */
												}


Leg::Leg 	(const int& ID_knee, const int& ID_hip, const int& ID_arm, const int& ID_wing,
			DNXServo* HipKnees, DNXServo* ArmsWings, const double& height_in) :
			
	// Instantiate Legs
	Joints[KNEE](ID_knee, HipKnees), Joints[HIP](ID_hip, HipKnees), Joints[ARM](ID_arm, ArmsWings), Joints[WING](ID_wing, ArmsWings){

	// Check if RIGHT or LEFT
	if(ID_knee>wkquad::knee_left_back) 	LegRight=-1.0;
	else 								LegRight=1.0;

	// Set ServoAngles. Order of HIP and KNEE calculation is important!
	ServoAngles[HIP] = CenterHip();
	ServoAngles[KNEE] = CenterKnee(height_in);
	ServoAngles[ARM] = 0.0;
	ServoAngles[WING] = 0.0;

	// Calculate AngleOffset
	if(ID_knee == wkquad::knee_left_front 	|| ID_knee == wkquad::knee_right_front) 	AngleOffset = wkquad::radians(30);
	if(ID_knee == wkquad::knee_left_middle 	|| ID_knee == wkquad::knee_right_middle) 	AngleOffset = wkquad::radians(30+60);
	if(ID_knee == wkquad::knee_left_back 	|| ID_knee == wkquad::knee_right_back) 		AngleOffset = wkquad::radians(30+120);

	// Initialize Robot state
	InitializeState(height_in);

	// Find Square Values
	for(int i=0; i<VAR_STEP; i++){
		StateVars[i+VAR_STEP] = pow(StateVars[i],2);
	}

	// Save Default Variables
	for(int i=0; i<VAR_COUNT; i++){
		DefaultVars[i] = StateVars[i];
	}

	// Save Default Angles
	for(int i=0; i<JOINT_COUNT; i++){
		DefaultAngles[i] = ServoAngles[i];
	}

}

Leg::~Leg(){}


/* -------------------------------------------- MANUAL LEG MANIPULATIONS -------------------------------------------- */


void Leg::Reset(){
	// Change State Variables
	for(int i=0; i<VAR_COUNT; i++){
		StateVars[i] = DefaultVars[i];
	}

	// Change State Angles
	for(int i=0; i<JOINT_COUNT; i++){
		ServoAngles[i] = DefaultAngles[i];
	}
}

// Centers leg for its current height. Assumes already lifted leg to allow for stability on writing the angles
void Leg::Center(){
	ServoAngles[ARM] = DefaultAngles[ARM];
	CenterHip();
	CenterKnee();
	InitializeState();
}	


void Leg::CopyState(const Leg& LegIn){
	for(int i=0; i<JOINT_COUNT; i++){
		ServoAngles[i] = LegIn.ServoAngles[i];
	}
	for(int i=0; i<VAR_COUNT; i++){
		StateVars[i] = LegIn.StateVars[i];
	}
}


int Leg::Straighten(){	
	return SetLegPosition(0.0, 0.0, 0.0, 0.0);	
}

// input equals height required for the end effector
void Leg::LiftLegUp(const double& height){
	ServoAngles[KNEE] = wkquad::PI - ServoAngles[KNEE] + acos(1 -pow(height,2) / Params[TIBIA_SQ]); 
}

// input equals current height of the end effector
void Leg::PutStraightDown(const double& height){
	ServoAngles[KNEE] = wkquad::PI - ServoAngles[KNEE] - acos(1 -pow(height,2) / Params[TIBIA_SQ]); 
}

/*void Leg::PutDownInDefault(){
	for(int i=0; i<JOINT_COUNT-1; i++){
		ServoAngles[i] = DefaultAngles[i];
	}

}*/

// Input is the currently used step size
// NB: Untested and not confirmed for negative input. Might need to invert the value for angle only
void Leg::PutDownForStepForward(const double& dist){
	double distSQ = pow(dist,2);

	// Cosine Rule to find new HipToEndSQ
	double ArmGToEndNewSQ = DefaultVars[ARMGTOEND_SQ] + distSQ - 
								2 * dist * DefaultVars[ARMGTOEND] * cos(wkquad::PI - AngleOffset) ;

	// Cosine Rule to find new Arm Servo Angle
	double ArmTmp = acos( (distSQ + DefaultVars[ARMGTOEND_SQ] - ArmGToEndNewSQ) / ( 2 * dist * DefaultVars[ARMGTOEND] ) );
	// Convert to actual angle for the servo
	ServoAngles[ARM] = wkquad::PI - AngleOffset - ArmTmp; 			// NB: Valid for a LEFT servo facing down

	UpdateStateVars(ARMGTOEND_SQ, ArmGToEndNewSQ);			// Automatically changes robot state in software	
}


/* -------------------------------------------- END MANUAL LEG MANIPULATIONS -------------------------------------------- */


/* -------------------------------------------- WALK RELATED MANIPULATIONS -------------------------------------------- */



// Valid calculations for negative input as well in the current form
void Leg::IKForward(const double& dist){

	double distSQ = pow(dist,2);

	// Cosine Rule to find new HipToEndSQ
	double ArmGToEndNewSQ = StateVars[ARMGTOEND_SQ] + distSQ - 
								2 * dist * StateVars[ARMGTOEND] * cos( AngleOffset + ServoAngles[ARM] ) ;

	double ArmGToEndNew = sqrt(ArmGToEndNewSQ);

	// Cosine Rule to find new Arm Servo Angle
	double ArmTmp = acos( (distSQ + ArmGToEndNewSQ - StateVars[ARMGTOEND_SQ]) / ( 2 * dist * ArmGToEndNew ) );
	// Convert to actual angle for the servo
	ServoAngles[ARM] = wkquad::PI - AngleOffset - ArmTmp; 			// NB: Valid for a LEFT servo facing down

	UpdateStateVars(ARMGTOEND, ArmGToEndNew, ArmGToEndNewSQ);		// Automatically changes robot state in software
}


void Leg::IKRotate(const double& angle){

	// Sine Rule to find rotation distance
	double RotDist = 2 * Params[DISTCENTER] * sin (std::abs(angle)/2);

	double RotDistSQ = pow(RotDist, 2);

	// Cosine Rule to find new ArmGToEndSQ
	double ArmGToEndNewSQ;
	if (angle>=0.0) ArmGToEndNewSQ = StateVars[ARMGTOEND_SQ] + RotDistSQ - 
									2 * RotDist * StateVars[ARMGTOEND] * cos( wkquad::PI/2 + angle/2 - ServoAngles[ARM] ) ;
	else ArmGToEndNewSQ = StateVars[ARMGTOEND_SQ] + RotDistSQ - 
									2 * RotDist * StateVars[ARMGTOEND] * cos( wkquad::PI/2 - angle/2 + ServoAngles[ARM] ) ;	

	double ArmGToEndNew = sqrt(ArmGToEndNewSQ);

	// Cosine Rule to find new Arm Servo Angle
	double ArmTmp = acos( (RotDistSQ + ArmGToEndNewSQ - StateVars[ARMGTOEND_SQ]) / ( 2 * RotDist * ArmGToEndNew ) );
	// Convert to actual angle for the servo - valid for a LEFT LEG servo facing down
	if (angle>=0.0)	ServoAngles[ARM] = wkquad::PI/2 - ArmTmp + angle/2;  // - (- wkquad::PI/2 + ServoAngles[ARM] - angle/2)
	else 			ServoAngles[ARM] = -wkquad::PI/2 + ArmTmp + angle/2;	// - (  wkquad::PI/2 - ServoAngles[ARM] - angle/2)

	UpdateStateVars(ARMGTOEND, ArmGToEndNew, ArmGToEndNewSQ);					// Automatically changes robot state in software		
}



/* -------------------------------------------- END WALK RELATED MANIPULATIONS -------------------------------------------- */


/* -------------------------------------------- FLIGHT RELATED MANIPULATIONS -------------------------------------------- */

// input equals height to be raised by; negative values also work
void Leg::LiftBodyUp(const double& hraise){

	UpdateStateVars(HEIGHT, (StateVars[HEIGHT] + hraise));			// HIPTOEND automatically gets updated

/*
	if (StateVars[HIPTOEND] > (Params[TIBIA] + Params[FEMUR]) ){
		UpdateStateVars(HIPTOEND, (Params[TIBIA] + Params[FEMUR]) );
		UpdateHeight();
	}

	else if (StateVars[HIPTOEND] < (Params[TIBIA] - Params[FEMUR]) ){ 
		UpdateStateVars(HIPTOEND, 1.5 * (Params[TIBIA] - Params[FEMUR]) );	 // Params[TIBIA] - Params[FEMUR] is impossible position
		// Think of better way to calculate minimal position - use Hip limit angle
		UpdateHeight();
	}
*/
}



void Leg::ConfigureQuadcopter(){
	double MotorAngle = asin( (Params[COXA]+Params[FEMUR])/Params[DISTCENTER] * sin(wkquad::PI/12) );

	// Angle valid for a LEFT LEG servo facing downwards
	if(AngleOffset > wkquad::PI/2) 	ServoAngles[ARM] =  -(MotorAngle + (wkquad::PI/12));
	else 					ServoAngles[ARM] = ARM, MotorAngle + (wkquad::PI/12);

	PrepareArmForTakeOff();
}

void Leg::ConfigureHexacopter(){
	ServoAngles[ARM] = 0.0;
	PrepareArmForTakeOff();
}



/* -------------------------------------------- END FLIGHT RELATED MANIPULATIONS -------------------------------------------- */


/* -------------------------------------------- WRITING TO SERVOS -------------------------------------------- */


// Write ServoAngles[] to physcial servos EXCEPT FOR WING SERVO
void Leg::WriteAngles(){
	for(int i=0; i<JOINT_COUNT-1; i++){
		Joints[i].SetGoalPosition(LegRight * ServoAngles[i]);
	}
}

// Write ServoAngles[] to physcial servos INCLUDING ALL SERVOS
void Leg::WriteAllAngles(){
	for(int i=0; i<JOINT_COUNT; i++){
		Joints[i].SetGoalPosition(LegRight * ServoAngles[i]);
	}
}

// WRITE only a single angle contained in ServoAngles[] TO PHYSCIAL SERVO
void Leg::WriteJoint(const int& idx){
	Joints[idx].SetGoalPosition(LegRight * ServoAngles[idx]);
}

/* -------------------------------------------- END WRITING TO SERVOS -------------------------------------------- */


/* -------------------------------------------- GETTERS -------------------------------------------- */


double Leg::GetLegParam(const int& idx, const int& param_type) const{
	if(param_type==SERVO_ANGLE) return ServoAngles[idx];
	else if(param_type==STATE_VAR) return StateVars[idx];
	else if(param_type==DEFAULT_VAR) return DefaultVars[idx];
	else if(param_type==DEFAULT_ANGLE) return DefaultAngles[idx];
	else if(param_type==PARAM) return Params[idx];
	else if(param_type==ANGLE_LIMIT) return AngleLimits[idx];
	else return 0;
}



/* ================================================= END OF PUBLIC METHODS ================================================= */



/* ================================================= PRIVATE METHODS ================================================= */




void Leg::PrepareArmForTakeOff(){
	ServoAngles[WING] = 0.0;
	WriteJoint(WING);
	LiftLegUp(5);
	WriteJoint(KNEE);
	WriteJoint(ARM);
	PutStraightDown(5);
	WriteJoint(KNEE);
	ServoAngles[KNEE] = wkquad::PI/2;
	ServoAngles[HIP] = 0.0;
	WriteJoint(HIP);
	WriteJoint(KNEE);
}

/* --------------------------------------- UPDATE LEG INSTANCE'S PARAMETERS AND STATE --------------------------------------- */


void Leg::UpdateStateVars(const int& idx, const double& value){
	
	StateVars[idx] = value;

	double base_idx = idx;
	// Provided argument is square
	if(idx>=VAR_STEP){
		StateVars[idx-VAR_STEP] = sqrt(StateVars[idx]);
		base_idx = idx-VAR_STEP;
	} 	

	// Provided argument is base number
	else StateVars[idx+VAR_STEP] = pow(StateVars[idx], 2);

	UpdateState(base_idx);	
}

void Leg::UpdateStateVars(const int& idx, const double& value, const double& valueSQ){
	StateVars[idx] = value;
	StateVars[idx+VAR_COUNT] = valueSQ;
	UpdateState(idx);	
}


void Leg::UpdateState(const int& idx){
	// Keep count static in order to keep its state in the recursion
	static int count = 1;

	if(count==1){
		// Make sure you do NOT enter an endless recursion 
		count = 0;
		switch(idx){
			case HIPTOEND:
				// Assumed that change is in END EFFECTOR and only ARMGTOEND is affected
				UpdateStateVars(ARMGTOEND, sqrt(StateVars[HIPTOEND_SQ]-StateVars[HEIGHT_SQ]) + Params[COXA]);
				break;
			case ARMGTOEND:
				// Only HIPTOEND is affected
				UpdateStateVars(HIPTOEND_SQ, pow((StateVars[ARMGTOEND]-Params[COXA]),2) + StateVars[HEIGHT_SQ]);
				break;
			case HEIGHT:
				// Only HIPTOEND is affected
				UpdateStateVars(HIPTOEND_SQ, pow((StateVars[ARMGTOEND]-Params[COXA]),2) + StateVars[HEIGHT_SQ]);
				break;
		}

		// All parameters that can be changed require the update of HIP and KNEE ANGLES
		UpdateHipKnee();			
		// Make sure Recursion will be allowed on next function call
		count = 1;
	}
}


// Updates Hip and Knee angles according to the current HIPTOEND and HEIGHT values
void Leg::UpdateHipKnee(){

	// Cosine Rule to find new Knee Servo Angle
	double KneeTmp = acos( (Params[FEMUR_SQ] + Params[TIBIA_SQ] - StateVars[HIPTOEND_SQ] ) / ( 2 * Params[FEMUR] * Params[TIBIA]) );
	// Convert to actual angle for the servo
	ServoAngles[KNEE] = wkquad::PI - KneeTmp;													// Input valid for a LEFT LEG

	// Cosine Rule to find new Knee Servo Angle
	double HipTmp = acos( (Params[FEMUR_SQ] + StateVars[HIPTOEND_SQ] - Params[TIBIA_SQ] ) / ( 2 * Params[FEMUR] * StateVars[HIPTOEND]) );
	// Convert to actual angle for the servo
	ServoAngles[HIP] = wkquad::PI/2 - HipTmp - acos(StateVars[HEIGHT]/StateVars[HIPTOEND]);		// Input valid for a LEFT LEG
}


void Leg::ClearState(){
	for(int i=0; i<VAR_COUNT; i++){
		StateVars[i]=0;
	}
}

/*
void Leg::VerifyState(){
	// Best to call this function at the end of each manipulation function after all variables have been configured to be true
	// This way you make sure no errors that don't exist are not found and by calling from the function you know what modifications
	// you need to do in order to correct the state


	for(int i=0; i<JOINT_COUNT; i++){
		if(ServoAngles[i]<AngleLimits[2*i]) throw wkquad::leg_state_t(2*i);
		if(ServoAngles[i]>AngleLimits[2*i+1]) throw wkquad::leg_state_t(2*i+1);
	}

	for(int i=0; i<VAR_STEP; i++){
		if(StateVars[i]<0) throw wkquad::leg_state_t(/*state vars less than 0*/);
/*	}

	// Triangle rule check
	double tri_side = StateVars[ARMGTOEND] - Params[COXA];
	if( StateVars[HIPTOEND] > (tri_side + StateVars[HEIGHT]) )	throw wkquad::leg_state_t(leg_state_vars_broken);
	if( StateVars[HEIGHT] > (tri_side + StateVars[HIPTOEND]) )	throw wkquad::leg_state_t(leg_state_vars_broken);
	if( tri_side > (StateVars[HIPTOEND] + StateVars[HEIGHT]) )	throw wkquad::leg_state_t(leg_state_vars_broken);


	// angle + var check
	// armgtoend<femur+tibia
	// torque check - is knee close enough - give it 20% freedom

}

*/
/* --------------------------------------- END UPDATE LEG INSTANCE'S PARAMETERS AND STATE --------------------------------------- */


/* --------------------------------------- FIND ANGLE/VARIABLE VALUES FOR LEG INSTANCE --------------------------------------- */



double Leg::CenterHip(){
	double hip_max=asin(Params[HIPKNEEMAXHDIST]/Params[FEMUR]);
	return wkquad::radians(90) - ( hip_max + ( wkquad::radians(90) - AngleLimits[HIP_MIN] ) )/2;
}

double Leg::CenterKnee(const double& height_hip /*=StateVars[HEIGHT]*/){
	double height_knee = height_hip + Params[FEMUR]*sin(ServoAngles[HIP]);
	double knee_angle = wkquad::radians(90) - ServoAngles[HIP] - acos(height_knee/Params[TIBIA]);
	return wkquad::radians(180) - knee_angle;
}


// Computes valid StateVars[] basing on height_in and assumes all StateVars[] are defined
void Leg::InitializeState(const double& height_in /*=StateVars[HEIGHT]*/){
	StateVars[HEIGHT] = height_in;
	UpdateStateVars(HIPTOEND, sqrt( pow(Params[FEMUR],2) + pow(Params[TIBIA],2) - 
									2 * Params[FEMUR] * Params[TIBIA] * cos(wkquad::PI - ServoAngles[KNEE]) )  );
}

// Computes valid StateVars[] basing on ServoAngles[] and assumes all StateVars[] can be defined
void Leg::InitializeState(){
	double knee_height = Params[TIBIA]*cos(wkquad::radians(90) - abs(ServoAngles[HIP]) - (wkquad::PI - ServoAngles[KNEE]) );
	StateVars[HEIGHT] = knee_height - Params[FEMUR]*sin(abs(ServoAngles[HIP]));
	UpdateStateVars(HIPTOEND, sqrt( pow(Params[FEMUR],2) + pow(Params[TIBIA],2) - 
									2 * Params[FEMUR] * Params[TIBIA] * cos(wkquad::PI - ServoAngles[KNEE]) )  );
}



/* --------------------------------------- END FIND ANGLE/VARIABLE VALUES FOR LEG INSTANCE --------------------------------------- */


/* ================================================= END OF PRIVATE METHODS ================================================= */






/* ============================ SET LEG POSITION MANUALLY BY PROVIDING ARGUMENTS FOR EACH SERVO ============================ */

/* ----------------------------------- DOUBLE OVERLOADING ----------------------------------- */

void Leg::SetLegPosition(const double& knee_pos, const double& hip_pos, const double& arm_pos, const double& wing_pos){
	
	SetJointPosition(ARM, arm_pos);
	SetJointPosition(HIP, hip_pos);
	SetJointPosition(KNEE, knee_pos);
	SetJointPosition(WING, wing_pos);
}

void Leg::SetLegPosition(const double& knee_pos, const double& hip_pos, const double& arm_pos){
	
	SetJointPosition(ARM, arm_pos);
	SetJointPosition(HIP, hip_pos);
	SetJointPosition(KNEE, knee_pos);
}

int Leg::SetJointPosition(const int& JointIdx, const double& position){
	UpdateServoAngle(JointIdx, position );
	return Joints[JointIdx].SetGoalPosition(LegRight * ServoAngles[JointIdx]);
}

/* ----------------------------------- END OF DOUBLE OVERLOADING ----------------------------------- */

