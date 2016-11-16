#include "Leg.h"

/* ===================================================== PUBLIC METHODS ===================================================== */

// Tripod constructor does not write to angles, so Leg constrcutor is only responsible for calculating the proper defaultPos
Leg::Leg 	(int ID_knee, int ID_hip, int ID_arm,
			DnxSerialBase* HipsKnees, DnxSerialBase* Arms, double height_in, const double robot_params[]) :
	
	state(height_in, robot_params), 
	// Instantiate joints
	joints	{ 	ServoJoint(ID_knee, HipsKnees), 
				ServoJoint(ID_hip, HipsKnees), 
				ServoJoint(ID_arm, Arms), 
			} {

	switch(ID_knee){
		case wkq::KNEE_LEFT_FRONT:
			angle_offset = wkq::radians(30);
			leg_id = wkq::LEG_LEFT_FRONT;
			leg_right = false;
			break;
		case wkq::KNEE_LEFT_MIDDLE:
			angle_offset = wkq::radians(30+60);
			leg_id = wkq::LEG_LEFT_MIDDLE;
			leg_right = false;
			break;
		case wkq::KNEE_LEFT_BACK:
			angle_offset = wkq::radians(30+120);
			leg_id = wkq::LEG_LEFT_BACK;
			leg_right = false;
			break;
		case wkq::KNEE_RIGHT_FRONT:
			angle_offset = wkq::radians(30);
			leg_id = wkq::LEG_RIGHT_FRONT;
			leg_right = true;
			break;
		case wkq::KNEE_RIGHT_MIDDLE:
			angle_offset = wkq::radians(30+60);
			leg_id = wkq::LEG_RIGHT_MIDDLE;
			leg_right = true;
			break;
		case wkq::KNEE_RIGHT_BACK:
			angle_offset = wkq::radians(30+120);
			leg_id = wkq::LEG_RIGHT_BACK;
			leg_right = true;
			break;
		default:
			pc.print_debug("WARNING: Invalid KNEE ID IN LEG CONSTRUCTOR\n");
	}
}

Leg::~Leg(){}

/* ------------------------------------------------- STATIC POSITIONS ------------------------------------------------- */

void Leg::standQuad(){
	state.legStand();
	confQuadArms();
}

void Leg::flatQuad(){
	confQuadArms();
	state.legFlatten();
}

void Leg::confQuadArms(){
	// MIDDLE LEGS
	if(leg_id == wkq::LEG_RIGHT_MIDDLE || leg_id == wkq::LEG_LEFT_MIDDLE) state.servo_angles[ARM] = 0.0;
	
	else{
		double arm_offset = 
			asin( state.params[DISTCENTER] * sin(wkq::PI/12)/(state.params[COXA]+state.params[FEMUR]-state.params[KNEEMOTORDIST]) );
		
		// FRONT LEGS
		if(leg_id == wkq::LEG_RIGHT_FRONT || leg_id == wkq::LEG_LEFT_FRONT) state.servo_angles[ARM] = arm_offset + (wkq::PI/12);
		// BACK LEGS
		else if(leg_id == wkq::LEG_RIGHT_BACK || leg_id == wkq::LEG_LEFT_BACK) state.servo_angles[ARM] = - (arm_offset + (wkq::PI/12));
	}
}

/* ------------------------------------------------- RAISE AND LOWER ------------------------------------------------- */

// input equals height required for the end effector; Untested for all joints and negative input
void Leg::liftUp(double height){
	state.servo_angles[KNEE] = wkq::PI - state.servo_angles[KNEE] + acos(1 - pow(height,2) / state.params[TIBIA_SQ]); 
}

// input equals current height of the end effector
void Leg::lowerDown(double height){
	state.servo_angles[KNEE] = wkq::PI - state.servo_angles[KNEE] - acos(1 - pow(height,2) / state.params[TIBIA_SQ]); 
}

/* 	
	Put End Effector down with ARM, HIP and KNEE centered. This is equivalent to properly terminating a movement forward
	Note that End Effector is still kept on the same line as HIP and KNEE are centered for this position
*/
void Leg::finishStep(){
	state.servo_angles[ARM] = 0.0;
	// center HIP and KNEE and Compute new state
	state.centerAngles();
}


/* ------------------------------------------------- WALKING ALGORITHMS ------------------------------------------------- */ 



// Valid calculations for negative input as well in the current form
void Leg::IKBodyForward(double step_size){

	double step_sizeSQ = pow(step_size,2);

	// Cosine Rule to find new HipToEndSQ
	double ArmGToEndNewSQ = state.get(STATE_VAR, ARMGTOEND_SQ) + step_sizeSQ - 
								2 * step_size * state.get(STATE_VAR, ARMGTOEND) * cos( angle_offset + state.servo_angles[ARM] ) ;

	double ArmGToEndNew = sqrt(ArmGToEndNewSQ);

	// Cosine Rule to find new Arm Servo Angle
	double ArmTmp = acos( (step_sizeSQ + ArmGToEndNewSQ - state.get(STATE_VAR, ARMGTOEND_SQ)) / ( 2 * step_size * ArmGToEndNew ) );
	// Convert to actual angle for the servo
	state.servo_angles[ARM] = wkq::PI - angle_offset - ArmTmp; 			// NB: Valid for a LEFT servo facing down

	state.updateVar(ARMGTOEND, ArmGToEndNew, ArmGToEndNewSQ);		// Automatically changes robot state in software
}


// Input is the currently used step size
void Leg::stepForward(double step_size){

	// Distance from Robot center to end effector for a fully centered robot at this height
	double ef_center = state.get(STATE_VAR, EFCENTER);

	// Create Point representing the new END EFFECTOR position
	double x_EFNew, y_EFNew;
	if(leg_id == wkq::LEG_RIGHT_MIDDLE || leg_id == wkq::LEG_LEFT_MIDDLE){			// Middle Joint
		//x_EFNew = ef_center - step_size/sqrt(3); - change when you have prev and current step size as inputs
		x_EFNew = ef_center - step_size/sqrt(3);
		y_EFNew = 0.0;
	}	
	else{
		x_EFNew = ef_center/2.0 + step_size/sqrt(3);
		y_EFNew = sqrt(3)*x_EFNew; 						// Front Joint

		if(leg_id == wkq::LEG_RIGHT_BACK || leg_id == wkq::LEG_LEFT_BACK) 	y_EFNew -= sqrt(3)*ef_center; 			// Back Joint
	}		
	wkq::Point EFNew(x_EFNew, y_EFNew);

	// Create Point representing current HIP position
	double hip_arg = wkq::PI/2 -angle_offset;
	wkq::Point Hip(state.params[DISTCENTER]*cos(hip_arg), state.params[DISTCENTER]*sin(hip_arg));

	// Create Point representing the new HIP position
	wkq::Point HipNew(Hip);
	HipNew.translate_y(step_size*2);

	// Calculate new ARMGTOEND
	double ArmGToEndNewSQ = Hip.dist_sq(EFNew);

	// Cosine Rule to find new ARM angle
	double ArgTriangle = 
			acos( (Hip.dist_sq(HipNew) +  Hip.dist_sq(EFNew) - HipNew.dist_sq(EFNew)) / (2*Hip.dist(HipNew)*Hip.dist_sq(EFNew)) );
	//state.servo_angles[ARM] = wkq::PI - angle_offset - ArgTriangle;
	state.servo_angles[ARM] = ArgTriangle - angle_offset ;		// Unconfirmed that valid for all joints

	state.updateVar(ARMGTOEND_SQ, ArmGToEndNewSQ);				// Automatically changes HIPTOEND, HIP and KNEE	
}



void Leg::IKBodyRotate(double angle){

	// Sine Rule to find rotation distance
	double RotDist = 2 * state.params[DISTCENTER] * sin (fabs(angle)/2);

	double RotDistSQ = pow(RotDist, 2);

	// Cosine Rule to find new ArmGToEndSQ
	double ArmGToEndNewSQ;
	if (angle>=0.0) ArmGToEndNewSQ = state.get(STATE_VAR, ARMGTOEND_SQ) + RotDistSQ - 
									2 * RotDist * state.get(STATE_VAR, ARMGTOEND) * cos( wkq::PI/2 + angle/2 - state.servo_angles[ARM] );
	else ArmGToEndNewSQ = state.get(STATE_VAR, ARMGTOEND_SQ) + RotDistSQ - 
									2 * RotDist * state.get(STATE_VAR, ARMGTOEND) * cos( wkq::PI/2 - angle/2 + state.servo_angles[ARM] );	

	double ArmGToEndNew = sqrt(ArmGToEndNewSQ);

	// Cosine Rule to find new Arm Servo Angle
	double ArmTmp = acos( (RotDistSQ + ArmGToEndNewSQ - state.get(STATE_VAR, ARMGTOEND_SQ)) / ( 2 * RotDist * ArmGToEndNew ) );
	// Convert to actual angle for the servo - valid for a LEFT LEG servo facing down
	if (angle>=0.0)	state.servo_angles[ARM] = wkq::PI/2 - ArmTmp + angle/2;  // - (- wkq::PI/2 + state.servo_angles[ARM] - angle/2)
	else 			state.servo_angles[ARM] = -wkq::PI/2 + ArmTmp + angle/2;	// - (  wkq::PI/2 - state.servo_angles[ARM] - angle/2)

	state.updateVar(ARMGTOEND, ArmGToEndNew, ArmGToEndNewSQ);					// Automatically changes robot state in software		
}


// Input is the currently used step size
void Leg::stepRotate(double step_size){
}


// input equals height to be raised by; negative values also work
void Leg::raiseBody(double hraise){

	state.updateVar(HEIGHT, (state.get(STATE_VAR, HEIGHT) + hraise));			// HIPTOEND, HIP and KNEE automatically get updated

/*
	if (vars[HIPTOEND] > (state.params[TIBIA] + state.params[FEMUR]) ){
		state.updateVar(HIPTOEND, (state.params[TIBIA] + state.params[FEMUR]) );
		UpdateHeight();
	}

	else if (vars[HIPTOEND] < (state.params[TIBIA] - state.params[FEMUR]) ){ 
		state.updateVar(HIPTOEND, 1.5 * (state.params[TIBIA] - state.params[FEMUR]) );	 // state.params[TIBIA] - state.params[FEMUR] is impossible position
		// Think of better way to calculate minimal position - use Hip limit angle
		UpdateHeight();
	}
*/
}

/* ------------------------------------------------- TESTING FUNCTIONS ------------------------------------------------- */

void Leg::quadSetup(){
	state.clear();
	state.servo_angles[ARM] = 0;
	state.servo_angles[HIP] = wkq::radians(50);
	state.servo_angles[KNEE] = wkq::PI/2;
}


/* ------------------------------------------------- WRITING TO SERVOS ------------------------------------------------- */


// Write state.servo_angles[] to physcial servos in order ARM, HIP, KNEE
void Leg::writeAngles(){
	if(!leg_right){
		joints[ARM].setGoalPosition(state.servo_angles[ARM]);
		joints[HIP].setGoalPosition(state.servo_angles[HIP]);
		joints[KNEE].setGoalPosition(state.servo_angles[KNEE]);
	}
	else{
		if(state.servo_angles[ARM] != 0)		joints[ARM].setGoalPosition(-state.servo_angles[ARM]);
		else								joints[ARM].setGoalPosition(state.servo_angles[ARM]);
		if(state.servo_angles[HIP] != 0)		joints[HIP].setGoalPosition(-state.servo_angles[HIP]);
		else								joints[HIP].setGoalPosition(state.servo_angles[HIP]);
		if(state.servo_angles[KNEE] != 0)	joints[KNEE].setGoalPosition(-state.servo_angles[KNEE]);
		else								joints[KNEE].setGoalPosition(state.servo_angles[KNEE]);
	}
}

// WRITE only a single angle contained in state.servo_angles[] TO PHYSCIAL SERVO
void Leg::writeJoint(int idx){
	if(!leg_right){
		joints[idx].setGoalPosition(state.servo_angles[idx]);
	}
	else{
		if(state.servo_angles[idx] != 0)		joints[idx].setGoalPosition(-state.servo_angles[idx]);
		else								joints[idx].setGoalPosition(state.servo_angles[idx]);
	}
}


/* ------------------------------------------------- GETTER AND COPY ------------------------------------------------- */

double Leg::get(int param_type, int idx) const{
	if 		(param_type==SERVO_ANGLE) 	return state.servo_angles[idx];
	else if (param_type==STATE_VAR) 	return state.get(STATE_VAR, idx);
	else if (param_type==defaultPos_VAR) 	return state.get(defaultPos_VAR, idx);
	else if (param_type==defaultPos_ANGLE) return state.get(defaultPos_ANGLE, idx);
	else if (param_type==PARAM) 		return state.params[idx];
	else if (param_type==ANGLE_LIMIT) 	return state.angle_limits[idx];
	else return 0.0;
}


void Leg::copyState(const Leg& LegIn){
	state = LegIn.state;
}

