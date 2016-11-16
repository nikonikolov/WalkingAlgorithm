#include "State_t.h"


/* ===================================================== PUBLIC METHODS ===================================================== */

/*
const double State_t::params[PARAM_COUNT] = { 10.95, 2.65, 17.5, 30.0, 12.0, 2.25,
										pow(10.95,2), pow(2.65,2), pow(17.5,2), pow(30.0,2), pow(12.0,2), pow(2.25,2)	};
*/

double State_t::params[PARAM_COUNT] = { 0.0 };

// Values are valid for a left leg and correspond to values that are written to the servo
const double State_t::angle_limits[JOINT_COUNT*2] = { 	wkq::radians(0), wkq::radians(150),					/* KNEE */ 
														wkq::radians(-(90-20)), wkq::radians(90-20),		/* HIP - JUST A GUESS */
														wkq::radians(-(90-20)), wkq::radians(90-20),		/* ARM - JUST A GUESS */
														//wkq::radians(-150), wkq::radians(150)				/* WING */
													};

double State_t::default_pos_vars[VAR_COUNT] 	= { 0.0 };
double State_t::default_pos_angles[JOINT_COUNT] = { 0.0 };

bool State_t::default_pos_calculated = false;


/* 	
	Tripod and Leg constructors do not write to angles, so State constrcutor is only responsible for initializing to meaningless
	 state and calculating the defaultPoss if not already
*/
State_t::State_t(double height_in, const double robot_params[]) : servo_angles { 0.0 }, vars{ 0.0 } {

	// Calculate defaultPoss only if not calculated already - note defaultPoss are static members for the class
	if(!default_pos_calculated){
		default_pos_calculated = true;

		// Initialize the parameters
		for(int i=0; i<PARAM_COUNT; i++){
			if(i<PARAM_STEP) params[i] = robot_params[i];
			else params[i] = pow(robot_params[i-PARAM_STEP],2);
		}

		// Find defaultPos servo_angles for Hip and Knee. Automatically computes the state and sets the height
		centerAngles(height_in);

		// Save defaultPos Variables
		for(int i=0; i<VAR_COUNT; i++){
			default_pos_vars[i] = vars[i]; 				
		}

		// Save defaultPos Angles
		for(int i=0; i<JOINT_COUNT; i++){
			default_pos_angles[i] = servo_angles[i];
		}

		// Initialize servo_angles to meaningless(center) state
		for(int i=0; i<JOINT_COUNT; i++){
			servo_angles[i]=0.0;
		}
	
		// Initialize Vars to meaningless values
		for(int i=0; i<VAR_COUNT; i++){
			vars[i] = 0.0;
		}
	}
}

State_t::~State_t(){}

/* -------------------------------------------- GETTER AND COPY -------------------------------------------- */


double State_t::get(int param_type, int idx) const{
	if 		(param_type==SERVO_ANGLE) 	return servo_angles[idx];
	else if (param_type==STATE_VAR) 	return vars[idx];
	else if (param_type==defaultPos_VAR) 	return default_pos_vars[idx];
	else if (param_type==defaultPos_ANGLE) return default_pos_angles[idx];
	else if (param_type==PARAM) 		return params[idx];
	else if (param_type==ANGLE_LIMIT) 	return angle_limits[idx];
	else return 0.0;
}

void State_t::operator=(const State_t& StateIn){
	if(this!=&StateIn){
		for(int i=0; i<JOINT_COUNT; i++){
			servo_angles[i] = StateIn.servo_angles[i];
		}
		for(int i=0; i<VAR_COUNT; i++){
			vars[i] = StateIn.vars[i];
		}
	}
}


/* -------------------------------------------- STATIC POSITIONS -------------------------------------------- */

void State_t::legDefaultPos(){	
	// Change State Variables
	for(int i=0; i<VAR_COUNT; i++){
		vars[i] = default_pos_vars[i];
	}
	// Change State Angles
	for(int i=0; i<JOINT_COUNT; i++){
		servo_angles[i] = default_pos_angles[i];
	}
}

void State_t::legCenter(){
	//servo_angles[WING] = default_pos_angles[WING];
	servo_angles[ARM] = default_pos_angles[ARM];
	centerAngles();
}

void State_t::legStand(){
	setAngles((wkq::PI)/2, 0.0, 0.0);		// Automatically calls computevars[] and computes new state
}

void State_t::legFlatten(){	
	servo_angles[KNEE]=0.0;
	servo_angles[HIP]=0.0;
	// ARM and WING are either set to 0.0 by defaultPos or are not to be changed
	clear();
}


/* ================================================= MAINTAINING LEG STATE ================================================= */


void State_t::updateVar(int idx, double value, bool update_state /*=true*/){
	
	vars[idx] = value;

	double base_idx = idx;
	// Provided argument is square
	if(idx>=VAR_STEP){
		vars[idx-VAR_STEP] = sqrt(vars[idx]);
		base_idx = idx-VAR_STEP;
	} 	

	// Provided argument is base number
	else vars[idx+VAR_STEP] = pow(vars[idx], 2);

	if(update_state) update(base_idx);	
}

void State_t::updateVar(int idx, double value, double valueSQ){
	vars[idx] = value;
	vars[idx+VAR_COUNT] = valueSQ;
	update(idx);	
}


void State_t::update(int idx){

	switch(idx){
		case HIPTOEND:
			// Assumed that change is in END EFFECTOR and only ARMGTOEND is affected
			updateVar(ARMGTOEND, sqrt(vars[HIPTOEND_SQ]-vars[HEIGHT_SQ]) + params[COXA], false);
			break;
		case ARMGTOEND:
			// Only HIPTOEND is affected
			updateVar(HIPTOEND_SQ, pow((vars[ARMGTOEND]-params[COXA]),2) + vars[HEIGHT_SQ], false);
			break;
		case HEIGHT:
			// Only EFCENTER and HIPTOEND affected. ARMGTOEND is not affected by height change
			updateVar(HIPTOEND_SQ, pow((vars[ARMGTOEND]-params[COXA]),2) + vars[HEIGHT_SQ], false);
			updateVar(EFCENTER, vars[ARMGTOEND]+params[DISTCENTER], false);
			break;
		//defaultPos:	// Good idea is to throw exception or signal somehow
	}

	// All parameters that can be changed require the update of HIP and KNEE angles
	updateAngles();			
}


// Update KNEE and HIP angles basing on the current HIPTOEND and HEIGHT
void State_t::updateAngles(){

	// Cosine Rule to find new Knee Servo Angle
	double KneeTmp = acos( (params[FEMUR_SQ] + params[TIBIA_SQ] - vars[HIPTOEND_SQ] ) / ( 2 * params[FEMUR] * params[TIBIA]) );
	// Convert to actual angle for the servo
	servo_angles[KNEE] = wkq::PI - KneeTmp;													// Input valid for a LEFT LEG

	// Cosine Rule to find new Hip Servo Angle
	double HipTmp = acos( (params[FEMUR_SQ] + vars[HIPTOEND_SQ] - params[TIBIA_SQ] ) / ( 2 * params[FEMUR] * vars[HIPTOEND]) );
	// Convert to actual angle for the servo
	servo_angles[HIP] = (wkq::PI)/2 - HipTmp - acos(vars[HEIGHT]/vars[HIPTOEND]);		// Input valid for a LEFT LEG
}





void State_t::clear(){
	for(int i=0; i<VAR_COUNT; i++){
		vars[i]=-1.0;
	}
}

/*
void State_t::Verify(){
	return;
	// Best to call this function at the end of each manipulation function after all variables have been configured to be true
	// This way you make sure no errors that don't exist are not found and by calling from the function you know what modifications
	// you need to do in order to correct the state


	for(int i=0; i<JOINT_COUNT; i++){
		if(servo_angles[i]<angle_limits[2*i]) throw wkq::leg_state_t(2*i);
		if(servo_angles[i]>angle_limits[2*i+1]) throw wkq::leg_state_t(2*i+1);
	}

	for(int i=0; i<VAR_STEP; i++){
		if(vars[i]<0) throw wkq::leg_state_t(/*state vars less than 0*);
	}

	// Triangle rule check
	double tri_side = vars[ARMGTOEND] - params[COXA];
	if( vars[HIPTOEND] > (tri_side + vars[HEIGHT]) )	throw wkq::leg_state_t(leg_state_vars_broken);
	if( vars[HEIGHT] > (tri_side + vars[HIPTOEND]) )	throw wkq::leg_state_t(leg_state_vars_broken);
	if( tri_side > (vars[HIPTOEND] + vars[HEIGHT]) )	throw wkq::leg_state_t(leg_state_vars_broken);


	// angle + var check
	// armgtoend<femur+tibia
	// torque check - is knee close enough - give it 20% freedom

}
*/

/* 	height should be 0.0 when the current StateVar[HEIGHT] should be used; if this variable has not been set to a meaningful value, 
	the height that needs to be used should be passed as parameter - computeEFVars() will set the value in StateVar[HEIGHT]
*/
void State_t::centerAngles(double height /*=0.0*/){
	// center HIP
	double hip_max_angle=asin(params[HIPKNEEMAXHDIST]/params[FEMUR]);
	servo_angles[HIP] = ( hip_max_angle + ( wkq::radians(90) - fabs(angle_limits[HIP_MIN]) ) )/2 - wkq::radians(90);
	
	// center KNEE
	double height_hip;
	if(height==0.0) height_hip = vars[HEIGHT];
	else height_hip = height;

	double height_knee = height_hip + params[FEMUR]*sin(fabs(servo_angles[HIP]));
	double knee_angle = wkq::PI/2 - fabs(servo_angles[HIP]) - acos(height_knee/params[TIBIA]);
	// if the argument of TIBIA relative to the ground is not acute but obtuse:
	if(knee_angle<0.0) knee_angle = wkq::PI/2 - fabs(servo_angles[HIP]) + acos(height_knee/params[TIBIA]);

	servo_angles[KNEE] = wkq::PI - knee_angle;

	computeEFVars(height);			// Update vars[] - needs to be passed the same argument
}

//void State_t::setAngles(double knee, double hip, double arm, double wing){
void State_t::setAngles(double knee, double hip, double arm){
	servo_angles[KNEE] = knee;
	servo_angles[HIP] = hip;
	servo_angles[ARM] = arm;
//	servo_angles[WING] = wing;
	computeVars();
}



// Computes valid vars[] basing on servo_angles[] and assumes all vars[] can be defined
void State_t::computeVars(){
	double knee_height = params[TIBIA]*cos(wkq::radians(90) - fabs(servo_angles[HIP]) - (wkq::PI - servo_angles[KNEE]) );
	
	double height = knee_height - params[FEMUR]*sin(fabs(servo_angles[HIP]));
	updateVar(HEIGHT, height, false);					// Only updates HEIGHT
	
	double hip_to_end_sq = pow(params[FEMUR],2) + pow(params[TIBIA],2) - 2*params[FEMUR]*params[TIBIA]*cos(wkq::PI - servo_angles[KNEE]);
	updateVar(HIPTOEND_SQ, hip_to_end_sq, false); 		// Only updates HIPTOEND

	double arm_g_to_end = sqrt(vars[HIPTOEND_SQ]-vars[HEIGHT_SQ]) + params[COXA];
	updateVar(ARMGTOEND, arm_g_to_end, false); 			// Only updates ARMGTOEND

	// Note that angles are already centered so EFCENTER will be fine
	updateVar(EFCENTER, vars[ARMGTOEND]+params[DISTCENTER], false);
}


void State_t::computeEFVars(double height /*=0.0*/){
	if(height!=0.0) updateVar(HEIGHT, height, false); 	// Only updates HEIGHT

	double hip_to_end_sq = pow(params[FEMUR],2) + pow(params[TIBIA],2) - 2*params[FEMUR]*params[TIBIA]*cos(wkq::PI - servo_angles[KNEE]);
	updateVar(HIPTOEND_SQ, hip_to_end_sq, false); 		// Only updates HIPTOEND

	double arm_g_to_end = sqrt(vars[HIPTOEND_SQ]-vars[HEIGHT_SQ]) + params[COXA];
	updateVar(ARMGTOEND, arm_g_to_end, false); 			// Only updates ARMGTOEND

	// Note that angles are already centered so EFCENTER will be fine
	if(height!=0.0) updateVar(EFCENTER, vars[ARMGTOEND]+params[DISTCENTER], false);
}
