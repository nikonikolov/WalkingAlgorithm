#include "Leg.h"

/* *********************************** DIRECT LEG POSITIONING ********************************** */

/* *********************************** DOUBLE OVERLOADING ********************************** */

void Leg::SetPosition(const double& knee_pos, const double& hip_pos, const double& arm_pos, const double& wing_pos){
	
	SetJointPosition(ARM, arm_pos);
	SetJointPosition(HIP, hip_pos);
	SetJointPosition(KNEE, knee_pos);
	SetJointPosition(WING, wing_pos);
}

void Leg::SetPosition(const double& knee_pos, const double& hip_pos, const double& arm_pos){
	
	SetJointPosition(ARM, arm_pos);
	SetJointPosition(HIP, hip_pos);
	SetJointPosition(KNEE, knee_pos);
}

int Leg::SetJointPosition(const int& JointIdx, const double& position){
	UpdateServoAngle(JointIdx, position );
	return Joints[JointIdx]->SetGoalPosition(position);
}

/* *********************************** END OF DOUBLE OVERLOADING ********************************** */


/* *********************************** INT OVERLOADING ********************************** */

void Leg::SetPosition(const int& knee_pos, const int& hip_pos, const int& arm_pos, const int& wing_pos){
	
	SetJointPosition(ARM, arm_pos);
	SetJointPosition(HIP, hip_pos);
	SetJointPosition(KNEE, knee_pos);
	SetJointPosition(WING, wing_pos);
}

void Leg::SetPosition(const int& knee_pos, const int& hip_pos, const int& arm_pos){
	
	SetJointPosition(ARM, arm_pos);
	SetJointPosition(HIP, hip_pos);
	SetJointPosition(KNEE, knee_pos);
}

int Leg::SetJointPosition(const int& JointIdx, const int& position){
	UpdateServoAngle(JointIdx, ConvertAngle(position) );
	return Joints[JointIdx]->SetGoalPosition(position);
}

// Function required in order to save the proper values in ServoAngle[] when using int inputs
// CW - positive output, CCW - negative output
double Leg::ConvertAngle(const int& angle){
	
	// 5.23598775598 rad = 300 degrees
	// 2.61799387799 rad = 150 degrees
	double result = 2.61799387799 - ((angle/1024)*5.23598775598);

	if (result>2.61799387799) return 2.61799387799;
	else if (result<-2.61799387799) return -2.61799387799; 
	else return result;
}

/* *********************************** END OF INT OVERLOADING ********************************** */

/* *********************************** END OF DIRECT LEG POSITIONING ********************************** */



/* *********************************** CONSTRUCTOR ********************************** */

Leg::Leg 	(ServoJoint& knee_in, ServoJoint& hip_in, ServoJoint& arm_in, ServoJoint& wing_in, 
			double& distcenter_in /*=10.9*/, double& coxa_in /*=2.65*/, double& femur_in /*=15.0*/, 
			double& tibia_in /*=25.0*/, double& heigh_in /*=default*/, double& angle_offset_in /*=default*/){

			Joints[KNEE]=&knee_in;
			Joints[HIP]=&hip_in;
			Joints[ARM]=&arm_in;
			Joints[WING]=&wing_in;

			// Have to calculate these dynamically or get them as inputs
			ServoAngle[KNEE]=;
			ServoAngle[HIP]=;
			ServoAngle[ARM]=;
			ServoAngle[WING]=0.0;

			Param[DISTCENTER] = distcenter_in;
			Param[COXA] = coxa_in;
			Param[FEMUR] = femur_in;
			Param[TIBIA] = tibia_in;
			Param[HEIGHT] = height_in;
			Param[ANGLE_OFFSET] = angle_offset_in;

			Param[HIPTOEND] = sqrt( pow(Param[COXA],2) + pow(Param[TIBIA],2) - 
									2 * Param[COXA] * Param[TIBIA] * cos( ServoAngle[KNEE] ) );


			for(int i=0; i<PARAMSQ_COUNT; i++){
				ParamSQ[i] = pow(Param[i],2);
			}

}


/*void Leg::Default(){
	
}*/

/* *********************************** END OF CONSTRUCTOR OVERLOADING ********************************** */



/* *********************************** SERVO MANIPULATION ********************************** */

void Leg::UpdateServoAngle(const int& idx, const double& pos){
	ServoAngle[idx]=pos;
}


void Leg::UpdateServoAngle(double NewPos[]){
	for(int i=0; i<JOINT_COUNT; i++){
		ServoAngle[i]=NewPos[i];
	}
}


int Leg::Straighten(){	
	return SetPosition(512, 512, 512, 512);	
}


// Have to come up with dynamic setting for that with given height	
int Leg::Stand(){
	return SetPosition(819, 512, 512, 512);	
}


// Works only for servos used in walking, does not write angle to Wing Servo
void Leg::WriteAngles(){
	for(int i=0; i<JOINTCOUNT-1; i++){
		Joints[i]->SetGoalPosition(ServoAngle[i]);
	}
}

/* *********************************** END OF SERVO MANIPULATION ********************************** */



/* *********************************** INVERSE KINEMATICS ********************************** */

// Valid calculations for negative input as well in the current form
void Leg::IKForward(const double& dist){

	double distSQ = pow(dist,2);

	// Cosine Rule to find new HipToEndSQ
	double ArmGToEndNewSQ = ParamSQ[ARMGTOEND] + distSQ - 
								2 * dist * Param[ARMGTOEND] * cos( Param[ANGLEOFFSET] + ServoAngle[ARM] ) ;

	double ArmGToEndNew = sqrt(ArmGToEndNewSQ);

	// Cosine Rule to find new Arm Servo Angle
	ServoAgnle[ARM] = acos( (distSQ + ArmGToEndNewSQ - ParamSQ[ARMGTOEND]) / ( 2 * dist * ArmGToEndNew ) );
	// Convert to actual angle for the servo
	ServoAngle[ARM] = - PI + Param[ANGLEOFFSET]) + ServoAngle[ARM]; 	// - (PI - Param[ANGLEOFFSET] - ServoAngle[ARM])

	UpdateParam(ARMGTOEND, ArmGToEndNew, ArmGToEndNewSQ);
	UpdateHipToEnd();
	IKUpdateHipKnee();
}


void Leg::IKRotate(const double& angle){

	// Sine Rule to find rotation distance
	double RotDist = 2 * Param[DISTCENTER] * sin (abs(angle)/2);

	double RotDistSQ = pow(RotDist, 2);

	// Cosine Rule to find new ArmGToEndSQ
	double ArmGToEndNewSQ;
	if (angle>=0.0) ArmGToEndNewSQ = ParamSQ[ARMGTOEND] + RotDistSQ - 
									2 * RotDist * Param[ARMGTOEND] * cos( PI/2 + angle/2 - ServoAngle[ARM] ) ;
	else ArmGToEndNewSQ = ParamSQ[ARMGTOEND] + RotDistSQ - 
									2 * RotDist * Param[ARMGTOEND] * cos( PI/2 - angle/2 + ServoAngle[ARM] ) ;	

	double ArmGToEndNew = sqrt(ArmGToEndNewSQ);

	// Cosine Rule to find new Arm Servo Angle
	ServoAgnle[ARM] = acos( (RotDistSQ + ArmGToEndNewSQ - ParamSQ[ARMGTOEND]) / ( 2 * RotDist * ArmGToEndNew ) );
	// Convert to actual angle for the servo
	if (angle>=0.0)	ServoAngle[ARM] =   PI/2 - ServoAngle[ARM] + angle/2;   // - (- PI/2 + ServoAngle[ARM] - angle/2)
	else 			ServoAngle[ARM] = -	PI/2 + ServoAngle[ARM] + angle/2;	// - (  PI/2 - ServoAngle[ARM] - angle/2)

	UpdateParam(ARMGTOEND, ArmGToEndNew, ArmGToEndNewSQ);
	UpdateHipToEnd();
	IKUpdateHipKnee();
}


// input equals height to be raised by; negative values also work
void Leg::BodyHeight(const double& hraise){

	UpdateParam(HEIGHT, (Param[HEIGHT] + hraise) ;

	UpdateParam(HIPTOEND, (ParamSQ[HEIGHT] + pow( (Param[AMRGTOEND]-Param[COXA]) ,2)) );

	if (Param[HIPTOEND] > (Param[TIBIA] + Param[FEMUR]) ){
		UpdateParam(HIPTOEND, (Param[TIBIA] + Param[FEMUR]) );
		UpdateHeight();
	}

	else if (Param[HIPTOEND] < (Param[TIBIA] - Param[FEMUR]) ){ 
		UpdateParam(HIPTOEND, 1.5 * (Param[TIBIA] - Param[FEMUR]) );	 // Param[TIBIA] - Param[FEMUR] is impossible position
		// Think of better way to calculate minimal position - use Hip limit angle
		UpdateHeight();
	}

	IKUpdateHipKnee();
}


// Updates Hip and Knee angles according to the current HIPTOEND and HEIGHT values
void Leg::IKUpdateHipKnee(){

	// Cosine Rule to find new Knee Servo Angle
	ServoAngle[KNEE] = acos( (ParamSQ[FEMUR] + ParamSQ[TIBIA] - ParamSQ[HIPTOEND] ) / ( 2 * Param[FEMUR] * Param[TIBIA]) );
	// Convert to actual angle for the servo
	ServoAngle[KNEE] = ServoAngle[KNEE] - PI;

	// Cosine Rule to find new Knee Servo Angle
	ServoAgnle[HIP] = acos( (ParamSQ[FEMUR] + ParamSQ[HIPTOEND] - ParamSQ[TIBIA] ) / ( 2 * Param[FEMUR] * Param[HIPTOEND]) );
	// Convert to actual angle for the servo
	ServoAngle[HIP] = ServoAngle[HIP] + acos(Param[HEIGHT]/Param[HIPTOEND]) - PI/2;
}


// input equals height required for the end effector
void Leg::Raise(const double& height){
	ServoAngle[KNEE] = ServoAngle[KNEE] + acos(1 -pow(height,2) / ParamSQ [TIBIA]); 
}


// input equals current height of the end effector
void Leg::Down(const double& height){
	ServoAngle[KNEE] = ServoAngle[KNEE] - acos(1 -pow(height,2) / ParamSQ [TIBIA]); 
}


void Leg::ArmQuadcopter(){
	double MotorAngle = asin( (Param[COXA]+Param[FEMUR])/Param[DISTCENTER] * sin(PI/12) );
	ServoAngle[ARM] = MotorAngle + (PI/12);

	int ArmID = Joints[ARM]->GetID();

	if( ArmID==ARM_RIGHT_FRONT || ArmID==ARM_LEFT_BACK) ServoAngle[ARM] = - ServoAngle[ARM];
}

void Leg::ArmHexacopter(){
	ServoAngle[ARM] = 0.0;
}

/* *********************************** END OF INVERSE KINEMATICS ********************************** */



/* *********************************** PARAMETER MANIPULATION ********************************** */

void Leg::UpdateHipToEnd(){
	UpdateParamSQ( HIPTOEND, (pow( (Param[ARMGTOEND]-Param[COXA]),2) + ParamSQ[HEIGHT]) );
}


void Leg::UpdateHeight(){
	UpdateParamSQ( HEIGHT, (ParamSQ[HIPTOEND] - pow( (Param[ARMGTOEND]-Param[COXA]),2) );
}


void Leg::UpdateParam(const int& idx, const double& value){
	Param[idx] = value;
	ParamSQ[idx] = pow(Param[idx], 2);
}


void Leg::UpdateParamSQ(const int& idx, const double& value){
	ParamSQ[idx] = value;
	Param[idx] = sqrt(ParamSQ[idx]);
}


void Leg::UpdateParam(const int& idx, const double& value, const double& valueSQ){
	Param[idx] = value;
	ParamSQ[idx] = valueSQ;
}

/* *********************************** END OF PARAMETER MANIPULATION ********************************** */



/* *********************************** SERVO MANIPULATION ********************************** */

void Leg::UpdateServoAngle(const int& idx, const double& pos){
	ServoAngle[idx]=pos;
}


void Leg::UpdateServoAngle(double NewPos[]){
	for(int i=0; i<JOINT_COUNT; i++){
		ServoAngle[i]=NewPos[i];
	}
}


int Leg::Straighten(){	
	return SetPosition(512, 512, 512, 512);	
}


// Have to come up with dynamic setting for that with given height	
int Leg::Stand(){
	return SetPosition(819, 512, 512, 512);	
}


// Works only for servos used in walking, does not write angle to Wing Servo
void Leg::WriteAngles(){
	for(int i=0; i<JOINTCOUNT-1; i++){
		Joints[i]->SetGoalPosition(ServoAngle[i]);
	}
}

/* *********************************** END OF SERVO MANIPULATION ********************************** */