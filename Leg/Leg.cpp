#include "Leg.h"

// DOUBLES OVERLOADING

void Leg::SetPosition(const double& knee_pos, const double& hip_pos, const double& arm_pos, const double& wing_pos/*=0.0*/){
	
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
	UpdatePos(JointIdx, position );
	return Joints[JointIdx]->SetGoalPosition(position);
}

// END OF DOUBLES OVERLOADING

// INTEGERS

void Leg::SetPosition(const int& knee_pos, const int& hip_pos, const int& arm_pos, const int& wing_pos/*=512*/){
	
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
	UpdatePos(JointIdx, angleScale(position) );
	return Joints[JointIdx]->SetGoalPosition(position);
}

// END OF INTEGERS OVERLOADING


Leg::Leg 	(ServoJoint& knee_in, ServoJoint& hip_in, ServoJoint& arm_in, ServoJoint& wing_in, 
			double& distcenter_in /*=default*/, double& coxa_in /*=default*/, double& femur_in /*=default*/, 
			double& tibia_in /*=default*/, double& heigh_in /*=default*/, double& angle_offset_in /*=default*/){

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


void Leg::UpdateArrayPos(const int& idx, const double& pos){
	ServoAngle[idx]=pos;
}


void Leg::UpdateArrayPos(double NewPos[]){
	for(int i=0; i<JOINT_COUNT; i++){
		ServoAngle[i]=NewPos[i];
	}
}


// CW - positive output, CCW - negative output
double Leg::angleScale(const int& angle){
	
	// 5.23598775598 rad = 300 degrees
	// 2.61799387799 rad = 150 degrees
	double result = 2.61799387799 - ((angle/1024)*5.23598775598);

	if (result>2.61799387799) return 2.61799387799;
	else if (result<-2.61799387799) return -2.61799387799; 
	else return result;
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


// Valid calculations for negative input as well in the current form
void Leg::IKForward(double dist){

	double distSQ = pow(dist,2);

	// Cosine Rule to find new HipToEndSQ
	double HipToEndNewSQ = ParamSQ[HIPTOEND] + distSQ - 
								2 * dist * Param[HIPTOEND] * cos( Param[ANGLEOFFSET] + ServoAngle[ARM] ) ;

	double HipToEndNew = sqrt(HipToEndNewSQ);


	// Cosine Rule to find new Arm Servo Angle
	ServoAgnle[ARM] = acos( distSQ + HipToEndNewSQ - ParamSQ[HIPTOEND]) / ( 2 * dist * HipToEndNew ) );
	// Convert to actual angle for the servo
	ServoAngle[ARM] = (PI - ServoAngle[ARM]) - param[ANGLEOFFSET];

	// Update new values
	Param[HIPTOEND]   = HipToEndNew;
	ParamSQ[HIPTOEND] = HipToEndNewSQ;

	IKUpdateHipKnee();
}


void Leg::IKRotate(double angle){

	// Sine Rule to find rotation distance
	double RotDist = 2 * Param[DISTCENTER] * sin (angle/2);

	double RotDistSQ = pow(RotDist, 2);

	// Cosine Rule to find new HipToEndSQ
	double HipToEndNewSQ = ParamSQ[HIPTOEND] + RotDistSQ - 
								2 * RotDist * Param[HIPTOEND] * cos( PI/2 - ServoAngle[ARM] + angle/2 ) ;

	double HipToEndNew = sqrt(HipToEndNewSQ);

	// Cosine Rule to find new Arm Servo Angle
	ServoAgnle[ARM] = acos( RotDistSQ + HipToEndNewSQ - ParamSQ[HIPTOEND]) / ( 2 * RotDist * HipToEndNew ) );
	// Convert to actual angle for the servo
	ServoAngle[ARM] = ServoAngle[ARM] - PI/2 - angle/2;

	// Update new values
	Param[HIPTOEND]   = HipToEndNew;
	ParamSQ[HIPTOEND] = HipToEndNewSQ;

	IKUpdateHipKnee();
}


// Updates Hip and Knee angles according to the current HIPTOEND and HEIGHT values
void Leg::IKUpdateHipKnee(){
	
	// Cosine Rule to find new Knee Servo Angle
	ServoAgnle[KNEE] = acos( ParamSQ[FEMUR] + ParamSQ[TIBIA] - ParamSQ[HIPTOEND] ) / ( 2 * Param[FEMUR] * Param[TIBIA]) );

	// Cosine Rule to find new Knee Servo Angle
	ServoAgnle[HIP] = acos( ParamSQ[FEMUR] + ParamSQ[HIPTOEND] - ParamSQ[TIBIA] ) / ( 2 * Param[FEMUR] * Param[HIPTOEND]) );
	// Convert to actual angle for the servo
	ServoAngle[HIP] = PI - ServoAngle[HIP] - acos(Param[HEIGHT]/Param[HIPTOEND]);
}
