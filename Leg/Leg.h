#ifndef LEG_H
#define LEG_H
#include "../ServoJoint/ServoJoint.h"
#include <cmath>

class Leg{

public:
	
	// FIX Constructos and DEFAULTS WHEN YOU KNOW THEM !!!
	Leg (ServoJoint& knee_in, ServoJoint& hip_in, ServoJoint& arm_in, ServoJoint& wing_in, 
		 double distcenter_in = 10.9, double coxa_in = 2.65, double femur_in = 25, double tibia_in = 25);

	// Use radians as inputs; Servo library automatically converts it to corresponding position
	void SetPosition(const double& knee_pos, const double& hip_pos, const double& arm_pos, const double& wing_pos);
	void SetPosition(const double& knee_pos, const double& hip_pos, const double& arm_pos);
	int SetJointPosition(const int& JointIdx, const double& position);
	
	// Overload with int type inputs to set directly the servo position
	void SetPosition(const int& knee_pos, const int& hip_pos, const int& arm_pos, const int& wing_pos);
	void SetPosition(const int& knee_pos, const int& hip_pos, const int& arm_pos);
	int SetJointPosition(const int& JointIdx, const int& position);

	int Straighten();
	int Stand();			// This one has to be repaired

	void IKForward(const double& dist);
	void IKRotate(const double& angle);
	void Raise(const double& height);
	void Down(const double& height);
	//void ConfigureQuadcopter();		// RaiseHeight function required for these to be implemented
	//void ConfigureHexacopter();
	void ArmQuadcopter();				// Update arm angle for ArmQuadcopter flight. No write to servo
	void ArmHexacopter();				// Update arm angle for HEXACOPTER flight. No write to servo

	void BodyHeight(const double& hraise);

	//void Default();

	void UpdateServoAngle(double NewPos[]);
	void UpdateServoAngle(const int& idx, const double& pos);

	void WriteAngles();

	// ADD FUNCTIONS TO WRITE ARRAY TO SERVOS, ACTIVATE AND WAIT FOR ACTIVATION WRITE

private:
	
	double ConvertAngle(const int& angle);

	void UpdateParam(const int& idx, const double& value);
	void UpdateParamSQ(const int& idx, const double& value);
	void UpdateParam(const int& idx, const double& value, const double& valueSQ);

	void IKUpdateHipKnee();
	void UpdateHipToEnd();
	void UpdateHeight();


	ServoJoint *Joints[JOINT_COUNT];
	double ServoAngle[JOINT_COUNT];		// In radians: 0.0 - center, positive - CW, negative - CCW
	double Param[PARAM_COUNT];
	double ParamSQ[PARAMSQ_COUNT];		// Holds Square Values of the parameters as they are frequently used
	
	double DefaultAngles[JOINT_COUNT]; 	// Store the dafault angles for a standing robot
};

const double PI = 3.14159265358979323846264338327950288419716939937510

// Indices for Joints and ServoAngle arrays
#define KNEE			0
#define HIP				1
#define ARM				2
#define WING 			3

#define JOINT_COUNT		4

// Indices for Param and ParamSQ array
#define DISTCENTER 		0			// Distance from Robot's Center to Coxa (Arm Mount Point)
#define COXA 			1
#define FEMUR 			2
#define TIBIA 			3
#define HIPTOEND		4			// Direct distance from hip mount point to the point where the end effector touches ground
#define HEIGHT 			5			// Vertical distance from hip mount point to ground
#define ARMGTOEND		6			// Distance from arm ground projection to end effector
#define ANGLEOFFSET 	7			// Agnle between Y-axis and servo orientation; always positive

#define PARAM_COUNT		8
#define PARAMSQ_COUNT	7


#endif// LEG_H
