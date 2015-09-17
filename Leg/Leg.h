#ifndef LEG_H
#define LEG_H
#include "../ServoJoint/ServoJoint.h"
#include <cmath>

class Leg{

public:
	
	// FIX Constructos and DEFAULTS WHEN YOU KNOW THEM !!!
	Leg (ServoJoint& knee_in, ServoJoint& hip_in, ServoJoint& arm_in, ServoJoint& wing_in, 
		 double distcenter_in = 15, double coxa_in = 2, double femur_in = 25, double tibia_in = 25);
	

	int Straighten();
	int Stand();

	// Use radians as inputs; Servo library automatically converts it to corresponding position
	void SetPosition(const double& knee_pos, const double& hip_pos, const double& arm_pos, const double& wing_pos);
	void SetPosition(const double& knee_pos, const double& hip_pos, const double& arm_pos);
	int SetJointPosition(const int& JointIdx, const double& position);
	
	// Overload with int type inputs to set directly the servo position
	void SetPosition(const int& knee_pos, const int& hip_pos, const int& arm_pos, const int& wing_pos);
	void SetPosition(const int& knee_pos, const int& hip_pos, const int& arm_pos);
	int SetJointPosition(const int& JointIdx, const int& position);

	void IKForward(const double& dist);
	void IKRotate(const double& angle);
	void Lift(const double& height);
	void Down(const double& height);


	void UpdateArrayPos(double NewPos[]);
	void UpdateArrayPos(const int& idx, const double& pos);

	void WriteAngles();

	// ADD FUNCTIONS TO WRITE ARRAY TO SERVOS, ACTIVATE AND WAIT FOR ACTIVATION WRITE

private:
	void IKUpdateHipKnee();
	double angleScale(const int& angle);

	ServoJoint *Joints[JOINT_COUNT];
	double ServoAngle[JOINT_COUNT];		// In radians: 0.0 - center, positive - CW, negative - CCW
	double Param[PARAM_COUNT];
	double ParamSQ[PARAMSQ_COUNT];		// Holds Square Values of the parameters as they are frequently used
	
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
#define ANGLEOFFSET 	6			// Agnle between Y-axis and servo orientation; always positive

#define PARAM_COUNT		7
#define PARAMSQ_COUNT	6


#endif// LEG_H
