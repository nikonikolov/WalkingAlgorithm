#ifndef ROBOT_TYPES
#define ROBOT_TYPES

#include <cmath>

#include "ServoJoint.h"
#include "wkq.h"

struct BodyParams{

    double DIST_CENTER;
#ifdef DOF3
    double COXA;
#endif
    double FEMUR;
    double TIBIA;
    double KNEE_TO_MOTOR_DIST;
    double DIST_CENTER_SQ;
    double COXA_SQ;
    double FEMUR_SQ;
    double TIBIA_SQ;
    double KNEE_TO_MOTOR_DIST_SQ;
    double MAX_HEIGHT;
    double MIN_HEIGHT;

    void compute_squares(){
        DIST_CENTER_SQ = pow(DIST_CENTER, 2);
#ifdef DOF3
        COXA_SQ = pow(COXA, 2);
#endif        
        FEMUR_SQ = pow(FEMUR, 2);
        TIBIA_SQ = pow(TIBIA, 2);
        KNEE_TO_MOTOR_DIST_SQ = pow(KNEE_TO_MOTOR_DIST, 2);
    }
};


/*
    @ Remember to update State_t::updateVar and State_t::update if you add or remove any members
*/
struct DynamicVars{

    void print();
    void operator=(const DynamicVars& obj_in);
    void operator=(double value);

    double center_ground_to_ef;             // Direct distance from the center ground projection to the end effector
    //double height;                          // Height of the robot center as it stays invariant
    //double


    double hip_to_end;                      // Direct distance from hip mount point to the point where the end effector touches ground
    double height;                          // Vertical distance from hip mount point to ground
    double arm_ground_to_ef;                // Distance from arm ground projection to end effector
    double ef_center;                       // Distance from End Effector to robot center for centered HIP and KNEE
    double hip_to_end_sq;
    double height_sq;
    double arm_ground_to_ef_sq;
    double ef_center_sq;
};  


struct LegAngles{
    
    void print();
    void operator=(const LegAngles& obj_in);
    void operator=(double value);
  
    double knee;
#ifdef DOF3
    double hip;
#endif
    double arm;
};


struct LegJoints{

#ifndef DOF3
    LegJoints(int ID_knee, DnxHAL* dnx_port_knee, int ID_arm, DnxHAL* dnx_port_arm) :
        knee(ID_knee, dnx_port_knee), arm(ID_arm, dnx_port_arm) {}
#else    
    LegJoints(int ID_knee, DnxHAL* dnx_port_knee, int ID_hip, DnxHAL* dnx_port_hip, int ID_arm, DnxHAL* dnx_port_arm) :
        knee(ID_knee, dnx_port_knee), hip(ID_hip, dnx_port_hip), arm(ID_arm, dnx_port_arm) {}
#endif

    ServoJoint knee;
#ifdef DOF3
    ServoJoint hip;
#endif
    ServoJoint arm;
};



/*
struct TripodLegs{
    Leg front_leg;
    Leg back_leg;
    Leg middle_leg;
};

struct JointLimits{

    JointLimits(knee_max_in, knee_min_in, hip_max_in, hip_min_in, arm_max_in, arm_min_in) :
        knee_max(knee_max_in), knee_min(knee_min_in), 
        hip_max(hip_max_in), hip_min(hip_min_in), 
        arm_max(arm_max_in), arm_min(arm_min_in) 
        {}

    const double knee_max;
    const double knee_min;

    const double hip_max;
    const double hip_min;

    const double arm_max;
    const double arm_min;
};
*/

#endif


