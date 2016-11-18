#ifndef ROBOT_TYPES
#define ROBOT_TYPES

#include "ServoJoint.h"
#include <cmath>

struct BodyParams{

//public:
    //void operator=(const BodyParams& obj_in);
 
    double DIST_CENTER;
    double COXA;
    double FEMUR;
    double TIBIA;
    double KNEE_TO_MOTOR_DIST;
    double DIST_CENTER_SQ;
    double COXA_SQ;
    double FEMUR_SQ;
    double TIBIA_SQ;
    double KNEE_TO_MOTOR_DIST_SQ;

/*
    BodyParams(double dist_center, double coxa, double femur, double tibia, double knee_to_motor_dist) : 
        DIST_CENTER(dist_center), COXA(coxa), FEMUR(femur), TIBIA(tibia), KNEE_TO_MOTOR_DIST(knee_to_motor_dist),
        DIST_CENTER_SQ(pow(dist_center, 2)), COXA_SQ(pow(coxa, 2)), FEMUR_SQ(pow(femur, 2)), TIBIA_SQ(pow(tibia, 2)), KNEE_TO_MOTOR_DIST_SQ(pow(knee_to_motor_dist, 2)) {}

    const double DIST_CENTER;
    const double COXA;
    const double FEMUR;
    const double TIBIA;
    const double KNEE_TO_MOTOR_DIST;
    const double DIST_CENTER_SQ;
    const double COXA_SQ;
    const double FEMUR_SQ;
    const double TIBIA_SQ;
    const double KNEE_TO_MOTOR_DIST_SQ;

    @Not included:
        #define HIPKNEEMAXHDIST     4
*/
};

struct DynamicVars{

//public:
    void operator=(const DynamicVars& obj_in);
    void operator=(double value);

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

//public:
    
    /*
    LegAngles(double knee_in, double hip_in, double arm_in) :
        knee(knee_in), hip(hip_in), arm(arm_in) {}
    */
    
    void operator=(const LegAngles& obj_in);
    void operator=(double value);
  
    double knee;
    double hip;
    double arm;
    //double wing;
};

struct LegJoints{

    ServoJoint knee;
    ServoJoint hip;
    ServoJoint arm;
};

/*
struct JointLimits{

//public:

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


