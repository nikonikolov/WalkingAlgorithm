#include "robot_types.h"

void LegAngles::operator=(const LegAngles& obj_in){
    if( this != &obj_in){
        this->knee  = obj_in.knee;
        this->hip   = obj_in.hip;
#ifdef DOF3
        this->arm   = obj_in.arm;
#endif
    }
}


void LegAngles::operator=(double value){
    knee    = value;
    hip     = value;
#ifdef DOF3
    arm     = value;
#endif
}

void LegAngles::print(){
    printf("LegAngles: knee(degrees) %f\n\r", wkq::degrees(knee));
    printf("LegAngles: hip(degrees) %f\n\r", wkq::degrees(hip));
#ifdef DOF3
    printf("LegAngles: arm(degrees) %f\n\r", wkq::degrees(arm));
#endif
}

void DynamicVars::operator=(const DynamicVars& obj_in){
    if( this != &obj_in){
        this->height                = obj_in.height;
        this->height_sq             = obj_in.height_sq;
#ifdef DOF3
        this->hip_to_end            = obj_in.hip_to_end;
        this->hip_to_end_sq         = obj_in.hip_to_end_sq;
        this->arm_ground_to_ef      = obj_in.arm_ground_to_ef;
        this->arm_ground_to_ef_sq   = obj_in.arm_ground_to_ef_sq;
        this->ef_center             = obj_in.ef_center;
        this->ef_center_sq          = obj_in.ef_center_sq;
#endif
    }
}


void DynamicVars::operator=(double value){
    height              = value;
    height_sq           = value;
#ifdef DOF3
    hip_to_end          = value;
    hip_to_end_sq       = value;
    arm_ground_to_ef    = value;
    arm_ground_to_ef_sq = value;
    ef_center           = value;
    ef_center_sq        = value;
#endif
}

void DynamicVars::print(){
    printf("DynamicVars: height %f\n\r", height);                          
    printf("DynamicVars: height_sq %f\n\r", height_sq);
#ifdef DOF3
    printf("DynamicVars: hip_to_end %f\n\r", hip_to_end);                      
    printf("DynamicVars: hip_to_end_sq %f\n\r", hip_to_end_sq);
    printf("DynamicVars: arm_ground_to_ef %f\n\r", arm_ground_to_ef);                
    printf("DynamicVars: arm_ground_to_ef_sq %f\n\r", arm_ground_to_ef_sq);
    printf("DynamicVars: ef_center %f\n\r", ef_center);                       
    printf("DynamicVars: ef_center_sq %f\n\r", ef_center_sq);
#endif
}
