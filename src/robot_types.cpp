#include "robot_types.h"

void LegAngles::operator=(const LegAngles& obj_in){
    if( this != &obj_in){
        this->knee  = obj_in.knee;
#ifdef DOF3
        this->hip   = obj_in.hip;
#endif
        this->arm   = obj_in.arm;
    }
}


void LegAngles::operator=(double value){
    knee    = value;
#ifdef DOF3
    hip     = value;
#endif
    arm     = value;
}

void LegAngles::print(){
    printf("LegAngles: knee(degrees) %f\n\r", wkq::degrees(knee));
#ifdef DOF3
    printf("LegAngles: hip(degrees) %f\n\r", wkq::degrees(hip));
#endif
    printf("LegAngles: arm(degrees) %f\n\r", wkq::degrees(arm));
}

void DynamicVars::operator=(const DynamicVars& obj_in){
    if( this != &obj_in){
        this->hip_to_end            = obj_in.hip_to_end;
        this->height                = obj_in.height;
        this->arm_ground_to_ef      = obj_in.arm_ground_to_ef;
        this->ef_center             = obj_in.ef_center;
        this->hip_to_end_sq         = obj_in.hip_to_end_sq;
        this->height_sq             = obj_in.height_sq;
        this->arm_ground_to_ef_sq   = obj_in.arm_ground_to_ef_sq;
        this->ef_center_sq          = obj_in.ef_center_sq;
    }
}


void DynamicVars::operator=(double value){
    hip_to_end          = value;
    height              = value;
    arm_ground_to_ef    = value;
    ef_center           = value;
    hip_to_end_sq       = value;
    height_sq           = value;
    arm_ground_to_ef_sq = value;
    ef_center_sq        = value;
}

void DynamicVars::print(){
    printf("DynamicVars: hip_to_end %f\n\r", hip_to_end);                      
    printf("DynamicVars: height %f\n\r", height);                          
    printf("DynamicVars: arm_ground_to_ef %f\n\r", arm_ground_to_ef);                
    printf("DynamicVars: ef_center %f\n\r", ef_center);                       
    printf("DynamicVars: hip_to_end_sq %f\n\r", hip_to_end_sq);
    printf("DynamicVars: height_sq %f\n\r", height_sq);
    printf("DynamicVars: arm_ground_to_ef_sq %f\n\r", arm_ground_to_ef_sq);
    printf("DynamicVars: ef_center_sq %f\n\r", ef_center_sq);
}
