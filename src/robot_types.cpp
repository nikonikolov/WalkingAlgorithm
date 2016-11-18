#include "robot_types.h"

void LegAngles::operator=(const LegAngles& obj_in){
    if( this != &obj_in){
        this->knee  = obj_in.knee;
        this->hip   = obj_in.hip;
        this->arm   = obj_in.arm;
    }
}


void LegAngles::operator=(double value){
    knee    = value;
    hip     = value;
    arm     = value;
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

