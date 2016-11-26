#include "Leg.h"

/* ===================================================== PUBLIC METHODS ===================================================== */

// Tripod constructor does not write to angles, so Leg constrcutor is only responsible for calculating the proper defaultPos
#ifdef DOF3
Leg::Leg(int ID_knee, int ID_hip, int ID_arm, DnxHAL* dnx_hips_knees, DnxHAL* dnx_arms, double height_in, const BodyParams& robot_params) :
    state(height_in, robot_params), 
    // Instantiate joints
    joints(ID_knee, dnx_hips_knees, ID_hip, dnx_hips_knees, ID_arm, dnx_arms) {
#else   
Leg::Leg(int ID_knee, int ID_hip, DnxHAL* dnx_hips_knees, DnxHAL* dnx_arms, double height_in, const BodyParams& robot_params) :
    state(height_in, robot_params), 
    // Instantiate joints
    joints(ID_knee, dnx_hips_knees, ID_hip, dnx_arms) {
#endif

    switch(ID_knee){
        case wkq::KNEE_LEFT_FRONT:
            angle_offset = wkq::radians(30);
            leg_id = wkq::LEG_LEFT_FRONT;
            leg_right = false;
            break;
        case wkq::KNEE_LEFT_MIDDLE:
            angle_offset = wkq::radians(30+60);
            leg_id = wkq::LEG_LEFT_MIDDLE;
            leg_right = false;
            break;
        case wkq::KNEE_LEFT_BACK:
            angle_offset = wkq::radians(30+120);
            leg_id = wkq::LEG_LEFT_BACK;
            leg_right = false;
            break;
        case wkq::KNEE_RIGHT_FRONT:
            angle_offset = wkq::radians(30);
            leg_id = wkq::LEG_RIGHT_FRONT;
            leg_right = true;
            break;
        case wkq::KNEE_RIGHT_MIDDLE:
            angle_offset = wkq::radians(30+60);
            leg_id = wkq::LEG_RIGHT_MIDDLE;
            leg_right = true;
            break;
        case wkq::KNEE_RIGHT_BACK:
            angle_offset = wkq::radians(30+120);
            leg_id = wkq::LEG_RIGHT_BACK;
            leg_right = true;
            break;
        default:
            printf("WARNING: Invalid KNEE ID IN LEG CONSTRUCTOR\n\r");
    }
}

Leg::~Leg(){}

/* ------------------------------------------------- STATIC POSITIONS ------------------------------------------------- */

void Leg::setPosition(wkq::RobotState_t robot_state){
    switch(robot_state){
        case wkq::RS_DEFAULT:
            state.legDefaultPos();
            break;              
        case wkq::RS_STANDING:
            state.legStand();
            break;             
        case wkq::RS_CENTERED:
            state.legCenter();
            break;             
        case wkq::RS_STANDING_QUAD:
            state.legStand();
            confQuadArms();
            break;        
        case wkq::RS_FLAT_QUAD:
            confQuadArms();
            state.legFlatten();
            break;            
        case wkq::RS_RECTANGULAR:
            confRectangular();
            break;          
        default:
            printf("ERROR: Leg::setPosition - input state not implemented\n\r");
    }
}


void Leg::confQuadArms(){
#ifdef DOF3
    // MIDDLE LEGS
    if(leg_id == wkq::LEG_RIGHT_MIDDLE || leg_id == wkq::LEG_LEFT_MIDDLE) state.servo_angles.arm = 0.0;
    
    else{
        double arm_offset = asin( state.params.DIST_CENTER * sin(wkq::PI/12)/(state.params.COXA+state.params.FEMUR-state.params.KNEE_TO_MOTOR_DIST) );

        // Remember that arm is pointing downwards
        // FRONT LEGS
        if(leg_id == wkq::LEG_RIGHT_FRONT || leg_id == wkq::LEG_LEFT_FRONT) state.servo_angles.arm = arm_offset + (wkq::PI/12);
        // BACK LEGS
        else if(leg_id == wkq::LEG_RIGHT_BACK || leg_id == wkq::LEG_LEFT_BACK) state.servo_angles.arm = - (arm_offset + (wkq::PI/12));
    }
#else
    // MIDDLE LEGS
    if(leg_id == wkq::LEG_RIGHT_MIDDLE || leg_id == wkq::LEG_LEFT_MIDDLE) state.servo_angles.hip = 0.0;
    
    else{
        double hip_offset = asin( state.params.DIST_CENTER / ( state.params.FEMUR - state.params.KNEE_TO_MOTOR_DIST) * sin(wkq::PI/12) ) ;

        // Remember that hip is pointing upwards
        // FRONT LEGS
        if(leg_id == wkq::LEG_RIGHT_FRONT || leg_id == wkq::LEG_LEFT_FRONT) state.servo_angles.hip = - (hip_offset + (wkq::PI/12));
        // BACK LEGS
        else if(leg_id == wkq::LEG_RIGHT_BACK || leg_id == wkq::LEG_LEFT_BACK) state.servo_angles.hip = hip_offset + (wkq::PI/12);
    }
#endif
}


void Leg::confRectangular(){
#ifdef DOF3
    state.servo_angles.arm = angle_offset - wkq::PI/2;
#else
    state.servo_angles.hip = angle_offset - wkq::PI/2;
    state.legCenter();
#endif
}


/* ------------------------------------------------- RAISE AND LOWER ------------------------------------------------- */

// input equals height required for the end effector; Untested for all joints and negative input
void Leg::liftUp(double height){
#ifdef DOF3
    state.servo_angles.knee = wkq::PI - state.servo_angles.knee + acos(1 - pow(height,2) / state.params.TIBIA_SQ); 
#else
    state.servo_angles.knee -= 2*asin( sqrt(2)/2 * height/ state.params.TIBIA); 
#endif
}

// input equals current height of the end effector
void Leg::lowerDown(double height){
#ifdef DOF3
    state.servo_angles.knee = wkq::PI - state.servo_angles.knee - acos(1 - pow(height,2) / state.params.TIBIA_SQ); 
#else
    state.servo_angles.knee += 2*asin( sqrt(2)/2 * height/ state.params.TIBIA); 
#endif
}

/*  
    Put End Effector down with ARM, HIP and KNEE centered. This is equivalent to properly terminating a movement forward
    Note that End Effector is still kept on the same line as HIP and KNEE are centered for this position
*/
void Leg::finishStep(){
    state.centerAngles();
}


/* ------------------------------------------------- WALKING ALGORITHMS ------------------------------------------------- */ 



// Valid calculations for negative input as well in the current form
void Leg::IKBodyForward(double step_size){

#ifdef DOF3
    double step_size_sq = pow(step_size,2);

    // Cosine Rule to find new hip_to_endSQ
    double arm_ground_to_ef_sq_new = state.vars.arm_ground_to_ef_sq + step_size_sq - 
                                2 * step_size * state.vars.arm_ground_to_ef * cos( angle_offset + state.servo_angles.arm ) ;

    double arm_ground_to_ef_new = sqrt(arm_ground_to_ef_sq_new);

    // Cosine Rule to find new Arm Servo Angle
    double ArmTmp = acos( (step_size_sq + arm_ground_to_ef_sq_new - state.vars.arm_ground_to_ef_sq) / ( 2 * step_size * arm_ground_to_ef_new ) );
    // Convert to actual angle for the servo
    state.servo_angles.arm = wkq::PI - angle_offset - ArmTmp;           // NB: Valid for a LEFT servo facing down

    state.updateVar(&(state.vars.arm_ground_to_ef), arm_ground_to_ef_new, &(state.vars.arm_ground_to_ef_sq), arm_ground_to_ef_sq_new);      // Automatically changes robot state in software
#else   
    double step_size_sq = pow(step_size,2);

    // Cosine Rule to find new hip_to_endSQ
    double hip_ground_to_ef = state.vars.hip_ground_to_ef_sq + step_size_sq - 
                                2 * step_size * state.vars.hip_ground_to_ef * cos( angle_offset + state.servo_angles.arm ) ;

    double arm_ground_to_ef_new = sqrt(hip_ground_to_ef);

    // Cosine Rule to find new Arm Servo Angle
    double ArmTmp = acos( (step_size_sq + hip_ground_to_ef - state.vars.hip_ground_to_ef_sq) / ( 2 * step_size * arm_ground_to_ef_new ) );
    // Convert to actual angle for the servo
    state.servo_angles.arm = wkq::PI - angle_offset - ArmTmp;           // NB: Valid for a LEFT servo facing down

    state.updateVar(&(state.vars.hip_ground_to_ef), arm_ground_to_ef_new, &(state.vars.hip_ground_to_ef_sq), hip_ground_to_ef);      // Automatically changes robot state in software

/*  double step_size_sq = pow(step_size,2);
    double dir_offset = 0.0;
    if(state.servo_angles.arm != 0.0){
        // Works for both positive and negative state.servo_angles.arm
        dir_offset = state.params.FEMUR / state.vars.center_ground_to_ef * (wkq::PI - sin(state.servo_angles.arm));
    }

    double ef_dir_angle = angle_offset - dir_offset;

    // Find the new center_ground_to_ef
    state.vars.center_ground_to_ef_sq = state.vars.center_ground_to_ef_sq + step_size_sq - 2 * step_size * state.vars.center_ground_to_ef * cos(ef_dir_angle);
    state.vars.center_ground_to_ef = sqrt(state.vars.center_ground_to_ef_sq);

    // Find the direct distance between center and end effector
    double center_to_ef_sq = center_ground_to_ef_new_sq + state.vars.height_sq;
    double center_to_ef = sqrt(center_to_ef_sq);

    double knee_new = acos( 
            (state.params.TIBIA_SQ + pow(state.params.FEMUR + state.params.DIST_CENTER, 2) - center_to_ef_sq) / ( 2 * state.params.TIBIA * (state.params.FEMUR + state.params.DIST_CENTER) )
                        );

    state.servo_angles.knee = wkq::PI - knee_new;
*/
#endif
}


// Assumes the leg starts from default position
void Leg::IKBodyForwardRectangularGait(double step_size){
#ifdef DOF3
    /*
        FILL IN
    */
#else
    // Non-sliding case
    //printf("FEMUR: %f, step size: %f, atan: %f\n\r", state.params.FEMUR, step_size, atan( state.params.FEMUR / step_size));
    //double hip_offset;
    state.servo_angles.hip -= atan( state.params.FEMUR / step_size) - wkq::PI/2;

    //hip_offset = 2*state.params.FEMUR_SQ 

    double hip_knee_dist = sqrt(pow(state.params.FEMUR, 2) + pow(step_size, 2));

    state.servo_angles.knee = wkq::PI/2 - asin( (hip_knee_dist - state.params.FEMUR) / state.params.TIBIA );


    // Sliding case
    /*
    state.servo_angles.hip = atan( state.params.FEMUR / step_size) - wkq::PI/2;

    double hip_knee_dist = sqrt(pow(state.params.FEMUR,2) + pow(step_size, 2));

    state.servo_angles.knee = wkq::PI/2 - asin( (hip_knee_dist - state.params.FEMUR) / state.params.TIBIA );
    */
#endif
}

// Assumes the leg starts from default position
void Leg::stepForwardRectangularGait(double step_size){
#ifdef DOF3
    /*
        FILL IN
    */
#else
    // Non-sliding case
    state.servo_angles.hip += wkq::PI/2 - atan( state.params.FEMUR / step_size);

    double hip_knee_dist = sqrt(pow(state.params.FEMUR,2) + pow(step_size, 2));

    state.servo_angles.knee = wkq::PI/2 - asin( (hip_knee_dist - state.params.FEMUR) / state.params.TIBIA );


    // Sliding case
    /*
    state.servo_angles.hip = atan( state.params.FEMUR / step_size) - wkq::PI/2;

    double hip_knee_dist = sqrt(pow(state.params.FEMUR,2) + pow(step_size, 2));

    state.servo_angles.knee = wkq::PI/2 - asin( (hip_knee_dist - state.params.FEMUR) / state.params.TIBIA );
    */
#endif
}


// Input is the currently used step size
void Leg::stepForward(double step_size){
#ifdef DOF3
    // Distance from Robot center to end effector for a fully centered robot at this height
    double ef_center = state.vars.ef_center;

    // Create Point representing the new END EFFECTOR position
    double x_EFNew, y_EFNew;
    if(leg_id == wkq::LEG_RIGHT_MIDDLE || leg_id == wkq::LEG_LEFT_MIDDLE){          // Middle Joint
        //x_EFNew = ef_center - step_size/sqrt(3); - change when you have prev and current step size as inputs
        x_EFNew = ef_center - step_size/sqrt(3);
        y_EFNew = 0.0;
    }   
    else{
        x_EFNew = ef_center/2.0 + step_size/sqrt(3);
        y_EFNew = sqrt(3)*x_EFNew;                      // Front Joint

        if(leg_id == wkq::LEG_RIGHT_BACK || leg_id == wkq::LEG_LEFT_BACK)   y_EFNew -= sqrt(3)*ef_center;           // Back Joint
    }       
    wkq::Point EFNew(x_EFNew, y_EFNew);

    // Create Point representing current HIP position
    double hip_arg = wkq::PI/2 -angle_offset;
    wkq::Point p_hip(state.params.DIST_CENTER*cos(hip_arg), state.params.DIST_CENTER*sin(hip_arg));

    // Create Point representing the new HIP position
    wkq::Point p_hip_new(p_hip);
    p_hip_new.translate_y(step_size*2);

    // Calculate new arm_ground_to_ef
    double arm_ground_to_ef_sq_new = p_hip.dist_sq(EFNew);

    // Cosine Rule to find new ARM angle
    double arg_triangle = 
            acos( (p_hip.dist_sq(p_hip_new) +  p_hip.dist_sq(EFNew) - p_hip_new.dist_sq(EFNew)) / (2*p_hip.dist(p_hip_new)*p_hip.dist_sq(EFNew)) );
    //state.servo_angles.arm = wkq::PI - angle_offset - arg_triangle;
    state.servo_angles.arm = arg_triangle - angle_offset ;      // Unconfirmed that valid for all joints

    state.updateVar(&(state.vars.arm_ground_to_ef_sq), arm_ground_to_ef_sq_new);                // Automatically changes hip_to_end, HIP and KNEE   
#else
    /*
        FILL IN
    */
#endif
}



void Leg::IKBodyRotate(double angle){
#ifdef DOF3
    // Sine Rule to find rotation distance
    double rot_dist = 2 * state.params.DIST_CENTER * sin (fabs(angle)/2);

    double rot_dist_sq = pow(rot_dist, 2);

    // Cosine Rule to find new arm_ground_to_efSQ
    double arm_ground_to_ef_sq_new;
    if (angle>=0.0) arm_ground_to_ef_sq_new = state.vars.arm_ground_to_ef_sq + rot_dist_sq - 
                                    2 * rot_dist * state.vars.arm_ground_to_ef * cos( wkq::PI/2 + angle/2 - state.servo_angles.arm );
    else arm_ground_to_ef_sq_new = state.vars.arm_ground_to_ef_sq + rot_dist_sq - 
                                    2 * rot_dist * state.vars.arm_ground_to_ef * cos( wkq::PI/2 - angle/2 + state.servo_angles.arm );   

    double arm_ground_to_ef_new = sqrt(arm_ground_to_ef_sq_new);

    // Cosine Rule to find new Arm Servo Angle
    double ArmTmp = acos( (rot_dist_sq + arm_ground_to_ef_sq_new - state.vars.arm_ground_to_ef_sq) / ( 2 * rot_dist * arm_ground_to_ef_new ) );
    // Convert to actual angle for the servo - valid for a LEFT LEG servo facing down
    if (angle>=0.0) state.servo_angles.arm = wkq::PI/2 - ArmTmp + angle/2;  // - (- wkq::PI/2 + state.servo_angles.arm - angle/2)
    else            state.servo_angles.arm = -wkq::PI/2 + ArmTmp + angle/2; // - (  wkq::PI/2 - state.servo_angles.arm - angle/2)

    state.updateVar(&(state.vars.arm_ground_to_ef), arm_ground_to_ef_new, &(state.vars.arm_ground_to_ef_sq), arm_ground_to_ef_sq_new);                  // Automatically changes robot state in software
#else
    /*
        FILL IN
    */
#endif
}


// Input is the currently used step size
void Leg::stepRotate(double step_size){
}


// input equals height to be raised by; negative values also work
void Leg::raiseBody(double hraise){

    state.updateVar(&(state.vars.height), (state.vars.height + hraise));            // hip_to_end, HIP and KNEE automatically get updated

/*
    if (&(state.vars.hip_to_end) > (state.params.TIBIA + state.params.FEMUR) ){
        state.updateVar(&(state.vars.hip_to_end), (state.params.TIBIA + state.params.FEMUR) );
        UpdateHeight();
    }

    else if (&(state.vars.hip_to_end) < (state.params.TIBIA - state.params.FEMUR) ){ 
        state.updateVar(&(state.vars.hip_to_end), 1.5 * (state.params.TIBIA - state.params.FEMUR) );     // state.params.TIBIA - state.params.FEMUR is impossible position
        // Think of better way to calculate minimal position - use Hip limit angle
        UpdateHeight();
    }
*/
}

/* ------------------------------------------------- WRITING TO SERVOS ------------------------------------------------- */


// Write state.servo_angles[] to physcial servos in order ARM, HIP, KNEE
void Leg::writeAngles(){
    if(debug_) printf("Leg: enter writeAngles\n\r");

    if(!leg_right){
        if(debug_) printf("Leg: writeAngles - left leg\n\r");
#ifdef DOF3
        joints.arm.setGoalPosition(state.servo_angles.arm);
#endif
        joints.hip.setGoalPosition(state.servo_angles.hip);
        joints.knee.setGoalPosition(state.servo_angles.knee);
    }
    else{
        if(debug_) printf("Leg: writeAngles - right leg\n\r");
    
#ifdef DOF3
        if(state.servo_angles.arm != 0)     joints.arm.setGoalPosition(-state.servo_angles.arm);
        else                                joints.arm.setGoalPosition(state.servo_angles.arm);
#endif
        if(state.servo_angles.hip != 0)     joints.hip.setGoalPosition(-state.servo_angles.hip);
        else                                joints.hip.setGoalPosition(state.servo_angles.hip);
        if(state.servo_angles.knee != 0)    joints.knee.setGoalPosition(-state.servo_angles.knee);
        else                                joints.knee.setGoalPosition(state.servo_angles.knee);
    }

    if(debug_) printf("Leg: done with writeAngles\n\r");
}

/*
// Write state.servo_angles[] to physcial servos in order ARM, HIP, KNEE
void Leg::writeKnee(){
    if(debug_) printf("Leg: enter writeAngles\n\r");

    if(!leg_right){
        if(debug_) printf("Leg: writeAngles - left leg\n\r");
        joints.knee.setGoalPosition(state.servo_angles.knee);
    }
    else{
        if(debug_) printf("Leg: writeAngles - right leg\n\r");
    
        if(state.servo_angles.knee != 0)    joints.knee.setGoalPosition(-state.servo_angles.knee);
        else                                joints.knee.setGoalPosition(state.servo_angles.knee);
    }

    if(debug_) printf("Leg: done with writeAngles\n\r");
}
*/
void Leg::copyState(const Leg& leg_in){
    if(this != &leg_in){
        this->state = leg_in.state;
    }
}

