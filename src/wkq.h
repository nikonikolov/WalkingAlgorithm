/* 

Walking Quadcopter Namespace
===========================================================================================

	Used to define some functions and types specific to the robot without introducing name clashes

-------------------------------------------------------------------------------------------

*/

#ifndef wkq_NAMESPACE_H
#define wkq_NAMESPACE_H

#include <cmath>

namespace wkq{


	const double PI = 3.14159265358979323846264338327950288419716939937510;

	inline double radians(double degrees){	
		return degrees/180.0*wkq::PI; 
	}

	inline double degrees(double radians){
		return radians/wkq::PI*180.0;
	}

	inline bool compare_doubles(double a, double b, double error = 0.01){
		return fabs(a - b) <= error;
	}

	/* 	SERVO NAMES: Number corresponds to ID of the servo
	 	NOTE: Try to keep IDs the same. Changing them will cause bugs in Tripod constructor parameters 
	 	and you will also have to change the ID of the physical servo */

	// SERIAL TX-9 RX-10 - KNEES
	const int KNEE_LEFT_FRONT =			11;				// M7
	const int KNEE_LEFT_MIDDLE =		12;				// M9
	const int KNEE_LEFT_BACK =			13;				// M1
	const int KNEE_RIGHT_FRONT =		14; 			// M11
	const int KNEE_RIGHT_MIDDLE =		15;				// M6
	const int KNEE_RIGHT_BACK =			16; 			// M5

	// SERIAL TX-9 RX-10 - HIPS
	const int HIP_LEFT_FRONT =			17; 			// M4
	const int HIP_LEFT_MIDDLE =			18; 			// M2
	const int HIP_LEFT_BACK =			19;				// M10
	const int HIP_RIGHT_FRONT =			20; 			// M12
	const int HIP_RIGHT_MIDDLE =		21; 			// M8
	const int HIP_RIGHT_BACK =			22; 			// M3

	// SERIAL TX-13 RX-14 - WINGS
	const int WING_LEFT_FRONT =			23; 			// M18
	const int WING_LEFT_MIDDLE =		24; 			// M13
	const int WING_LEFT_BACK =			25; 			// M14
	const int WING_RIGHT_FRONT =		26; 			// M17
	const int WING_RIGHT_MIDDLE =		27; 			// M16
	const int WING_RIGHT_BACK =			28; 			// M15

	// SERIAL TX-13 RX-14 - ARMS
	const int ARM_LEFT_FRONT =			29; 			// M18
	const int ARM_LEFT_MIDDLE =			30; 			// M13
	const int ARM_LEFT_BACK =			31; 			// M14
	const int ARM_RIGHT_FRONT =			32; 			// M17
	const int ARM_RIGHT_MIDDLE =		33; 			// M16
	const int ARM_RIGHT_BACK =			34; 			// M15

	const int knees_start = 			11;
	const int hips_start = 				17;
	const int wings_start = 			23;
	const int arms_start = 				29;

	// LIMITS FOR ROTATION ANGLES FOR SERVO
	const int KNEE_MIN =				0;
	const int KNEE_MAX =				1024;
	const int HIP_MIN =					0;
	const int HIP_MAX =					1024;
	const int WING_MIN =				0;
	const int WING_MAX =				1024;	
	const int ARM_MIN =					0;
	const int ARM_MAX =					1024;


	enum LegID{
		LEG_LEFT_FRONT 			= 1,
		LEG_LEFT_MIDDLE 		= 2,
		LEG_LEFT_BACK 			= 3,
		LEG_RIGHT_FRONT 		= 4,
		LEG_RIGHT_MIDDLE 		= 5,
		LEG_RIGHT_BACK 			= 6
	};

	
	/* 	@Note: Be very careful when introducing new states - if the new state is meaningless in terms of leg configuration
		on the ground, i.e. backwards consistency with current position does no need to be kept, you need to signify that
		somehow in Robot::stand()
	*/
	enum RobotState_t{
		RS_DEFAULT 				= 0,			// all servos configured to default positions, i.e. centralized for stable stand
		RS_STANDING				= 1,			// body height equal to knee height, all knees to 90 degrees, the rest centered
		RS_CENTERED				= 2,			// body height equal to knee height, all knees to 90 degrees, the rest centered
		RS_STANDING_QUAD 		= 4, 			// standing with legs configured in Quad mode and configured to stand on the ground
		RS_FLAT_QUAD			= 5,			// standing with legs configured in Quad mode and fully flat
		RS_QUAD_SETUP			= 6, 			// Legs set so that pixhawk configuration can be perfromed 
		RS_RECTANGULAR 			= 7, 			// Legs set in a manner for Retangular Gait

		RS_FLY_STANDING_QUAD 	= 20, 			// Flying with legs configured in Quad mode but not fully flat
		RS_FLY_STRAIGHT_QUAD 	= 21, 			// Flying with legs configured in Quad mode and fully flat
		/*
		RS_idle 		= 0,			// not doing anything
		RS_in_flight 	= 1,			// flying with legs fully flat
		RS_walking 		= 2,			// walking
		RS_landing 		= 3,			// currently landing - knees on 90 degrees for landing
		RS_error 		= 4,			// not sure what is happening
		RS_hex			= 6,			// not completely defined yet
		RS_quad			= 7,			// not completely defined yet
		*/
	}; 

	enum RobotMovement_t{
		RM_HEXAPOD_GAIT 		= 0,
		RM_RECTANGULAR_GAIT 	= 1,
		RM_ROTATION_HEXAPOD 	= 2,
		RM_ROTATION_RECTANGULAR = 3,


		/*RM_BODY_FORWARD 		= 0,
		RM_STEP_FORWARD 		= 1,
		RM_BODY_FORWARD_RECT	= 2,
		RM_STEP_FORWARD_RECT	= 3,
		RM_BODY_ROTATE 			= 4,
		RM_STEP_ROTATE 			= 5,

		RM_LIFT_UP 				= 6,
		RM_LOWER_DOWN 			= 7,
		*/
	}; 


	enum LegStatus_t{
		LS_KNEE_MAX_OVERFLOW 	= 0,
		LS_KNEE_MIN_OVERFLOW 	= 1,
		LS_HIP_MAX_OVERFLOW 	= 2,
		LS_HIP_MIN_OVERFLOW 	= 3,
		LS_ARM_MAX_OVERFLOW 	= 4,
		LS_ARM_MIN_OVERFLOW 	= 5,
		LS_WING_MAX_OVERFLOW 	= 6,
		LS_WING_MIN_OVERFLOW 	= 7,

	}; 


/* ------------------------------------------------- POINT ------------------------------------------------- */ 

	struct Point{

		//Point();
		//Point(double x_in, double y_in, double mod_in, double arg_in);
		Point(double x_in, double y_in);
		Point(const Point& p_in);
		~Point();
		
		inline double get_x() const;
		inline double get_y() const;
		inline double get_mag() const;
		inline double get_arg() const;
		
		double origin_dist() const;
		double dist(const Point& p_in) const;
		double dist_sq(const Point& p_in) const;
		double line_arg(const Point& p_in);

		void translate_y(double delta_y);
		void translate_x(double delta_x);
		void rotate(double delta_arg);
		//void translate(const Point& p2);
		
	
		//void set_arg(double arg_in);
		//void set_mag(double mag_in);
	/*
		friend bool operator<(const Point& p1, const Point& p2);
		friend bool operator==(const Point& p1, const Point& p2);
	*/

	private:

		void update_rect_coord();
		void update_polar_coord();

		double x = 0;
		double y = 0;

		double mag = 0;
		double arg = 0;
	};


	double Point::get_x() const{
		return x;
	}
	double Point::get_y() const{
		return y;
	}
	double Point::get_arg() const{
		return arg;
	}
	double Point::get_mag() const{
		return mag;
	}



/* ------------------------------------------------- END POINT ------------------------------------------------- */ 



}


#endif

