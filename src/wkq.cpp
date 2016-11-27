#include "wkq.h"

wkq::Point::Point() : x(0.0), y(0.0) {}

wkq::Point::Point(double x_in, double y_in) : x(x_in), y(y_in) {}

wkq::Point::Point(const Point& p_in) : x(p_in.x), y(p_in.y) {}


wkq::Point::~Point(){}

wkq::Point::Point(double x_in, double y_in, double mag_in, double arg_in) : mag(mag_in), arg(arg_in) {}


void wkq::Point::translate_y(double delta_y){
	y += delta_y;
	update_polar_coord();
}
void wkq::Point::translate_x(double delta_x){
	x += delta_x;
	update_polar_coord();
}

void wkq::Point::update_rect_coord(){
	x=mag*cos(arg);
	y=mag*sin(arg);
}

void wkq::Point::update_polar_coord(){
	arg = atan(y/x);
	mag = sqrt(y*y + x*x);
}


void wkq::Point::set_arg(double arg_in){
	arg=arg_in;
	update_rect_coord();
}

void wkq::Point::set_mag(double mag_in){
	mag=mag_in;
	update_rect_coord();
}


/*
void Point::origin_symmetric(){
	x=-x;
	y=-y;
}

	void Point :: translate(const Point& p2){
		x=x+p2.x;
		y=y+p2.y;
	}

bool operator<(const Point& p1, const Point& p2){
	return (p1.origin_dist() < p2.origin_dist());
}

bool operator==(const Point& p1, const Point& p2){
	return (p1.x==p2.x)&&(p1.y==p2.y);
}
*/




