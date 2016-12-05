#include "wkq.h"


wkq::Point::Point(double x_in, double y_in) : x(x_in), y(y_in) {
	update_polar_coord();
}

wkq::Point::Point(const Point& p_in) : x(p_in.x), y(p_in.y) {
	update_polar_coord();
}

//wkq::Point::Point() : x(0.0), y(0.0) {}
//wkq::Point::Point(double x_in, double y_in, double mag_in, double arg_in) : mag(mag_in), arg(arg_in) {}

wkq::Point::~Point(){}



double wkq::Point::origin_dist() const{
	return sqrt(x*x + y*y);
}

double wkq::Point::dist(const Point& p_in) const{
	return sqrt(dist_sq(p_in));
}

double wkq::Point::dist_sq(const Point& p_in) const{
	return pow((x-p_in.x),2) + pow((y-p_in.y),2);
}

double wkq::Point::line_arg(const Point& p_in){
	//return atan( (p_in.y - y) / (p_in.x - x) );
	return atan2( p_in.y - y, p_in.x - x );
}


void wkq::Point::translate_y(double delta_y){
	y += delta_y;
	update_polar_coord();
}

void wkq::Point::translate_x(double delta_x){
	x += delta_x;
	update_polar_coord();
}

void wkq::Point::translate(const Point& p2){
	x += p2.x;
	y += p2.y;
	update_polar_coord();
}


void wkq::Point::rotate(double delta_arg){
	arg -= delta_arg;
	//arg += delta_arg;
	update_rect_coord();
}


void wkq::Point::update_rect_coord(){
	x = mag * cos(arg);
	y = mag * sin(arg);
}

void wkq::Point::update_polar_coord(){
	//arg = atan(y/x);
	arg = atan2(y, x);
	mag = sqrt(y*y + x*x);
}




/*
void wkq::Point::set_arg(double arg_in){
	arg=arg_in;
	update_rect_coord();
}

void wkq::Point::set_mag(double mag_in){
	mag=mag_in;
	update_rect_coord();
}

void Point::origin_symmetric(){
	x=-x;
	y=-y;
}


bool operator<(const Point& p1, const Point& p2){
	return (p1.origin_dist() < p2.origin_dist());
}

bool operator==(const Point& p1, const Point& p2){
	return (p1.x==p2.x)&&(p1.y==p2.y);
}
*/




