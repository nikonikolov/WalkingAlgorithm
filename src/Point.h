#ifndef POINT_H
#define POINT_H

#include <cmath>

class Point{

public:
	Point();
	Point(const double& x_in, const double& y_in);
	Point(const Point& p_in);
	~Point();
	
	inline double get_x();
	inline double get_y();
	
	inline double origin_dist() const;
	//void origin_symmetric();

	inline void translate_y(const double& delta_y);
	inline void translate_x(const double& delta_x);
	inline void translate(const Point& p2);
	
	inline double dist(const Point& p_in) const;
	inline double dist_sq(const Point& p_in) const;
/*
	friend bool operator<(const Point& p1, const Point& p2);
	friend bool operator==(const Point& p1, const Point& p2);
*/

private:
	double x;
	double y;

};


double Point::get_x(){
	return x;
}

double Point::get_y(){
	return y;
}

double Point::origin_dist() const{
	return sqrt(pow(x,2) + pow(y,2));
}

double Point::dist(const Point& p_in) const{
	return sqrt( pow((x-p_in.x),2) + pow((y-p_in.y),2) );
}

double Point::dist_sq(const Point& p_in) const{
	return pow((x-p_in.x),2) + pow((y-p_in.y),2);
}

void Point::translate_y(const double& delta_y){
	y += delta_y;
}

void Point::translate_x(const double& delta_x){
	x += delta_x;
}


#endif
