/*#ifndef ROBOTIM_H
#define ROBOTIM_H

#include "wkq.h"
#include "opencv2/highgui/highgui.hpp"


#define SERVOS 18


class GeomView{

public:
	GeomView(int rows_in, int cols_in, const double& robot_params[]);

	int SetGoalPosition(int idx, double pos);

	void ViewLeg(int idx=1);
	void ViewTop();

private:
	inline int ID_to_idx(const int& idx_in){
		if(idx_in<=22) return idx_in-11;
		else return idx_in-11-6;	
	}


	int rows;
	int cols;

	cv::Point origin;

	// First batch of 6 are knees, second batch of 6 are hips, third batch of 6 are arms
	wkq::Point joints[SERVOS];

};

typedef GeomView DNXServo;

#endif
*/
