/*#include "GeomView.h"


GeomView::GeomView(int rows_in, int cols_in, const double& robot_params) : 
	rows(rows_in), cols(cols_in), origin(cols_in/2, rows_in/2), joints{ Point(0, 0)} {
		for(int i=0; i<SERVOS; i++){
			if(i<SERVOS/3) joints[i].set_mag(robot_params[3]);
			else if(i>SERVOS/3 && i<SERVOS*2/3) joints[i].set_mag(robot_params[2]);
			else joints[i].set_mag(robot_params[2]);
		}
	}

// not ready
int GeomView::SetGoalPosition(int ID, double pos){
	int idx = ID_to_idx(ID);
	if(idx<SERVOS/3){
		joints[idx].set_arg(wkq::PI - std::fabs(pos) - joints[idx+6].get_arg());
		joints[idx].translate(joints[idx-6]);
	}
	else if(idx>SERVOS/3 && idx<SERVOS*2/3){
		joints[idx].set_arg(std::fabs(pos));
		joints[idx].translate_y(pos);
	}
	else{

	}
}

void GeomView::ViewLeg(int idx=1){
	cv::Mat img(rows, cols, cv::CV_8UC3);	
	// draw horizontal line representing ground
	cv::line(img, cv::Point(0, origin.y), cv::Point(cols, origin.y), Scalar(255,0,0), 5);		



}

void GeomView::ViewTop(){

}



*/



