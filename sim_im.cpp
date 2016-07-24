#include <iostream>
//#include "opencv2/imgproc/imgproc.hpp"
//#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"

using namespace cv;

/*
 * main() for simulating the robot behaviour and debugging the algorithm using opencv
 * 
 * To compile run $ make bin/sim
 * 
 */


int main(){

	Mat img(800, 800, CV_8UC3);

	Point pt1(20,30);
	Point pt2(120,30);

	line(img, pt1, pt2, Scalar(255,0,0), 5);

	namedWindow("Front View", WINDOW_AUTOSIZE);		// Create a window for display.
    imshow("Front View", img);                   	// Show our image inside it.

    waitKey(0);

    return 0;
}





