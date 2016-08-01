#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "iostream"

using namespace cv;
using namespace std;

int main( int argc, char** argv ) {
	//Scalar greenLower = (29, 86, 6);
	//Scalar greenUpper = (64, 255, 255);
	Scalar greenLower = (0, 0, 0);
	Scalar greenUpper = (0, 0, 255);

	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened()) // check if we succeeded
		return -1;

	Mat edges;
	Mat mask;
	Mat hsv;
	namedWindow("hsv", 1);

	for (;;) {
		Mat frame;
		cap >> frame; // get a new frame from camera
		cvtColor(frame, hsv, COLOR_BGR2HSV);

		imshow("original", hsv);
		inRange(hsv, Scalar(100,80,80), Scalar(115,120,120), mask);
		// Apply erosion or dilation on the image
		// (in, out, kernel, anchor, iteration, bordertype, bordervalue)
		erode( mask, mask, Mat(), Point(-1, -1), 2, BORDER_CONSTANT, morphologyDefaultBorderValue() );
		dilate( mask, mask, Mat(), Point(-1, -1), 2, BORDER_CONSTANT, morphologyDefaultBorderValue() );

		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		RNG rng(12345);
		findContours( mask, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, Point(0, 0) );
		
		Mat drawing = Mat::zeros( mask.size(), CV_8UC3 );
		for( int i = 0; i< contours.size(); i++ ){
		    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		    drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
		}     
		  
		imshow("hsv", mask);
		imshow("contours", drawing);
		if (waitKey(30) >= 0) break;
	}

	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}
