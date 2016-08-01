#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

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
		inRange(hsv, Scalar(0,100,100), Scalar(50,255,255), mask);

		// Apply erosion or dilation on the image
		// (in, out, kernel, anchor, iteration, bordertype, bordervalue)
		//erode( mask, mask, Mat(), Point(-1, -1), 2, BORDER_CONSTANT, morphologyDefaultBorderValue() );
		//dilate( mask, mask, Mat(), Point(-1, -1), 2, BORDER_CONSTANT, morphologyDefaultBorderValue() );

		//findContours( contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );


		imshow("hsv", mask);
		if (waitKey(30) >= 0) break;
	}

	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}
