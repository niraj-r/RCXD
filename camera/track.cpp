#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

Mat src;
char window_name[30] = "HSV Segmentation";

static void onMouse( int event, int x, int y, int f, void* ){
 Mat image=src.clone();
 Vec3b rgb=image.at<Vec3b>(y,x);
 int B=rgb.val[0];
 int G=rgb.val[1];
 int R=rgb.val[2];

  Mat HSV;
  Mat RGB=image(Rect(x,y,1,1));
  cvtColor(RGB, HSV,CV_BGR2HSV);

    Vec3b hsv=HSV.at<Vec3b>(0,0);
    int H=hsv.val[0];
    int S=hsv.val[1];
    int V=hsv.val[2];

    char name[30];
    sprintf(name,"B=%d",B);
    putText(image,name, Point(150,40) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );

    sprintf(name,"G=%d",G);
    putText(image,name, Point(150,80) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );

    sprintf(name,"R=%d",R);
    putText(image,name, Point(150,120) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );

    sprintf(name,"H=%d",H);
    putText(image,name, Point(25,40) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );

    sprintf(name,"S=%d",S);
    putText(image,name, Point(25,80) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );

    sprintf(name,"V=%d",V);
    putText(image,name, Point(25,120) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,255,0), 2,8,false );

    sprintf(name,"X=%d",x);
    putText(image,name, Point(25,300) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,0,255), 2,8,false );

    sprintf(name,"Y=%d",y);
    putText(image,name, Point(25,340) , FONT_HERSHEY_SIMPLEX, .7, Scalar(0,0,255), 2,8,false );

 //imwrite("hsv.jpg",image);
 imshow( window_name, image );
}




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
		src = hsv;
		inRange(hsv, Scalar(40,100,100), Scalar(50,255,255), mask);

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
