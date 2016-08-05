// g++ -std=c++11 track.cpp -o track `pkg-config --cflags --libs opencv`

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "iostream"

using namespace cv;
using namespace std;

void NonUniformIlluminationMorph(const cv::Mat &src, cv::Mat &dst, int minThickess = 5, bool restoreMean = true, cv::Mat *background=NULL);

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
		Mat frame, illum;
		cap >> frame; // get a new frame from camera
		
		//NonUniformIlluminationMorph(frame, illum);
		//imwrite("illum.jpg",illum);
		GaussianBlur(frame, illum, Size(21,21), 0, 0);
		imshow("blur", illum);
		cvtColor(illum, hsv, COLOR_BGR2HSV);
		imshow("hsv", hsv);
		
		// Apply erosion or dilation on the image
		// (in, out, kernel, anchor, iteration, bordertype, bordervalue)
		// hsv(0-360, 0-100, 0-100) => hsv(0-180, 0-255, 0-255)
		inRange(hsv, Scalar(5,165,110), Scalar(20,195,130), mask);
		
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
				  
		imshow("contours", drawing);
		if (waitKey(30) >= 0) break;
	}

	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}


/** @brief Remove non-uniform illumination using morphology
Morphology OPEN can detects bright structures larger that a given size.
If you consider large structures as background you can use OPEN
to detect background than remove it from the original image.
This is same as to do MORPH_TOPHAT.
@Param [in]src input image GRAY, BGR or BGRA.
With BGR(A) image this function uses Brightness from image HSV.
@Param [out]dst destination image. If alpha channel is present in src it will be cloned in dst
@Param minThickess size used by morphology operation to estimate background. Use small size to
enhance details flatting larger structures.
@c minThickess should be just larger than maximum thickness in objects you want to keep.
Example:
- Take thickest object, suppose is circle 100 * 100px
- Measure its maximum thickness let's say is 20px: In this case @c minThickess could be 20+5.
- If the circle is filled than thickness=diameter, consequently @c minThickess should be 100+5px
@Param restoreMean if true, the mean of input brightness will be restored in destination image.
if false, the destination brightness will be close to darker region of input image.
@Param [out]background if not NULL the removed background will be returned here.
This will be Mat(src.size(),CV_8UC1)
*/
void NonUniformIlluminationMorph(const cv::Mat &src, cv::Mat &dst, int minThickess, bool restoreMean, cv::Mat *background)
{
    CV_Assert(minThickess >= 0);
    CV_Assert((src.type() == CV_8UC1) || (src.type() == CV_8UC3) || (src.type() == CV_8UC4));
    cv::Mat brightness, src_hsv;
    vector<cv::Mat> hsv_planes;

    // GET THE BRIGHTNESS
    if (src.type() == CV_8UC1)
        src.copyTo(brightness);
    else if (src.type() == CV_8UC3)
    {
        cv::cvtColor(src, src_hsv, cv::COLOR_BGR2HSV);
        cv::split(src_hsv, hsv_planes);
        brightness = hsv_planes[2];
    }
    else if (src.type() == CV_8UC4)
    {
        cv::cvtColor(src, src_hsv, cv::COLOR_BGRA2BGR);
        cv::cvtColor(src_hsv, src_hsv, cv::COLOR_BGR2HSV);
        cv::split(src_hsv, hsv_planes);
        brightness = hsv_planes[2];
    }

    //to restore previous brightness we need its current mean
    Scalar m;
    if (restoreMean)
        m = mean(brightness);

    // REMOVE THE BACKGROUND
    int size = minThickess / 2;
    Point anchor = Point(size, size);
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2 * size + 1, 2 * size + 1), anchor);
    if (background != NULL) // to keep background we need to use MORPH_OPEN
    {
        //get the background
        cv::Mat bkg(brightness.size(), CV_8UC1);
        morphologyEx(brightness, bkg, MORPH_OPEN, element, anchor);
        //save the background
        (*background) = bkg;
        //remove the background
        brightness = brightness - bkg;
    }
    else //tophat(I)  <=> open(I) - I; 
    {
        //remove background
        morphologyEx(brightness, brightness, MORPH_TOPHAT, element, anchor);
    }

    // RESTORE PREVIOUS BRIGHTNESS MEAN
    if (restoreMean)
        brightness += m(0);

    // BUILD THE DESTINATION
    if (src.type() == CV_8UC1)
        dst = brightness;
    else if (src.type() == CV_8UC3)
    {
        merge(hsv_planes, dst);
        cvtColor(dst, dst, COLOR_HSV2BGR);
    }
    // restore alpha channel from source 
    else if (src.type() == CV_8UC4)
    {
        cv::Mat bgr;
        vector<cv::Mat> bgr_planes = { hsv_planes[0], hsv_planes[1], hsv_planes[2]};
        merge(bgr_planes, bgr);
        cvtColor(bgr, bgr, COLOR_HSV2BGR);

        int from_toA[] = { 0, 0, 1, 1, 2, 2 };
        src.copyTo(dst);
        cv::mixChannels(&bgr, 1, &dst, 1, from_toA, 3);
    }

    imshow("NonUniformIlluminationMorph:SRC", src);
    imshow("NonUniformIlluminationMorph:DST", dst);
    if ((background != NULL) && (!background->empty()))
        imshow("NonUniformIlluminationMorph:BKG", *background);
}
