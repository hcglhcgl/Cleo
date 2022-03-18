#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include "AppleDetector.h"

using namespace std;
using namespace cv;

AppleDetector::AppleDetector(void) {

}

Mat getCirclesMask(Mat gray) {
	//detect circles in a gray image
	vector<Vec3f> circles;
	//HoughCircles(gray, circles, HOUGH_GRADIENT, 1, 25, 100, 30, 10, 30);
	//HoughCircles(gray, circles, HOUGH_GRADIENT, 1, 25);
	HoughCircles(gray, circles, HOUGH_GRADIENT, 1, 50, 90, 30, 20, 80);

	//create mask => used for getting the new hsv image only with the detected circles
	Mat mask = cv::Mat::zeros(gray.rows, gray.cols, CV_8UC1);

	//loop through the circles
	for (size_t i = 0; i < circles.size(); i++) {
		Vec3i c = circles[i];
		Point center = Point(c[0], c[1]);
		int radius = c[2];
		circle(gray, center, radius, Scalar(255, 0, 255), 3, LINE_AA); //draw found circles on the gray image - for debugging
		circle(mask, center, radius, Scalar(255, 255, 255), -1, 8, 0); //draw circles on the mask
	}
	return mask;
}

Mat imageReducer(Mat gray, int percentage) {
	//turnes percentage of the rows black in a top-down approach
	int noOfRows = int(gray.rows / 100 * percentage);
	for (int y = 0; y < noOfRows; ++y) {
		for (int x = 0; x < gray.cols; ++x)
		{
			gray.at<bool>(y, x) = 0;
		}
	}
	return gray;
}

Mat AppleDetector::findWhiteApples(Mat image) {

		//Mat img = imread(src);
		Mat img = image;
		Mat gray;
		Mat tresholded;
		Mat hsv;
		Mat mask;
		Mat mapped;
		Mat output;

		//tresholds for the white HSV values
		int wLowH = 30;
		int wHighH = 255;

		int wLowS = 0;
		int wHighS = 40;

		int wLowV = 100;
		int wHighV = 255;

		cvtColor(img, gray, COLOR_BGR2GRAY);
		cvtColor(img, hsv, COLOR_BGR2HSV);

		//remove upper x % of the picture
		gray = imageReducer(gray, 40);

		//get mask of found circles
		mask = getCirclesMask(gray);

		//create mapped image from the original hsv using the mask
		hsv.copyTo(mapped, mask);

		//treshold the mapped image using the tresholds
		inRange(mapped, Scalar(wLowH, wLowS, wLowV), Scalar(wHighH, wHighS, wHighV), tresholded);

		//create structuring elements for the morphological operations
		Mat horizontalElement = getStructuringElement(MORPH_RECT, Size(1, 7), Point(-1, -1));
		Mat verticalElement = getStructuringElement(MORPH_RECT, Size(7, 1), Point(-1, -1));
		Mat circleElement = getStructuringElement(MORPH_ELLIPSE, Size(7, 9), Point(-1, -1));

		//CLOSING - filling in the holes => EROSION - remove vertical lines => EROSION - remove horizontal lines
		morphologyEx(tresholded, output, MORPH_CLOSE, circleElement, Point(-1, -1), 2);
		morphologyEx(output, output, MORPH_ERODE, horizontalElement, Point(-1, -1), 1);
		morphologyEx(output, output, MORPH_ERODE, verticalElement, Point(-1, -1), 1);

		//imshow("Display window", output);
		return output;
}

Mat AppleDetector::findOrangeApples(Mat image) {

	//Mat img = imread(src);
	Mat img = image;
	Mat gray;
	Mat tresholded;
	Mat hsv;
	Mat mask;
	Mat mapped;
	Mat output;

	//tresholds for the orange HSV values
	int lowH = 0;
	int highH = 50;

	int lowS = 140;
	int highS = 255;

	int lowV = 90;
	int highV = 255;

	cvtColor(img, gray, COLOR_BGR2GRAY);
	cvtColor(img, hsv, COLOR_BGR2HSV);

	//remove upper x % of the picture
	gray = imageReducer(gray, 40);

	//get mask of found circles
	mask = getCirclesMask(gray);

	//create mapped image from the original hsv using the mask
	hsv.copyTo(mapped, mask);

	//treshold the mapped image using the tresholds
	inRange(mapped, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), tresholded);

	//create structuring elements for the morphological operations
	Mat circleElement = getStructuringElement(MORPH_ELLIPSE, Size(11, 11), Point(-1, -1));

	//CLOSING - filling in the holes
	morphologyEx(tresholded, output, MORPH_CLOSE, circleElement, Point(-1, -1), 2);

	//imshow("Display window", output);
	return output;
}