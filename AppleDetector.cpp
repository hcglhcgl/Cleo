#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include "AppleDetector.h"

using namespace std;
using namespace cv;


AppleDetector::AppleDetector(void) {
	//vector<Vec3f> circles;
}

vector<Vec3f> AppleDetector::getCircles(Mat tresholded) {
	vector<Vec3f> goodCircles;

	for (size_t i = 0; i < circles.size(); i++) {
		Vec3i c = circles[i];
		//Point center = Point(c[0], c[1]);
		int radius = c[2];
		int left_x = c[0] - radius;
		int left_y = c[1] - radius;

		Rect crop(left_x, left_y, (2*radius), (2 * radius));
		Mat res = tresholded(crop);

		int white = countNonZero(res);
		int full = res.rows * res.cols;
		//cout << "white: " << res.rows;
		//cout << "full: " << res.cols;
		float percentage = ((float)white / (float)full * 100);
		//int black = (res.rows * res.cols) - white;

		if (percentage > 50) {
			goodCircles.push_back(c);
			cout << "percentage: " << percentage << "radius" << radius;
			imshow("Display window", res);
			waitKey(0);
		}
	}

	return goodCircles;
}

Mat AppleDetector::getCirclesMask(Mat gray, bool small) {
	//detect circles in a gray image
	////vector<Vec3f> circles;
	//HoughCircles(gray, circles, HOUGH_GRADIENT, 1, 25, 100, 30, 10, 30);
	//HoughCircles(gray, circles, HOUGH_GRADIENT, 1, 25);
	//HoughCircles(gray, circles, HOUGH_GRADIENT, 1, 50, 90, 30, 20, 80);
	// //good one
	//HoughCircles(gray, circles, HOUGH_GRADIENT, 1, 10, 200, 25, 2, 35);
	/////HoughCircles(gray, circles, HOUGH_GRADIENT, 1, 20, 230, 20, 10, 35);
	//HoughCircles(gray, circles, HOUGH_GRADIENT, 1, 10, 120, 12, 2, 10);
	HoughCircles(gray, circles, HOUGH_GRADIENT, 1, 18, 180, 18, 10, 25);//small
	//HoughCircles(gray, circles, HOUGH_GRADIENT, 1, 38, 180, 28, 20, 40);//big

	if (small) {
		HoughCircles(gray, circles, HOUGH_GRADIENT, 1, 18, 180, 18, 10, 25);
	}
	else {
		HoughCircles(gray, circles, HOUGH_GRADIENT, 1, 38, 180, 28, 20, 45);
	}

	//create mask => used for getting the new hsv image only with the detected circles
	Mat mask = cv::Mat::zeros(gray.rows, gray.cols, CV_8UC1);
	medianBlur(gray, gray, 3 );
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
		mask = getCirclesMask(gray, true);

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

Mat AppleDetector::findOrangeApples(Mat image, bool small) {

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

	int lowS = 90;
	int highS = 255;

	int lowV = 80;
	int highV = 255;

	cvtColor(img, gray, COLOR_BGR2GRAY);
	//Mat bgr[3];
	//split(image, bgr);
	//gray = bgr[2];
	cvtColor(img, hsv, COLOR_BGR2HSV);

	//remove upper x % of the picture
	gray = imageReducer(gray, 40);

	//get mask of found circles
	mask = getCirclesMask(gray, small);
	//return mask; 
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

Vec3i AppleDetector::getOrangeAppleCoordinates(Mat image) {
	//Search for big balls first
	Mat res = findOrangeApples(image, false);
	vector<Vec3f> resultVector = getCircles(res);

	if (resultVector.empty()) {
		//Search for small balls
		Mat res = findOrangeApples(image, true);
		resultVector = getCircles(res);
	}

	Vec3i c = resultVector[0];
	//Point center = Point(c[0], c[1]);
	//int radius = c[2];
	return c;
}

float AppleDetector::getDistance(int radius) {
	//calculation of the focal length in pixels
	//float f_mm = 3.04;
	//float s_w_mm = 3.674;
	//float img_w_p = 932;
	//float f = (f_mm / s_w_mm) * img_w_p;

	float width_ball_mm = 42;
	int width_ball_pixels = (int)radius * 2;
	int f = 771.17;
	float distance = ((width_ball_mm / width_ball_pixels) * f) / 10;
	return distance;
}

pose_t AppleDetector::getOrangeApplePose(Mat image) {
	pose_t orange_apple_pose;
	orange_apple_pose.valid = false;

	//Search for big balls first
	Mat res = findOrangeApples(image, false);
	vector<Vec3f> resultVector = getCircles(res);

	if (resultVector.empty()) {
		//Search for small balls
		Mat res = findOrangeApples(image, true);
		resultVector = getCircles(res);
	}

	//If resultvector is still empty, no balls have been found
	if (resultVector.empty()) {
		orange_apple_pose.valid = false;
		return orange_apple_pose;
	}

	Vec3i vector = resultVector[0];

	int radius = vector[2];
	orange_apple_pose.radius = radius;

	orange_apple_pose.x = vector[0];
	orange_apple_pose.y = vector[1];
	orange_apple_pose.z = getDistance(radius);
	return orange_apple_pose;
}