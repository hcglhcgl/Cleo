#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>

#include "types.h"

using namespace std;
using namespace cv;
class AppleDetector {

public:
	vector<Vec3f> circles;
	AppleDetector();
	vector<Vec3f> getCircles(Mat tresholded);
	Mat getCirclesMask(Mat gray, bool small);
	Mat findWhiteApples(Mat image);
	Mat findOrangeApples(Mat image, bool small);
	Mat findOrangeApples2(Mat image);
	pose_t getOrangeApplePose(Mat image);
	float getDistance(int radius);
};
