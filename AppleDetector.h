#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>

using namespace std;
using namespace cv;

class AppleDetector {

public:
	AppleDetector();
	Mat findWhiteApples(Mat image);
	Mat findOrangeApples(Mat image);
};
