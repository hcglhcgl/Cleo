#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/core.hpp"
#include "types.h"

#define RED 0
#define WHITE 1

using namespace cv;
using namespace std;

struct HSV 
{
    int HSV_lower[3];
    int HSV_upper[3];
};

class BallFinder
{
    public:
        BallFinder();
        ~BallFinder();
        pose_t find_ball(cv::Mat frame, bool red_or_white, bool debug);
        Mat imageReducer(Mat image, int percentage,bool reverse, int width_percentage);
        float getDistance(int radius);
        float getDistanceTree(int radius);
        pose_t treeID(cv::Mat frame, bool red_or_white, bool debug);
        pose_t trunkFinder(cv::Mat frame, bool debug);
        Mat imageReducerReverse(Mat image, int percentage);
    private:
        pose_t ballPose;
        Mat cameraMatrix;
	    Mat distCoeffs;
        HSV orangeHSV;
        HSV whiteHSV;
        HSV greenHSV;
        pose_t stubPose;
        float width_ball_mm = 42;
};


