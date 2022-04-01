#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/core.hpp"
# include <opencv2/aruco.hpp> //added

#define RED 0
#define WHITE 1


using namespace cv;
using namespace std;

struct pose_t
{
    bool valid;
    double x;
    double y;
    double z;
    double th;
    long num_pose;
    int id;
};

class Aruco_finder
{
    public:
        Aruco_finder();
        ~Aruco_finder();
        pose_t find_aruco(cv::Mat * frame, bool show_image, bool red_or_white);
    private:
        pose_t aruco_pose;
        Mat cameraMatrix;
	    Mat distCoeffs;
        Mat R33 = Mat::eye(3,3,CV_64FC1);
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        double distance(double x, double y);
};


