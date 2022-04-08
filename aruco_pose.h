#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include "aruco/aruco.hpp"

#include <lccv.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define RED 0
#define WHITE 1

class Aruco_pose
{
    public:
        Aruco_pose();
        ~Aruco_pose();
        void init(bool show_stream);
        void shutdown(void);
        pose_t aruco_pose(bool which_aruco);
    private:
        Aruco_finder ar_finder
        cv::Mat image;
        cv::Mat output;
        lccv::PiCamera cam;
        bool stream = false;
}