#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include "aruco.hpp"
#include "AppleDetector.h"
#include "balls.hpp"

#include <lccv.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

#define RED 0
#define WHITE 1

class CVPositions
{
    public:
        CVPositions();
        ~CVPositions();
        void init(bool show_stream,bool save_video);
        void shutdown(void);
        pose_t find_aruco_pose(bool which_aruco);
        pose_t find_apple_pose(bool which_color);
        pose_t treeID(bool which_color);
    private:
        Aruco_finder ar_finder;
        AppleDetector apple_detector;
        BallFinder ball_finder;

        cv::Mat image;
        cv::Mat output;

        lccv::PiCamera cam;
        cv::VideoWriter video;
        bool stream = false;
        bool save = false;
};