#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include "aruco.hpp"

#include <lccv.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


int main( int argc, char** argv )
{
    std::cout<<"Program for LCCV video capture"<<std::endl;
    std::cout<<"Press ESC to stop."<<std::endl;

    cv::Mat image;
    cv::Mat output;

    //Aruco variables
    pose_t aruco_location;
    Aruco_finder ar_finder = Aruco_finder();
    ar_finder.set_markersize(0.15);

    lccv::PiCamera cam;
    //Aspect ratio of pi cam 2 = 1.3311
    cam.options->video_width=932;
    cam.options->video_height=700;
    cam.options->framerate=30;
    cam.options->verbose=true;

    cv::namedWindow("Aruco",cv::WINDOW_NORMAL);
    cam.startVideo();

    int ch=0;
    while(ch!=27){
        if(!cam.getVideoFrame(image,1000)){
            std::cout<<"Timeout error"<<std::endl;
        }
        else{
            aruco_location = ar_finder.find_aruco(&image,true, WHITE);
            if(aruco_location.valid == true) {
                cout << "ID: " << aruco_location.id << " x: " << aruco_location.x << " y: " << aruco_location.y << " z: " << aruco_location.z << endl;
            }
            cv::imshow("Aruco",image);
            ch=cv::waitKey(10);
        }
    }
    cam.stopVideo();
    cv::destroyWindow("Video");
}