#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include "aruco/aruco.hpp"

#include <lccv.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

Aruco_pose::Aruco_finder()
{
    this->ar_finder = Aruco_finder();
    ar_finder.set_markersize(0.15);

    //Aspect ratio of pi cam 2 = 1.3311
    this->cam.options->video_width=932;
    this->cam.options->video_height=700;
    this->cam.options->framerate=30;
    this->cam.options->verbose=true;
}
void Aruco_pose::init(bool show_stream) 
{
    if (show_stream) {
        this->stream = true;
        cv::namedWindow("Aruco",cv::WINDOW_NORMAL);
    }
    this->cam.startVideo();
}

void Aruco_pose::shutdown(void) 
{
    cam.stopVideo();

    if(this->stream) {
        cv::destroyWindow("Aruco");
    }
}

pose_t Aruco_pose::find_pose(bool which_aruco)
{
    if (which_aruco == WHITE) {
        std::cout<<"Searching for white aruco"<<std::endl;
    }
    else if (which_aruco == RED) {
        std::cout<<"Searching for orange aruco"<<std::endl;
    } 

    int ch=0;
    while(ch!=27){
        if(this->!cam.getVideoFrame(image,1000)){
            std::cout<<"Timeout error"<<std::endl;
        }
        else{
            this->aruco_location = ar_finder.find_aruco(&image,true, WHITE);
            if(this->aruco_location.valid == true) {
                cout << "ID: " << this->aruco_location.id << " x: " << this->aruco_location.x << " y: " << this->aruco_location.y << " z: " << this->aruco_location.z << endl;
            }
            if(this->stream) { 
                cv::imshow("Aruco",image);
                ch=cv::waitKey(10);
            }
        }
    }
    
    
}