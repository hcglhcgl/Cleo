#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include "AppleDetector.h"

int main()
{
    std::cout<<"Sample program for LCCV video capture"<<std::endl;
    std::cout<<"Press ESC to stop."<<std::endl;
    cv::Mat image;
    cv::Mat output;
    AppleDetector apple_detector = AppleDetector();
    lccv::PiCamera cam;
    cam.options->video_width=600;
    cam.options->video_height=480;
    cam.options->framerate=30;
    cam.options->verbose=true;
    cv::namedWindow("Video",cv::WINDOW_NORMAL);
    cv::namedWindow("Video_original",cv::WINDOW_NORMAL);
    cam.startVideo();
    int ch=0;
    while(ch!=27){
        if(!cam.getVideoFrame(image,1000)){
            std::cout<<"Timeout error"<<std::endl;
        }
        else{
            //cv::imshow("Video",image);
            output = apple_detector.findOrangeApples(image);
            cv::imshow("Video",output);
            cv::imshow("Video_original",image);
            ch=cv::waitKey(10);
        }
    }
    cam.stopVideo();
    cv::destroyWindow("Video");
}