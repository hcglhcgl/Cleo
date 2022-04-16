#include "apple_aruco_pose.hpp"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>

#include <lccv.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

CVPositions::CVPositions()
{
    this->ar_finder = Aruco_finder();
    ar_finder.set_markersize(0.15);

    this->apple_detector = AppleDetector();

    this->ball_finder = BallFinder();

    //Aspect ratio of pi cam 2 = 1.3311
    this->cam.options->video_width=932;
    this->cam.options->video_height=700;
    this->cam.options->framerate=30;
    this->cam.options->verbose=true;
}

CVPositions::~CVPositions() {
}

void CVPositions::init(bool show_stream,bool save_video) 
{
    if (show_stream) {
        this->stream = true;
        cv::namedWindow("Video",cv::WINDOW_NORMAL);
    }
    if (save_video) {
        this->save = true;
        this->video.open("outcpp.avi", VideoWriter::fourcc('M','J','P','G'), 10, Size(932,700),true);
    }
    this->cam.startVideo();
}

void CVPositions::shutdown(void) 
{
    cam.stopVideo();

    if(this->stream) {
        cv::destroyWindow("Aruco");
    }
    if(this->save) {
        video.release();
    }
}

pose_t CVPositions::find_aruco_pose(bool which_aruco)
{
    /*
    if (which_aruco == WHITE) {
        std::cout<<"Searching for white aruco"<<std::endl;
    }
    else if (which_aruco == RED) {
        std::cout<<"Searching for orange aruco"<<std::endl;
    } */

    pose_t aruco_location;

    if(!this->cam.getVideoFrame(image,1000)){
        std::cout<<"Timeout error"<<std::endl;
    }
    else {
        aruco_location = ar_finder.find_aruco(&image,true, which_aruco);
        
        if(this->save) {
            this->video.write(image);
        }
    }
    return aruco_location;
}

pose_t CVPositions::find_apple_pose(bool which_color)
{
    if (which_color == WHITE) {
        std::cout<<"Searching for white balls"<<std::endl;
    }
    else if (which_color == RED) {
        std::cout<<"Searching for orange balls"<<std::endl;
    } 

    pose_t apple_pose;
    int ch=0;

    while(ch!=27){
        if(!this->cam.getVideoFrame(image,1000)){
            std::cout<<"Timeout error"<<std::endl;
        }
        else {
            if(which_color == RED) {
                apple_pose = apple_detector.getOrangeApplePose(image);
            }
            
            if(this->stream) { 
                if(apple_pose.valid) {
                    Point center = Point(apple_pose.x, apple_pose.y);
                    string text = to_string(apple_pose.z);

                    circle(image, center, apple_pose.radius, Scalar(255, 0, 255), 3, LINE_AA);
                    
                    putText(image,
                    text,
                    Point(10, image.rows / 2), //top-left position
                    FONT_HERSHEY_DUPLEX,
                    1.0,
                    CV_RGB(118, 185, 0), //font color
                    2);
                }
    
                cv::imshow("Video",image);
                ch=cv::waitKey(10);
            }

            if(apple_pose.valid == true) {
                cout << "x: " << apple_pose.x << " y: " << apple_pose.y << " z: " << apple_pose.z << endl;
            }
            
        }
    }
    shutdown();
    return apple_pose;
}

pose_t CVPositions::treeID(bool which_color)
{
    if (which_color == WHITE) {
        //std::cout<<"Searching for tree with white balls"<<std::endl;
    }
    else if (which_color == RED) {
        //std::cout<<"Searching for tree with orange balls"<<std::endl;
    } 

    pose_t treeColorPose;

    if(!this->cam.getVideoFrame(image,1000)){
        std::cout<<"Timeout error"<<std::endl;
    }
    else {
        treeColorPose = ball_finder.treeID(image,which_color,false);
            
        if(this->stream) { 
            if(treeColorPose.valid) {
                Point center = Point(treeColorPose.x, treeColorPose.y);
                string text = to_string(treeColorPose.z);

                circle(image, center, treeColorPose.radius, Scalar(255, 0, 255), 3, LINE_AA);
                
                putText(image,
                text,
                Point(10, image.rows / 2), //top-left position
                FONT_HERSHEY_DUPLEX,
                1.0,
                CV_RGB(118, 185, 0), //font color
                2);
            }
            cv::imshow("Video",image);
        }
        if(this->save) {
            this->video.write(image);
        }
        
        if(treeColorPose.valid == true) {
            //Debugging messages

            /*
            cout << "x: " << treeColorPose.x << " y: " << treeColorPose.y << " z: " << treeColorPose.z << endl;
            if (treeColorPose.id == WHITE){
                cout << "WHITE TREE" << endl;
            }
            else if (treeColorPose.id == RED) {
                cout << "RED TREE" << endl;
            }
            else {
                cout << "UNABLE TO ID TREE" << endl;
            }
            */
        }
    }
    return treeColorPose;
}

pose_t CVPositions::trunkPos(void)
{
    pose_t trunk_pos;

    if(!this->cam.getVideoFrame(image,1000)){
        std::cout<<"Timeout error"<<std::endl;
    }
    else {
        trunk_pos = ball_finder.trunkFinder(image,false);

        if(this->save) {
            this->video.write(image);
        }
    }
    return trunk_pos;
}

void CVPositions::determineMovement(pose_t object_position,bool &go_straight, bool &go_left, bool &go_right)
{
    if(object_position.x < 440) {
        go_left = true;
    }

    if(object_position.x > 460) {
        go_right = true;
    }

    if(object_position.z > 20) {
        go_straight = true;
    }
}