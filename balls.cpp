# include "balls.hpp"
# include <stdio.h> 
# include <vector>
# include <map>
// openCV libs
# include "opencv2/highgui/highgui.hpp"
# include "opencv2/imgproc/imgproc.hpp"
# include "opencv2/opencv.hpp"
# include "opencv2/core/core.hpp"

# include <sys/time.h>

# include <math.h>

using namespace cv;
using namespace std;

BallFinder::BallFinder()
{
    ballPose.x = 0;
	ballPose.y = 0;
	ballPose.z = 0;
	ballPose.radius = 0;
    ballPose.id = 0;
    ballPose.valid = false;
    //HSV Values
    //Orange
    orangeHSV.HSV_lower[0] = 0;
    orangeHSV.HSV_lower[1] = 150;
    orangeHSV.HSV_lower[2] = 50;

    orangeHSV.HSV_upper[0] = 15;
    orangeHSV.HSV_upper[1] = 255;
    orangeHSV.HSV_upper[2] = 255;
    //White
    whiteHSV.HSV_lower[0] = 30;
    whiteHSV.HSV_lower[1] = 20;
    whiteHSV.HSV_lower[2] = 50;

    whiteHSV.HSV_upper[0] = 180;
    whiteHSV.HSV_upper[1] = 80;
    whiteHSV.HSV_upper[2] = 255;
    //Green
    greenHSV.HSV_lower[0] = 31;
    greenHSV.HSV_lower[1] = 110;
    greenHSV.HSV_lower[2] = 70;

    greenHSV.HSV_upper[0] = 49;
    greenHSV.HSV_upper[1] = 255;
    greenHSV.HSV_upper[2] = 255;
}

BallFinder::~BallFinder(){

};

pose_t BallFinder::find_ball(cv::Mat frame, bool show_image, bool red_or_white,bool debug) {
    Mat cropped;
    Mat blurred;
    Mat mask;
    Mat mask_eroded;
    Mat mask_dilated;
    Mat HSV;

    ballPose.valid = false;

    //Crop the top 20%
    cropped = imageReducer(frame,20,false,0);
    if (debug) {
        imshow("cropped", cropped);
        waitKey(0);
    }
    
    //Blurred
    GaussianBlur(cropped,blurred,Size(11, 11),0);

    int width; int height;
    height = cropped.size[0];
    width = cropped.size[1];

    cvtColor(blurred, HSV, COLOR_BGR2HSV);

    if (red_or_white == RED) {
        inRange(HSV, Scalar (orangeHSV.HSV_lower[0],orangeHSV.HSV_lower[1],orangeHSV.HSV_lower[2]),
    Scalar (orangeHSV.HSV_upper[0],orangeHSV.HSV_upper[1],orangeHSV.HSV_upper[2]),mask);
    }
    else if (red_or_white == WHITE){
        inRange(HSV, Scalar (whiteHSV.HSV_lower[0],whiteHSV.HSV_lower[1],whiteHSV.HSV_lower[2]),
    Scalar (whiteHSV.HSV_upper[0],whiteHSV.HSV_upper[1],whiteHSV.HSV_upper[2]),mask);
    }

    if (debug) {
        imshow("mask", mask);
        waitKey(0);
    }
    
    // Create a structuring element (SE)
    Mat element = getStructuringElement(MORPH_RECT, Size(11,11));
    //Mat element = getStructuringElement(MORPH_ELLIPSE, Size(11,11));

    Mat erod, dill;

    cv::erode(mask,mask_eroded,element,Point(-1, -1), 2);

    if (debug){
        imshow("Eroded", mask_eroded);
        waitKey(0);
    }
    

    dilate(mask_eroded,mask_dilated,element,Point(-1, -1), 2);
    
    if (debug) {
        imshow("Dilated", mask_dilated);
        waitKey(0);
    }
    
    vector<vector<Point>> contours;
    cv::findContours(mask_dilated,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    //vector<Vec3f> contours;
    //HoughCircles(mask_dilated, contours, HOUGH_GRADIENT, 1, 18, 180, 18, 10, 25);

    // Approximate contours to polygons + get bounding circles
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Point2f>centers( contours.size() );
    vector<float>radius( contours.size() );
    
    int idx = -1, max_area=0;
    for( size_t i = 0; i < contours.size(); i++ ){
        approxPolyDP( contours[i], contours_poly[i], 3, true );
        minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
        int area = (int)contourArea(contours_poly[i]);     
        if (area > max_area){
            idx = (int)i;
            max_area = area;
        }
    }

  
    if (idx>-1){
        cout << "Ball detected!" << endl;

        Point2f center = centers[idx];
        circle( cropped, center, (int)radius[idx], Scalar(0,255,0), 2 );
        circle( cropped, center, 3, Scalar(255,0,0), -1 );

        ballPose.valid = true;
        ballPose.x = center.x;
        ballPose.y = center.y;
        ballPose.z = getDistance((int)radius[idx]);
    }

    if(debug) {
        imshow("Ball", frame);

        waitKey(0); // Wait for any keystroke in the window

        destroyAllWindows(); //destroy all opened windows
    }
    
    return ballPose;
}

float BallFinder::getDistance(int radius) {    
	int width_ball_pixels = radius * 2;
	int f = 771.17;
	float distance = ((width_ball_mm / width_ball_pixels) * f) / 10;

    return distance;
}

float BallFinder::getDistanceTree(int radius) {    
	int width_ball_pixels = radius * 2;
	int f = 771.17;
	float distance = ((width_ball_mm / width_ball_pixels) * f) / 10;

    return distance;
}

Mat BallFinder::imageReducer(Mat image, int percentage,bool reverse, int width_percentage) {
	Mat cropped_image;

    int width; int height;
    height = image.size[0];
    width = image.size[1];

    int newHeight;
    newHeight = (height/100)*percentage;

    if (reverse) {
        if (width_percentage != 0) {
            int newWidth = (width/100)*percentage;
            cropped_image = image(Range(0,newHeight),Range(newWidth,width));
        }
        else {
            cropped_image = image(Range(0,newHeight),Range(0,width));
        }
    }
    else {
        if (width_percentage != 0) {
            
        }
        else {
            cropped_image = image(Range(newHeight,height),Range(0,width));
        }
    }
    
    return cropped_image;
}

pose_t BallFinder::treeID(cv::Mat frame, bool show_image, bool red_or_white,bool debug) {
    Mat cropped;
    Mat blurred;
    Mat mask;
    Mat mask_eroded;
    Mat mask_dilated;
    Mat HSV;

    ballPose.valid = false;

    //Crop the top 20%
    cropped = imageReducer(frame,20,false,0);
    if (debug) {
        imshow("cropped", cropped);
        waitKey(0);
    }
    
    //Blurred
    GaussianBlur(cropped,blurred,Size(11, 11),0);

    int width; int height;
    height = cropped.size[0];
    width = cropped.size[1];

    cvtColor(blurred, HSV, COLOR_BGR2HSV);

    if (red_or_white == RED) {
        inRange(HSV, Scalar (orangeHSV.HSV_lower[0],orangeHSV.HSV_lower[1],orangeHSV.HSV_lower[2]),
    Scalar (orangeHSV.HSV_upper[0],orangeHSV.HSV_upper[1],orangeHSV.HSV_upper[2]),mask);
    }
    else if (red_or_white == WHITE){
        inRange(HSV, Scalar (whiteHSV.HSV_lower[0],whiteHSV.HSV_lower[1],whiteHSV.HSV_lower[2]),
    Scalar (whiteHSV.HSV_upper[0],whiteHSV.HSV_upper[1],whiteHSV.HSV_upper[2]),mask);
    }

    if (debug) {
        imshow("mask", mask);
        waitKey(0);
    }
    
    // Create a structuring element (SE)
    Mat element = getStructuringElement(MORPH_RECT, Size(11,11));
    //Mat element = getStructuringElement(MORPH_ELLIPSE, Size(11,11));

    Mat erod, dill;

    cv::erode(mask,mask_eroded,element,Point(-1, -1), 2);

    if (debug){
        imshow("Eroded", mask_eroded);
        waitKey(0);
    }
    

    dilate(mask_eroded,mask_dilated,element,Point(-1, -1), 2);
    
    if (debug) {
        imshow("Dilated", mask_dilated);
        waitKey(0);
    }
    
    vector<vector<Point>> contours;
    cv::findContours(mask_dilated,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);

    // Approximate contours to polygons + get bounding circles
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Point2f>centers( contours.size() );
    vector<float>radius( contours.size() );
    
    int ids[5] = {0};
    int min_area=0;
    int ballCount = 0;
    for( size_t i = 0; i < contours.size(); i++ ){
        approxPolyDP( contours[i], contours_poly[i], 3, true );
        minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
        int area = (int)contourArea(contours_poly[i]);     
        if (area > min_area){
            ids[ballCount] = (int)i;
            ballCount++;
        }
    }

  
    if (ballCount>0){
        cout << ballCount << " ball(s) detected!" << endl;
        int id = 0;
        for (int i  = 0; i < ballCount ; i++) {
            id = ids[i];
            Point2f center = centers[id];
            circle( cropped, center, (int)radius[id], Scalar(0,255,0), 2 );
            circle( cropped, center, 3, Scalar(255,0,0), -1 );

            ballPose.valid = true;
            ballPose.x = center.x;
            ballPose.y = center.y;
            ballPose.z = getDistance((int)radius[id]);
        }
        if (ballCount == 1) {
            ballPose.id = WHITE;
        }
        else if (ballCount == 2) {
            ballPose.id = RED;
        }
        else {
            ballPose.id = -1;
        }
    }

    if(debug) {
        imshow("Ball", frame);

        waitKey(0); // Wait for any keystroke in the window

        destroyAllWindows(); //destroy all opened windows
    }
    
    return ballPose;
}

pose_t BallFinder::trunkFinder(cv::Mat frame, bool show_image ,bool debug) {
    Mat cropped;
    Mat blurred;
    Mat mask;
    Mat mask_eroded;
    Mat mask_dilated;
    Mat HSV;

    stubPose.valid = false;

    //Crop the bottom 70%
    cropped = imageReducer(frame,30,true,0);
    if (debug) {
        imshow("cropped", cropped);
        waitKey(0);
    }
    
    //Blurred
    GaussianBlur(cropped,blurred,Size(11, 11),0);

    int width; int height;
    height = cropped.size[0];
    width = cropped.size[1];

    cvtColor(blurred, HSV, COLOR_BGR2HSV);

    inRange(HSV, Scalar (greenHSV.HSV_lower[0],greenHSV.HSV_lower[1],greenHSV.HSV_lower[2]),
    Scalar (greenHSV.HSV_upper[0],greenHSV.HSV_upper[1],greenHSV.HSV_upper[2]),mask);

    if (debug) {
        imshow("mask", mask);
        waitKey(0);
    }
    
    // Create a structuring element (SE)
    //Mat element = getStructuringElement(MORPH_RECT, Size(11,11));
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(11,11));

    Mat erod, dill;

    cv::erode(mask,mask_eroded,element,Point(-1, -1), 1);

    if (debug){
        imshow("Eroded", mask_eroded);
        waitKey(0);
    }
    

    dilate(mask_eroded,mask_dilated,element,Point(-1, -1), 2);
    
    if (debug) {
        imshow("Dilated", mask_dilated);
        waitKey(0);
    }
    
    vector<vector<Point>> contours;
    cv::findContours(mask_dilated,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);

    // Approximate contours to polygons + get bounding circles
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Point2f>centers( contours.size() );
    vector<float>radius( contours.size() );

    int idx = -1, max_area=0;
    for( size_t i = 0; i < contours.size(); i++ ){
        approxPolyDP( contours[i], contours_poly[i], 3, true );
        minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
        int area = (int)contourArea(contours_poly[i]);     
        if (area > max_area){
            idx = (int)i;
            max_area = area;
        }
    }

  
    if (idx>-1){
        Point2f center = centers[idx];
        circle( cropped, center, (int)radius[idx], Scalar(0,255,0), 2 );
        circle( cropped, center, 3, Scalar(255,0,0), -1 );

        stubPose.valid = true;
        stubPose.x = center.x;
        stubPose.y = center.y;
        stubPose.z = getDistanceTree((int)radius[idx]);
    }

    if(debug) {
        imshow("Ball", frame);

        waitKey(0); // Wait for any keystroke in the window

        destroyAllWindows(); //destroy all opened windows
    }
    return stubPose;
}