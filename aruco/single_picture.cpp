#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include "aruco.hpp"

using namespace std;
using namespace cv;


int main( int argc, char** argv )
{
    pose_t aruco_location;
    Aruco_finder ar_finder = Aruco_finder();

    Mat image = imread("aruco.jpg");

    aruco_location = ar_finder.find_aruco(&image,true, WHITE);
    
    cout << "ID: " << aruco_location.id << " x: " << aruco_location.x << " y: " << aruco_location.y << " z: " << aruco_location.z << endl;

    namedWindow("Aruco");
    imshow("Aruco", image);

    waitKey(0); // Wait for any keystroke in the window

    destroyAllWindows(); //destroy all opened windows
}