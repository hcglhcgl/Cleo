# include "aruco.hpp"
# include <stdio.h> 
# include <vector>
# include <map>
// openCV libs
# include "opencv2/highgui/highgui.hpp"
# include "opencv2/imgproc/imgproc.hpp"
# include "opencv2/opencv.hpp"
# include "opencv2/core/core.hpp"
# include <opencv2/aruco.hpp> //added

# include <sys/time.h>

# include <math.h>

using namespace cv;
using namespace std;

Aruco_finder::Aruco_finder()
{
    cameraMatrix=(Mat_<double>(3,3)<<633.06058204,0,330.28981083,0,631.01252673,226.42308878,0,0,1);
	distCoeffs=(Mat_<double>(1,5)<< 0.0503468649,-0.0438421987, -0.000252895273 , 0.00191361583,-0.490955908);
    
    aruco_pose.x = 0;
	aruco_pose.y = 0;
	aruco_pose.z = 0;
	aruco_pose.radius = 0;
    aruco_pose.id = 0;
    aruco_pose.valid = false;
    
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

}

Aruco_finder::~Aruco_finder()
{

}

double Aruco_finder::distance(double x, double y) {
    double dist = sqrt(x*x + y*y);
    return dist;
}

void Aruco_finder::set_markersize(float marker_size) {
    this->markerSize = marker_size;
}
pose_t Aruco_finder::find_aruco(cv::Mat *frame, bool show_image, bool red_or_white) {

    Mat gray;
    cvtColor(*frame, gray,COLOR_BGR2GRAY);

    vector<int> detectedIDs;
    
	vector<vector<Point2f> > corners;
	vector<Vec3d> rvecs, tvecs;

    int wantedIDIndex = 0;

    //Set QR found = false
    aruco_pose.valid = false;

    if(frame->empty() ) {
		cout << "ERROR NO PICTURE!!!\n";
		aruco_pose.valid = false;
		return aruco_pose;
	}


    int wanted_id = 0;
    if(red_or_white == RED) {
        //cout << "Looking for RED" << endl;
        wanted_id = 19;
    }
    else if (red_or_white == WHITE){
        //cout << "Looking for WHITE" << endl;
        wanted_id = 5;
    }

    aruco::detectMarkers(gray, dictionary, corners, detectedIDs); 

    if (detectedIDs.size() > 0) { //We detected some markers
        //Estimate position of marker
        aruco::estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, distCoeffs, rvecs, tvecs);

        //Draw the markers
        if( show_image ) {
            aruco::drawDetectedMarkers(*frame, corners, detectedIDs);
        }

        //Find only the wanted ID
        for (unsigned int i = 0; i < detectedIDs.size();i++){
            //aruco::drawAxis(*frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerSize);
            
            if (detectedIDs[i] == wanted_id) {
                wantedIDIndex = i;
                aruco_pose.valid = true;
            }
        }
        //Camera coordinate system is different from the drones - the x & y are swapped.
        aruco_pose.x = tvecs[wantedIDIndex][0];
        aruco_pose.y = tvecs[wantedIDIndex][1];
        aruco_pose.z = tvecs[wantedIDIndex][2];

        aruco_pose.radius = 0;
        //Aruco ID
        aruco_pose.id = detectedIDs[wantedIDIndex];
    }
    return aruco_pose;
}