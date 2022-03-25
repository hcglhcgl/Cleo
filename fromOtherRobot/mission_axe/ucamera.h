/***************************************************************************
*   Copyright (C) 2016-2020 by DTU (Christian Andersen)                        *
*   jca@elektro.dtu.dk                                                    *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU Lesser General Public License as        *
*   published by the Free Software Foundation; either version 2 of the    *
*   License, or (at your option) any later version.                       *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU Lesser General Public License for more details.                   *
*                                                                         *
*   You should have received a copy of the GNU Lesser General Public      *
*   License along with this program; if not, write to the                 *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/

#ifndef UCAMERA_H
#define UCAMERA_H


#include <iostream>
#include <sys/time.h>
#include <thread>
// #include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
//#include <string>

#include "urun.h"
#include "ubridge.h"
#include "utime.h"
// #include "u2dline.h"
#include "uaruco.h"
// this should be defined in the CMakeList.txt ? or ?
#ifdef raspicam_CV_LIBS
#include <raspicam/raspicam.h>
#include <raspicam/raspicam_cv.h>
#endif


using namespace std;

/**
 * The camera class has the functions
 * to open, close, configure and
 * capture images from the raspberry pi
 * camera */
class UCamera : public URun
{ // raw camera functions
public:
  // flag to save an image to disk
  bool saveImage = false;
  // flad to do ArUcoAnalysis
  bool doArUcoAnalysis = false;
  /// do loop-test (aruco log)
  bool doArUcoLoopTest = false;
  // opened OK
  bool cameraOpen = false;
  // detected ArUco markers
  ArUcoVals * arUcos = NULL;
  // camera position on robot
  cv::Vec3d camPos = {0.03, 0.03, 0.27}; /// x=fwd, y=left, z=up
  cv::Vec3d camRot = {0, 10*M_PI/180.0, 0}; /// roll, tilt, yaw (right hand rule, radians)
  cv::Mat cam2robot;
  //
  /** camera matrix is a 3x3 matrix (raspberry PI typical values)
   *    pix    ---1----  ---2---  ---3---   -3D-
   *  1 (x)      980        0       640     (X)
   *  2 (y)       0        980      480     (Y)
   *  3 (w)       0         0        1      (Z)
   * where [1,1] and [2,2] is focal length,
   * and   [1,3] is half width  center column (assuming image is 1280 pixels wide)
   * and   [2,3] is half height center row    (assuming image is 960 pixels high)
   * [X,Y,Z] is 3D position (in camera coordinated (X=right, Y=down, Z=front),
   * [x,y,w] is pixel position for 3D position, when normalized, so that w=1
  */
  //cv::Mat cameraMatrix = cv::Mat(3,3, CV_64F, c_m);
  const cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 
                       980,    0,    640, 
                       0,    980,    480,  
                       0,       0,     1);
  /**
   * camera radial distortion vector 
   * 1 (k1)   0.14738
   * 2 (k2)   0.0117267
   * 3 (p1)   0
   * 4 (p2)   0
   * 5 (k3)  -0.14143
   * where k1, k2 and k3 is radial distortion params
   * and p1, p2 are tangential distortion 
   * see https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html 
   *   */
  const cv::Mat distortionCoefficients = (cv::Mat_<double>(1,5) << 
                       0.14738, 
                       0.0117267, 
                       0,  
                       0, 
                      -0.14143);
public:
  /** Constructor */
  UCamera(UBridge * reg);
  /** destructor */
  ~UCamera();
  void stop();
  /**
   * find ArUco */
//   int doArUcoProcessing(cv::Mat frame, int frameNumber);
  /**
   * print status for camera and image based functions */
  void printStatus();
  /**
   * set camera tilt angle 
   * \tilt is tilt angle in degrees (positive down, 0=horizontal)
   * */
  void setTilt(float tilt);
  void setPan(float pan);
  void setRoll(float roll);
  void setPos(float x, float y, float z);
  
private:
  // pointer to regbot interface
  UBridge * bridge;
  // image saved number
  int imageNumber = 0;
  // time image was taken
  UTime imTime, im2Time;
  // logfile for images
  FILE * logImg = NULL;
//   /// logfile for ArUco extract
//   FILE * logArUco = NULL;

public:
  /**
   * Save image to flashdisk */
  void saveImageAsPng(cv::Mat im, const char * filename = NULL);
  
protected:
  /**
   * The raw raspberry pi camera device, as defined by the
   * Ava Group of the University of Cordoba */
#ifdef raspicam_CV_LIBS
  raspicam::RaspiCam_Cv camDev;
#endif
  /**
   * conver position and rotation of camera to 
   * coordinate conversion matrix 
   * \param camPos (position vector)
   * \param camRot (rotation vector in radians )
   * \result cam2robot 4x4 matrix 
   * */
  void makeCamToRobotTransformation();
  /**
   * Aruco functions */
//   string extractSingleMarker(cv::Vec3d rotationVectors, cv::Vec3d translationVectors, int markerId, cv::Mat rob_Ar_H);
  /**
   * convert radians to degree */
//   inline double rad2deg( double a) { return a*180/M_PI;};
//   inline double deg2rad( double a) { return a/180*M_PI;};
  /**
   * print out the values of tempArUcoVal to a log-file
   * */
//public:
  /** Capture an image and load image to cv::Mat structure
   * \param image is the destination for the image
   * \returns timestamp when the image was grabbed. */
  timeval capture(cv::Mat &image);
  /**
   * Configure camera */
  bool setupCamera()
  { 
    bool isOpen = false;
    // image should be a multible of 320x240
    // or the image will be cropped
#ifdef raspicam_CV_LIBS
    int w = (2592/640)*320; // (2592/320)*320; // 320*5; //
    int h = (1992/480)*240; // (1992/240)*240; // 240*3; //
    printf("image size %dx%d\n", h, w);
    //     camDev.setWidth(w);
    camDev.set( CV_CAP_PROP_FORMAT, CV_8UC3 );
    camDev.set( CV_CAP_PROP_FRAME_HEIGHT, h);
    camDev.set( CV_CAP_PROP_FRAME_WIDTH, w);
    cout<<"Connecting to camera"<<endl;
    if ( !camDev.open() ) {
      cerr<<"Error opening camera"<<endl;
      return false;
    }
    else
      isOpen = true;
    for (int i = 0; i < 30; i++)
      // just to make sure camera settings has reached steady state
      camDev.grab();
    cout<<"Connected to pi-camera ='"<<camDev.getId() << "\r\n";
#endif
    return isOpen;
  }
public:
  /**
   * open logfile for saved images - inc timing */
  void openCamLog();
  /**
   * open logfile for ArUco decoding images - inc timing */
  void openArucoLog();
  /// close logfile
  void closeCamLog();
  /// close logfile
  void closeArucoLog();
  /// return true if log is open
  bool logCamIsOpen()
  { return logImg != NULL; }
  /// return true if log is open
//   bool logArucoIsOpen()
//   { return logArUco != NULL; }
  /**
   * method to do the image analysis - see ucamera.cpp */
  void run();
};

#endif
