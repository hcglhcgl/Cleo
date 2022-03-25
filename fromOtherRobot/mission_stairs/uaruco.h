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

#ifndef UCAMERA_ARUCO_H
#define UCAMERA_ARUCO_H


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
#include "ulibpose.h"
// #include "u2dline.h"
// this should be defined in the CMakeList.txt ? or ?
#ifdef raspicam_CV_LIBS
#include <raspicam/raspicam.h>
#include <raspicam/raspicam_cv.h>
#endif


using namespace std;

//// forward declaration
//class UCamera;

// should be OK version

/**
 * The class ArUcoVal contains the values related to one ArUco marker
 * \var markerId is the ID of the marker extracted from the parameter markerIds found by the cv::aruco::detectMarkers function
 * \var rVec and tVec are also found by cv::aruco::detectMarkers.
 * \var disteance2marker id the disteance from the robot to the marker in meters
 * \var angle2markerX and angle2markerY are the angle between the robot and the x or y axis of the mar frame respectetly 
 * */
class ArUcoVal 
{
public:
  /// markerId is the ID of the marker ectracted from the parameter markerIds found på the cv::aruco::detectMarkers function
  int markerId = -1;
  /// linux time when image was captured
  UTime imageTime;
  /// the image (frame) number, as numbered by the capture function
  int frameNumber = 0; 
  /** Rotation    vector 
   * - x (left) ,y (down) ,z (forward), length is angle in radians.
   * - as found by cv::aruco::detectMarkers
   * In camera coordinates
   * */
  cv::Vec3f rVec = 0;
  /**
   * Translation (position) of marker  x (left) ,y (down) ,z (forward)
   * as found by cv::aruco::detectMarkers
   * */
  cv::Vec3f tVec = 0; 
  /**
   * Coordinate conversion matrix [4x4] from a position in marker coordinates (left, up, out)
   * to robot coordinates (forward, left, up) */
  cv::Mat marker2Cam;
  /**
   * Marker position in robot coordinates
   * x = forward, y=left, z=up */
  cv::Mat markerPosition;
  /// euclidean distance in meters
  float distance2marker = 0;
  /**
   * Marker angle, if vertical (on a wall), zero degree is facing robot, parallel to robot orientation.
   * if not vertical (assumed horizontal), then zero angle is when marker up is forward and alligned with robot
   * */
  float markerAngle = 0; 
  /**
   * Vertical means marker is mostly vertical (like on a wall), else mostly horizontal */
  bool markerVertical = true;
  /**
   * Is the value from the newest search */
  bool isNew = false;
  /**
   * Lock to ensure data is consistent (detected in a thread) */
  std::mutex lock;
  /**
   * Robot pose when image was taken (assuming stationary during frame capture) */
  UPose robotPose;
public:
  /**
   * make marker to camera coordinate conversion matrix 
   * based on vectors found by the detectMarkers function.
   */
  cv::Mat makeMarkerToCam4x4matrix(cv::Vec3f rVec, cv::Vec3f tVec);
  /**
   * Detect marker position and orientation in robot coordinates
   */
  void markerToRobotCoordinate(cv::Mat cam2robot);
  /**
   * Debug print status for camera and ArUco markers to console
   * */
  void printStatus();
}; 


//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////


class UCamera;

/**
 * Simple class to hold all possible ArUco value IDs, potentially
 * allow one picture with all 100 markers */
class ArUcoVals
{
public:
  /**
   * Up to 100 markers */
  static const int MAX_VAL_CNT = 100;
  /**
   * Marker array */
  ArUcoVal arucos[MAX_VAL_CNT];
  /** debug image */
  bool debugImages = false;
  /**
   * Pointer to camera info */
  UCamera * cam;
  /**
   * Number of frames analized */
  int frameCnt = 0;
  
public:
  /** constructor 
   * \param iCam is pointer to source calera. */
  ArUcoVals(UCamera * iCam)
  {
    cam = iCam;
  }
  /** Destructor */
  ~ArUcoVals()
  {
    closeArucoLog();
  }
  /**
   * Get values for marker with this ID
   * \param id marker ID to look for, range [0, 100[.
   * \Returns a pointer to the relevant marker. or NULL if out of range
   * markerID = -1 id data is not valid
   * NB! data should be locked to avoid being updated while reading.
   * */
  ArUcoVal * getID(int id);
  /**
   * Get first new marker
   * \param n (optional) if N=0 get the first, else get the N'th of the new markers.
   * \return NULL if none qualified.
   * NB! data should be locked to avoid being updated while reading.
   * */
  ArUcoVal * getFirstNew(int n = 0);
  
  /**
   * Get first new marker
   * \param n (optional) if N=0 get the first, else get the N'th of the new markers.
   * \return NULL if none qualified.
   * NB! data should be locked to avoid being updated while reading.
   * */
  int getMarkerCount(bool newOnly = true);
  /**
   * clear newest flag */
  void setNewFlagToFalse();
  /**
   * print status for all markers,
   * mostly for console debug (use s for status on console)
   * */
  void printStatus();
  /**
   * Investigate frame for ArUco markers Andersenif found convert marker position to robot coordinates
   * \param frame is the image (in RGB format) to investigate
   * \param framenumber is the image number for the frame
   * \param imTime is time for frame capture
   * \returns number of codes found.
   * */
  int doArUcoProcessing(cv::Mat frame, int frameNumber, UTime imTime);
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
  string extractSingleMarker(cv::Vec3d rotationVectors, cv::Vec3d translationVectors, int markerId, cv::Mat rob_Ar_H);
  /**
   * convert radians to degree */
  inline double rad2deg( double a) { return a*180/M_PI;};
  inline double deg2rad( double a) { return a/180*M_PI;};
  /**
   * Open logfile 
   */
  void openArucoLog();
  /**
   * Close logfile 
   * */
  void closeArucoLog();
  /** return true if log is open
   * */
  bool logArucoIsOpen()
  { return logArUco != NULL; }
  /**
   * add robot pose to all newly found markers */
  void setPoseAtImageTime(float x, float y, float_t h);
private:
  /// logfile for ArUco extract
  FILE * logArUco = NULL;
  
};


#endif
