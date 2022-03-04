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

/***************************************************************************
*   August 2019 ArUco functions and Camera calibration functions added    *
*   by Michael Teglgaard                                                  *
*   s130067@student.dtu.dk                                                *
*                                                                         *
*   Part of this code is inspired by code from the YouTube playlist       *
*   OpenCv Basic https://youtu.be/HNfPbw-1e_w                             *
***************************************************************************/

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include "urun.h"
#include "uaruco.h"
#include "ubridge.h"
#include "utime.h"
#include "ucamera.h"


using namespace std; 

//////////////////////////////////////////////////
//////////////////////////////////////////////////
////////////// ArUcoVal class ////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////


cv::Mat ArUcoVal::makeMarkerToCam4x4matrix(cv::Vec3f rVec, cv::Vec3f tVec)
{
  cv::Mat R, camH;
  Rodrigues(rVec,R); 		 				                // 3 cols, 3 rows
  cv::Mat col = cv::Mat(tVec);  	              // 1 cols, 3 rows
  hconcat(R, col, camH);                        // 4 cols, 3 rows
  float tempRow[4] = {0,0,0,1};
  cv::Mat row = cv::Mat(1, 4, CV_32F, tempRow); // 4 cols, 1 row
  camH.push_back(row); // 4 cols, 4 rows
  return camH; // 4 cols, 4 rows
}


////////////////////////////////////////////////////

void ArUcoVal::markerToRobotCoordinate(cv::Mat cam2robot)
{ // make marker to camera coordinate conversion matrix for this marker
  marker2Cam = makeMarkerToCam4x4matrix(rVec, tVec);
  // combine with camera to robot coordinate conversion
  cv::Mat marker2robot = cam2robot * marker2Cam;
  //
  if (false)
  { // print matrices - for debug
    cout << "ID "  << markerId << " marker2cam \n" << marker2Cam << "\n";
    cout << "ID "  << markerId << " cam2robot \n" << cam2robot << "\n";
    cout << "ID "  << markerId << " marker2robot \n" << marker2robot << "\n";
  }
  // get position of marker center 
  cv::Vec4f zeroVec = {0,0,0,1};
  markerPosition = marker2robot * cv::Mat(zeroVec);
  // get position of 10cm in z-marker direction - out from marker
  cv::Vec4f zeroVecZ = {0,0,0.1,1};
  cv::Mat marker10cmVecZ = marker2robot * cv::Mat(zeroVecZ);
  // get marker z-vector (in robot coordinate system)
  cv::Mat dz10cm = marker10cmVecZ - markerPosition;
  if (false)
  { // debug
    cout << "ID " << markerId << " 0,0,0 \n" << markerPosition <<"\n";
    cout << "ID " << markerId << " 10cmZ \n" << dz10cm <<"\n";
  }
  // if marker z-axis extends (most) in the robot Z dimension, then the marker is not vertical
  markerVertical = dz10cm.at<float>(0,2) < 0.0707; // 10cm marker angle 45 deg
  if (markerVertical)
  { // marker mostly vertical
    // rotation of marker Z vector in robot coordinates around robot Z axis
    markerAngle = atan2(-dz10cm.at<float>(0,1), -dz10cm.at<float>(0,0));
  }
  else
  { // then mostly horizontal
//     printf("# marker horizontal - orientation of marker Y is used\n");
    cv::Vec4f zeroVecY = {0,0.1,0,1};
    cv::Mat marker10cmVecY = marker2robot * cv::Mat(zeroVecY);
    // get marker z-vector (in robot coordinate system)
    cv::Mat dy10cm = marker10cmVecY - markerPosition;
    if (false)
    { // debug
      cout << "ID " << markerId << " 10cmY \n" << dz10cm <<" - marker horizontal\n";
    }
    // rotation of marker Y vector in robot coordinates around robot Z axis
    markerAngle = atan2(dy10cm.at<float>(0,1), dy10cm.at<float>(0,0));
  }
  // in plane distance sqrt(x^2 + y^2) only - using hypot(x,y) function
  distance2marker = hypotf(markerPosition.at<float>(0,0), markerPosition.at<float>(0,1));
  if (true)
  { // debug
    printf("# ArUco ID %02d at(%.3fx, %.3fy, %.3fz) (robot coo), plane dist %.3fm, angle %.3f rad, or %.1f deg, vertical=%d\n", 
           markerId, 
           markerPosition.at<float>(0,0), markerPosition.at<float>(0,1), markerPosition.at<float>(0,2),
           distance2marker, markerAngle, markerAngle*180/M_PI, markerVertical);
  }
}


void ArUcoVal::printStatus()
{
  printf("# ID=%3d, from frame %d, time %ld.%06ld, plane distance=%.3fm, angle=%.1f deg, vertical=%d, new=%d\n", 
         markerId, 
         frameNumber, imageTime.getSec(), imageTime.getMilisec(), 
         distance2marker, 
         markerAngle*180/M_PI, 
         markerVertical, isNew);
  printf("#         marker position (x,y,z) = (%6.2f, %6.2f, %6.2f) m (robot coo)\n",
         markerPosition.at<float>(0,0), 
         markerPosition.at<float>(0,1), 
         markerPosition.at<float>(0,2));
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
////////////// ArUcoVals class ///////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////


void ArUcoVals::printStatus()
{
  printf("# ------- ArUco ------------\n");
  ArUcoVal * v = NULL;
  int cnt = 0;
  for (int i = 0; i < MAX_VAL_CNT; i++)
  {
    v = &arucos[i];
    if (v->markerId >= 0)
      cnt++;
  }
  printf("# Detected %d codes, %d attempts\n", cnt, frameCnt);
  for (int i = 0; i < MAX_VAL_CNT; i++)
  {
    v = &arucos[i];
    if (v->markerId >= 0)
      v->printStatus();
  }
}

//////////////////////////////////////////////////

ArUcoVal * ArUcoVals::getID(int id)
{
  ArUcoVal * v = NULL;
  if (id >= 0 and id < MAX_VAL_CNT)
    v = &arucos[id];
  return v;
}

//////////////////////////////////////////////////

ArUcoVal * ArUcoVals::getFirstNew(int n)
{
  ArUcoVal * v = NULL;
  for (int i = 0; i < MAX_VAL_CNT; i++)
  {
    v = getID(i);
    if (v != NULL and v->isNew)
    {
      if (n == 0)
        break;
      else
        n--;
    }
  }
  return v;
}

//////////////////////////////////////////////////

int ArUcoVals::getMarkerCount(bool newOnly)
{
  ArUcoVal * v = NULL;
  int n = 0;
  for (int i = 0; i < MAX_VAL_CNT; i++)
  {
    v = getID(i);
    if (v->markerId >= 0)
    {
      if (newOnly)
      {
        if (v->isNew)
          n++;
      }
      else
        n++;
    }
  }
  return n;
}

////////////////////////////////////////////////////

void ArUcoVals::setNewFlagToFalse()
{
  for (int i = 0; i < MAX_VAL_CNT; i++)
  {
    arucos[i].isNew = false;
  }
}


void ArUcoVals::openArucoLog()
{
  // make logfile
  const int MDL = 32;
  const int MNL = 128;
  char date[MDL];
  char name[MNL];
  UTime appTime;
  appTime.now();
  appTime.getForFilename(date);
  // construct filename ArUco
  snprintf(name, MNL, "log_ArUco_%s.txt", date);
  logArUco = fopen(name, "w");
  if (logArUco != NULL)
  {
    const int MSL = 50;
    char s[MSL];
    fprintf(logArUco, "%% Mission ArUco log started at %s\n", appTime.getDateTimeAsString(s));
    fprintf(logArUco, "%% 1   Time [sec]\n");
    fprintf(logArUco, "%% 3   image number\n");
    fprintf(logArUco, "%% 4   ArUco ID\n");
    fprintf(logArUco, "%% 5-7 Position (x,y,z) [m] (robot coordinates)\n");
    fprintf(logArUco, "%% 8   Distance to marker [m]\n");
    fprintf(logArUco, "%% 9   Marker angle [radians] - assumed vertical marker.\n");
    fprintf(logArUco, "%% 10  Marker is vertical (on a wall)\n");
    fprintf(logArUco, "%% 11  Processing time [sec].\n");
    fflush(logArUco);
  }
  else
    printf("#UCamera:: Failed to open image logfile\n");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void ArUcoVals::closeArucoLog()
{
  if (logArUco != NULL)
  {
    fclose(logArUco);
    logArUco = NULL;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

int ArUcoVals::doArUcoProcessing(cv::Mat frame, int frameNumber, UTime imTime)
{
  cv::Mat frameAnn;
  const float arucoSqaureDimensions = 0.100;      //meters
  vector<int> markerIds;
  vector<vector<cv::Point2f>> markerCorners; //, rejectedcandidates;
  cv::aruco::DetectorParameters parameters;
  //seach DICT on docs.opencv.org
  cv::Ptr < cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50); 
  // marker position info
  vector<cv::Vec3d> rotationVectors, translationVectors;
  UTime t; // timing calculation (for log)
  t.now(); // start timing
  // clear all flags (but maintain old information)
  setNewFlagToFalse();
  /** Each marker has 4 marker corners into 'markerCorners' in image pixel coordinates (float x,y).
   *  Marker ID is the detected marker ID, a vector of integer.
   * */
  cv::aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
  // marker size is same as detected markers
  // printf("# Found %ld marker corners\n", markerCorners.size());
  if (markerIds.size() > 0)
  { // there are markers in image
    if (debugImages)
    { // make a copy for nice saved images
      frame.copyTo(frameAnn);
      cv::aruco::drawDetectedMarkers(frameAnn, markerCorners, markerIds);
    }
    // 
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 
                                         arucoSqaureDimensions, 
                                         cam->cameraMatrix, 
                                         cam->distortionCoefficients, 
                                         rotationVectors, 
                                         translationVectors);
  }
  else
    printf("# No markers found\n");
  // extract markers
  int i = 0;
  for(unsigned int j = 0; j < markerIds.size(); j++)
  { // for all detected markers in this frame
    i = markerIds[j];
    ArUcoVal * v = getID(i);
    if (v != NULL)
    { // save marker info and do coordinate conversion for marker
      v->lock.lock();
      v->markerId = i;
      // set time and frame-number for this detection
      v->imageTime = imTime;
      v->frameNumber = frameNumber;
      // v->frame = frame; // might use frame.copyTo(v->frame) insted
      v->rVec = rotationVectors[j];
      v->tVec = translationVectors[j];
      //
      if (debugImages)
      { // show detected orientation in image X:red, Y:green, Z:blue.
        cv::aruco::drawAxis(frameAnn, cam->cameraMatrix, cam->distortionCoefficients, v->rVec, v->tVec, 0.03f); //X:red, Y:green, Z:blue.
      }
      //
      v->markerToRobotCoordinate(cam->cam2robot);
      v->isNew = true;
      v->lock.unlock();
      //       v->done = true;
//       printf("# debug images = %d, logArUco = %d\n", debugImages, logArUco != NULL);
      // maybe also log of data
      if (logArUco != NULL)
      {
        /*
         *      fprintf(logArUco, "%% data log started at %s\n", t.getDateTimeAsString(s));
         *      fprintf(logArUco, "%% 1   Time [sec]\n");
         *      fprintf(logArUco, "%% 2   Regbot time [sec]\n");
         *      fprintf(logArUco, "%% 3   image number\n");
         *      fprintf(logArUco, "%% 4   ArUco ID\n");
         *      fprintf(logArUco, "%% 5-7 Position (x,y,z) [m] (robot coordinates)\n");
         *      fprintf(logArUco, "%% 8   Distance to marker [m]\n");
         *      fprintf(logArUco, "%% 9   Marker angle [radians] - assumed vertical marker.\n");
         * */
        fprintf(logArUco, "%ld.%03ld %d %d %.3f %.3f %.3f  %.3f %.4f %d %.3f\n", imTime.getSec(), imTime.getMilisec(), 
                v->frameNumber, 
                v->markerId,
                v->markerPosition.at<float>(0,0), v->markerPosition.at<float>(0,1), v->markerPosition.at<float>(0,2),
                v->distance2marker, v->markerAngle, v->markerVertical, t.getTimePassed()
               );
      }
    }
  }
  if (markerIds.size() > 0 and debugImages)
    cam->saveImageAsPng(frameAnn, "annotated");
  frameCnt++;
  return markerIds.size();
}

/////////////////////////////////////////////////////////

void ArUcoVals::setPoseAtImageTime(float x, float y, float_t h)
{
  for (int i = 0; i < MAX_VAL_CNT; i++)
  {
    ArUcoVal * v = &arucos[i];
    if (v->isNew)
      v->robotPose.set(x, y, h);
  }
}
