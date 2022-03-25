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
#include "ucamera.h"
#include "ubridge.h"
#include "utime.h"


using namespace std; 


//////////////////////////////////////////////////
//////////////////////////////////////////////////
////////////// camera class //////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

void UCamera::stop()
{
  th1stop = true;
  if (th1 != NULL)
    th1->join();
#ifdef raspicam_CV_LIBS
  camDev.release();
#endif
  printf("Camera closed\n");
}

//////////////////////////////////////////////////

void UCamera::printStatus()
{
  printf("# ------------ camera ------------\n");
  printf("# camera open=%d, frame number %d\n", cameraOpen, imageNumber);
  printf("# focal length = %.0f pixels\n", cameraMatrix.at<double>(0,0));
  printf("# Camera position (%.3fx, %.3fy, %.3fz) [m]\n", camPos[0], camPos[1], camPos[2]);
  printf("# Camera rotation (%.1froll, %.1fpitch, %.1fpan) [degrees]\n", 
         camRot[0] * 180 / M_PI, 
         camRot[1] * 180 / M_PI, 
         camRot[2] * 180 / M_PI);
  #ifdef raspicam_CV_LIBS
  printf("# frame size (h,w)=(%g, %g), framerate %g/s\n", 
         camDev.get(CV_CAP_PROP_FRAME_HEIGHT), 
         camDev.get(CV_CAP_PROP_FRAME_WIDTH),
         camDev.get(CV_CAP_PROP_FPS)
        );
#endif
  arUcos->printStatus();
}

//////////////////////////////////////////////////

/** Constructor */
UCamera::UCamera(UBridge * reg)
{
  th1 = NULL;
  th1stop = false;
  saveImage = false;
  bridge = reg;
  arUcos = new ArUcoVals(this);
  cameraOpen = setupCamera();
  // initialize coordinate conversion
  makeCamToRobotTransformation();
  if (cameraOpen)
  { // start camera thread
    th1 = new thread(runObj, this);
  }
  else
  {
    printf("#UCamera:: Camera setup failed - no camera available!  ################################\n");
  }
}


void UCamera::openCamLog()
{
  // make logfile
  const int MDL = 32;
  const int MNL = 128;
  char date[MDL];
  char name[MNL];
  imTime.now();
  imTime.getForFilename(date);
  // construct filename Image
  snprintf(name, MNL, "log_image_%s.txt", date);
  logImg = fopen(name, "w");
  //
  if (logImg != NULL)
  {
    UTime t;
    t.setTime(bridge->info->bootTime);
    const int MSL = 50;
    char s[MSL];
    fprintf(logImg, "%% mission image log started at %s\n", t.getDateTimeAsString(s));
    fprintf(logImg, "%% 1 Time [sec]\n");
    fprintf(logImg, "%% 2 Regbot time [sec]\n");
    fprintf(logImg, "%% 3 image number\n");
    fprintf(logImg, "%% 4 save image\n");
    fprintf(logImg, "%% 5 do ArUco analysis\n");
    fflush(logImg);
  }
  else
    printf("#UCamera:: Failed to open image logfile\n");
  //
}


void UCamera::closeCamLog()
{
  if (logImg != NULL)
    fclose(logImg);
}

// void UCamera::closeArucoLog()
// {
//   if (logArUco != NULL)
//     fclose(logArUco);
// }


//////////////////////////////////////////////////
/** destructor */
UCamera::~UCamera()
{
  printf("#UCamera::destructor - closing\n");
  closeCamLog();
  stop();
}

//////////////////////////////////////////////////

/**
  * Implementation of capture and timestamp image */
timeval UCamera::capture(cv::Mat &image)
{
  timeval imageTime;
#ifdef raspicam_CV_LIBS
  camDev.grab();
  gettimeofday(&imageTime, NULL);
  image.create(camDev.get(CV_CAP_PROP_FRAME_HEIGHT), camDev.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC3);
  camDev.retrieve ( image);
#else
  gettimeofday(&imageTime, NULL);
#endif
  return imageTime;
}

//////////////////////////////////////////////////

/**
 * Simpel use of image pixel values.
 * \return average intensity over full image 
 */
int getAverageIntensity(cv::Mat im)
{
  int sum = 0; // of red
  int n = 0;
  for (int row = 2; row < im.rows; row+=55)
  {
    for (int col= 2; col < im.cols; col+=15)
    {
      n++;
      cv::Vec3b pix = im.at<cv::Vec3b>(row, col);
      sum += pix.val[2]; // format is BGR, so use red
    }
    //       printf("# row=%d, n=%d sum=%d, avg=%d\n", row, n, sum, sum/n);
  }
  return sum/n;
}

//////////////////////////////////////////////////

/**
 * Thread that keeps frame buffer empty
 * and handles all image events 
 */
void UCamera::run()
{
  cv::Mat im; //, frame;
//   cv::Mat im2;
//   cv::Mat imd; // differense image
//   int lineState = 0;
//   UTime imTime, im2Time;
//   bool isOK = false;
//   printf("# camera thread started\n");
  UTime t;
  float dt = 0;
  saveImage = false;
  doArUcoAnalysis = false;
  doArUcoLoopTest = false;
  int arucoLoop = 100;
  while (not th1stop)
  {
    if (cameraOpen)
    {
      // capture RGB image to a Mat structure
      imTime = capture(im);
      if (im.rows > 10 and im.cols > 10)
      { // there is an image
        imageNumber++;
        if (logImg != NULL)
        { // save to image logfile
          fprintf(logImg, "%ld.%03ld %.3f %d %d %d\n", 
                  imTime.getSec(), imTime.getMilisec(), 
                  bridge->info->regbotTime, imageNumber,
                  saveImage, doArUcoAnalysis);
        }
        // test function to access pixel values
        //imgAverage = getAverageIntensity(im);
        // 
        // test for required actions
        if (saveImage)
        { // save image as PNG file (takes lots of time to compress and save to flash)
          //printf("# saving - avg=%d\n", imgAverage);
          saveImageAsPng(im);
          printf("Image saved\n");
          //
          saveImage = false;
        }
        if (doArUcoAnalysis)
        { // do ArUco detection
          arUcos->doArUcoProcessing(im, imageNumber, imTime);
          doArUcoAnalysis = false;
          // robot pose is set after the processing, it is more likely that
          // the pose is updated while processing.
          // this is a bad idea, if robot is moving while grabbing images.
          arUcos->setPoseAtImageTime(bridge->pose->x, bridge->pose->y, bridge->pose->h);
        }
        if (doArUcoLoopTest and arucoLoop > 0)
        { // timing test - 100 ArUco analysis on 100 frames
          if (arucoLoop == 100)
            dt = 0;
          arucoLoop--;
          t.now();
          arUcos->doArUcoProcessing(im, imageNumber, imTime);
          dt += t.getTimePassed();
          usleep(10000);
          if (arucoLoop == 0)
          { // finished
            printf("# average ArUco analysis took %.2f ms\n", dt/100 * 1000);
            doArUcoLoopTest = false;
            arucoLoop = 100;
          }
        }
      }
    }
    else if (doArUcoAnalysis or saveImage)
    { // no camera
      printf("# ------  sorry, no camera is available ---------------\n");
      saveImage = false;
      doArUcoAnalysis = false;
      sleep(1);
    }
    // wait a bit
    usleep(1000);
  }
}

//////////////////////////////////////////////////

/**
 * Save image as png file
 * \param im is the 8-bit RGB image to save
 * \param filename is an optional image filename, if not used, then image is saved as image_[timestamp].png
 * */
void UCamera::saveImageAsPng(cv::Mat im, const char * filename)
{
  const int MNL = 120;
  char date[25];
  char name[MNL];
  const char * usename = filename;
  saveImage = false;
  // use date in filename
  // get date as string
  if (usename == NULL)
  {
    usename = "ucamera";
  }
  imTime.getForFilename(date);
  // construct filename
  snprintf(name, MNL, "i1%04d_%s_%s.png", imageNumber, usename, date);
  // convert to RGB
  //cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
  // make PNG option - compression level 6
  vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(6);
  // save image
  cv::imwrite(name, im, compression_params);
  // debug message
  printf("saved image to: %s\n", name);
  if (logImg != NULL)
  { // save to image logfile
    fprintf(logImg, "%ld.%03ld %.3f %d 0 0 '%s'\n", imTime.getSec(), imTime.getMilisec(), bridge->info->regbotTime, imageNumber, name);
    fflush(logImg);
  }
}


//////////////////////////////////////////////////////////////////

void UCamera::makeCamToRobotTransformation()
{
  //making a homegeneous transformation matrix from camera to robot robot_cam_H
  float tx 	= camPos[0]; // 0.158094; //meter - forward
  float ty  = camPos[1]; // 0.0; // meter - left
  float tz 	= camPos[2]; // 0.124882; //meter - up
  cv::Mat tranH = (cv::Mat_<float>(4,4) << 
              1,0,0, tx,    
              0,1,0, ty,   
              0,0,1, tz,  
              0,0,0,  1);
  
  float angle 	= camRot[0]; // degree positiv around xcam__axis - tilt
  float co 	= cos(angle);
  float si 	= sin(angle);
  cv::Mat rotxH = (cv::Mat_<float>(4,4) << 
              1,  0,  0, 0,  
              0, co, -si, 0,  
              0, si, co, 0,  
              0,  0,  0, 1);
  
  angle 	= camRot[1]; // degree positiv around ycam__axis - (roll?)
  co 	= cos(angle);
  si 	= sin(angle);
  // rotation matrix
  cv::Mat rotyH = (cv::Mat_<float>(4,4) << 
              co,  0, si, 0,   
               0,  1,  0, 0,   
               -si, 0, co, 0,  
               0,  0,  0, 1);

  angle 	= camRot[2]; // 2nd rotation around temp zcam__axis -- pan
  co 	= cos(angle);
  si 	= sin(angle);
  // rotation matrix
  cv::Mat rotzH = (cv::Mat_<float>(4,4) << 
               co,-si, 0, 0,  
               si, co, 0, 0,   
                0,  0, 1, 0,  
                0,  0, 0, 1);
  // coordinate shift - from camera to robot orientation
  cv::Mat cc = (cv::Mat_<float>(4,4) << 
               0, 0, 1, 0,
              -1, 0, 0, 0,
               0,-1, 0, 0,
               0, 0, 0, 1);
  // combine to one matrix
  cam2robot = tranH * rotzH * rotyH * rotxH * cc;
}


void UCamera::setRoll(float roll)
{
  camRot[0] = roll;
  makeCamToRobotTransformation();
}

void UCamera::setTilt(float tilt)
{
  camRot[1] = tilt;
  makeCamToRobotTransformation();
}

void UCamera::setPan(float pan)
{
  camRot[2] = pan;
  makeCamToRobotTransformation();
}

void UCamera::setPos(float x, float y, float z)
{
  camPos[0] = x;
  camPos[1] = y;
  camPos[2] = z;
  makeCamToRobotTransformation();
}
