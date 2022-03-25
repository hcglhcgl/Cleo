
/***************************************************************************
*   Copyright (C) 2016 by DTU (Christian Andersen)                        *
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

#include <iostream>
#include <sys/time.h>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include "../Downloads/raspicam-0.0.5/src/raspicam.h"

// #define PI_CAM
#ifdef PI_CAM
#include <raspicam/raspicam.h>
#endif
#include "urun.h"
#include "ucamera.h"
#include "ubridge.h"
#include "umission.h"
#include "ulibpose2pose.h"
// #include "ujoy.h"

using namespace std;



void printHelp(char * name)
{ // show help
  printf("\nUsage: %s [<from mission> [<to mission>]] [n IP] [h]\n\n", name);
  printf("<from mission part> and <to mission part>:\n");
  printf("         number in the range 1..998, and the code\n");
  printf("         run only the mission parts in this range.\n");
  printf(" n=IP    IP is direct IP or URL (default is 127.0.0.1)\n");
  printf(" h       This help text\n\n");
  printf("E.g.: './%s 2 2' runs mission part 2 only\n\n", name);
  printf("NB!  Robot may continue to move if this app is stopped with ctrl-C.\n\n");
}

bool readCommandLineParameters(int argc, char ** argv, 
                               int * firstMission, 
                               int * lastMission, 
                               const char ** bridgeIp)
{
  // are there mission parameters
  bool startNumber = true;
  bool isHelp = false;
  for (int i = 0; i < argc; i++)
  {
    if (isHelp)
      break;
    char c = argv[i][0];
    switch (c)
    {
      case '-':
      case 'h':
        printHelp(argv[0]);
        isHelp = true;
        break;
      case 'n':
        // skip the n, but use the rest of this parameter
        *bridgeIp = &argv[i][1];
        while (((*bridgeIp)[0] <= ' ' and (*bridgeIp)[0] > '\0') or (*bridgeIp)[0] == '=')
          (*bridgeIp)++;
        printf("n-parameter '%s'\n", *bridgeIp);
        break;
      default:
        if (isdigit(argv[i][0]))
        {
          if (startNumber)
          {
            *firstMission = strtol(argv[i], NULL, 10);
            *lastMission = *firstMission;
            startNumber = false;
          }
          else
          {
            *lastMission = strtol(argv[i], NULL, 10);
          }
        }
        break;
    }
  }
  if (not isHelp)
  { // not help - create modules
    if (argc > 1)
    { // get mission range
      if (isdigit(argv[1][0]))
      {
        *firstMission = strtol(argv[1], NULL, 10);
        *lastMission = *firstMission;
        if (argc > 2)
          if (isdigit(argv[2][0]))
          {
            *lastMission = strtol(argv[2], NULL, 10);
          }
      }
    }
  }
  return not isHelp;
}

/**
 * Get a line from keyboard, but return also, when 
 * bission is finished or connection to bridge is
 * broken.
 * \param s is a buffer string, where the line is returned.
 * \param MSL is the length of the string buffer
 * \param conn is a pointer to a connection flag - should return, if false.
 * \param fin  is a pointer to a finished flag - should return when flag is set.
 * \returns number of characters in line (0 if empty line or finished or connection is false.
 * */
int getLineFromKeyboard(char * s, const int MSL, volatile bool * conn, volatile bool * fin)
{
  char * p1 = s;
  *p1 = '\0';
  while (((p1 - s) < MSL) and *conn and not *fin)
  {
    int n = read(stdin->_fileno, p1, 1);
    if (*p1 == '\n')
    { // terminate and stop on newline
      *p1 = '\0';
      break;
    }
    if (n == 1)
      p1++;
    else
      usleep(1000);
  }
  return strlen(s);
}

/////////////////////////////////////////////////////////////////////

/**
 * Try turn-straight-turn manoeuvre calculation
 * \param s holds a string with destination position */
void toPositionTest4(char * s)
{ // read also next parameter
  char * p1 = &s[1];
  // marker position
  float x = strtof(p1, &p1);
  float y = strtof(p1, &p1);
  float h = strtof(p1, &p1) * M_PI / 180.0;
  float d = 0.35; // m target position distance in front of marker
  if (*p1 != '\0')
    // there is a distance parameter
    d = strtof(p1, &p1);
  printf("# Marker      at (x=%6.3f, y=%6.3f, h=%6.3f) target dist=%g\n", x, y, h * 180/ M_PI, d);
  // calculate target position for robot, at distance d from marker.
  x -= d * cos(h);
  y -= d * sin(h);
  printf("# Destination at (x=%6.3f, y=%6.3f, h=%6.3f) \n", x, y, h * 180/ M_PI);
  // acceleration and radius limits
  const float acc = 1.0; // m/s2 - max acceleration - turn and linear
  //const float minimumTurnRadius = 0.05; // m
  // calculation object
  UPose2pose p2p(x, y, h, 0.01);
  // set target position
  // result variables
  // calculate manoeuvre parameters
  bool isOK = p2p.calculateALA(0.3, acc);
  // print result on console
  p2p.printMan();
  printf("# manoeuvre is OK=%d ------------\n", isOK);
}

////////////////////////////////////////////////////////////////////
/**
 * main function.
 * creates all modules - bridge, camera and mission functions,
 * and then allow console functions.
 * */

int main ( int argc,char **argv ) 
{
  int firstMissionPart = 1;
  int lastMissionPart = 998;
  const char * bridgeIp = "127.0.0.1"; // default connection IP to bridge
  const int MSL = 250;
  char s[MSL];
  //
  bool isOK = readCommandLineParameters(argc, argv, &firstMissionPart, &lastMissionPart, &bridgeIp);
  if (isOK)
  { // create connection to Regbot board through bridge 
    // (IP number (127.0.0.1 is localhost, 2. param is logOpen)
    UBridge bridge(bridgeIp, false);
    // create camera interface
    UCamera cam(&bridge);
    // start camera setup and capture
    cam.start();
    // create mission control
    UMission mission(&bridge, &cam);
    // set mission range (default is 1..988 (all))
    mission.fromMission = firstMissionPart;
    mission.toMission = lastMissionPart;
    // start mission thread
    mission.start();
    //
    // the rest is just to allow some command line input
    //
    sleep(1); // wait a bit to allow connection to bridge
    printf("Press h for help (q for quit)\n");
    printf(">> "); // command prompt
    fflush(stdout); // make sure command prompt gets displayed
    // make stdin (keyboard) non-blocking
    int flags = fcntl(stdin->_fileno, F_GETFL, 0); 
    fcntl(stdin->_fileno, F_SETFL, flags | O_NONBLOCK);
    //
    while (bridge.connected and not mission.finished)
    { // Just wait for quit
      int n = getLineFromKeyboard(s, MSL, &bridge.connected, &mission.finished);
//       const char * p1 = s;
      if (n > 0)
      {
        printf("# got '%s' n=%d\n", s, n);
        switch (s[0])
        {
          case 'q':
            mission.stop();
            sleep(1);
            break;
          case 'a':
            printf("# setting flag to do ArUco analysis\n");
            cam.doArUcoAnalysis = true;
            break;
          case 'b':
            printf("# Trying to restart bridge. - will stop this mission app too\n");
            bridge.send("bridge restart\n");
            mission.stop();
            sleep(1);
            break;
          case 'd':
            if (n > 1)
            {
              bool d = strtol(&s[1], NULL, 10);
              cam.arUcos->debugImages = d;
              printf("# setting flag to save debug images = %d\n", cam.arUcos->debugImages);
            }
            else
              printf("# Use either 'd0' or 'd1' - debug still %d\n", cam.arUcos->debugImages);
            break;
          case 'c':
            printf("# setting flag to capture and save image\n");
            cam.saveImage = true;
            break;
          case 'o':
            printf("# Starting loop-test\n");
            cam.doArUcoLoopTest = true;
            break;
          case 'l':
            {
              int n = bridge.decodeLogOpenClose(s);
              if (strstr(s, "cam") != NULL)
              {
                if (s[1] == 'o')
                  cam.openCamLog();
                else
                  cam.closeCamLog();
                n++;
              }
              if (strstr(s, "aruco") != NULL)
              {
                if (s[1] == 'o')
                  cam.arUcos->openArucoLog();
                else
                  cam.arUcos->closeArucoLog();
                n++;
              }
              if (strstr(s, "mission") != NULL)
              {
                if (s[1] == 'o')
                  mission.openLog();
                else
                  mission.closeLog();
                n++;
              }
              if (n == 0)
                printf("# logfile not found in '%s' (see help)\n", s);
            }
            break;
          case 's':
            printf("# Status:\n");
            cam.printStatus();
            bridge.printStatus();
            mission.printStatus();
            printf("# -------------------------\n");
            break;
          case 'r':
            { // read also next parameter
              if (n > 1)
              { // convert string from s[1] to float
                float v = strtof(&s[1], NULL);
                printf("# Roll changed from %g to %g deg\n", cam.camRot[0] * 180/ M_PI, v);
                // set new value in radians
                cam.setRoll(v *  M_PI / 180.0);
              }
            }
            break;
          case 't':
          { // read also next parameter
            if (n > 1)
            { // convert string from s[1] to float
              float v = strtof(&s[1], NULL);
              printf("# Tilt changed from %g to %g deg\n", cam.camRot[1] * 180/ M_PI, v);
              // set new value in radians
              cam.setTilt(v *  M_PI / 180.0);
            }
          }
          break;
          case 'p':
          { // read also next parameter
            if (n > 1)
            { // convert string from s[1] to float
              float p = strtof(&s[1], NULL);
              printf("# Pan (Yaw) changed from %g to %g deg\n", cam.camRot[2] * 180/ M_PI, p);
              // set new value in radians
              cam.setPan(p *  M_PI / 180.0);
            }
          }
          break;
          case '2':
            toPositionTest4(s);
            break;
          case '3': // set position of camera
            if (n > 1)
            {
              char * p1 = &s[1];
              float x = strtof(p1, &p1);
              float y = strtof(p1, &p1);
              float z = strtof(p1, &p1);
              cam.setPos(x, y, z);
            }
            break;
          case 'h':
            printf("# mission command options\n");
            printf("#    a    Do an ArUco analysis of a frame\n");
            printf("#    b    Bridge restart\n");
            printf("#    c    Capture an image and save to disk (image*.png)\n");
            printf("#    d 1/0 Set/clear flag to save ArUco debug images, is=%d\n", cam.arUcos->debugImages);
            printf("#    h    This help\n");
            printf("#    lo xxx  Open log for xxx (pose %d, hbt %d, bridge %d, imu %d\n"
                   "#               ir %d, motor %d, joy %d, event %d, cam %d, aruco %d, mission %d)\n",
                   bridge.pose->logIsOpen(), 
                   bridge.info->logIsOpen(),
                   bridge.logIsOpen(), 
                   bridge.imu->logIsOpen(),
                   bridge.irdist->logIsOpen(),
                   bridge.motor->logIsOpen(),
                   bridge.joy->logIsOpen(),
                   bridge.event->logIsOpen(),
                   cam.logCamIsOpen(),
                   cam.arUcos->logArucoIsOpen(),
                   mission.logIsOpen()
                  );
            printf("#    lc xxx  Close log for xxx\n");
            printf("#    o    Loop-test for steady ArUco marker (makes logfile)\n");
            printf("#    p 99 Camera pan (Yaw) degrees (positive CCV) is %.1f deg\n", cam.camRot[2] * 180 / M_PI);
            printf("#    q    Quit now\n");
            printf("#    r 99 Camera roll degrees (positive left), is %.1f deg\n", cam.camRot[0] * 180 / M_PI);
            printf("#    s    Status (all)\n");
            printf("#    t 99 Camera tilt degrees (positive down), is %.1f deg\n", cam.camRot[1] * 180 / M_PI);
            printf("#    2 x y h d    To face destination (x,y,h) at dist d \n");
            printf("#    3 x y h      Camera position on robot (is %.3f, %.3f, %.3f) [m]\n",
                   cam.camPos[0],cam.camPos[1],cam.camPos[2]);
            printf("#\n");
            printf("# NB!  Robot may continue to move if this app is stopped with ctrl-C.\n\n");
            break;
          default:
            printf("# unknown command '%s'\n", s);
            break;
        }
      }
      // wait long enough for function to finish.
      usleep(300000);
      printf(">> ");
      fflush(stdout);
      //mission.runMission(firstMission, lastMission);
    }
    printf("Main ended (connected=%d finished=%d)\n", bridge.connected, mission.finished);
  }
}
