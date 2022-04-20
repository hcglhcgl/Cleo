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


#include <sys/time.h>
#include <cstdlib>
#include "umission.h"
#include "utime.h"
#include "ulibpose2pose.h"
#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include "types.h"

bool alreadyParked = false;

UMission::UMission(UBridge * regbot/*, UCamera * camera*/) {
  //cam = camera;
  bridge = regbot;
  threadActive = 100;
  // initialize line list to empty
  for (int i = 0; i < missionLineMax; i++) { // add to line list 
    lines[i] = lineBuffer[i];    
    // terminate c-strings strings - good practice, but not needed
    lines[i][0] = '\0';
  }
  // start mission thread
  th1 = new thread(runObj, this);
//   play.say("What a nice day for a stroll\n", 100);
//   sleep(5);
  computerVision = new CVPositions();
}

UMission::~UMission() {
  printf("Mission class destructor\n");
}

void UMission::run() {
  while (not active and not th1stop)
    usleep(100000);
//   printf("UMission::run:  active=%d, th1stop=%d\n", active, th1stop);
  if (not th1stop)
    runMission();
  printf("UMission::run: mission thread ended\n");
}
  
void UMission::printStatus() {
  printf("# ------- Mission ----------\n");
  printf("# active = %d, finished = %d\n", active, finished);
  printf("# mission part=%d, in state=%d\n", mission, missionState);
}
  
/**
 * Initializes the communication with the robobot_bridge and the REGBOT.
 * It further initializes a (maximum) number of mission lines 
 * in the REGBOT microprocessor. */
void UMission::missionInit() { // stop any not-finished mission
  bridge->send("robot stop\n");
  // clear old mission
  bridge->send("robot <clear\n");
  //
  // add new mission with 3 threads
  // one (100) starting at event 30 and stopping at event 31
  // one (101) starting at event 31 and stopping at event 30
  // one (  1) used for idle and initialisation of hardware
  // the mission is started, but staying in place (velocity=0, so servo action)
  //
  bridge->send("robot <add thread=1\n");
  // Irsensor should be activated a good time before use 
  // otherwise first samples will produce "false" positive (too short/negative).
  bridge->send("robot <add irsensor=1,vel=0:dist<0.2\n");
  //
  // alternating threads (100 and 101, alternating on event 30 and 31 (last 2 events)
  bridge->send("robot <add thread=100,event=30 : event=31\n");
  for (int i = 0; i < missionLineMax; i++)
    // send placeholder lines, that will never finish
    // are to be replaced with real mission
    // NB - hereafter no lines can be added to these threads, just modified
    bridge->send("robot <add vel=0 : time=0.1\n");
  //
  bridge->send("robot <add thread=101,event=31 : event=30\n");
  for (int i = 0; i < missionLineMax; i++)
    // send placeholder lines, that will never finish
    bridge->send("robot <add vel=0 : time=0.1\n");
  usleep(10000);

  // send subscribe to bridge
  bridge->pose->subscribe();
  bridge->edge->subscribe();
  bridge->motor->subscribe();
  bridge->event->subscribe();
  bridge->joy->subscribe();
  bridge->motor->subscribe();
  bridge->info->subscribe();
  bridge->irdist->subscribe();
  bridge->imu->subscribe();
  usleep(10000);
  // there maybe leftover events from last mission
  bridge->event->clearEvents();
}

void UMission::sendAndActivateSnippet(char ** missionLines, int missionLineCnt) {
  // Calling sendAndActivateSnippet automatically toggles between thread 100 and 101. 
  // Modifies the currently inactive thread and then makes it active. 
  const int MSL = 100;
  char s[MSL];
  int threadToMod = 101;
  int startEvent = 31;
  // select Regbot thread to modify
  // and event to activate it
  if (threadActive == 101) {
    threadToMod = 100;
    startEvent = 30;
  }
  if (missionLineCnt > missionLineMax) {
    printf("# ----------- error - too many lines ------------\n");
    printf("# You tried to send %d lines, but there is buffer space for %d only!\n", missionLineCnt, missionLineMax);
    printf("# set 'missionLineMax' to a higher number in 'umission.h' about line 57\n");
    printf("# (not all lines will be send)\n");
    printf("# -----------------------------------------------\n");
    missionLineCnt = missionLineMax;
  }
  // send mission lines using '<mod ...' command
  for (int i = 0; i < missionLineCnt; i++) { // send lines one at a time
    if (strlen((char*)missionLines[i]) > 0) { // send a modify line command
      snprintf(s, MSL, "<mod %d %d %s\n", threadToMod, i+1, missionLines[i]);
      bridge->send(s); 
    }
    else
      // an empty line will end code snippet too
      break;
  }
  // let it sink in (10ms)
  usleep(10000);
  // Activate new snippet thread and stop the other  
  snprintf(s, MSL, "<event=%d\n", startEvent);
  bridge->send(s);
  // save active thread numbers
  threadActive = threadToMod;
}

void UMission::parkArm() {
  int line = 0;
  int parkLoc = 450;

  printf("Inside parkArm()\n");

  // if (!alreadyParked) {
  //   Can NOT use 'time' here
  //   snprintf(lines[line++], MAX_LEN, "servo=2, pservo=%d", parkLoc);
  //   snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-%d", parkLoc);
    
  //   Waiting for 1 sec before continuing
  //   snprintf(lines[line++], MAX_LEN, "vel=0 : time=1");
  //   snprintf(lines[line++], MAX_LEN, "event=9, vel=0 : time=1");
  //   sendAndActivateSnippet(lines, line);

  //   while(!(bridge->event->isEventSet(9))) {
  //       alreadyParked = false;
  //   }

  //   disableArm();

  //   alreadyParked = true;
  // }


  snprintf(lines[line++], MAX_LEN, "servo=2, pservo=%d", parkLoc);
  snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-%d", parkLoc);

  //Waiting for 1 sec before continuing
  snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");
  snprintf(lines[line++], MAX_LEN, "event=1, vel=0 : time=1");
  
  sendAndActivateSnippet(lines, line);

  while(!(bridge->event->isEventSet(1))) {
    //do nothing
  }

  disableArm();
}

void UMission::disableArm() {

  printf("Inside disableArm()\n");

  int line = 0;

  //Can NOT use 'time' here
  snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000");
  snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000");
  
  snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");
  snprintf(lines[line++], MAX_LEN, "event=1, vel=0 : time=1");

  sendAndActivateSnippet(lines, line);

  while(!(bridge->event->isEventSet(1))) {
    //do nothing
  }
}

void UMission::setArm(int armPose) {

  printf("Inside setArm()\n");

  int line = 0;

  snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-%d, vservo=0", armPose);
  snprintf(lines[line++], MAX_LEN, "servo=3, pservo=%d, vservo=0", armPose);

  snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");
  snprintf(lines[line++], MAX_LEN, "event=1, vel=0 : time=1");

  sendAndActivateSnippet(lines, line);

  while(!(bridge->event->isEventSet(1))) {
    //do nothing
  }
}

//////////////////////////////////////////////////////////

/**
 * Thread for running the mission(s)
 * All missions segments are called in turn based on mission number
 * Mission number can be set at parameter when starting mission command line.
 * 
 * The loop also handles manual override for the gamepad, and resumes
 * when manual control is released.
 * **/
void UMission::runMission() {
  mission = fromMission;
  int missionOld = mission;
  bool regbotStarted = false;
  // end flag for current mission
  bool ended = false;
  // manuel override - using gamepad
  bool inManual = false;
  // debug loop counter
  int loop = 0;
  // keeps track of mission state
  missionState = 0;
  int missionStateOld = missionState;
  // fixed string buffer
  const int MSL = 120;
  char s[MSL];
  /// initialize robot mission to do nothing (wait for mission lines)
  missionInit();
  /// start (the empty) mission, ready for mission snippets.
  bridge->send("start\n"); // ask REGBOT to start controlled run (ready to execute)
  bridge->send("oled 3 waiting for REGBOT\n");
  //play.say("Waiting for robot data.", 100);
  
  for (int i = 0; i < 3; i++) {
    if (not bridge->info->isHeartbeatOK()) { // heartbeat should come at least once a second
      sleep(2);
    }
  }
  if (not bridge->info->isHeartbeatOK()) { // heartbeat should come at least once a second
    //play.say("Oops, no usable connection with robot.", 100);
    //system("espeak \"Oops, no usable connection with robot.\" -ven+f4 -s130 -a60 2>/dev/null &"); 
    bridge->send("oled 3 Oops: Lost REGBOT!");
    printf("# ---------- error ------------\n");
    printf("# No heartbeat from robot. Bridge or REGBOT is stuck\n");
    //printf("# You could try restart ROBOBOT bridge ('b' from mission console) \n");
    printf("# -----------------------------\n");

    if (false)
      // for debug - allow this
      stop();
  }
  // loop in sequence every mission until they report ended
  while (not finished and not th1stop) { // stay in this mission loop until finished
    loop++;
    // test for manuel override (joy is short for joystick or gamepad)
    if (bridge->joy->manual) { // just wait, do not continue mission
      usleep(20000);
      if (not inManual) {
        //system("espeak \"Mission paused.\" -ven+f4 -s130 -a40 2>/dev/null &"); 
        //play.say("Mission paused.", 90);
      }
      inManual = true;
      bridge->send("oled 3 GAMEPAD control\n");
    }
    else { // in auto mode
      if (not regbotStarted) { // wait for start event is received from REGBOT
        // - in response to 'bot->send("start\n")' earlier
        if (bridge->event->isEventSet(33)) { // start mission (button pressed)
          //printf("Mission::runMission: starting mission (part from %d to %d)\n", fromMission, toMission);
          regbotStarted = true;
        }
      }
      else { // mission in auto mode
        if (inManual) { // just entered auto mode, so tell.
          inManual = false;
          //system("espeak \"Mission resuming.\" -ven+f4 -s130 -a40 2>/dev/null &");
          //play.say("Mission resuming", 90);
          //bridge->send("oled 3 running AUTO\n");
        }
        switch(mission) {
          case 1:
            ended = mission_guillotine(missionState);
            break;
          case 2:
            ended = mission_ball_1(missionState);
            break;
          case 3:
            ended = mission_seesaw(missionState);
            break;
          case 4:
            ended = mission_ball_2(missionState);
            break;
          case 5:
            ended = mission_stairs(missionState);
            break;
          case 6:
            ended = mission_parking(missionState);
            break;
          // case 7:
          //   ended = mission_parking_without_closing(missionState);
          //   break;
          case 7:
            ended = mission_parking_with_closing(missionState);
            break;
          // case 7:
          //   ended = mission_skipping_parking(missionState);
          //   break;
          // case 8:
          //   ended = mission_appleTree_Identifier(missionState);
          //   break;
          case 8:
            ended = mission_racetrack(missionState);
            break;
          // case 9:
          //   ended = mission_circleOfHell(missionState);
          //   break;
          case 9:
            ended = mission_skipping_circleOfHell(missionState);
            break;
          case 10:
            ended = mission_appleTree_Identifier_Kids_Edition(missionState);
            break;
          case 11:
            ended = mission_go_to_goal(missionState);
            break;
          default:
            // no more missions - end everything
            finished = true;
            break;
        }
        if (ended) { // start next mission part in state 0
          printf("Mission ended\n");
          mission++;
          ended = false;
          missionState = 0;
        }
        // show current state on robot display
        if (mission != missionOld or missionState != missionStateOld) { // update small O-led display on robot - when there is a change
          UTime t;
          t.now();
          snprintf(s, MSL, "oled 4 mission %d state %d\n", mission, missionState);
          bridge->send(s);
          if (logMission != NULL) {
            fprintf(logMission, "%ld.%03ld %d %d\n", 
                    t.getSec(), t.getMilisec(),
                    missionOld, missionStateOld
            );
            fprintf(logMission, "%ld.%03ld %d %d\n", 
                    t.getSec(), t.getMilisec(),
                    mission, missionState
            );
          }
          missionOld = mission;
          missionStateOld = missionState;
        }
      }
    }
    //
    // check for general events in all modes
    // gamepad buttons 0=green, 1=red, 2=blue, 3=yellow, 4=LB, 5=RB, 6=back, 7=start, 8=Logitech, 9=A1, 10 = A2
    // gamepad axes    0=left-LR, 1=left-UD, 2=LT, 3=right-LR, 4=right-UD, 5=RT, 6=+LR, 7=+-UD
    // see also "ujoy.h"
    if (bridge->joy->button[BUTTON_RED]) { // red button -> save image
      /*if (not cam->saveImage) {
        printf("UMission::runMission:: button 1 (red) pressed -> save image\n");
        cam->saveImage = true;
      }*/
    }
    if (bridge->joy->button[BUTTON_YELLOW]) { // yellow button -> make ArUco analysis
      /*if (not cam->doArUcoAnalysis) {
        printf("UMission::runMission:: button 3 (yellow) pressed -> do ArUco\n");
        cam->doArUcoAnalysis = true;
      }*/
    }
    // are we finished - event 0 disables motors (e.g. green button)
    if (bridge->event->isEventSet(0)) { // robot say stop
      finished = true;
      printf("Mission:: insist we are finished\n");
    }
    else if (mission > toMission) { // stop robot
      // make an event 0
      bridge->send("stop\n");
      // stop mission loop
      finished = true;
    }
    // release CPU a bit (10ms)
    usleep(10000);
  }
  bridge->send("stop\n");
  snprintf(s, MSL, "Robot%s finished.\n", bridge->info->robotname);
  // system(s); 
  //play.say(s, 100);
  printf("%s", s);
  bridge->send("oled 3 finished\n");
}

////////////////////////////////////////////////////////////

bool UMission::mission_guillotine(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Press green to start.\n");
      state = 1;
    } break;

    case 1: {
      if (bridge->joy->button[BUTTON_GREEN])
        state = 10;
      state = 10;
      printf(">> Starting mission guillotine\n");
      //play.say("Starting mission guillotine", 100);
    } break;

    case 10: {
      int line = 0;

      parkArm();

      snprintf(lines[line++], MAX_LEN, "vel=0.6, edgel=0, white=1 : xl>15");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=1, vel=0.4 : dist=1");

      // send lines to Cleo
      sendAndActivateSnippet(lines, line);

      state = 11;
      featureCnt = 0;
    } break;

    case 11: {
      if (bridge->event->isEventSet(1)) {
        state = 999;
      } 
    } break;

    case 999:
    default:
      printf(">> Mission guillotine ended \n");
      finished = true;
      break;
  }
  return finished;
}

bool UMission::mission_ball_1(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission_ball_1\n");
      //play.say("Starting mission ball 1", 100);

      state = 10;
    } break;

    case 10: {
      int line = 0;

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edger=0, white=1  : tilt>0");
      snprintf(lines[line++], MAX_LEN, "vel=0.3, edger=0, white=1  : dist=0.37");
      snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn=87");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.13");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=2, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);

      state = 11;
      featureCnt = 0;
    } break;

    case 11: {
      if (bridge->event->isEventSet(2)) {
        state = 12;
      }
    } break;

    case 12: {
      int line = 0;

      setArm(500);

      disableArm();

      
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.50");
      snprintf(lines[line++], MAX_LEN, "vel=-0.3 : dist=0.14");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-95");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, edger=0, white=1 : dist=0.4");



      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.32");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");


      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=3, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);

      state = 13;
      featureCnt = 0;
    } break;

    case 13: {
      if (bridge->event->isEventSet(3)) {
        state = 14;
      }
    } break;

    case 14: {

      int line = 0;

      setArm(650);

      snprintf(lines[line++], MAX_LEN, "vel=0.27, tr=0 : turn=-27");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0.27, tr=0 : turn=27");

      snprintf(lines[line++], MAX_LEN, "vel=0.3 : dist=0.03");
      snprintf(lines[line++], MAX_LEN, "vel=0.27, tr=0 : turn=-27");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0.27, tr=0 : turn=27");
      
      snprintf(lines[line++], MAX_LEN, "vel=-0.3 : dist = 0.1");
      snprintf(lines[line++], MAX_LEN, "vel=0.27, tr=0 : turn=-27");


      // snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=30");
      // snprintf(lines[line++], MAX_LEN, "vel=0.3 : dist=0.05");

      // snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-30");
      // snprintf(lines[line++], MAX_LEN, "vel=0.3 : dist=0.05");¨

            // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=3, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);

      state = 15;

    } break;

      case 15: {
      if (bridge->event->isEventSet(3)) {
        state = 16;
      }
    } break;

    case 16: {
      int line = 0;
      
      parkArm();

      snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn=220");

      snprintf(lines[line++], MAX_LEN, "vel=0.3, edgel=0, white=1 : dist=1.1");
      snprintf(lines[line++], MAX_LEN, "vel=0.3, edger=0, white=1 : xl>15");
      snprintf(lines[line++], MAX_LEN, "vel=0.3, edger=0, white=1 : dist=0.03");

      snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn=-115");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=3, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);

      state = 17;
      featureCnt = 0;
    } break;

      case 17: {
      if (bridge->event->isEventSet(3)) {
        state = 999;
      }
    } break;


    case 999:
    default:
      printf(">> Mission_ball_1 ended\n");

      finished = true;
      break;
  }
  return finished;
}

bool UMission::mission_seesaw(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission_seesaw\n");
      //play.say("Starting mission seesaw", 100);

      state = 10;
    } break;

    case 10: {
      int line = 0;

      setArm(550);
      
      snprintf(lines[line++], MAX_LEN, "vel=0.3, : dist=0.2");
      snprintf(lines[line++], MAX_LEN, "vel=0.3, edgel=0, white=1 : dist=0.91");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");
      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-810, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=810, vservo=0");
      // snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : dist>0.1");
      // snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : ir1<0.3");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : time=0.2");   //This line needs to be here for the 'lv=0' to work
      snprintf(lines[line++], MAX_LEN, "vel=0.5, edgel=0, white=1 : lv=0");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=4, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);

      state = 11;
      featureCnt = 0;
    } break;

    case 11: {
      if (bridge->event->isEventSet(4)) {
        state = 12;
      }
    } break;

    case 12: {
      int line = 0;

      disableArm();

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : time=1.5");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-75");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.2");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=10");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : xl > 15");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=20");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.1");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : xl > 15 ");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.1");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=5, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);

      state = 13;
      featureCnt = 0;
    } break;

    case 13: {
      if (bridge->event->isEventSet(5)) {
        state = 999;
      }
    } break;

    case 999:
    default:
      printf(">> Mission_seesaw ended\n");

      finished = true;
      break;
  }
  return finished;
}

bool UMission::mission_ball_2(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission_ball_2\n");
      //play.say("Starting mission balls 2", 100);

      state = 10;
    } break;

    case 10: {
      int line = 0;

      snprintf(lines[line++], MAX_LEN, "vel=0.5, tr=0 : turn=-140");

      snprintf(lines[line++], MAX_LEN, "vel=-0.4 : dist=0.2");

      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");
      
      snprintf(lines[line++], MAX_LEN, "vel=0.5, edgel=0, white=1 : dist=2.3");
      // snprintf(lines[line++], MAX_LEN, "vel=0.5, edgel=0, white=1 : tilt > 0"); //Not working

      snprintf(lines[line++], MAX_LEN, "vel=0.5, edgel=0, white=1 : ir1 < 0.3");

      snprintf(lines[line++], MAX_LEN, "vel=0.6, edgel=0, white=1 : dist=0.3");
      snprintf(lines[line++], MAX_LEN, "vel=0.6 : dist=0.61");

      snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn=30");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=5, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
    

      state = 11;
      featureCnt = 0;
    } break;

    case 11: {
      if (bridge->event->isEventSet(5)) {
        state = 12;
      }
    } break;

    case 12: {
      int line = 0;

      setArm(650);

      snprintf(lines[line++], MAX_LEN, "vel=0.25, tr=0 : turn=-27");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0.25, tr=0 : turn=27");

      snprintf(lines[line++], MAX_LEN, "vel=0.3 : dist=0.03");
      snprintf(lines[line++], MAX_LEN, "vel=0.25, tr=0 : turn=-27");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0.25, tr=0 : turn=27");
          
      snprintf(lines[line++], MAX_LEN, "vel=-0.3 : dist = 0.1");
      snprintf(lines[line++], MAX_LEN, "vel=0.25, tr=0 : turn=-27");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=5, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
    

      state = 13;
      featureCnt = 0;
    } break;

    case 13: {
      if (bridge->event->isEventSet(5)) {
        state = 14;
      }
    } break;

    case 14: {
      int line = 0;

      // snprintf(lines[line++], MAX_LEN, "vel=0.35, tr=0 : turn=-10");
      // snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      // snprintf(lines[line++], MAX_LEN, "vel=0.35, tr=0 : turn=35");
      // snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      parkArm();

      // snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-150, vservo=0");
      // snprintf(lines[line++], MAX_LEN, "servo=3, pservo=150, vservo=0");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-80");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=5, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
    

      state = 15;
      featureCnt = 0;
    } break;

    case 15: {
      if (bridge->event->isEventSet(5)) {
        state = 999;
      }
    } break;

    case 999:
    default:
      printf(">> Mission_ball_2 ended\n");

      finished = true;
      break;
  }
  return finished;
}

bool UMission::mission_stairs(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission_stairs\n");
      //play.say("Starting mission stairs", 100);

      state = 10;
    } break;

    case 10: {

      parkArm();

      int line = 0;

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : dist=1");

      
      snprintf(lines[line++], MAX_LEN, "vel=0.4, edger=0, white=1 : xl > 15");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=235");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : dist=0.6");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1 : turn=30");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.5");
      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-800, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=800, vservo=0");

      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.3, edge=0, white=1 : dist=0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0.45, edge=0, white=1 : dist=0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0.45, edge=0, white=1 : dist=0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0.45, edge=0, white=1 : dist=0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0.45, edge=0, white=1 : dist=0.4");


      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=6, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
    

      state = 11;
      featureCnt = 0;
    } break;

    case 11: {
      if (bridge->event->isEventSet(6)) {
        state = 12;
      }
    } break;

    case 12: {

      parkArm();

      int line = 0;

      snprintf(lines[line++], MAX_LEN, "vel=-0.4 : time=3");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=20");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : xl > 15");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=40");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");


      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=6, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
    

      state = 13;
      featureCnt = 0;
    } break;

    case 13: {
      if (bridge->event->isEventSet(6)) {
        state = 999;
      }
    } break;

    case 999:
    default:
      printf(">> Mission_stairs ended\n");

      finished = true;
      break;
  }
  return finished;
}

bool UMission::mission_parking(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission parking\n");
      // play.say("Starting mission parking", 100);

      state = 10;
    } break;

    case 10: {
      int line = 0;

    

      snprintf(lines[line++], MAX_LEN, "vel=0, edgel=0, white=1 : time=1");
      snprintf(lines[line++], MAX_LEN, "vel=0.35, edgel=0, white=1 : lv=0");    //vel=0.25
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.6");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.0 : turn=-90");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
      
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist>0.95");  //0.47
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.2 : turn=92");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-700, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=700, vservo=0");

      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=8, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 11;
      featureCnt = 0;
    } break;
    
    case 11: {
      if (bridge->event->isEventSet(8)) {
        state = 12;
      }
    } break;

    case 12: {
      int line = 0;


      disableArm();

      snprintf(lines[line++], MAX_LEN, "vel=0.8 : time=4");


      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=8, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 13;
      featureCnt = 0;
    } break;
    
    case 13: {
      if (bridge->event->isEventSet(8)) {
        state = 14;
      }
    } break;

    case 14: {
      int line = 0;


      parkArm();

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn= 90");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=-0.3 : dist=0.2");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : ir1 > 0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.40");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-87");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.8");


      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn= -100");
      //snprintf(lines[line++], MAX_LEN, "vel=0.4 : ir2 < 0.2");
      snprintf(lines[line++], MAX_LEN, "vel=-0.4 : dist=0.2");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-700, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=700, vservo=0");

      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");


      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=8, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 15;
      featureCnt = 0;
    } break;
    
    case 15: {
      if (bridge->event->isEventSet(8)) {
        state = 16;
      }
    } break;

    case 16: {
      int line = 0;


      disableArm();

      snprintf(lines[line++], MAX_LEN, "vel=0.8 : time=3");

      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=450, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-450, vservo=0");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time= 0.5");

      //snprintf(lines[line++], MAX_LEN, "vel=0.3 : dist=0.18");
      snprintf(lines[line++], MAX_LEN, "vel=0.3 : time=2");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");


      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-86");


      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=8, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 17;
      featureCnt = 0;
    } break;
    
    case 17: {
      if (bridge->event->isEventSet(8)) {
        state = 18;
      }
    } break;

    case 18: {
      int line = 0;

      parkArm();


      snprintf(lines[line++], MAX_LEN, "vel=1 : dist=0.45");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");


      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.25 : turn=187");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-700, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=700, vservo=0");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time= 0.5");
      

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : ir1 > 0.3");
      //snprintf(lines[line++], MAX_LEN, "vel=0.4 : ir1 > 0.6");
      
      snprintf(lines[line++], MAX_LEN, "event=9, vel=0 : dist=1");


      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 19;
      featureCnt = 0;
    } break;
    
    case 19: {
      if (bridge->event->isEventSet(9)) {
        state = 999;
      }
    } break;

    case 999:
    default:
      printf(">> Mission parking ended\n");
      finished = true;
      break;
  }
  return finished;
}

bool UMission::mission_parking_without_closing(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission parking without closing\n");
      // play.say("Starting mission parking without closing", 100);

      state = 10;
    } break;

    case 10: {
      int line = 0;

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.45");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-25");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.2");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : dist=1");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=8, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 11;
      featureCnt = 0;
    } break;
    
    case 11: {
      if (bridge->event->isEventSet(9)) {
        state = 999;
      }
    } break;
    case 999:
    default:
      printf(">> Mission parking without closing ended\n");
      finished = true;
      break;
  }
  return finished;
}

bool UMission::mission_parking_with_closing(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission parking with closing\n");
      // play.say("Starting mission parking with closing", 100);

      state = 10;
    } break;

    case 10: {
      int line = 0;

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.4");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-120");

      snprintf(lines[line++], MAX_LEN, "vel=0.5 : time=6");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=450, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-450, vservo=0");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time= 0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.1");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-92");



      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=8, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 11;
      featureCnt = 0;
    } break;
    
    case 11: {
      if (bridge->event->isEventSet(8)) {
        state = 12;
      }
    } break;

    case 12: {
      int line = 0;

      setArm(700);
      disableArm();

      snprintf(lines[line++], MAX_LEN, "vel=0.6 : dist=1");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.5, tr=0 : turn=-100");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.6 : dist=0.15");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-110");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : time=3");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=30");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=450, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-450, vservo=0");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.1");


      snprintf(lines[line++], MAX_LEN, "vel=0.4 : ir1 > 0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.35");

      
      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=9, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 13;
      featureCnt = 0;
    } break;
    
    case 13: {
      if (bridge->event->isEventSet(9)) {
        state = 14;
      }
    } break;

    case 14: {
      int line = 0;

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-90");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.6");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-90");

      snprintf(lines[line++], MAX_LEN, "vel=-0.3 : dist=0.3");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-700, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=700, vservo=0");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time= 0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.8 : time=3");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      
      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=9, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 15;
      featureCnt = 0;
    } break;
    
    case 15: {
      if (bridge->event->isEventSet(9)) {
        state = 16;
      }
    } break;

    case 16: {
      int line = 0;

      parkArm();

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.15");


      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=89");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-700, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=700, vservo=0");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");


      snprintf(lines[line++], MAX_LEN, "vel=0.5 : time=4");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");


      snprintf(lines[line++], MAX_LEN, "vel=-0.3 : dist=0.2");

      
      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=9, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 17;
      featureCnt = 0;
    } break;
    
    case 17: {
      if (bridge->event->isEventSet(9)) {
        state = 18;
      }
    } break;


    case 18: {
      int line = 0;


      // parkArm();

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-110");

      // snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-700, vservo=0");
      // snprintf(lines[line++], MAX_LEN, "servo=3, pservo=700, vservo=0");
      // snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      // snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
      // snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");
      // snprintf(lines[line++], MAX_LEN, "vel=0 : time= 0.5");



      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.1");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-60");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : time=1");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=30");

      snprintf(lines[line++], MAX_LEN, "vel=0.7 : time = 4");

      snprintf(lines[line++], MAX_LEN, "vel=-0.3 : dist = 0.2");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo= 450, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo= -450, vservo=0");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time= 0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=90");


      
      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=9, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 19;
      featureCnt = 0;
    } break;
    
    case 19: {
      if (bridge->event->isEventSet(9)) {
        state = 999;
      }
    } break;

    case 999:
    default:
      printf(">> Mission parking with closing ended\n");
      finished = true;
      break;
  }
  return finished;
}

bool UMission::mission_skipping_parking(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission parking\n");
      // play.say("Starting mission parking", 100);

      state = 10;
    } break;

    case 10: {
      int line = 0;

      parkArm();

      snprintf(lines[line++], MAX_LEN, "vel=0, edgel=0, white=1 : time=1");
      snprintf(lines[line++], MAX_LEN, "vel=0.35, edgel=0, white=1 : lv=0");    //vel=0.25
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.6");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.0 : turn=-90");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
      
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=1.5");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");



      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-700, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=700, vservo=0");

      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : time=2");



      

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=8, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 11;
      featureCnt = 0;
    } break;
    
    case 11: {
      if (bridge->event->isEventSet(8)) {
        state = 12;
      }
    } break;

    case 12: {
      int line = 0;

      parkArm();

      snprintf(lines[line++], MAX_LEN, "vel=0.3 : dist=0.22");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.0 : turn=85");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.3 : dist=1.2");

      

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-700, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=700, vservo=0");

      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");




      snprintf(lines[line++], MAX_LEN, "vel=0.3 : time=7");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");



      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=8, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 13;
      featureCnt = 0;
    } break;
    
    case 13: {
      if (bridge->event->isEventSet(8)) {
        state = 14;
      }
    } break;

    case 14: {
      int line = 0;

      parkArm();

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.0 : turn=90");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.2");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.0 : turn=-90");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.4");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : dist=1");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=8, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 15;
      featureCnt = 0;
    } break;
    
    case 15: {
      if (bridge->event->isEventSet(8)) {
        state = 999;
      }
    } break;

    case 999:
    default:
      printf(">> Mission parking ended\n");
      finished = true;
      break;
  }
  return finished;
}

bool UMission::mission_racetrack(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission racetrack\n");
      //play.say("Starting mission racetrack", 100);

      state = 10; // 10 
    } break;

    case 10: {
      int line = 0;

      parkArm();

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : dist=0.3");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : ir1 < 0.15");  //ir1 < 0.2
      snprintf(lines[line++], MAX_LEN, "vel=0 : ir2 < 0.5");  //0.5
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.1");
      snprintf(lines[line++], MAX_LEN, "vel=0 : ir2 > 0.5");  //0.5

      snprintf(lines[line++], MAX_LEN, "vel=2.1, acc=1, edgel=0, white=1 : dist = 7.3");

      // snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-700, vservo=0");
      // snprintf(lines[line++], MAX_LEN, "servo=3, pservo=700, vservo=0");

      // snprintf(lines[line++], MAX_LEN, "vel=0.5, edgel=0, white=1 : ir1 < 0.2");

      // snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
      // snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");

      snprintf(lines[line++], MAX_LEN, "vel=1, acc=1, edgel=0, white=1 : ir1 < 0.4");

      snprintf(lines[line++], MAX_LEN, "vel=0 : time=1");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=9, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 11;
      featureCnt = 0;
    } break;

    case 11: {
      if (bridge->event->isEventSet(9)) {
        state = 12;
      }
    } break;

    case 12: {
      int line = 0;

      parkArm();

      // snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-190");
      // snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.2");


      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-110");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=1");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-90");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=1.5");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-90");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : xl > 15 ");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=90");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");


      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=9, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 13;
      featureCnt = 0;
    } break;

    case 13: {
      if (bridge->event->isEventSet(9)) {
        state = 14;
      }
    } break;

    case 14: {
      int line = 0;

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : ir1 < 0.3");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : dist = 1.47");


      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=10, vel=0 : dist=1"); //

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 15;
      featureCnt = 0;
    } break;
    
    case 15: {
      if (bridge->event->isEventSet(10)) {
        state = 999;
      }
    } break;

    case 999:
    default:
      printf(">> Mission racetrack ended\n");
      finished = true;
      break;
  }
  return finished;
}

bool UMission::mission_appleTree_Identifier_Kids_Edition(int & state) {
  bool finished = false;

  // Create the elements that will receive the movement instructions
  bool straight = false, left = false, right = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission identify appletree\n");
      computerVision->init(false,true);
      state = 20;
    } break;
    
    case 20: {
      int line = 0;
      //Follow line and drive to the first tree (right)
      snprintf(lines[line++], MAX_LEN, "vel=0, edgel=0, white=1 : time=1");
      snprintf(lines[line++], MAX_LEN, "vel=0.35, edgel=0, white=1 : lv=0");    //vel=0.25
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      //Go straight a little bit
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.4");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
      //Turn sharp left
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=90");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=1");
      //Occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=20, vel=0 : dist=1");
      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 40;
    } break;


    case 40: {
      int line = 0;
      if(bridge->event->isEventSet(20)) {

        //Lower the servo to the correct height
        snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-640, vservo=0");
        snprintf(lines[line++], MAX_LEN, "servo=3, pservo=640, vservo=0");
        snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
        
        snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
        snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");
        snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
        //Allign with the wall
        snprintf(lines[line++], MAX_LEN, "vel=0.8 : time=3");
        //Raise the servo to the correct height

        snprintf(lines[line++], MAX_LEN, "vel=-0.4 : dist=0.05");

        snprintf(lines[line++], MAX_LEN, "servo=2, pservo=450, vservo=0");
        snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-450, vservo=0");
        snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
        
        snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
        snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");
        snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
        //Turn sharp right
        snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-90");

        //Occupy Robot
        snprintf(lines[line++], MAX_LEN, "event=3, vel=0 : dist=1");

        // send lines to REGBOT
        sendAndActivateSnippet(lines, line);

        state = 50;
        
      }
      
    } break;

    case 50:
    {
      if(bridge->event->isEventSet(3))
      {
        int line = 0;
        //Go straight 
        snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.2");

        //Lower the servo to the correct height
        snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-640, vservo=0");
        snprintf(lines[line++], MAX_LEN, "servo=3, pservo=640, vservo=0");
        snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

        //Occupy Robot
        snprintf(lines[line++], MAX_LEN, "event=4, vel=0 : dist=1");

        // send lines to REGBOT
        sendAndActivateSnippet(lines, line);
      }
      if(bridge->event->isEventSet(4))
      {
        state = 60;
      }
    } break;

    case 60: 
    {
      // Drive to the first tree and grab it
      pose_t trunk_pos;
      static pose_t trunk_pos_avg;

      int avg = 3;

      static int avg_counter = 0;

      cout << "Get trunk pose" << endl;

      trunk_pos = computerVision->trunkPos();
        if(trunk_pos.valid) 
        {
          cout << "x: " << trunk_pos.x << " y: " << trunk_pos.y << " z: " << trunk_pos.z << endl;

          if (avg_counter < avg) {
            trunk_pos_avg.x += trunk_pos.x;
            avg_counter++;
          }
          else {
            trunk_pos_avg.x = trunk_pos_avg.x/(double)avg;
            computerVision->determineMovement(trunk_pos_avg,straight,left,right);
            printf("Determining movement. x = %f\n",trunk_pos_avg.x);
            avg_counter = 0;
            trunk_pos_avg.x = 0;
            new_event_ready = true;
          }
          
          //Movements
          int line = 0;
          static int correctionCounter = 0;
          // Line to send in case every action is false.
          if(new_event_ready)
          {
            new_event_ready = false;
            if(straight && correctionCounter > 9){
              printf("Going straight.\n"); 

              straight = false;
              event_nr = 7;

            }
            else if(right){
              printf("Going right.\n"); 
              snprintf(lines[line++], MAX_LEN, "vel=0.2, tr=0.0: turn=1");
              snprintf(lines[line++], MAX_LEN, "vel=0.2: dist=0.02");
              correctionCounter++;
              right = false;
            }
            else if(left){
              printf("Going left.\n");
              snprintf(lines[line++], MAX_LEN, "vel=0.2, tr=0.0: turn=-1");
              snprintf(lines[line++], MAX_LEN, "vel=0.2: dist=0.02");
              correctionCounter++;
              left = false;
            }
            else if(!left && !right && correctionCounter <= 9){
              snprintf(lines[line++], MAX_LEN, "vel=0.2: dist=0.02");
              correctionCounter++;
            }
            // occupy Robot
            snprintf(lines[line++], MAX_LEN, "event=%d, vel=0 : dist=1", event_nr);
            //printf(lines[line-1]);
            sendAndActivateSnippet(lines, line);
            
          }
          // send lines to REGBOT
          if(event_nr == 7)
          {
            cout << "grabbed the tree.." << endl;
            state = 70;

          }
          else if(event_nr == 8)
          {
            if(bridge->event->isEventSet(8))
            {
              line = 0;
              event_nr = 9;
            }

          }
          else if(event_nr == 9)
          {
            if(bridge->event->isEventSet(9))
            {
              line = 0;
              event_nr = 8;
            }
          }

          
        }
    } break;

    case 70: {
      int line = 0;
      //Navigate to the correct Aruco!
      if(bridge->event->isEventSet(7))
      {
        cout << "Going to the left aruco.." << endl;
        //Reset event_nr
        event_nr = 8;
        //Arm up a little bit
        snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-600, vservo=0");
        snprintf(lines[line++], MAX_LEN, "servo=3, pservo=600, vservo=0");

        snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
        //Go to tree
        snprintf(lines[line++], MAX_LEN, "vel=0.25: dist=0.45");        
        
        //Drive forward
        snprintf(lines[line++], MAX_LEN, "vel=0.25: time=11.5");
        //snprintf(lines[line++], MAX_LEN, "vel=0.25 : ir2 < 0.35");
        //Small bump
        snprintf(lines[line++], MAX_LEN, "vel=-0.3: dist=0.1");
        snprintf(lines[line++], MAX_LEN, "vel=0.6: dist=0.22");
        snprintf(lines[line++], MAX_LEN, "vel=0: time=2");
        // occupy Robot
        snprintf(lines[line++], MAX_LEN, "event=11, vel=0 : dist=1");
        sendAndActivateSnippet(lines, line);
        state = 80;
        }
    } break;

    case 80: {
      int line = 0;
      if (bridge->event->isEventSet(11)) {
        printf("Going back to initial position \n");
        // Go back
        snprintf(lines[line++], MAX_LEN, "vel=-0.30: dist=0.2");
        //Arm Up
        snprintf(lines[line++], MAX_LEN, "servo=2, pservo=450, vservo=0");
        snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-450, vservo=0");

        snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
        
        snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
        snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");

        snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

        //Turn slightly left
        snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0.0: turn=3");
        //Go straight back
        snprintf(lines[line++], MAX_LEN, "vel=-0.40: dist=2");
        //Turn back & left
        snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0.0: turn=90");
        //Lower arm
        snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-600, vservo=0");
        snprintf(lines[line++], MAX_LEN, "servo=3, pservo=600, vservo=0");

        snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
        
        snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
        snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");

        snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
        //Allign with the wall
        snprintf(lines[line++], MAX_LEN, "vel=0.8 : time=3");
        // occupy Robot
        snprintf(lines[line++], MAX_LEN, "event=13, vel=0 : dist=1");
        sendAndActivateSnippet(lines, line);
        state = 90;
      }
    } break;

    case 90: {
      int line = 0;
      if (bridge->event->isEventSet(13)) {
        printf("Alligning to the next tree...\n");
        //Drive backwards
        snprintf(lines[line++], MAX_LEN, "vel=-0.3 : dist=0.66");
        //Turn right
        snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0.0: turn=-88");
        snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
        //Lower arm
        snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-640, vservo=0");
        snprintf(lines[line++], MAX_LEN, "servo=3, pservo=640, vservo=0");

        snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
        // occupy Robot
        snprintf(lines[line++], MAX_LEN, "event=14, vel=0 : dist=1");
        sendAndActivateSnippet(lines, line);
      }
      if (bridge->event->isEventSet(14)) {
        state = 100;
        printf("Tracking next trunk...\n");
      }
    } break;

    case 100: {
           // Drive to the first tree and grab it
      pose_t trunk_pos;
      static pose_t trunk_pos_avg;

      int avg = 3;

      static int avg_counter = 0;


      trunk_pos = computerVision->trunkPos();
        if(trunk_pos.valid) 
        {
          cout << "x: " << trunk_pos.x << " y: " << trunk_pos.y << " z: " << trunk_pos.z << endl;

          if (avg_counter < avg) {
            trunk_pos_avg.x += trunk_pos.x;
            avg_counter++;
          }
          else {
            trunk_pos_avg.x = trunk_pos_avg.x/(double)avg;
            computerVision->determineMovement(trunk_pos_avg,straight,left,right);
            printf("Determining movement. x = %f\n",trunk_pos_avg.x);
            avg_counter = 0;
            trunk_pos_avg.x = 0;
            new_event_ready = true;
          }
          
          //Movements
          int line = 0;
          static int correctionCounter = 0;
          // Line to send in case every action is false.
          if(new_event_ready)
          {
            new_event_ready = false;
            if(straight && correctionCounter > 8){
              printf("Going straight.\n"); 

              straight = false;
              event_nr = 7;

            }
            else if(right){
              printf("Going right.\n"); 
              snprintf(lines[line++], MAX_LEN, "vel=0.2, tr=0.0: turn=1");
              snprintf(lines[line++], MAX_LEN, "vel=0.2: dist=0.01");
              correctionCounter++;
              right = false;
            }
            else if(left){
              printf("Going left.\n");
              snprintf(lines[line++], MAX_LEN, "vel=0.2, tr=0.0: turn=-1");
              snprintf(lines[line++], MAX_LEN, "vel=0.2: dist=0.01");
              correctionCounter++;
              left = false;
            }
            else if(!left && !right && correctionCounter <= 9){
              snprintf(lines[line++], MAX_LEN, "vel=0.2: dist=0.01");
              correctionCounter++;
            }
            // occupy Robot
            snprintf(lines[line++], MAX_LEN, "event=%d, vel=0 : dist=1", event_nr);
            //printf(lines[line-1]);
            sendAndActivateSnippet(lines, line);
            
          }
          // send lines to REGBOT
          if(event_nr == 7)
          {
            cout << "grabbed the tree.." << endl;
            state = 110;

          }
          else if(event_nr == 8)
          {
            if(bridge->event->isEventSet(8))
            {
              line = 0;
              event_nr = 9;
            }

          }
          else if(event_nr == 9)
          {
            if(bridge->event->isEventSet(9))
            {
              line = 0;
              event_nr = 8;
            }
          }          
        }
        if(signal_var) {
          signal_var = false;
          state = 999;
      }
    } break;

    case 110: 
    {
      int line = 0;
      //Navigate to the correct Aruco!
      if(bridge->event->isEventSet(7))
      {
      printf("bringing ball to right aruco\n");
      //Arm up a little bit
      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-600, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=600, vservo=0");

      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
      //Go to tree
      snprintf(lines[line++], MAX_LEN, "vel=0.25: dist=0.45");        
      
      //Drive forward
      snprintf(lines[line++], MAX_LEN, "vel=0.25: time=12.5");
      //Small bump
      snprintf(lines[line++], MAX_LEN, "vel=-0.4: dist=0.1");
      snprintf(lines[line++], MAX_LEN, "vel=0.6: dist=0.22");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=2");
      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=5, vel=0 : dist=1");
      sendAndActivateSnippet(lines, line);
      state = 120;
      }
    }break;

    case 120:
    {      
      int line = 0;
      if(bridge->event->isEventSet(5))
      {
        printf("Going back to initial position \n");
        // Go back
        snprintf(lines[line++], MAX_LEN, "vel=-0.40: dist=1.7");
        //Turn left and allign
        snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0.0: turn=90");
        snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

        snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
        snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");

        snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
        snprintf(lines[line++], MAX_LEN, "vel=0.7: time=3");
        //move up arm
        snprintf(lines[line++], MAX_LEN, "servo=2, pservo=450, vservo=0");
        snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-450, vservo=0");
        snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
        //relax arm
        snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
        snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");
        snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
        //Go back a bit
        snprintf(lines[line++], MAX_LEN, "vel=-0.40: dist=0.25");
        //Turn around 90 more degrees
        snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0.0: turn=87");
        //Drive straight towards line
        snprintf(lines[line++], MAX_LEN, "vel=0.40: dist=0.4");
        // occupy Robot
        snprintf(lines[line++], MAX_LEN, "event=6, vel=0 : dist=1");
        sendAndActivateSnippet(lines, line);
      }
      if(bridge->event->isEventSet(6)){

        printf("MADE IT!!");
        state = 999;
      }
    } break;

    case 999:
    default:
      printf("Mission treeID ended \n");
      computerVision->shutdown();
      finished = true;
      break;
  }
  return finished;
}

bool UMission::mission_appleTree_Identifier(int & state) {
  bool finished = false;
  static bool currentTreeColor;

  // Create the elements that will receive the movement instructions
  bool straight = false, left = false, right = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission identify appletree\n");
      computerVision->init(false,true);
      state = 20;
    } break;

    case 1: {
      printf("Testing tree-finding and video saving\n");
    
      pose_t result;
      while (!signal_var) {
        result = computerVision->treeID(RED);
      }
      signal_var = false;
      state = 999;
    } break;

    case 20: {
      int line = 0;
      //Follow line and drive to the first tree (right)
      snprintf(lines[line++], MAX_LEN, "vel=0, edgel=0, white=1 : time=1");
      snprintf(lines[line++], MAX_LEN, "vel=0.35, edgel=0, white=1 : lv=0");    //vel=0.25
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.7");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-35");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=3");

      //Occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=20, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      
      state = 30;
    } break;
    
    case 30: {
      if (bridge->event->isEventSet(20)) {
        //Determine which color the tree has
        pose_t result;
        bool tempColor;
        bool colorDetermined = false;
        int colorCertainty = 0;

        while (colorCertainty < 10) {
          result = computerVision->treeID(RED);
          tempColor = result.id;

          if (tempColor == colorDetermined) {
            colorCertainty++;
          }
          else {
            colorCertainty = 0;
          }
          colorDetermined = tempColor;
        }
        // Now we are sure of the color
        currentTreeColor = colorDetermined;

        if (currentTreeColor == RED) {
          printf("Tree is red\n");
        }
        else printf("Tree is white\n");
        
        state = 40;
      }
    } break;


    case 40: {
      int line = 0;
      //Lower the servo to the correct height
      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-640, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=640, vservo=0");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=8, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      if(bridge->event->isEventSet(8))
      {
        state = 50;
      }
      
    } break;

    case 50: 
    {
      
      // Drive to the first tree and grab it
      pose_t trunk_pos;
      static pose_t trunk_pos_avg;

      int avg = 3;

      static int avg_counter = 0;
      
      trunk_pos = computerVision->trunkPos();
        if(trunk_pos.valid) 
        {
          cout << "x: " << trunk_pos.x << " y: " << trunk_pos.y << " z: " << trunk_pos.z << endl;

          if (avg_counter < avg) {
            trunk_pos_avg.x += trunk_pos.x;
            avg_counter++;
          }
          else {
            trunk_pos_avg.x = trunk_pos_avg.x/(double)avg;
            computerVision->determineMovement(trunk_pos_avg,straight,left,right);
            printf("Determining movement. x = %f\n",trunk_pos_avg.x);
            avg_counter = 0;
            trunk_pos_avg.x = 0;
            new_event_ready = true;
          }
          
          //Movements
          int line = 0;
          // Line to send in case every action is false.
          //snprintf(lines[line++], MAX_LEN, "vel=0.0: time=1.0");
          if(new_event_ready)
          {
            new_event_ready = false;
            if(straight){
              printf("Going straight.\n"); 
              snprintf(lines[line++], MAX_LEN, "vel=0.4: dist=0.15");
              snprintf(lines[line++], MAX_LEN, "vel=0.25: dist=0.30");

              straight = false;
              event_nr = 7;

            }
            else if(right){
              printf("Going right.\n"); 
              snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0.0: turn=2");
              right = false;
            }
            else if(left){
              printf("Going left.\n");
              snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0.0: turn=-2");
              left = false;
            }
            // occupy Robot
            snprintf(lines[line++], MAX_LEN, "event=%d, vel=0 : dist=1", event_nr);
            //printf(lines[line-1]);
            sendAndActivateSnippet(lines, line);
            
          }
          // send lines to REGBOT
          if(event_nr == 7)
          {
            cout << "grabbed the tree.." << endl;
            state = 60;

          }
          else if(event_nr == 8)
          {
            cout << "event: " << event_nr << endl;
            if(bridge->event->isEventSet(8))
            {
              cout << "event: " << event_nr << endl;
              line = 0;
              event_nr = 9;
              //new_event_ready = true;
            }

          }
          else if(event_nr == 9)
          {
            cout << "event: " << event_nr << endl;
            if(bridge->event->isEventSet(9))
            {
              cout << "event: " << event_nr << endl;
              line = 0;
              event_nr = 8;
              //new_event_ready = true;
            }
          }

          
        }
    } break;

    case 60: {
      int line = 0;
      //Navigate to the correct Aruco!
      if(bridge->event->isEventSet(7))
      {
        cout << "ready to go to aruco.." << endl;
        state = 999;
        if (currentTreeColor == WHITE) {
          cout << "Going to the white tree.." << endl;
          snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
          snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-600, vservo=0");
          snprintf(lines[line++], MAX_LEN, "servo=3, pservo=600, vservo=0");
          snprintf(lines[line++], MAX_LEN, "vel=0.25, tr=0.2: turn=45");
          snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");


          snprintf(lines[line++], MAX_LEN, "vel=0.25: dist=1.25");

          snprintf(lines[line++], MAX_LEN, "vel=-0.40: dist=1.2");
          // occupy Robot
          snprintf(lines[line++], MAX_LEN, "event=11, vel=0 : dist=1");
          sendAndActivateSnippet(lines, line);
          state = 70;
        }
        else if (currentTreeColor == RED){
          cout << "Going to the red tree.." << endl;
          snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-600, vservo=0");
          snprintf(lines[line++], MAX_LEN, "servo=3, pservo=600, vservo=0");
          snprintf(lines[line++], MAX_LEN, "vel=0.25, tr=0.2: turn=45");
          snprintf(lines[line++], MAX_LEN, "vel=0: time=0.5");
          
          //Go straight a bit
          snprintf(lines[line++], MAX_LEN, "vel=0.3 : dist=0.3");
          //Turn left
          snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0.2: turn=45");
          //Go straight a little bit
          snprintf(lines[line++], MAX_LEN, "vel=0.3 : dist=0.1");
          //Turn right
          snprintf(lines[line++], MAX_LEN, "vel=0: time=1");
          snprintf(lines[line++], MAX_LEN, "vel=0.25, tr=0.2: turn=-45");
          //Go straight
          snprintf(lines[line++], MAX_LEN, "vel=0.3 : dist=0.3");
          // occupy Robot
          snprintf(lines[line++], MAX_LEN, "event=12, vel=0 : dist=1");
          sendAndActivateSnippet(lines, line);
          state = 70;
        }

      }
    } break;

    case 70: {
      int line = 0;
      if (bridge->event->isEventSet(11)||bridge->event->isEventSet(12)) {
        
        if(currentTreeColor == WHITE) {
          printf("Going back to initial position - WHITE\n");
          //Go straight back
          snprintf(lines[line++], MAX_LEN, "vel=-0.40: dist=1.8");
          //Turn back & left a bit 
          snprintf(lines[line++], MAX_LEN, "vel=-0.3, tr=0.2: turn=45");
          //Go straight back
          snprintf(lines[line++], MAX_LEN, "vel=-0.40: dist=0.8");
          // occupy Robot
          snprintf(lines[line++], MAX_LEN, "event=13, vel=0 : dist=1");
          sendAndActivateSnippet(lines, line);

        }
        else if (currentTreeColor == RED) {
          printf("Going back to initial position - RED\n");
          //Go straight back
          snprintf(lines[line++], MAX_LEN, "vel=-0.40: dist=0.8");
          //Turn back & right a bit 
          snprintf(lines[line++], MAX_LEN, "vel=-0.3, tr=0.2: turn=-45");
          //Go straight back
          snprintf(lines[line++], MAX_LEN, "vel=-0.40: dist=0.8");
          //Turn back & right a bit 
          snprintf(lines[line++], MAX_LEN, "vel=-0.3, tr=0.2: turn=-45");
          //Go straight back
          snprintf(lines[line++], MAX_LEN, "vel=-0.40: dist=0.8");
          //Turn to face the new tree
          snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0.0: turn=45");
          // occupy Robot
          snprintf(lines[line++], MAX_LEN, "event=14, vel=0 : dist=1");
          sendAndActivateSnippet(lines, line);
        }
        state = 999;
      }
    } break;

    case 999:
    default:
      printf("Mission treeID ended \n");
      computerVision->shutdown();
      finished = true;
      break;
  }
  return finished;
}

bool UMission::mission_circleOfHell(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission circle of Hell\n");
      //play.say("Starting mission circle of hell", 100);

      state = 10;
    } break;

    case 10: {
      int line = 0;

      snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn= 90");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time = 0.5");

      snprintf(lines[line++], MAX_LEN, "vel=0 : ir2 < 0.5");  //0.5
      snprintf(lines[line++], MAX_LEN, "vel=0 : ir2 > 0.5");  //0.5
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=1");

      snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn= 180");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");
      snprintf(lines[line++], MAX_LEN, "vel=-0.5 : dist = 1.17");  //0.45
      snprintf(lines[line++], MAX_LEN, "vel=0 : time = 0.5");

      // snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=40");//32 // old

      // snprintf(lines[line++], MAX_LEN, "vel=0.7 : dist=1.15");
      // snprintf(lines[line++], MAX_LEN, "vel=0.25 : time = 7"); //old

      // snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-600, vservo=0"); //old
      // snprintf(lines[line++], MAX_LEN, "servo=3, pservo=600, vservo=0"); //old
      // snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5"); //old
      // snprintf(lines[line++], MAX_LEN, "vel=0.7 : time=1.3"); //old

      // snprintf(lines[line++], MAX_LEN, "servo=2, pservo=400, vservo=0");
      // snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-400, vservo=0");
      // snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.5");

      // snprintf(lines[line++], MAX_LEN, "servo=2, pservo=2000, vservo=0");
      // snprintf(lines[line++], MAX_LEN, "servo=3, pservo=2000, vservo=0");

      // snprintf(lines[line++], MAX_LEN, "vel=0.3 : dist=0.10"); //old
      

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=10, vel=0 : dist=1"); //Old

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 11;
      featureCnt = 0;
    } break;
    
    case 11: {
      if (bridge->event->isEventSet(10)) {
        state = 12;
      }
    } break;

    case 12: {
      int line = 0;

      snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn=110");
      snprintf(lines[line++], MAX_LEN, "vel=0.0 : time = 0.5");
      //snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.5, ir1 < 0.25");
      // snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.4 : turn=360, ir1 < 0.3");
      // snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.37 : turn=360, time = 0.6");
      // //snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.2");
      // //snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.5 : turn=10");
      // snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.37 : turn=360, ir1 < 0.3");

      // snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.08");

      // //snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.5 : turn=10");
      // snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.34 : turn=360, ir1 < 0.22");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.33 : turn=320");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.3");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=10, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 13;
      featureCnt = 0;
    } break;
    
    case 13: {
      if (bridge->event->isEventSet(10)) {
        state = 14;
      }
    } break;

    case 14: {
      int line = 0;


      snprintf(lines[line++], MAX_LEN, "vel=0 : ir2 < 0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0 : ir2 > 0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=2");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : lv=1");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-150");


      //  ---- Move to appleTree -----

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edger=0, white=1 : dist=3.6");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=1.3");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=90");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : xl>15");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-90");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edger=0, white=1 : dist=0.2");

      // --------- Move to goal ------

      // snprintf(lines[line++], MAX_LEN, "vel=0.4, edger=0, white=1 : dist=2.1");

      // snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=30");
      // snprintf(lines[line++], MAX_LEN, "vel=0.5, edgel=0, white=1 : time=10");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=10, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 15;
      featureCnt = 0;
    } break;
    
    case 15: {
      if (bridge->event->isEventSet(10)) {
        state = 999;
      }
    } break;

    case 999:
    default:
      printf(">> Mission circleOfHell ended\n");
      finished = true;
      break;
  }
  return finished;
}

bool UMission::mission_skipping_circleOfHell(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission skipping circle of Hell\n");
      //play.say("Starting mission circle of hell", 100);

      state = 10;
    } break;

    case 10: {
      int line = 0;

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : dist=0.2");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : xl>15");
      
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=90");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edger=0, white=1 : dist=0.1");

      snprintf(lines[line++], MAX_LEN, "vel=0 : ir2 < 0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0 : ir2 > 0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=1.5");

      //------- Going to appleTree -----

      snprintf(lines[line++], MAX_LEN, "vel=0.45, edger=0, white=1 : dist=2.4");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-80");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edger=0, white=1 : xl>15");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-40");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edger=0, white=1 : dist=0.4");

      //------- Going to goal -----

      // snprintf(lines[line++], MAX_LEN, "vel=0.4, edger=0, white=1 : dist=2.4");
      // snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=90");

      // snprintf(lines[line++], MAX_LEN, "vel=0.4 : xl > 15");
      // snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.1");

      // snprintf(lines[line++], MAX_LEN, "vel=0.4 : xl > 15");
      // snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-40");
      // snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : dist=0.7");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=10, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 11;
      featureCnt = 0;
    } break;
    
    case 11: {
      if (bridge->event->isEventSet(10)) {
        state = 999;
      }
    } break;

    case 999:
    default:
      printf(">> Mission skipping circleOfHell ended\n");
      finished = true;
      break;
  }
  return finished;
}

bool UMission::mission_go_to_goal(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission dummy\n");
      //play.say("Starting mission dummy", 100);

      state = 10;
    } break;

    case 10: {
      int line = 0;

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : dist=0.3");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-10");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=1.7");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : xl>15");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-30");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : time=10");

      //Occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=11, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      
      state = 11;
      featureCnt = 0;
      break;
    }
    
    case 11: {
      if (bridge->event->isEventSet(11)) {
        state = 999;
      }
    } break;

    case 999:
    default:
      printf("Mission dummy ended \n");
      finished = true;
      break;
  }
  return finished;
}

////////////////////////////////////////////////////////////

void UMission::openLog() {
  // make logfile
  const int MDL = 32;
  const int MNL = 128;
  char date[MDL];
  char name[MNL];
  UTime appTime;
  appTime.now();
  appTime.getForFilename(date);
  // construct filename ArUco
  snprintf(name, MNL, "log_mission_%s.txt", date);
  logMission = fopen(name, "w");
  if (logMission != NULL) {
    const int MSL = 50;
    char s[MSL];
    fprintf(logMission, "%% Mission log started at %s\n", appTime.getDateTimeAsString(s));
    fprintf(logMission, "%% Start mission %d end mission %d\n", fromMission, toMission);
    fprintf(logMission, "%% 1  Time [sec]\n");
    fprintf(logMission, "%% 2  mission number.\n");
    fprintf(logMission, "%% 3  mission state.\n");
  }
  else
    printf("#UCamera:: Failed to open image logfile\n");
}

void UMission::closeLog() {
  if (logMission != NULL) {
    fclose(logMission);
    logMission = NULL;
  }
}

