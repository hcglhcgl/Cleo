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
  // save active thread number
  threadActive = threadToMod;
}

void UMission::parkArm() {
  int line = 0;
  int parkLoc = 150;

  if (!alreadyParked) {
    //Can NOT use 'time' here
    snprintf(lines[line++], MAX_LEN, "servo=2, pservo=%d", parkLoc);
    snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-%d", parkLoc);
    
    //Waiting for 1 sec before continuing
    snprintf(lines[line++], MAX_LEN, "vel=0 : time=1");
    snprintf(lines[line++], MAX_LEN, "event=9, vel=0 : time=1");
    sendAndActivateSnippet(lines, line);

    while(!(bridge->event->isEventSet(9))) {
        alreadyParked = false;
    }

    alreadyParked = true;
  }
}

void UMission::setArm(int armPose) {
  int line = 0;

  //Can NOT use 'time' here 
  snprintf(lines[line++], MAX_LEN, "servo=2, pservo=%d, vservo=0", armPose);
  snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-%d, vservo=0", armPose);

  sendAndActivateSnippet(lines, line);

  alreadyParked = false;
}

//////////////////////////////////////////////////////////

/**
 * Thread for running the mission(s)
 * All missions segments are called in turn based on mission number
 * Mission number can be set at parameter when starting mission command line.
 * 
 * The loop also handles manual override for the gamepad, and resumes
 * when manual control is released.
 * */
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
          case 10:
            ended = mission_guillotine(missionState);
            break;
          case 20:
            ended = mission_seesaw(missionState);
            break;
          case 1:
            ended = mission_balls_1(missionState);
            break;
           case 2:
            ended = mission_balls_2(missionState);
            break;
          case 40:
            ended = mission_parking(missionState);
            break;
          case 50:
            ended = mission_racetrack(missionState);
            break;
          case 60:
            ended = mission_circleOfHell(missionState);
            break;
          case 70:
            ended = mission_dummy(missionState);
            break;
          default:
            // no more missions - end everything
            finished = true;
            break;
        }
        if (ended) { // start next mission part in state 0
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
    } break;

    case 10: {
      int line = 0;

      parkArm();

      //snprintf(lines[line++], MAX_LEN, "vel=0.5, acc=0.5, edgel=0, white=1 : dist=0.1");
      snprintf(lines[line++], MAX_LEN, "vel=0.6, edgel=0, white=1 : xl>15");
      snprintf(lines[line++], MAX_LEN, "vel=0.6, edger=0, white=1 : dist=0.09");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=1, vel=0 : dist=1");

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

bool UMission::mission_seesaw(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission_seesaw\n");

      state = 10;
    } break;

    case 10: {
      int line = 0;

      parkArm();

      //snprintf(lines[line++], MAX_LEN, "vel=0.6, acc=0.6, edger=0, white=1 : dist=0.1");

      snprintf(lines[line++], MAX_LEN, "vel=0.5, tr=0 : turn=90");

      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-700, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=700, vservo=0");

      snprintf(lines[line++], MAX_LEN, "vel=0.3, : dist=0.2");
      snprintf(lines[line++], MAX_LEN, "vel=0.3, edger=0, white=1 : dist=0.99");

      snprintf(lines[line++], MAX_LEN, "vel=0, time=0.1");
      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-900, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=900, vservo=0");

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edger=0, white=1 : dist>0.1");


      snprintf(lines[line++], MAX_LEN, "vel=0.4, edger=0, white=1 : ir1<0.4");


      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.6");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-90");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : xl > 15 ");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.2");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : xl > 15 ");

      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.1");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-125");


      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=2, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);

      state = 11;
      featureCnt = 0;
    } break;

    case 11: {
      if (bridge->event->isEventSet(2)) {
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

bool UMission::mission_balls_1(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission_balls\n");

      state = 10;
    } break;

    case 10: {
      int line = 0;

      snprintf(lines[line++], MAX_LEN, "vel=0, time=0.1");
      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-900, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=900, vservo=0");

      snprintf(lines[line++], MAX_LEN, "vel=0.6, edgel=0, white=1 : dist=2");
      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-990, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=990, vservo=0");

      snprintf(lines[line++], MAX_LEN, "vel=0.5, edgel = 0, white = 1, log = 1: tilt > 0");
      snprintf(lines[line++], MAX_LEN, "vel=0.6, edgel=0, white=1 : dist=0.1"); //0.47
      snprintf(lines[line++], MAX_LEN, "vel=0.6 : dist=0.37");
      snprintf(lines[line++], MAX_LEN, "vel=0.25, tr=0 : turn=25");

      snprintf(lines[line++], MAX_LEN, "vel=0, time=0.1");
      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-150, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=150, vservo=0");
      snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn=-95");
      //snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : tilt < 0");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : xl > 15");
      snprintf(lines[line++], MAX_LEN, "vel=-0.4, edgel=0, white=1 : tilt > 0");


      snprintf(lines[line++], MAX_LEN, "vel=0, time=2");
      



      //snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn=90");
      //snprintf(lines[line++], MAX_LEN, "vel=0, time=0.1");
      //snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn=-40");
      //snprintf(lines[line++], MAX_LEN, "vel=0, time=0.1");
      //snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn=40");
      //snprintf(lines[line++], MAX_LEN, "vel=0, time=0.1");
      //snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn=-180");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=2, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
    

      state = 11;
      featureCnt = 0;
    } break;

    case 11: {
      if (bridge->event->isEventSet(2)) {
        state = 999;
      }
    } break;

    case 999:
    default:
      printf(">> Mission_balls ended\n");

      finished = true;
      break;
  }
  return finished;
}

bool UMission::mission_balls_2(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission_balls\n");

      state = 10;
    } break;

    case 10: {
      int line = 0;

     
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.2");
      snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn=-85"); // -90
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.10"); //10 
      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-900, vservo=0");
      snprintf(lines[line++], MAX_LEN, "servo=3, pservo=900, vservo=0");
      snprintf(lines[line++], MAX_LEN, "vel=0: time=0.2"); // new
      snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn=-140");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, edger=0, white=1 : dist=0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.32"); 
      snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn=-50"); //make more realiable solution? works 70-80% of the time




      //snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn=90");
      //snprintf(lines[line++], MAX_LEN, "vel=0, time=0.1");
      //snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn=-40");
      //snprintf(lines[line++], MAX_LEN, "vel=0, time=0.1");
      //snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn=40");
      //snprintf(lines[line++], MAX_LEN, "vel=0, time=0.1");
      //snprintf(lines[line++], MAX_LEN, "vel=0.3, tr=0 : turn=-180");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=2, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);

      state = 11;
      featureCnt = 0;
    } break;

    case 11: {
      if (bridge->event->isEventSet(2)) {
        state = 999;
      }
    } break;

    case 999:
    default:
      printf(">> Mission_balls ended\n");

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
      state = 10;
    } break;

    case 10: {
      int line = 0;

      parkArm();

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : lv=0");
      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-800 : time=1");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.4");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-90");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.55");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=90");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=1");
      snprintf(lines[line++], MAX_LEN, "vel=0.6, tr=0 : turn=-180");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.1");
      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=0 : time=1");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.4 : turn=180");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=0.1");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=3, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 11;
      featureCnt = 0;
    } break;
    
    case 11: {
      if (bridge->event->isEventSet(3)) {
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
      state = 10;
    } break;

    case 10: {
      int line = 0;

      parkArm();

      snprintf(lines[line++], MAX_LEN, "vel=0.4, edgel=0, white=1 : ir1 < 0.2");
      snprintf(lines[line++], MAX_LEN, "vel=0 : ir2 < 0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0 : ir2 > 0.5");
      snprintf(lines[line++], MAX_LEN, "vel=2, acc=1.5, edger=0, white=1 : dist = 2");
      snprintf(lines[line++], MAX_LEN, "vel=2, acc=1.5, edger=0, white=1 : ir1 < 0.2");

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=4, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 11;
      featureCnt = 0;
    } break;
    
    case 11: {
      if (bridge->event->isEventSet(4)) {
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

bool UMission::mission_circleOfHell(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission circle of Hell\n");
      state = 10;
    } break;

    case 10: {
      int line = 0;

      parkArm();

      snprintf(lines[line++], MAX_LEN, "vel=-0.4 : dist=0.2");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=180");
      snprintf(lines[line++], MAX_LEN, "vel=0.4,edgel=0,white=1 : xl > 15");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.1 : turn= 90 ");
      snprintf(lines[line++], MAX_LEN, "vel=0.4,edgel=0,white=1 : dist=0.85");
      snprintf(lines[line++], MAX_LEN, "vel=0 : ir2 < 0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0 : ir2 > 0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0 : time=1");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=45");
      snprintf(lines[line++], MAX_LEN, "vel=0.6 : dist=1.1");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-70");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.5, ir1 < 0.25");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.2");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.5 : turn=10");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.4 : turn=360, ir1 < 0.2");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.2");
      /*snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.5 : turn=10");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0.4 : turn=360, ir1 < 0.2");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : dist=0.3");
      snprintf(lines[line++], MAX_LEN, "vel=0 : ir1 < 0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0 : ir1 > 0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0.4 : lv=1");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=-150 ");
      snprintf(lines[line++], MAX_LEN, "vel=0.6, edgel=0, white=1 : dist=3");
      snprintf(lines[line++], MAX_LEN, "vel=0.6, edgel=0, white=1 : ir1 < 0.5");
      snprintf(lines[line++], MAX_LEN, "vel=0.6, edgel=0, white=1 : dist=0.25");
      snprintf(lines[line++], MAX_LEN, "vel=0.4, tr=0 : turn=110");
      snprintf(lines[line++], MAX_LEN, "vel=0.6, edgel=0, white=1 : ir2 < 0.1");*/

      // occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=5, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      state = 11;
      featureCnt = 0;
    } break;
    
    case 11: {
      if (bridge->event->isEventSet(5)) {
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

bool UMission::mission_dummy(int & state) {
  bool finished = false;

  switch (state) {
    case 0: {
      printf(">> Starting mission dummy\n");
      state = 10;
    } break;

    case 10: {
      int line = 0;

      parkArm();

      //Occupy Robot
      snprintf(lines[line++], MAX_LEN, "event=10, vel=0 : dist=1");

      // send lines to REGBOT
      sendAndActivateSnippet(lines, line);
      
      state = 11;
      featureCnt = 0;
      break;
    }
    
    case 11: {
      if (bridge->event->isEventSet(10)) {
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
