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

#ifndef UMISSION_H
#define UMISSION_H


#include <sys/time.h>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include "urun.h"
#include "ucamera.h"
#include "ubridge.h"
#include "ujoy.h"
#include "uplay.h"


/**
 * Base class, that makes it easier to starta thread
 * from the method runObj
 * The run method should be overwritten in the real class */
class UMission : public URun
{
public:
  /// flag to finish all (pending) missions and RC
  bool finished = false;
private:
  /**
   * Pointer to communication and data part of this mission application */
  UBridge * bridge;
  /**
   * Pointer to camera (class) of this mission application */
  //UCamera * cam;
  /** is thread active */
  bool active = false;
  // thread on Regbot
  int threadActive = false;
  /// space for fabricated lines
  const static int MAX_LINES = 100;
  const static int MAX_LEN = 100;
  const static int missionLineMax = 30; //20
  /** definition of array with c-strings for the mission snippets */
  char lineBuffer[missionLineMax][MAX_LEN];
  /** an array of pointers to mission lines */
  char * lines[missionLineMax];
  /** logfile for mission state */
  FILE * logMission = NULL;
  
public:
  /**
   * Constructor */
  UMission(UBridge * regbot);
  /**
   * Destructor */
  ~UMission();
  /**
   * Initialize regbot part to accept mission commands */
  void missionInit();
  void openLog();
  void closeLog();
  inline bool logIsOpen() { return logMission != NULL; };

  void parkArm();
  void setArm(int armPose);
  /**
   * Run the missions
   * \param fromMission is first mission element (default is 1)
   * \param toMission is last mission to run (default is 998)
   * so 'runMission(3,3)' runs just mission part 3 */
  void runMission();
  /**
   * Thread run function */
  void run();
  /**
   * Activate mission */
  void start()
  {
    active = true;
    // printf("UMission::start active=true\n");
  }
  /**
   * Stop all missions */
  void stop()
  {
    finished = true;
    th1stop = true;
    usleep(11000);
    if (th1 != NULL)
      th1->join();
  }
  /**
   * Print status for mission */
  void printStatus();
  
  /** which missions to run 
   * These values can be set as parameters, when starting the mission */
  int fromMission;
  int toMission;
  int mission;
  int missionState;
private:
  /**
   * Mission parts
   * \param state is the current state of the mission
   * \return true, when missionpart is finished */
  bool mission_guillotine(int & state);
  bool mission_balls_1(int & state);
  bool mission_seesaw(int & state);
  bool mission_balls_2(int & state);
  bool mission_stairs(int & state);
  bool mission_parking(int & state);
  bool mission_racetrack(int & state);
  bool mission_circleOfHell(int & state);
  bool mission_dummy(int & state);
  bool camera_mission(int & state);
  
  
  
private:
  /**
   * Send a number of lines to the REGBOT in a dormant thread, and 
   * make these lines (mission snippet) active - stopping the last set of lines.
   * \param missionLines is a pointer to an array of c-strings
   * \param missionLineCnt is the number of strings to be send from the missionLine array. */
  void sendAndActivateSnippet(char * missionLines[], int missionLineCnt);
  /**
   * Object to play a soundfile as we go */
  USay play;
  /**
   * turn count, when looking for feature */
  int featureCnt;
};


#endif
