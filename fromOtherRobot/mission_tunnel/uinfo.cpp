/***************************************************************************
 *   Copyright (C) 2019 by DTU (Christian Andersen)                        *
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

#include <string.h>
#include "ubridge.h"

UInfo::UInfo(UBridge * bridge_ptr, bool openlog)
{
  bridge = bridge_ptr;
  gettimeofday(&bootTime, NULL);
  // default
  strncpy(robotname, "no name", MAX_NAME_LENGTH);
  if (openlog)
    openLog();
}

void UInfo::openLog()
{ // open pose log
  UData::openLog("log_hbt_mission");
  if (logIsOpen())
  {
    fprintf(logfile, "%% robobot mission heartbeat logfile\n");
    fprintf(logfile, "%% 1 Timestamp in seconds\n");
    fprintf(logfile, "%% 2 REGBOT time\n");
    fprintf(logfile, "%% 3 bridge load [%%]\n");
    fprintf(logfile, "%% 4 battery voltage\n");
    fprintf(logfile, "%% 5 mission running\n");
    fprintf(logfile, "%% 6 Mission line number\n");
    fprintf(logfile, "%% 7 Thread\n");
    fprintf(logfile, "%% 8 REGBOT load [%%]\n");
  }
}

//   snprintf(reply, MRL,  "hbt %g %g %d %d %d %lu\r\n", missionTime, batVoltInt * batVoltIntToFloat, 
//                     control.controlActive, control.missionState, remoteControl, controlUsedTime[2] / (F_CPU/1000000));

void UInfo::decodeHbt(char * msg)
{ // "hbt 2399.4 12.1"
  char * p1 = &msg[3];
  regbotTime = strtof(p1, &p1);
  batteryVoltage = strtof(p1, &p1);
  strtol(p1, &p1, 10); // control active (always)
  strtol(p1, &p1, 10); // mission state = 2 - always
  strtol(p1, &p1, 10); // remote control (manuel control) - fetched from bridge already
  controlTime = strtol(p1, &p1, 10); // used microseconds in each 1ms cycle
  timeval t;
  gettimeofday(&t, NULL);
  float dt = getTimeDiff(t, regbotTimeAtLinuxTime);
  if (dt < 36000 and dt > 2)
    printf("# heartbeat time too slow (%.3fsec) - lost REGBOT?\n", dt);
  regbotTimeAtLinuxTime = t;
  updated();
  saveDataToLog();
}

void UInfo::saveDataToLog()
{
  if (logfile != NULL)
  {
    timeval t;
    gettimeofday(&t, NULL);
    fprintf(logfile, "%ld.%03ld %.3f %.1f %.2f %d %d %d %.1f %d\n", t.tv_sec, t.tv_usec / 1000, 
            regbotTime, bridgeLoad,
            batteryVoltage,
            missionRunning, missionLineNum, missionThread, float(controlTime)*100.0/1000.0, msgCnt1sec
    );
  }
}

bool UInfo::isHeartbeatOK()
{
  timeval t;
  gettimeofday(&t, NULL);
  float dt = getTimeDiff(t, regbotTimeAtLinuxTime);
  bool isOK = dt < 2.0;
  //  printf("# heartbeat time not OK (%.3fsec)\n", dt);
  return isOK;
}


void UInfo::decodeId(char * msg)
{ // rid 84 0.206 9.68 48 0.04 0.04 0 1 9.89802 6 Cleo
  // snprintf(reply, MRL, "rid %d %g %g %d %g %g %g %d %g %d %s\r\n",
  //   robotId, 
  //   odoWheelBase, gear, 
  //   pulsPerRev,
  //   odoWheelRadius[0], odoWheelRadius[1],
  //   balanceOffset,
  //   batteryUse, batteryIdleVoltageInt * batVoltIntToFloat,
  //   robotHWversion,
  //   robotname[robotId]
  //   ); 
  char * p1 = &msg[3];
  robotId = strtol(p1, &p1, 10);
  odoWheelBase = strtof(p1, &p1);
  gear = strtof(p1, &p1);
  pulsPerRev = strtol(p1, &p1, 10);
  odoWheelRadius[0] = strtof(p1, &p1);
  odoWheelRadius[1] = strtof(p1, &p1);
  balanceOffset = strtof(p1, &p1);
  batteryUse = strtol(p1, &p1, 10);
  batteryIdleVoltage = strtof(p1, &p1);
  robotHWversion = strtol(p1, &p1, 10);
  strncpy(robotname, p1, MAX_NAME_LENGTH-1);
  updated();
}

void UInfo::decodeMission(char * msg)
{
  /* snprintf(s, MSL, "mis %d %d %d '%s' %d %d\r\n",
   *         mission, missionState , missionLineNum, 
   *         missionName[mission],
   *         0, // unused (was sendStatusWhileRunning)
   *         misThread
   * */
  //     printf("Uinfo::decodeMission - got %s\n", msg); 
  char * p1 = &msg[3];
  /*int midx =*/ strtol(p1, &p1, 0);
  int misState = strtol(p1, &p1, 0);
  missionRunning = misState == 2;
  missionLineNum = strtol(p1, &p1, 0);
  while (*p1 > ' ' and *p1 != '\'')
    p1++;
  p1++; // skip the ' char
  // look for next ' char
  p1 = strchr(++p1, '\'');
  if (p1 != NULL)
  { // get thread number
    p1++;
    int n = strtol(p1, &p1, 10);
    if (n != 0)
      printf("UInfo::decodeMission error in format (5) (%s) unused parameter error\n", msg);
    //
    missionThread = strtol(p1, &p1, 10);
    // when mission is started, mission state is 1 for 1.0 seconds, then 2 when mission is running
    // printf("# Mission message: (thread=%d) '%s'\n", missionThread, msg);
  }
  else
    printf("UInfo::decodeMission error in format (4) (%s)\n", msg);
  // timestamp data
  updated();
  saveDataToLog();
}

float UInfo::getTime()
{
  timeval t;
  gettimeofday(&t, NULL);
  return getTimeDiff(t, bootTime);
}

void UInfo::subscribe()
{
  bridge->send("mis subscribe 2\n"); // mission data
  bridge->send("hbt subscribe 1\n"); // time and mission info  
  bridge->send("rid subscribe 3\n"); // get robot ID values
  bridge->send("robot u4\n"); // request ID values from robot (once should be enough)
}


void UInfo::printStatus()
{
  printf("# ------- Info ----------\n");
  printf("# %s (%d), HW=%d\n", robotname, robotId, robotHWversion);
  printf("# REGBOT time %.3fs, batteryVoltage %.1f\n", regbotTime, batteryVoltage);
  printf("# Wheel base %gm, gear=%g, encoder ppr=%d, wheel radius (l=%gm, r=%gm)\n",
         odoWheelBase, gear, pulsPerRev, odoWheelRadius[0], odoWheelRadius[1]);
  printf("# Mission from robot: running=%d, line number=%d, thread running=%d\n",
         missionRunning, missionLineNum, missionThread);
  printf("# data age %.3fs\n", getTimeSinceUpdate());
  printf("# logfile active=%d\n", logfile != NULL);
}
