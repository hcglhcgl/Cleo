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

#ifndef UREGBOT_H
#define UREGBOT_H

// #include <iostream>
#include <sys/time.h>
#include <cstdlib>
// #include <sstream>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include "urun.h"
#include "tcpCase.h"
#include "utime.h"

using namespace std;
// forward declaration
class UBridge;

/////////////////////////////////////////////////////////////

/**
 * base class for data items from robot and bridge */
class UData
{ // base class for data
public:
  timeval dataTime;
  UBridge * bridge = NULL;
  FILE * logfile = NULL;
  /** destructor  */
  ~UData()
  { // make sure file is closed
    closeLog();
  }
  // open logfile
  virtual void openLog() {};
  // create logfile
  void openLog(const char * prename)
  {
    const int MDL = 32;
    const int MNL = 128;
    char date[MDL];
    char name[MNL];
    UTime time;
    time.now();
    time.getForFilename(date);
    snprintf(name, MNL, "%s_%s.txt", prename, date);
    logfile = fopen(name, "w");
  }
  // close logfile
  void closeLog()
  {
    if (logfile != NULL)
      fclose(logfile);
    logfile = NULL;
  }
  // is log open
  inline bool logIsOpen() { return logfile != NULL; }
  // set update time
  inline void updated()
  {
    gettimeofday(&dataTime, NULL);
  }
  // get time since update
  float getTimeSinceUpdate()
  {
    timeval t;
    gettimeofday(&t, NULL);
    return getTimeDiff(t, dataTime);
  }
  // just a default reply
  // implement in real class when needed
//   virtual bool tick()
//   {
//     return false;
//   }
  // subscribe to data
  virtual void subscribe() {};
};

/**
 * base class for data items from IR distance sensor */
class UIRdist : public UData
{
public:
  float dist[2] = {0,0};
  int raw[2] = {0,0};
  // constructor
  UIRdist(UBridge * bridge_ptr, bool openLog);
  //
  void decode(char * msg);
  //
  virtual void subscribe();
  // open logfile for all updates
  void openLog();
  /**
   * Print status for bridge and all data elements */
  void printStatus();
};

/**
 * base class for data items from Accelerometer and Gyro */
class UAccGyro : public UData
{
public:
  float acc[3] = {0};
  float gyro[3] = {0};
  // constructor
  UAccGyro(UBridge * bridge_ptr, bool openLog);
  //
  void decode(char * msg);
  //
  virtual void subscribe();
  // open logfile for all updates
  void openLog();
  /**
   * Print status for bridge and all data elements */
  void printStatus();
  /**
   * get turnrate */
  float turnrate();
};

/**
 * robot pose info */
class UPoseInfo : public UData
{
public:
  float x = 0.0;
  float y = 0.0;
  float h = 0.0;
  float tilt = 0.0;
  float dist = 0.0;
  // constructor
  UPoseInfo(UBridge * bridge_ptr, bool openLog);
  //
  void decode(char * msg);
  //
  virtual void subscribe();
  // open logfile
  void openLog();
  /**
   * Print status for bridge and all data elements */
  void printStatus();
};

/////////////////////////////////////////////////////////////

/**
 * Line sensor data */
class UEdge  : public UData
{
public:
  // edge detect
  bool edgeValidLeft = false;
  bool edgeValidRight = false;
  bool edgeCrossingBlack = false;
  bool edgeCrossingWhite = false;
  //
  UEdge(UBridge * bridge_ptr, bool openLog);
  void decode(char * msg);
  void subscribe() override;
  
  /**
   * Print status for bridge and all data elements */
  void printStatus();
};

/////////////////////////////////////////////////////////////

/**
 * Other info like mission line and battery voltage */
class UInfo  : public UData
{
public:
  float regbotTime = 0.0;
  timeval regbotTimeAtLinuxTime;
  timeval bootTime;
  float batteryVoltage = 0.0;
  float controlTime = 0.0;
  bool missionRunning = false;
  int missionLineNum = 0;
  int missionThread = 0;
  // basic robot info
  int robotId; 
  float odoWheelBase; // meter
  float gear;         // ratio
  int   pulsPerRev;   // on motor encoder
  float odoWheelRadius[2]; // in meter
  float balanceOffset; // offset to balance
  bool  batteryUse;    // running on battery
  float batteryIdleVoltage; // Volts, vhen robot turns off
  int   robotHWversion;  // hardware configuration 1..6
  static const int MAX_NAME_LENGTH = 32;
  char robotname[MAX_NAME_LENGTH];
  float bridgeLoad = 0.0;
  int   msgCnt1sec = 0;
  
  // methods
  // constructor
  UInfo(UBridge * bridge_ptr, bool openLog);
  // open the logfile
  void openLog();
  // public methods
  void decodeId(char * msg);
  void decodeHbt(char * msg);
  void decodeMission(char * msg);
  float getTime();
  
  void subscribe() override;
  bool isHeartbeatOK();  
  /**
   * Print status for bridge and all data elements */
  void printStatus();
  /**
   * save status to log - if log is open */
  void saveDataToLog();  
};

/////////////////////////////////////////////////////////////

class UEvent  : public UData
{
public:
  static const int MAX_EVENT_FLAGS = 34;
  bool eventFlags[MAX_EVENT_FLAGS];
  mutex eventUpdate;
  bool firstEvent = true;
  
  UEvent(UBridge * bridge_ptr, bool openLog);
  //
  void decode(char * msg);
  /** set event flag */
  void setEvent(int eventNumber);
  /**
   * print set event flags to console */
  void printEvents();
  // open logfile
  void openLog();
  /**
   * Clear all event flags to false */
  void clearEvents();
  /**
   * Requests if this event has occured.
   * \param event the event flag to test
   * \returns true and resets event, if it was set */
  bool isEventSet ( int event );
  
  void subscribe() override;
  /**
   * Print status for bridge and all data elements */
  void printStatus();
};

/////////////////////////////////////////////////////////////

class UJoy  : public UData
{
public:
  int axes[8] = {0,0,0,0,0,0,0};
  // axis 0 is left hand left-right axis
  // axis 1 is left hand up-down axis (servo)
  // axis 2 is left front speeder
  // axis 3 is right hand left-right (turn)
  // axis 4 is right hand up-down (velocity)
  // axis 5 is right front speeder
  // axis 6 id digital left-write
  // axis 7 is digital up-down 
  bool button[11] = {0,0,0,0,0,0,0,0,0,0,0};
  // first 4 is [0]=green, [1]=red, [2]=blue, [3]=yellow
  // [4] = LB, [5] = RB, [6]=back, [7]=start, 
  // [8]=logitech logo, [9]=left knob, [10]=right knob
  bool manual = false;
  //
  UJoy(UBridge * bridge_ptr, bool openLog);
  /** open logfile and write initial comment */
  void openLog() override;
  /// decode gamepad message from bridge
  void decode(char * msg);
  /// subscribe to joystick data
  void subscribe() override;
  /**
   * Print status for bridge and all data elements */
  void printStatus();
};

/////////////////////////////////////////////////////////////

class UMotor  : public UData
{
public:
  float velocity[2];
  float current[2];
  
  UMotor(UBridge * bridge_ptr, bool openLog);
  //
  void decodeVel(char * msg);
  void decodeCurrent(char * msg);
  void openLog();
  //
  void subscribe() override;
  /**
   * Print status for bridge and all data elements */
  void printStatus();
  /**
   * Get velocity */
  float getVelocity()
  {
    return (velocity[0] + velocity[1])/2.0;
  }
};

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/**
 * The robot class handles the 
 * port to the REGBOT part of the robot through the bridge,
 * REGBOT handles the most real time issues and the bridge a few
 * and this class is the interface to that. */
class UBridge : public URun, public tcpCase
{ // REGBOT interface
public:
  // data items
  // the second parameter is 
  UPoseInfo * pose = new UPoseInfo(this, false);
  UEdge * edge = new UEdge(this, false);
  UInfo * info = new UInfo(this, false);
  UEvent * event = new UEvent(this, false);
  UJoy * joy = new UJoy(this, false);
  UMotor * motor = new UMotor(this, false);
  UIRdist * irdist = new UIRdist(this, false);
  UAccGyro * imu = new UAccGyro(this, false);
  // debug log
  FILE * botlog;
  
private:
  // mutex to ensure commands to regbot are not mixed
  mutex sendMtx;
  mutex logMtx;
  // receive buffer
  static const int MAX_RX_CNT = 500;
  char rx[MAX_RX_CNT];
  // number of characters in rx buffer
  int rxCnt;
  // status message sequence (fast (sensors))
  int tickClassIdx;
  // status message sequence (slow ("static" info))
  int statusMsgIdx2;
  
public:
  /** constructor
   * \param server is a string with either IP address or hostname.
   * connects to port 24001
   */
  UBridge(const char * server, bool openLog);
  /** destructor */
  ~UBridge();
  /** open logfile for communication interface */
  void openLog();
  void closeLog();
  inline bool logIsOpen() { return botlog != NULL; }
  /**
   * Decode log open and log close command (from keyboard)
   * \param s is the command string, e.g. 'lo motor' to open logfile for motor values
   * */
  int decodeLogOpenClose(const char* s);
  /**
   * Decode open or close log based for this item
   * \param c is either 'o' or 'c' for open or close.
   * \param item is item op open or close logfile for.
   * \returns 1
   * */
  int decodeLogOpenOrClose(const char c, UData * item);
  /**
   * send a string to the serial port */
  void send(const char * cmd);
  /**
   * receive thread */
  void run();
  /**
   * clear events */
  void clearEvents();
  /**
   * test and reset event */
  bool eventSet(int event);
  /**
   * Set an event flag
   * \param eventNumber is event to set (in range 0..33) */
  void setEvent(int eventNumber);
  /**
   * stop interface */
  void stop();
  /**
   * Print status for bridge and all data elements */
  void printStatus();
  
private:
  /**
   * Open the connection.
   * \returns true if successful */
//   bool openToRegbot();
  /**
   * decode messages from REGBOT */
  void decode(char * msg);
  /** decode event message */
//   void decodeEvent(char * msg);
  /** decode heartbeat message */
  void decodeHbt(char * msg);
  /** decode heartbeat message */
  void decodePose(char * msg);
  /** decode line sensor edge message */
  void decodeEdge(char * msg);
  /** decode mission status - running or not */
  void decodeMissionStatus(char * msg);
  /** send regulat status requests, if no other traffic */
  void requestDataTick();
  /**
   * time (for debug print) */
  UTime t;
};

#endif
