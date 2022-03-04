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

// #include <iostream>
#include <sys/time.h>
#include <cstdlib>
// #include <fstream>
// #include <sstream>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <termios.h>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

#include "ubridge.h"

using namespace std;

//////////////////////////////////////////////////////////////////

/** constructor */
UBridge::UBridge(const char * server, bool openlog)
{
  th1stop = false;
  th1 = NULL;
  tickClassIdx = 0;
//   printf("UBridge:: opening socket to bridge\n");
  createSocket("24001", server);
  tryConnect();
  if (connected)
  {
//     printf("UBridge:: connected to bridge\n");
    th1 = new thread(runObj, this);
  }
  botlog = NULL;
  if (openlog)
    openLog();
}

/////////////////////////////////////////////////////////

/** destructor */
UBridge::~UBridge()
{ // stop all activity before close
  printf("Bridge destructor\n");
  stop();
  if (botlog != NULL)
    fclose(botlog);
}

/////////////////////////////////////////////////////////

void UBridge::openLog()
{
  const int MDL = 32;
  const int MNL = 128;
  char date[MDL];
  char name[MNL];
  UTime time;
  time.now();
  time.getForFilename(date);
  snprintf(name, MNL, "log_rx_tx_%s.txt", date);
  botlog=fopen(name,"w");
  if (botlog != NULL)
  {
    timeval t;
    gettimeofday(&t, NULL);
    float dt = getTimeDiff(t, info->bootTime);
    logMtx.lock();
    fprintf(botlog, "%% log of mission messages transmitted to bridge (outgoing) (<-)\n");
    fprintf(botlog, "%% received from bridge (incoming) (->)\n");
    fprintf(botlog, "%% 1 Linux timestamp (seconds since 1 jan 1970)\n");
    fprintf(botlog, "%% 2 mission time (sec)\n");
    fprintf(botlog, "%% 3 direction flag ->: or <-:\n");
    fprintf(botlog, "%% 4 string send or received\n");
    fprintf(botlog, "%lu.%03ld %6.3f->: %s %d\n", t.tv_sec, t.tv_usec/1000, dt, "Connected to bridge ", connected);
    fflush(botlog);
    logMtx.unlock();
  }
}

/////////////////////////////////////////////////////////

void UBridge::closeLog()
{
  if (botlog != NULL)
  {
    fclose(botlog);
    botlog = NULL;
  }
}

/////////////////////////////////////////////////////////

void UBridge::stop()
{
  th1stop = true;
  if (th1 != NULL)
    th1->join();
  printf("UBridge:: closed bridge port client\n");
}

/////////////////////////////////////////////////////////

/**
  * send a string to the serial port */
void UBridge::send(const char * cmd)
{ // this function may be called by more than one thread
  // so make sure that only one send at any one time
//   printf("UBridge::send 0\n");
  sendMtx.lock();
  if (connected)
  { // send data
//     sleep(1);
//    printf("UBridge::send 2-\n");
    sendData(cmd);
//     printf("UBridge::send 2+ %s", cmd);
    // debug
    {
      timeval t;
      gettimeofday(&t, NULL);
      float dt = getTimeDiff(t, info->bootTime);
      if (botlog != NULL)
      {
        logMtx.lock();
        fprintf(botlog, "%lu.%03ld %6.3f->: %s", t.tv_sec, t.tv_usec/1000, dt, cmd);
        fflush(botlog);
        logMtx.unlock();
      }
//       printf("UBridge::send %6.3f->: %s", dt, cmd);
    }
  }
  // 
  usleep(4000); // Short sleep helps communication be more reliable when many command lines are send consecutively.
  sendMtx.unlock();
}

/////////////////////////////////////////////////////////

/**
  * receive thread */
void UBridge::run()
{ // read thread for REGBOT messages
  int n = 0;
  rxCnt = 0;
  int msgCnt = 0;
  int msgCntSec =0;
  timeval idleTime;
  gettimeofday(&idleTime, NULL);
  bool sockErr = false;
  UTime t, tsec;
  t.Now();
  tsec = t + 1;
  int loopSec = 0;
  int sleepUs = 1000;
  // get robot name
  //send("u4\n");
  int loop = 0;
  while (not th1stop)
  {
    char c;
    n = readChar(&c, &sockErr);
    if (n == 1)
    { // not an error or hangup
      if (c >=' ' or c == '\n')
      { // got a valid character
        rx[rxCnt] = c;
        if (rx[rxCnt] == '\n')
        { // terminate string
          rx[rxCnt] = '\0';
          // printf("#### Received: (loop=%d, rxCnt=%d) '%s'\n",loop, rxCnt, rx);
          decode(rx);
          rxCnt = 0;
          msgCnt++;
          // clear buffer for old message
          memset(rx, '\0', MAX_RX_CNT);
          n = 0;
          gettimeofday(&idleTime, NULL);
        }
        else if (rxCnt < MAX_RX_CNT)
        { // prepare for next byte
          rxCnt++;
          continue;
        }
        else
        { // buffer overflow
          printf("UBridge::run: receiver overflow\n");
          rxCnt = 0;
        }
      }
    }
    else if (sockErr)
    {
      perror("UBridge:: port error");
      usleep(100000);
    }
    // go idle a bit
    usleep(sleepUs);
    loop++;
    t.now();
    if (t > tsec)
    {
      int loops = loop - loopSec;
      loopSec = loop;
      int sleptUs = loops * sleepUs;
      info->bridgeLoad = (1000000 - sleptUs) * 100.0 / 1000000.0;
      info->msgCnt1sec = msgCnt - msgCntSec;
      msgCntSec = msgCnt;
//       printf("# bridge load = %.1f %% (loops = %d, sleepUs=%d, msg cnt=%d/sec)\n", info->bridgeLoad, loops, sleepUs, info->msgCnt1sec);
      tsec += 1;
      info->saveDataToLog();
    }
  }
  printf("UBridge:: thread stopped\n");
}  

/////////////////////////////////////////////////////////

/**
  * decode messages from REGBOT */
void UBridge::decode(char * message)
{
  // skip whitespace
  while (*message <= ' ' and *message > '\0')
    message++;
  // debug
  // printf("UBridge::decode:: got:%s\n", message);
  // debug end
  if (*message != '\0')
  {
    if (strncmp(message, "hbt ", 4) == 0)
      info->decodeHbt(message); // it is a heartbeat message (time and battry voltage)
    else if (strncmp(message, "pse ", 4) == 0)
      pose->decode(message); // it is a pose message
    else if (strncmp(message, "lip ", 4) == 0)
      edge->decode(message); // it is a line edge message
    else if (strncmp(message, "event", 5)==0)
    {
      event->decode(message);
//       t.now();
//       printf("%ld.%03ld UBridge::decode: %s\n", t.getSec(), t.getMilisec(), message);
    }
    else if (strncmp(message, "mis ", 4)==0)
      info->decodeMission(message); // mission status skipped
    else if (strncmp(message, "rid ", 4)==0)
      info->decodeId(message); // mission status skipped
    else if (strncmp(message, "joy ", 4)==0)
    {
      joy->decode(message); // 
    }
    else if (strncmp(message, "wve ", 4)==0)
    { // wheel velocity
      motor->decodeVel(message);
    }
    else if (strncmp(message, "mca ", 4)==0)
    { // motor current
      motor->decodeCurrent(message);
    }
    else if (strncmp(message, "irc ", 4)==0)
    { // motor current
      irdist->decode(message);
    }
    else if (strncmp(message, "acw ", 4)==0)
    { // motor current
//       printf("\n#UBridge:: decoding acw\n");
      imu->decode(message);
//       printf("#UBridge:: decoded acw\n");
    }
    else if (strncmp(message, "gyw ", 4)==0)
    { // motor current
      imu->decode(message);
    }
    else if (strncmp(message, "rid ", 4)==0)
      ; // robot ID skipped
    else if (strncmp(message, "bridge", 6)==0)
      ; // skipped bridge
    else if (*message == '#')
      // just a message from Regbot
      printf("%s\n", message);
    else
    { // not yet supported message - ignore
      printf("UBridge:: unhandled message: '%s'\n", message);
    }
  }
  if (botlog != NULL)
  {
    timeval t;
    gettimeofday(&t, NULL);
    float dt = getTimeDiff(t, info->bootTime);
    logMtx.lock();
    fprintf(botlog, "%lu.%03ld %.1f%%, %6.3f<-: %s\n", t.tv_sec, t.tv_usec/1000, info->bridgeLoad, dt, message);
    logMtx.unlock();
  }
}

//////////////////////////////////////////////////


void UBridge::printStatus()
{
  printf("# ------- Bridge ----------\n");
  printf("# logfile active=%d\n", botlog != NULL);
  printf("# load=%.0f %%, message rate %d/sec\n", info->bridgeLoad, info->msgCnt1sec);
  pose->printStatus();
  edge->printStatus();
  info->printStatus();
  event->printStatus();
  joy->printStatus();
  motor->printStatus();
  irdist->printStatus();
  imu->printStatus();
}

int UBridge::decodeLogOpenOrClose(const char c, UData * item)
{
  if (c == 'o')
    item->openLog();
  else
    item->closeLog();
  return 1;
}


int UBridge::decodeLogOpenClose(const char * s)
{
  const char * p1 = s;
  p1++; // skip 'l'
  int n = 0;
  if (*p1 == 'o' or *p1 == 'c')
  { // open file
    if (strstr(s, "hbt") != NULL)
      n += decodeLogOpenOrClose(s[1], info);
    if (strstr(s, "imu") != NULL)
      n += decodeLogOpenOrClose(s[1], imu);
    if (strstr(s, "motor") != NULL)
      n += decodeLogOpenOrClose(s[1], motor);
    if (strstr(s, "joy") != NULL)
      n += decodeLogOpenOrClose(s[1], joy);
    if (strstr(s, "ir") != NULL)
      n += decodeLogOpenOrClose(s[1], irdist);
    if (strstr(s, "event") != NULL)
      n += decodeLogOpenOrClose(s[1], event);
    if (strstr(s, "pose") != NULL)
      n += decodeLogOpenOrClose(s[1], pose);
    if (strstr(s, "bridge") != NULL)
    { // may be bridge
      if (s[1] == 'o')
        openLog();
      else
        closeLog();
      n++;
    }
  }
  else
    printf("# unknown log command '%s'\n", s); 
  return n;
}
