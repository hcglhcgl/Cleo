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

#include "ubridge.h"

UEvent::UEvent(UBridge * bridge_ptr, bool openlog)
{
  clearEvents();
  bridge = bridge_ptr;
  if (openlog)
    openLog();
}

void UEvent::openLog()
{ // open pose log
  const int MDL = 32;
  const int MNL = 128;
  char date[MDL];
  char name[MNL];
  UTime time;
  time.now();
  time.getForFilename(date);
  snprintf(name, MNL, "log_event_%s.txt", date);
  logfile = fopen(name, "w");
  if (logfile != NULL)
  {
    fprintf(logfile, "%% robobot IR distance log\n");
    fprintf(logfile, "%% 1 Timestamp in seconds\n");
    fprintf(logfile, "%% 2 event set (-1=not set)\n");
    fprintf(logfile, "%% 3 event cleared (-1=not cleared)\n");
  }
}


//
void UEvent::decode(char * msg)
{ // "event 33"
  // skip the first 5 characters
  char * p1 = &msg[5];
  int eventNumber = strtol(p1, &p1,0);
  // if first event is 0, then it is result of last mission
  // still maintained in bridge - just ignore
  if (eventNumber > 0 or not firstEvent)
  {
    setEvent(eventNumber);
    firstEvent = false;
  }
  updated();
  timeval t;
  gettimeofday(&t, NULL);
  float dt = getTimeDiff(t, bridge->info->bootTime);
  
  // debug
  if (false)
  { // print received event message to console
    switch (eventNumber)
    {
      case  0: printf("# %.3f got event %d (stopped): '%s'\n", dt, eventNumber, msg); break;
      case 33: printf("# %.3f got event %d (start): '%s'\n", dt, eventNumber, msg); break;
      case 30: printf("# %.3f got event %d (next): '%s'\n", dt, eventNumber, msg); break;
      case 31: printf("# %.3f got event %d (next): '%s'\n", dt, eventNumber, msg); break;
      default: printf("# %.3f got event %d: '%s'\n", dt, eventNumber, msg); break;
    }
  }
  if (logfile != NULL)
  {
    fprintf(logfile, "%ld.%03ld %2d -1\n", dataTime.tv_sec, dataTime.tv_usec / 1000, eventNumber);
  }
}
/** set event flag */
void UEvent::setEvent(int eventNumber)
{
  if (eventNumber < MAX_EVENT_FLAGS and eventNumber >= 0)
  { // event 33 is start button, event 0 is misson stop
    eventUpdate.lock();
    //printf("# Event received: %d\n", eventNumber);
    eventFlags[eventNumber] = true;
    eventUpdate.unlock();
  }
}
/**
 * print all event flags to console */
void UEvent::printEvents()
{
  for (int i = 0; i < MAX_EVENT_FLAGS; i++)
  {
    if (eventFlags[i])
      printf("#UEvent::print: event %2d is set\n", i);
  }
}
/**
 * Clear all event flags to false */
void UEvent::clearEvents()
{
  for (int i = 0; i < MAX_EVENT_FLAGS; i++)
    eventFlags[i] = false;
}
/**
 * Requests if this event has occured.
 * \param event the event flag to test
 * \returns true and resets event, if it was set */
bool UEvent::isEventSet ( int event )
{
  bool set = false;
  if (event <= MAX_EVENT_FLAGS and event >= 0)
  { // make sure it is not updated between test and set
    eventUpdate.lock();
    set = eventFlags[event];
    eventFlags[event] = 0;
    updated();
    eventUpdate.unlock();
    if (logfile != NULL and set)
    { // flag is cleared - put in log
      switch (event)
      {
        case  0: fprintf(logfile, "%ld.%03ld -1 %2d (stop)\n", dataTime.tv_sec, dataTime.tv_usec / 1000, event); break;
        case 33: fprintf(logfile, "%ld.%03ld -1 %2d (start)\n", dataTime.tv_sec, dataTime.tv_usec / 1000, event); break;
        case 30: fprintf(logfile, "%ld.%03ld -1 %2d (next snippet)\n", dataTime.tv_sec, dataTime.tv_usec / 1000, event); break;
        case 31: fprintf(logfile, "%ld.%03ld -1 %2d (next snippet)\n", dataTime.tv_sec, dataTime.tv_usec / 1000, event); break;
        default: fprintf(logfile, "%ld.%03ld -1 %2d\n", dataTime.tv_sec, dataTime.tv_usec / 1000, event); break;
      }
    }
  }
  return set;
}

void UEvent::subscribe()
{ // subscribe to data from bridge
  clearEvents();
  bridge->send("event subscribe 6\n"); // start, stop and mission events
  bridge->send("event get\n"); // start, stop and mission events
}

void UEvent::printStatus()
{
  int eventCnt = 0;
  printf("# ------- Events ----------\n");
  for (int i = 0; i < MAX_EVENT_FLAGS; i++)
  {
    if (eventFlags[i])
    {
      printf("# event %d is set", i);
      eventCnt++;
    }
  }
  if (eventCnt == 0)
    printf("# No events active.\n");
  printf("# data age %.3fs for event from bridge\n", getTimeSinceUpdate());
  printf("# logfile active=%d\n", logfile != NULL);
}
