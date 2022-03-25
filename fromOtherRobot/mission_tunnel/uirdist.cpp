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


UIRdist::UIRdist(UBridge* bridge_ptr, bool openlog)
{
  bridge = bridge_ptr;
  if (openlog)
    openLog();
}


void UIRdist::openLog()
{ // open pose log
  UData::openLog("log_irdist");
  if (logfile != NULL)
  {
    fprintf(logfile, "%% robobot mission IR distance log\n");
    fprintf(logfile, "%% 1 Timestamp in seconds\n");
    fprintf(logfile, "%% 2 IR 1 distance [meter]\n");
    fprintf(logfile, "%% 3 IR 2 distance [meter]\n");
    fprintf(logfile, "%% 4 IR 1 raw [AD value]\n");
    fprintf(logfile, "%% 5 IR 2 raw [AD value]\n");
  }
}
//

void UIRdist::decode(char* msg)
{ // assuming msg = "irc ..."
  char * p1 = &msg[3];
  dist[0] = strtof(p1, &p1);
  dist[1] = strtof(p1, &p1);
  raw[0] = strtol(p1, &p1, 10);
  raw[1] = strtol(p1, &p1, 10);
  updated();
  if (logfile != NULL)
  {
    fprintf(logfile, "%ld.%03ld %.3f %.3f %d %d\n", dataTime.tv_sec, dataTime.tv_usec / 1000, dist[0], dist[1], raw[0], raw[1]);
  }
}

void UIRdist::subscribe()
{ // tell bridge to forward IR messages (type irc)
  bridge->send("irc subscribe 1\n"); 
  // tell REGBOT to generate IR messages (at priority 1 and msg type 2)
  bridge->send("robot sub 1 1 2\n");   
}

void UIRdist::printStatus()
{
  printf("# ------- IR distance----------\n");
  for (int i = 0; i < 2; i++)
  {
    printf("# distance %d: %.3fm (raw=%d)\n", i, dist[i], raw[i]);
  }
  printf("# data age %.3fs\n", getTimeSinceUpdate());
  printf("# logfile active=%d\n", logfile != NULL);
}

