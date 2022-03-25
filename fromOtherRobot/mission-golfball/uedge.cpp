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

UEdge::UEdge(UBridge * bridge_ptr, bool openLog)
{
  bridge = bridge_ptr;
}


void UEdge::decode(char * msg)
{ /*   snprintf(reply, MRL, "lip %d %d %.4f %d %.4f %d %d %d %d %d %d %d %d\r\n" ,
  *           lineSensorOn, 
  *           lsIsWhite,
  *           lsLeftSide, lsLeftValid,
  *           lsRightSide, lsRightValid, 
  *           mission_line_LeftEdge,  // not visible in client
  *           crossingWhiteLine , crossingBlackLine, crossingWhiteCnt, crossingBlackCnt,
  *           lsPowerHigh, lsPowerAuto
  */
  char * p1 = &msg[3];
  int d = strtol(p1, &p1, 0);
  if (d)
  { // line sensor is on - so read the rest
    d = strtol(p1, &p1, 0);
    /*float f =*/ strtof(p1, &p1);
    edgeValidLeft = strtol(p1, &p1, 0);
    /*f =*/ strtof(p1, &p1); 
    edgeValidRight = strtol(p1, &p1, 0);
    d = strtol(p1, &p1, 0);
    edgeCrossingBlack = strtol(p1, &p1, 0);
    edgeCrossingWhite = strtol(p1, &p1, 0);
    updated();
  }  
}


void UEdge::subscribe()
{
  bridge->send("lip subscribe 1\n"); // Line sensor result
}

void UEdge::printStatus()
{
  printf("# ------- Edge sensor ----------\n");
  printf("# edge left valid=%d, edge right valid=%d\n", edgeValidLeft, edgeValidRight);
  printf("# crossing black line = %d, crossing white line = %d\n", edgeCrossingBlack, edgeCrossingWhite);
  printf("# data age %.3fs\n", getTimeSinceUpdate());
  printf("# logfile active=%d\n", logfile != NULL);
}
