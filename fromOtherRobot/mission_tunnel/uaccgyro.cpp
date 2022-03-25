/***************************************************************************
 *   Copyright (C) 2019-2020 by DTU (Christian Andersen)                        *
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


UAccGyro::UAccGyro(UBridge* bridge_ptr, bool openlog)
{
  bridge = bridge_ptr;
  if (openlog)
    openLog();
}

void UAccGyro::openLog()
{ // open pose log
  UData::openLog("log_accgyro");
  if (logfile != NULL)
  {
    fprintf(logfile, "%% robobot mission Accelerometer and gyro log\n");
    fprintf(logfile, "%% 1 Timestamp in seconds\n");
    fprintf(logfile, "%% 2-4 accelerometer x,y,z [m/s^2]\n");
    fprintf(logfile, "%% 5-7 gyro x,y,z [deg/s]\n");
  }
}


void UAccGyro::decode(char* msg)
{ // assuming msg = "irc ..."
  char * p1 = &msg[3];
  bool isOK = true;
  if (strncmp(msg, "acw", 3) == 0)
  {
    acc[0] = strtof(p1, &p1);
    acc[1] = strtof(p1, &p1);
    acc[2] = strtof(p1, &p1);
  }
  else if (strncmp(msg, "gyw", 3) == 0)
  {
    gyro[0] = strtof(p1, &p1);
    gyro[1] = strtof(p1, &p1);
    gyro[2] = strtof(p1, &p1);
  }
  else 
    isOK = false;
  if (isOK)
  {
    updated();
    if (logfile != NULL)
    {
      fprintf(logfile, "%ld.%03ld %.3f %.3f %.3f %.3f %.3f %.3f\n", dataTime.tv_sec, dataTime.tv_usec / 1000, acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
    }
  }
}

void UAccGyro::subscribe()
{ // ask bridge to forward all acc messages
  bridge->send("acw subscribe 1\n"); 
  // ask bridge to forward all gyro messages
  bridge->send("gyw subscribe 1\n");
  // ask regbot to generate acc messages
  bridge->send("robot sub 1 1 4\n");
  // ask regbot to generate gyro messages
  bridge->send("robot sub 1 1 5\n");
}

void UAccGyro::printStatus()
{
  printf("# ------- Accelerometer and gyro ----------\n");
  printf("# accelerometer (x, y, z) = (%6.3f, %6.3f, %6.3f) m/s^2\n", acc[0], acc[1], acc[2]);
  printf("# gyro          (x, y, z) = (%6.3f, %6.3f, %6.3f) deg/s\n", gyro[0], gyro[1], gyro[2]);
  printf("# data age %.3fs\n", getTimeSinceUpdate());
  printf("# logfile active=%d\n", logfile != NULL);
}

float UAccGyro::turnrate()
{
  return sqrt(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
}

