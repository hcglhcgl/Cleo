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

#include "ujoy.h"


UJoy::UJoy(UBridge * bridge_ptr, bool openLog)
{
  bridge = bridge_ptr;
}

/**
 * open logfile and print description */
void UJoy::openLog()
{
  UData::openLog("log_joy");
  if (logIsOpen())
  {
    fprintf(logfile, "%% robobot mission joystick (gamepad) log\n");
    fprintf(logfile, "%% 1 Timestamp in seconds\n");
    fprintf(logfile, "%% 2-8 Axis position\n");
    fprintf(logfile, "%% 9-19 buttons pressed\n");
  }
}
//
void UJoy::decode(char * msg)
{ // joy 1 1 8 11 0 -2 -32767 0 -7513 -32767 0 0 0 0 0 0 0 0 0 0 0 0 0
  // 1 : joyRunning, 
  // 1 : manOverride, 
  // 8 : number_of_axes, 
  // 11 : number_of_buttons    
  // 0 -2 -32767 0 -7513 -32767 0 0 : axis values
  // 0 0 0 0 0 0 0 0 0 0 0 : button values (1=pressed)
  char * p1 = &msg[4];
  int n = strtol(p1, &p1, 10);
  if (n == 0)
    printf("UJoy::decode: No joystick / gamepad\n");
  manual = strtol(p1, &p1, 10) == 1;
  int a = strtol(p1, &p1, 10);
  int b = strtol(p1, &p1, 10);
  if (b > 11)
    b = 11;
  for (int i = 0; i < a; i++)
  {
    if (i < 8)
      axes[i] = strtol(p1, &p1, 10);
  }      
  for (int i = 0; i < b; i++)
  {
    if (i < 11)
    {
      button[i] = strtol(p1, &p1, 10);
      if (button[i])
      {
        printf("UJoy::decode: button %d pressed\n", i);
      }
    }
  }
  updated();
  if (logIsOpen())
  {
    fprintf(logfile, "%ld.%03ld ", 
            dataTime.tv_sec, dataTime.tv_usec / 1000);
    for (int i = 0; i < 8; i++)
      fprintf(logfile, "%d ", axes[i]);
    for (int i = 0; i < 11; i++)
      fprintf(logfile, "%d ", button[i]);
    fprintf(logfile,"\n");
  }
}

void UJoy::subscribe()
{
  bridge->send("joy subscribe 1\n"); // button and axis
  bridge->send("joy get\n");          // get data now
}

void UJoy::printStatus()
{
  printf("# ------- JOY (gamepad) ----------\n");
  printf("# manual control = %d\n", manual);
  printf("# axis      1 to 8 : ");
  for (int i = 0; i < 8; i++)
    printf("%d, ", axes[i]);
  printf("\n# buttons 1 to 11: ");
  for (int i = 0; i < 11; i++)
    printf("%d, ", button[i]);
  printf("\n# data age %.3fs\n", getTimeSinceUpdate());
  printf("# logfile active=%d\n", logfile != NULL);
}
