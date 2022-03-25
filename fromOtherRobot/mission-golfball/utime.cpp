 /***************************************************************************
 *   Copyright (C) 2006-2020 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "utime.h"

/////////////////////////////////////////

UTime::UTime()
{
  clear();
}

/////////////////////////////////////////////

UTime::~UTime()
{
}

/////////////////////////////////////////

void UTime::clear()
{ // clear to zero
  time.tv_sec = 0;
  time.tv_usec = 0;
  valid = false;
}

unsigned long UTime::getSec()
{
  if (valid)
    return time.tv_sec;
  else
    return 0;
}

/////////////////////////////////////////

float UTime::getDecSec()
{
  if (valid)
    return float(time.tv_sec) + float(time.tv_usec) * 1e-6;
  else
    return 0;
}

/////////////////////////////////////////

float UTime::getDecSec(UTime t1)
{ // get time compared to t1
  return float(time.tv_sec - t1.time.tv_sec) + float(time.tv_usec - t1.time.tv_usec) * 1e-6;
}

/////////////////////////////////////////

float UTime::getTimePassed()
{
  UTime t;
  t.Now();
  return (t - *this);
}

/////////////////////////////////////////

long UTime::getMilisec()
{
  if (valid)
    return time.tv_usec / 1000;
  else
    return 0;
}

///////////////////////////////////////////////

unsigned long UTime::getMicrosec()
{
  if (valid)
    return time.tv_usec;
  else
    return 0;
}

//////////////////////////////////////////////////

int UTime::getTimeAsString(char * info, bool local)
{ // writes time to string in format "hh:mm:ss.msec"
  struct tm ymd;
  //
  if (local)
    localtime_r(&time.tv_sec, &ymd);
  else
    gmtime_r(&time.tv_sec, &ymd);
  //
  sprintf(info, "%2d:%02d:%02d.%03d", ymd.tm_hour,
            ymd.tm_min, ymd.tm_sec, (int)getMilisec());
  return strlen(info);
}

//////////////////////////////////////////

char * UTime::getForFilename(char * info, bool local /*= true*/)
{
  struct tm ymd;
  //
  if (local)
    localtime_r(&time.tv_sec, &ymd);
  else
    gmtime_r(&time.tv_sec, &ymd);
  //
  sprintf(info, "%04d%02d%02d_%02d%02d%02d.%03d",
            ymd.tm_year+1900, ymd.tm_mon+1, ymd.tm_mday,
            ymd.tm_hour,
            ymd.tm_min, ymd.tm_sec, (int)getMilisec());
  return info;
}

//////////////////////////////////////////

char * UTime::getDateTimeAsString(char * info, bool local /*= true*/)
{
  struct tm ymd;
  //
  if (local)
    localtime_r(&time.tv_sec, &ymd);
  else
    gmtime_r(&time.tv_sec, &ymd);
  //
  sprintf(info, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
          ymd.tm_year+1900, ymd.tm_mon+1, ymd.tm_mday,
          ymd.tm_hour,
          ymd.tm_min, ymd.tm_sec, (int)getMilisec());
  return info;
}


//////////////////////////////////////////

void UTime::setTime(timeval iTime)
{
  time = iTime;
  valid = true;
}

/////////////////////////////////////////

void UTime::setTime(long sec, long uSec)
{
  time.tv_sec = sec;
  time.tv_usec = uSec;
  valid = true;
}

/////////////////////////////////////////

struct tm UTime::getTimeTm(bool local)
{
  struct tm ymd;
  //
  if (local)
    localtime_r(&time.tv_sec, &ymd);
  else
    gmtime_r(&time.tv_sec, &ymd);
  //
  return ymd;
}

/////////////////////////////////////////////

UTime UTime::operator+ (float seconds)
{
  UTime t = *this;
  t.add(seconds);
  return t;
}

/////////////////////////////////////////////

UTime UTime::operator- (float seconds)
{
  UTime t = *this;
  t.sub(seconds);
  return t;
}

/////////////////////////////////////////////

void UTime::add(float seconds)
{
  float rem = seconds - long(seconds);
  long remL = (unsigned long)(rem * 1000000.0);
  //
  time.tv_sec += (unsigned long)seconds;
  if (time.tv_usec + remL > 1000000)
  { // adjust if overflow
    time.tv_sec++;
    time.tv_usec += remL - 1000000;
  }
  else
    time.tv_usec += remL;
}

void UTime::sub(float seconds)
{
  float rem = seconds - long(seconds);
  long remL = (unsigned long)(rem * 1000000.0);
  //
  time.tv_sec -= (unsigned long)seconds;
  if (time.tv_usec < remL)
  { // adjust if overflow
    time.tv_sec--;
    time.tv_usec += 1000000 - remL;
  }
  else
    time.tv_usec -= remL;
}
/////////////////////////////////////////////



