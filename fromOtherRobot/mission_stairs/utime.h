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

#ifndef UTIME_H
#define UTIME_H

#include <sys/time.h>


/**
Class encapsulation the time structure used by 'gettimeofday'
with resolution in years down to micro-seconds.
The class has functions to make simple time calculations and
conversion to and from string in localized format. */
class UTime
{
public:
  /**
  Constructor */
  UTime();
  /**
  Destructor */
  ~UTime();
  /**
  Clear to 0.0 */
  void clear();
  /**
  Get time value in seconds (since 1970) */
  unsigned long getSec();
  /**
  Get milisecond value within second in range 0..999 */
  long getMilisec();
  /**
  Get microsecond value within second in range 0..999999 */
  unsigned long getMicrosec();
  /**
  Get second value with microsecond as decimals */
  float getDecSec();
  /**
  Get time since t1 as decimal seconds. */
  float getDecSec(UTime t1);
  /**
  Get time past since this time */
  float getTimePassed();
  /**
  Set time value to system time now using gettimeofday() */
  inline void Now()
     { gettimeofday(&time, NULL); valid = true; }
  /**
  Set time value to system time now using gettimeofday() */
  inline void now()
  { gettimeofday(&time, NULL); valid = true; }
  /**
  Set time from a timeval structure */
  void setTime(timeval iTime);
  /**
  Set time using seconds and microseconds. */
  void setTime(long sec, long uSec);
  /**
   * Writes time to INFO in format "hh:mm:ss.msec"
   * \param info destination buffer, must be at least 13 characters long
   * \param local converts time to local time (is system time is UTM or somthing)
  */
  int getTimeAsString(char * info, bool local = true);
  /**
  Writes time to INFO in format "yyyyMMdd_hhmmss.msec"
   * \param info is a bugger for the string, must be at least 19 characters long.
   * \param local should time be in local time (else UTM if set on computer)
   * \returns pointer to the info buffer */
  char * getForFilename(char * info, bool local = true);
  /**
  Writes time to INFO in format "yyyy-MM-dd hh:mm:ss.msec"
   * \param info is a bugger for the string, must be at least 24 characters long.
   * \param local should time be in local time (else UTM if set on computer)
   * \returns pointer to the info buffer */
  char * getDateTimeAsString(char * info, bool local = true);
  /**
   *  Compare two times */
  inline UTime operator=(timeval newTime)
  {
    time = newTime;
    valid = true;
    return *this;
  };
  /**
  Compare two times */
  inline bool operator==(UTime other)
  {
    bool result;
    if ((time.tv_sec == other.time.tv_sec) and (time.tv_usec == other.time.tv_usec))
      result = true;
    else
      result = false;
    return result;
  };
  /**
  Compare two times */
  inline bool operator> (UTime other)
  {
    bool result;
    if ((time.tv_sec > other.time.tv_sec) or
         ((time.tv_sec == other.time.tv_sec) and (time.tv_usec > other.time.tv_usec)))
      result = true;
    else
      result = false;
    return result;
  };
  /**
  Compare two times */
  inline bool operator>= (UTime other)
  { return not (*this < other); };
  /**
  Compare two times */
  inline bool operator< (UTime other)
  {
    bool result;
    if ((time.tv_sec < other.time.tv_sec) or
         ((time.tv_sec == other.time.tv_sec) and (time.tv_usec < other.time.tv_usec)))
      result = true;
    else
      result = false;
    return result;
  };
  /**
  Compare two times, where other is a float float */
  inline bool operator< (float other)
  {
    return ((getDecSec() - other) < 0.0);
  };
  /**
  Compare two times, where other is a float float */
  inline bool operator> (float other)
  {
    return ((getDecSec() - other) > 0.0);
  };
  /**
  Compare two times, where other is a float float */
  inline bool operator<= (float other)
  {
    return ((getDecSec() - other) <= 0.0);
  };
  /**
  Compare two times, where other is a float float */
  inline bool operator>= (float other)
  {
    return ((getDecSec() - other) >= 0.0);
  };
  /**
  Compare two times */
  inline bool operator<= (UTime other)
  { return not (*this > other); };
  /**
  Compare two times */
  inline bool operator!=(UTime other)
  {
    return not (*this == other);
  };
  /**
  Subtract two UTime values and get result in decimal seconds */
  inline float operator- (UTime old)
  { return getDecSec(old);};
  /**
  Add a number of seconds to this time */
  UTime operator+ (float seconds);
  /**
  sub a number of seconds to this time */
  UTime operator- (float seconds);
  /**
  Add a number of decimal seconds to this time. */
  inline void operator+= (float seconds)
    { add(seconds); };
  /**
  Add a number of decimal seconds to this time. */
  inline void operator-= (float seconds)
    { sub(seconds); };
  /**
  Add this number of seconds to the current value */
  void add(float seconds);
  /**
  Subtract a number of seconds from this time.
  Can not handle negative time, and seconds must be positive. */
  void sub(float seconds);
  /**
  Convert seconds to time_tm strucure.
  \param when 'local' is true the local time is returned, else GMT.
  \return the structure with year (year 1900 == 0), month, day, hour, min and sec. */
  struct tm getTimeTm(bool local = true);
  /**
  Get copy of timevalue structure */
  inline struct timeval getTimeval()
  {
    return time;
  }
  /**
  Get month number form 3 character string.
  String value must match one of:
  Jan Feb Mas Apr May Jun Jul Aug Sep Oct Nov Dec.
  Returns 0 if no match were found. */
  int getMdrFromString(const char * month3char);
  /**
  Show date and time on console */
  void show(const char * prestring = NULL);
  /**
  print date and time on console */
  inline void print(const char * prestring = NULL)
    { show(prestring); };
  /**
  Code time in XML-like format.
  If 'name' is NULL, then no name attribute is included.
  The tag name is 'time', and format is like:\n
  \< time name="name" sec=107096665 usec=123456/>
  (in seconds and microsec since 1970).
  Returns pointer to provided buffer. */
  char * getAsSml(const char * name, char * buff, int buffCnt);
public:
  /**
  Time as 'timeval' - i.e. same format as in 'timeofday' call. */
  timeval time;
  /**
  A valid flag, that are used when setting the time */
  bool valid;
};


#endif
