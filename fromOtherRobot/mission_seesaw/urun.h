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
 
#ifndef URUN_H
#define URUN_H
 
#include <stdio.h>
#include <thread>

/**
 * calculatin difference between two timestamps as a float - in seconds
 * \param newest is a timestamp created with gettimeofday(&timeval) of type timeval (defined in \<sys/time.h> ))
 * \param oldest is a timestamp created with gettimeofday(&timeval) of type timeval (defined in \<sys/time.h> ))
 * \returns difference in decimal seconds
 * */
float getTimeDiff(timeval newest, timeval oldest);

inline int mini(int a, int b)
{
  if (a < b)
    return a;
  else
    return b;
}

inline int maxi(int a, int b)
{
  if (a > b)
    return a;
  else
    return b;
}


class URun;

/** start a thread in the provided object */
void runObj(URun * obj);

/**
 * Base class, that makes it easier to starta thread
 * from the method runObj
 * The run method should be overwritten in the real class */
class URun
{
public:
  
  ~URun()
  {
//     printf("instance of run stopping\n");
    stop();
  }
  
  ////////////////////////////////////////////////
  
  virtual void run()
  { // must be overwritten
    printf("Should not show (nothing to run)\n");
  }
  
  ////////////////////////////////////////////////
  /**
   * Start the thread and run until thStop is set */
  bool start()
  {
    th1stop = false;
    if (th1 == NULL)
      th1 = new std::thread(runObj, this);
    return th1 != NULL;
  }
  
  virtual void stop()
  {
    th1stop = true;
    if (th1 != NULL)
    {
      if (th1->joinable())
        th1->join();
      th1 = NULL;
    }
  }
protected:
  void runFinished()
  {
    th1stop = true;
    th1 = NULL;
  }
  // read thread handle
  std::thread * th1 = NULL;
  // set true to stop thread
  bool th1stop = false;
};

#endif
