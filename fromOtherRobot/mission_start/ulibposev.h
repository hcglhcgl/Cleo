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
#ifndef UPOSEV_H
#define UPOSEV_H

#include "ulibpose.h"

/**
A robot pose and speed, i.e. a robot state

@author Christian Andersen
*/
class UPoseV : public UPose
{
public:
  /**
  Constructor */
  UPoseV();
  /**
  Constructor with initial value */
  UPoseV(float_t x, float_t y, float_t h, float_t v);
  /**
  Destructor */
  virtual ~UPoseV();
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "posev";  };
  /**
  Subtract 2 poses to get a delta pose from ref to base.
  i.e. if P1 and P2 is two poses, and
  deltaPose Pd = P2 - P1, then P2 is at
  position Pd in local P1 coordinates, and
  P2 = P1 + Pd. Velocity (vel) is difference in velovity P2.vel - P1.vel. */
  UPoseV operator-(UPoseV pRef);
  /**
  Add a delta pose to a base pose to get a new pose.
  The delta pose must be in base pose perspective.
  i.e. To get from a reference position P1 to a new position
  P2 after a movement of 'pDelta' */
  UPoseV operator+(UPoseV pDelta);
  /**
  Assign from base pose */
  inline UPoseV operator= (UPose source)
  {
    x = source.x;
    y = source.y;
    h = source.h;
    return *this;
  };
  /**
  Assign from base pose */
  inline UPoseV operator= (UPoseV source)
  {
    x = source.x;
    y = source.y;
    h = source.h;
    vel = source.getVel();
    return *this;
  };
  /**
  Add a delta pose to a base pose to get a new pose.
  The delta pose must be in base pose perspective.
  i.e. To get from a reference position P1 to a new position
  P2 after a movement of 'pDelta' */
  UPoseV operator+(UPose pDelta);
  /**
  Set all variables from float_ts */
  inline void set(float_t ix, float_t iy, float_t ih, float_t iv)
  {
    x = ix;
    y = iy;
    h = ih;
    vel = iv;
  };
  /**
  Set all variables from pose and velocity */
  inline void set(UPose pose, float_t iv)
  {
    x = pose.x;
    y = pose.y;
    h = pose.h;
    vel = iv;
  };
  /**
  Get velocity */
  inline float_t getVel()
  { return vel; };
  /**
  Get the UPose part only */
  inline UPose getPose()
  {
    UPose result(x, y, h);
    return result;
  };
  /**
  Set velocity */
  inline void setVel(float_t velocity)
  { vel = velocity; };
  /**
  Print object status */
  virtual void fprint(FILE * fd, const char * prestring);
  /**
  Print tatus to string.
  Deprecated call format - use snprint(...) */
  inline void print(const char * prestring, char * buff, const int buffCnt)
  { snprint(prestring, buff, buffCnt); };
  /**
  Print status for this structure */
  virtual void snprint(const char * preString, char * buff, const int buffCnt);

  /**
  Print tatus to string */
  inline void print(const char * prestring)
  {
    const int MSL = 50;
    char s[MSL];
    snprint(prestring, s, MSL);
    printf("%s", s);
  }

public:
  /**
  Actual velocity. velocity may be negative for
  reverse speed. */
  float_t vel;
};

#endif
