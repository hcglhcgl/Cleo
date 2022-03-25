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
#ifndef ULIBPOSE_H
#define ULIBPOSE_H

//#include <ugen4/u3d.h>
//#include <ugen4/ucommon.h>
//#include <ugen4/udatabase.h>
#include "ulib2dline.h"
#include "utime.h"

class UPoseTime;
class UPoseV;
// class UPoseTVQ;


/**
 *Ensure this angle 'v' is within plus/minus PI. */
inline float_t limitToPi(float_t v)
{
  float_t result = v;
  if (fabs(result) > 1e6)
    // insane value, but avoids endless loop
    result = 0.0;
  else
  {
    while (result < -M_PI)
      result += 2.0 * M_PI;
    while (result > M_PI)
      result -= 2.0 * M_PI;
  }
  return result;
}

/**
 *Ensure this angle 'v' is within plus/minus PI. */
inline float_t limitTo2Pi(float_t v)
{
  float_t result = v;
  if (fabs(result) > 1e6)
    // insane value, but avoids endless loop
    result = 0.0;
  else
  {
    while (result < 0.0)
      result += 2.0 * M_PI;
    while (result > 2.0*M_PI)
      result -= 2.0 * M_PI;
  }
  return result;
}



/**
Position for a Robot and functions related to such function. <br>
The main thing is the data in format [x,y,Theta]

@author Christian Andersen
*/
class UPose // : public UDataBase
{
public:
  /**
  Constructor */
  UPose();
  /**
  Constructor with initial value */
  UPose(float_t x, float_t y, float_t h);
  /**
  Constructor, tahe data from other object */
  UPose(UPose * source);
  /**
  Destructor */
  virtual ~UPose() {};
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "pose";  };
  /**
  Clear to zero. */
  inline virtual void clear()
  {
    x = 0.0;
    y = 0.0;
    h = 0.0;
  };
  /**
  get a pose from a pose-time object */
  UPose operator= (UPoseTime source);
  /**
  get a pose from a pose-velocity object */
  UPose operator= (UPoseV source);
  /**
  Advance the pose 'dist' distance with an heading change
  over the distance of 'headingChange'. */
  void add(float_t dist, float_t headingChange);
  /**
  Set value from these data */
  void set(float_t ix, float_t iy, float_t ih);
  /**
  Add this increment to present pose */
  void add(float_t ix, float_t iy, float_t ih);
  /**
  Print pose in single line to console, with
  the provided prestring in front of data. */
  void fprint(FILE * fd, const char * prestring);
  /**
  Print pose in single line to a string buffer of length 'bufLng', with
  the provided prestring in front of data.
  Deprecated call - use snprint(...). */
  inline void print(char * buf, const char * prestring, int bufLng)
  { snprint(prestring, buf, bufLng); };
  /**
  Print pose in single line to a string buffer of length 'bufLng', with
  the provided prestring in front of data. */
  inline void print(const char * str)
  {
    const int MSL = 200;
    char s[MSL];
    snprint(str, s, MSL);
    fprintf(stdout, "%s\n", s);
  }
  /**
  Print status for this structure into a string */
  virtual void snprint(const char * preString, char * buff, const int buffCnt);
  /**
  Returns heading in 0-360 degree range. */
  float_t getHeadingDeg();
  /**
  Returns heading in radians. */
  inline float_t getHeadingRad()
     { return h; };
  /**
  Get distance to another pose. */
  inline float_t getDistance(UPose other)
     { return hypot(other.x - x, other.y - y); };
  /**
  Get distance to another pose. */
  inline float_t getDistance(UPose * other)
  { return hypot(other->x - x, other->y - y); };
  /**
  Get heading difference to another pose. I.e.
  how much angle should I add th this heading to get the other
  (return limitToPi(other.h - h)). */
  inline float_t getHeadingDiff(UPose other)
     { return limitToPi(other.h - h); };
  /**
  Negate this delta pose. */
  UPose neg();
  /**
  Convert this local position coordinate to map (global) coordinates. The z value is maintained */
//   UPosition getPoseToMap(UPosition posePos);
  /**
  Convert this local position coordinate to map (global) coordinates. */
  U2Dpos getPoseToMap(U2Dpos posePos);
  /**
  Convert this map position to local pose coordinates.
  This is done by translating map coordinates to pose position
  followed by rotation to pose-heading. */
  U2Dpos getMapToPose(U2Dpos mapPos);
  /**
  Convert this map pose (x,y,h) to local pose coordinates.
  This is done by translating map coordinates to pose position
  followed by rotation to pose-heading.
  See also getPoseToMapPose(UPose * mapPose)*/
  UPose getMapToPosePose(UPose * mapPose);
  inline UPose getMapToPosePose(UPose mapPose)
  { return getMapToPosePose(&mapPose); };
  /**
  Convert this local pose position coordinate to map (global) coordinates.
  This is done by rotating with the heading and translating
  with the pose position.
  See also getMapToPosePose(UPose * mapPose)*/
  UPose getPoseToMapPose(UPose poseLocal);
  /**
  Convert this local pose position coordinate to map (global) coordinates.
  This is done by rotating with the heading and translating
  with the pose position.
  See also getMapToPosePose(UPose * mapPose)*/
  inline UPose getPoseToMapPose(float_t x, float_t y, float_t h)
  {
    UPose a(x,y,h);
    return getPoseToMapPose(a);
  };
  /**
  Subtract 2 poses to get a delta pose from ref to base.
  i.e. if P1 and P2 is two poses, and
  deltaPose Pd = P2 - P1, then P2 is at
  position Pd in local P1 coordinates, and
  P2 = P1 + Pd */
  inline UPose operator-(UPose pRef)
     { return subCed(pRef); };
  UPose operator-(UPoseV pRef);
  UPose operator-(UPoseTime pRef);
//   UPose operator-(UPoseTVQ pRef);
  /**
  Add a delta pose to a base pose to get a new pose.
  The delta pose must be in base pose perspective.
  i.e. To get from a reference position P1 to a new position
  P2 after a movement of 'pDelta' */
  inline UPose operator+(UPose pDelta)
     { return addCed(pDelta); };
  UPose operator+(UPoseV pDelta);
  UPose operator+(UPoseTime pDelta);
  /**
   * Get distance to line described by this line (signed)
   * \param Px,Py is the position to be evaluated
   * \returns the signed distance (left is positive) */
  float_t getDistToPoseLineSigned(const float_t Px, const float_t Py);
  /**
   * Get distance to line described by this line
   * \param Px,Py is the position to be evaluated
   * \returns the distance */
  inline float_t getDistToPoseLine(const float_t Px, const float_t Py)
  { return fabs(getDistToPoseLineSigned(Px, Py)); };
protected:

  /**
  Compounding operation Va = Vbase o+ D, where
  Va, Vbase and are all poses and D is the movement from Vb to
  Va in Vb local coordinates, whereas Va and Vbase are in "global"
  coordinates.
  Va = [xa, ya, ha] Vbase = [xb, yb , hb] D = [x, y, h].
  xa = xb + x cos(hb) - y sin(hb);
  ya = yb + x sin(hb) + y cos(hb);
  ha = hb + h;
  Returns the new pose Va.
  (from Lu and Milios (1997)) */
  void asAddC(UPose Vbase, UPose D);
  inline UPose addCed(UPose D)
  { return addCed(&D); };
  UPose addCed(UPose * D);
  /**
  Inverse compounding operation calculating the
  pose change from Vbase to Va, i.e. D = Va o- Vb.
  Va = [xa, ya, ha] Vbase = [xb, yb , hb] D = [x, y, h].
  x = (xa - xb)cos(hb) + (ya - yb)sin(hb);
  y = -(xa - xb)sin(hb) + (ya - yb)cos(hb);
  h = ha - hb;
  Returns the inverse compound pose.
  (from Lu and Milios (1997)) */
  void asSubC(UPose Va, UPose Vbase);
  void subC(UPose * Vbase);
  inline UPose subCed(UPose Vbase)
  { return subCed(&Vbase); };
  UPose subCed(UPose * Vbase);
  /**
  Reverse the relative pose D, so that it reverses the change from Vbase to Va
  Returns [0,0,0] o- D, or
  -D = subc([0,0,0], D); */
  void asNegC(UPose D);

public:
  /**
  x position (forward or east) */
  float_t x; //
  /**
  y position (left or north) */
  float_t y; //
  /**
  heading (Theta) zero in x direction and positive towards y.
  That is right hand coordinates with z pointing up. */
  float_t h; //
};

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

/**
Class to pose information and time of pose information */
class UPoseTime : public UPose
{
public:
  /**
  Constructor */
  UPoseTime();
  /**
  Destructor */
  virtual ~UPoseTime() {};
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "posetime";  };
  /**
  Clear to zero. */
  inline virtual void clear()
  {
    x = 0.0;
    y = 0.0;
    h = 0.0;
    t.clear();
  };
  /**
  Constructor with initial value */
  UPoseTime(float_t ix, float_t iy, float_t ih, UTime it)
  {
    setPt(ix, iy, ih, it);
  }
  /**
  Constructor with initial value */
  UPoseTime(UPose pose, UTime it)
  {
    setPt(pose, it);
  }
  /**
  Set value */
  inline void setPt(float_t ix, float_t iy, float_t ih, UTime it)
  {
    x = ix;
    y = iy;
    h = ih;
    t = it;
  }
  /**
  Set value */
  inline void setPt(UPose pose, UTime it)
  {
    x = pose.x;
    y = pose.y;
    h = pose.h;
    t = it;
  }
  /**
  Make interpolation between this pose
  and the 'otherPose', for 'atTime'.
  The method uses direct interpolation for small movements
  or small heading changes, and a slightly better method
  for larger movements, bu not suited for more
  than approx 45 deg heading change.
  If this and 'otherPose' has same time, this pose is returned.
  if 'atTime' is outside interval, pose is extrapolated! */
  UPose getPoseAtTime(UPoseTime otherPose, UTime atTime);
  /**
  Get just pose part */
  UPose getPose()
  {
    UPose result(x, y, h);
    return result;
  };
  /**
  Print value, preceded by 'prestring' */
  void fprint(FILE * fd, const char * preString);
  /**
  Print status for this structure */
  virtual void snprint(const char * preString, char * buff, const int buffCnt);
  /**
   * Set a UPose time from a source pose. This sets the x,y,h only, the time part is untouched. */
  inline UPoseTime operator= (UPose source)
  {
    x = source.x; y = source.y; h = source.h;
    return *this;
  };

public:
  /**
  Time valid for pose */
  UTime t;
};

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////

/**
Class to pose information and time of pose information */
class UPoseTVQ : public UPoseTime
{
public:
    /**
  Constructor */
  UPoseTVQ();
  /**
  Constructor with initial value */
  UPoseTVQ(UPose pose, UTime t, float_t v, float_t q);
  /**
  Constructor with initial value */
  UPoseTVQ(float_t x, float_t y, float_t h, UTime t, float_t v, float_t q);
  /**
  Destructor */
  virtual ~UPoseTVQ()
  {};
  /**
  Get (end) type of this structure */
  virtual const char * getDataType()
  { return "posetvq";  };
  /**
  Clear to zero. */
  inline virtual void clear()
  {
    x = 0.0;
    y = 0.0;
    h = 0.0;
    t.clear();
    vel = 0.0;
    q = 0.0;
  };
  /**
  Subtract 2 poses to get a delta pose from ref to base.
  i.e. if P1 and P2 is two poses, and
  deltaPose Pd = P2 - P1, then P2 is at
  position Pd in local P1 coordinates, and
  P2 = P1 + Pd */
  UPoseTVQ operator-(UPoseV pRef);
  UPose operator-(UPose pRef)
  { return subCed(&pRef); };
  UPose operator-(UPoseTime pRef)
  { return subCed(&pRef); };
  UPose operator-(UPoseTVQ pRef)
  { return subCed(&pRef); };
  /**
  Add a delta pose to a base pose to get a new pose.
  The delta pose must be in base pose perspective.
  i.e. To get from a reference position P1 to a new position
  P2 after a movement of 'pDelta' */
  UPoseTVQ operator+(UPoseV pDelta);
  UPose operator+(UPoseTVQ pDelta)
  { return addCed(&pDelta); };
  UPose operator+(UPoseTime pDelta)
  { return addCed(&pDelta); };
  /**
  Assign from base pose */
  inline virtual UPoseTVQ operator= (UPose source)
  {
    x = source.x;
    y = source.y;
    h = source.h;
    return *this;
  };
  /**
  Add a delta pose to a base pose to get a new pose.
  The delta pose must be in base pose perspective.
  i.e. To get from a reference position P1 to a new position
  P2 after a movement of 'pDelta' */
  UPoseTVQ operator+(UPose pDelta);
  /**
  Set all variables from float_ts */
  inline void set(float_t ix, float_t iy, float_t ih, UTime time, float_t iv, float_t qual)
  {
    x = ix;
    y = iy;
    h = ih;
    t = time;
    vel = iv;
    q = qual;
  };
  /**
  Set all variables from pose and velocity */
  inline void set(UPose pose, UTime time, float_t iv, float_t qual)
  {
    x = pose.x;
    y = pose.y;
    h = pose.h;
    t = time;
    vel = iv;
    q = qual;
  };
  /**
  Set all variables from other pose */
  inline void set(UPoseTVQ * pose)
  {
    x = pose->x;
    y = pose->y;
    h = pose->h;
    t = pose->t;
    vel = pose->vel;
    q = pose->q;
  };
  /**
  Set all variables from other pose */
  inline void set(UPoseTime * pose)
  {
    x = pose->x;
    y = pose->y;
    h = pose->h;
    t = pose->t;
    vel = 0.0;
    q = 0.0;
  };
  /**
  Set all variables from other pose */
  void set(UPoseV * pose);
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
  Get the UPose part only */
  UPoseV getPoseV();
  /**
  Get the UPose part only */
  inline UPoseTime getPoseTime()
  {
    UPoseTime result(x, y, h, t);
    return result;
  };
  /**
  Set velocity */
  inline void setVel(float_t velocity)
  { vel = velocity; };
  /**
  Print tatus to string.
  Deprecated call format - use snprint(...) */
  inline void print(const char * prestring, char * buff, const int buffCnt)
  { snprint(prestring, buff, buffCnt); };
  /**
  Print status for this structure */
  virtual void snprint(const char * preString, char * buff, const int buffCnt);
  /**
  Print tatus to console */
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
    /**
     * Pose quality - this may be dependent on use, but basically a float_t value associated with pose */
    float_t q;
};


//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////


#endif
