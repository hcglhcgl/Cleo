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


#include "ulibpose.h"
#include "ulibposev.h"
#include "utime.h"

////////////////////////////////////////////

UPose::UPose()
{
}


////////////////////////////////////////////

UPose::UPose(float_t ix, float_t iy, float_t ih)
{
  x = ix;
  y = iy;
  h = ih;
}

////////////////////////////////////////////

UPose::UPose(UPose * source)
{
  *this = *source;
}

/////////////////////////////////////////////////

void UPose::set(float_t ix, float_t iy, float_t ih)
{
  x = ix;
  y = iy;
  h = ih;
}

/////////////////////////////////////////////////

void UPose::add(float_t ix, float_t iy, float_t ih)
{
  x += ix;
  y += iy;
  h  = limitToPi(h + ih);
}

/////////////////////////////////////////////////

void UPose::fprint(FILE * fd, const char * prestring)
{ // prints the 2D position and heading only
  // assumes indoor coordinate system in meters
  fprintf(fd, "%s (x,y)=(%6.2f,%6.2f) meter %6.1f deg\n",
      prestring, x, y, getHeadingDeg());
}

/////////////////////////////////////////////////

void UPose::snprint(const char * prestring, char * s, int bufLng)
{ // prints the 2D position and heading only
  // assumes indoor sized coordinate system in meters
  snprintf(s, bufLng, "%s (x,y)=(%.2f,%.2f) meter %.1f deg",
           prestring, x, y, getHeadingDeg());
}

/////////////////////////////////////////////////

float_t UPose::getHeadingDeg()
{ // returns heading in 0-360 degrees range
  float_t result;
  //
  result = limitToPi(h);
  result *= 180.0 / M_PI;
  if (result < 0.0)
    result += 360.0;
  //
  return result;
}

///////////////////////////////////
/*  Va = [xa, ya, ha] Vbase = [xb, yb , hb] D = [x, y, h].
    xa = xb + x cos(hb) - y sin(hb);
    ya = yb + x sin(hb) + y cos(hb);
    ha = hb + h;
    Returns the new pose Va.
    (from Lu and Milios (1997)) */
void UPose::asAddC(UPose Vbase, UPose D)
{
  float_t shb, chb;
  //
  shb = sin(Vbase.h);
  chb = cos(Vbase.h);
  x = Vbase.x + D.x * chb - D.y * shb;
  y = Vbase.y + D.x * shb + D.y * chb;
  h = limitToPi(Vbase.h + D.h);
}

///////////////////////////////////////

UPose UPose::addCed(UPose * D) 
{
  UPose result;
  result.asAddC(*this, *D);
  return result;
}

///////////////////////////////////////

void UPose::add(float_t dist, float_t headingChange)
{
  x += dist * cos(h + headingChange / 2.0);
  y += dist * sin(h + headingChange / 2.0);
  h = limitToPi(h + headingChange);
}

///////////////////////////////////////
/*
  Inverse compounding operation calculating the
  pose change from Vbase to Va, i.e. D = Va o- Vb.
  Va = [xa, ya, ha] Vbase = [xb, yb , hb] D = [x, y, h].
  x = (xa - xb)cos(hb) + (ya - yb)sin(hb);
  y = -(xa - xb)sin(hb) + (ya - yb)cos(hb);
  h = ha - hb;
  Returns the inverse compound pose.
  (from Lu and Milios (1997)) */

void UPose::asSubC(UPose Va, UPose Vbase)
{
  float_t shb, chb;
  //
  shb = sin(Vbase.h);
  chb = cos(Vbase.h);
  x =   (Va.x - Vbase.x) * chb + (Va.y - Vbase.y) * shb;
  y = - (Va.x - Vbase.x) * shb + (Va.y - Vbase.y) * chb;
  h = limitToPi(Va.h - Vbase.h);
}

//////////////////////////////////////////////

void UPose::subC(UPose * Vbase)
{
  asSubC(*this, Vbase);
}

//////////////////////////////////////////////

UPose UPose::subCed(UPose * Vbase)
{
  UPose result;
  result.asSubC(*this, *Vbase);
  return result;
}

//////////////////////////////////////////////

void UPose::asNegC(UPose D)
{
  UPose V0(0.0, 0.0, 0.0);
  asSubC(V0, D);
}

//////////////////////////////////////////////

UPose UPose::neg()
{
  UPose result;
  result.asNegC(*this);
  return result;
}


//////////////////////////////////////////////

U2Dpos UPose::getMapToPose(U2Dpos mapPos)
{
  U2Dpos result;
  float_t ch, sh;
  float_t lx, ly; // local coordinates
  //
  ch = cos(h);
  sh = sin(h);
  lx = mapPos.x - x;
  ly = mapPos.y - y;
  //
  result.x =  ch * lx + sh * ly;
  result.y = -sh * lx + ch * ly;
  //
  return result;
}

//////////////////////////////////////////////

UPose UPose::getMapToPosePose(UPose * mapPose)
{
  UPose result;
  float_t ch, sh;
  float_t lx, ly; // local coordinates
  //
  ch = cos(h);
  sh = sin(h);
  lx = mapPose->x - x;
  ly = mapPose->y - y;
  //
  result.x =  ch * lx + sh * ly;
  result.y = -sh * lx + ch * ly;
  result.h = limitToPi(mapPose->h - h);
  return result;
}

///////////////////////////////////////////

UPose UPose::getPoseToMapPose(UPose poseLocal)
{
  UPose result;
  double ch, sh;
  //
  ch = cos(h);
  sh = sin(h);
  //
  result.x =  ch * poseLocal.x - sh * poseLocal.y + x;
  result.y =  sh * poseLocal.x + ch * poseLocal.y + y;
  result.h = limitToPi(poseLocal.h + h);
  return result;
}


//////////////////////////////////////////////

U2Dpos UPose::getPoseToMap(U2Dpos posePos)
{
   U2Dpos result;
   float_t ch, sh;
   //
   ch = cos(h);
   sh = sin(h);
   //
   result.x =  ch * posePos.x - sh * posePos.y + x;
   result.y =  sh * posePos.x + ch * posePos.y + y;
   //
   return result;
}

//////////////////////////////////////////////



///////////////////////////////////////////

UPose UPose::operator= (UPoseTime source)
{
  x = source.x; y = source.y; h = source.h;
  return *this;
}

//////////////////////////////////////////////

UPose UPose::operator= (UPoseV source)
{
  x = source.x; y = source.y; h = source.h;
  return *this;
}

/////////////////////////////////////////////////

float_t UPose::getDistToPoseLineSigned(const float_t Px, const float_t Py)
{
  float_t A = -sin(h);
  float_t B = cos(h);
  float_t C = -A * x - B * y;
  float_t d = hypot(A,B);
  //
  if (d > 1e-50)
  { // normalize line
    A /= d;
    B /= d;
    C /= d;
  }
  // evaluete distance
  d = A * Px + B * Py + C; // sqrt(sqr(lA) + sqr(lB)) == 1;
  return d;
}

/////////////////////////////////////////////////


////////////////////////////////

UPose UPose::operator-(UPoseV pRef)
{
  return subCed(&pRef);
}

////////////////////////////////

UPose UPose::operator+(UPoseV pDelta)
{
  return addCed(&pDelta);
}

////////////////////////////////

UPose UPose::operator-(UPoseTime pRef)
{
  return subCed(&pRef);
}

////////////////////////////////

UPose UPose::operator+(UPoseTime pDelta)
{
  return addCed(&pDelta);
}



/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////

UPoseTime::UPoseTime()
{
}

///////////////////////////////////////////////////////////////

UPose UPoseTime::getPoseAtTime(UPoseTime otherPose, UTime atTime)
{
  UPose result;
  UPose pv;
  float_t dt;
  float_t part, dh, dx, dy, ds;
  bool isOK;
  //
  dt = (otherPose.t - t);
  isOK = fabsf(dt) > 1e-6;
  if (isOK)
  {
    part = (atTime - t)/dt;
    dh = limitToPi(otherPose.h - h);
    dx = otherPose.x - x;
    dy = otherPose.y - y;
    ds = hypot(dx, dy);
    // direct position interpolation (as if same heading or
    // heading change only)
    result.x = x + part * dx;
    result.y = y + part * dy;
    result.h = limitToPi(h + part * dh);
    // if both movement and turn, so do a little more
    if ((fabsf(dh) > (1.0 * M_PI / 180.0)) and (ds > 0.01))
    { // combined move - follow curve
      // get end position if followed start heading
      pv.x = x + part * ds * cos(h);
      pv.y = y + part * ds * sin(h);
      // interpolate between this and other
      result.x = pv.x + part * (result.x - pv.x);
      result.y = pv.y + part * (result.y - pv.y);
    }
  }
  else
    // poses is assumed to be identical, so return this pose
    result = this->getPose();
  return result;
}

/////////////////////////////////////////////////////

void UPoseTime::fprint(FILE * fd, const char * preString)
{
  const int MSL = 100;
  char s[MSL];
  const int MTL = 30;
  char st[MTL];
  //
  t.getTimeAsString(st, true);
  snprintf(s, MSL, "%s %s", preString, st);
  UPose::fprint(stdout, s);
}

/////////////////////////////////////////////////////

void UPoseTime::snprint(const char * preString, char * buff, const int buffCnt)
{
  const int MSL = 100;
  char s[MSL];
  const int MTL = 30;
  char st[MTL];
  //
  t.getTimeAsString(st, true);
  snprintf(s, MSL, "%s %s", preString, st);
  UPose::snprint(s, buff, buffCnt);
}

/////////////////////////////////////////////////////

