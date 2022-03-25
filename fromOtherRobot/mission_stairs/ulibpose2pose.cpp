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
#include "ulibpose2pose.h"
#include "ulib2dline.h"

//////////////////////////////////////////////////

UPose2pose::UPose2pose()
 : UPoseV()
{
}

//////////////////////////////////////////////////

UPose2pose::UPose2pose(float_t ix, float_t iy, float_t ih, float_t iv)
  : UPoseV(ix, iy, ih, iv)
{
}

//////////////////////////////////////////////////

UPose2pose::~UPose2pose()
{
}


void UPose2pose::printMan()
{
  printf("# ---------- manoeuvre ----------------\n");
  switch(mode)
  {
    case 0: printf("#mode %d, left-straight-left\n", mode); break;
    case 1: printf("#mode %d, left-straight-right\n", mode); break;
    case 2: printf("#mode %d, right-straight-left\n", mode); break;
    case 3: printf("#mode %d, right-straight-right\n", mode); break;
  }
  printf("# going to %g, %g, %g\n", x, y, h * 180/M_PI);
  printf("# initial  dist =%5.3fm to velocity %5.3f m/s\n", initialBreak, straightVel);
  printf("# turn 1 radius =%5.3fm, turn=%5.1f deg\n", radius1, turnArc1 * 180/M_PI);
  printf("# straight path =%5.3fm\n", straightDist);
  printf("# turn 2 radius =%5.3fm, turn=%5.1f deg\n", radius2, turnArc2 * 180/M_PI);
  printf("# final    dist =%5.3fm to velocity %5.3f m/s\n", finalBreak, vel);
  printf("# acc turn = %.3f m/s^2\n", sqr(straightVel)/radius1);
}


bool UPose2pose::calculateALA(float maxVel, float acc, 
                              float minimumTurnRadius)
{
  float startVel = maxVel;
  int mode = 0;
  // result variables
  bool isOK = get2hereLALA(&mode, startVel, acc, acc, minimumTurnRadius, 
                           &initialBreak, &straightVel, 
                           &radius1, &turnArc1, 
                           &straightDist, 
                           &radius2, &turnArc2, 
                           &finalBreak);
  if (isOK)
  { // a solution is found - and more that 3cm away.
    // set turn signs (left is positive)
    switch(mode)
    { // case 0: left straight left
      case 1: // left straight right
        turnArc2 *= -1.0;
        break;
      case 2: // right straight left
        turnArc1 *= -1.0;
        break;
      case 3: // right straight right
        turnArc1 *= -1.0;
        turnArc2 *= -1.0;
        break;
      default:
        break;
    }
  }
  return isOK;
}




////////////////////////////////////////////////////

bool UPose2pose::get2RightLineLeft(float_t initVel, float_t maxAcc, float_t maxTurnAcc,
                                       float_t * radius1, float_t * arc1,
                                       float_t * dist,
                                       float_t * radius2, float_t * arc2,
                                       FILE * logFp)
{ // we are in 0,0 and want to this pose
  // max speed of first angle is start speed,
  // max speed of exit arc is end speed.
  UPose c1; // center of first turn
  UPose c2; // center of second turn
  UPose t1; // first tangent point
  UPose t2; // second tangent point
  float_t r1, r2; // arc radius
  float_t dcc; // distance center to enter
  float_t dtt; // distance from tangent point to tangent point
  bool result = false;
  float_t h2;
  const float_t halfPi = M_PI / 2.0;
  float_t a, b, dce;
  const float_t MIN_SPEED = 0.2; /* m/sec */
  //float_t MAX_BREAK_ACC = 2.0 * maxAcc;
  // a = angle of line from c1 to c2
  //
  r1 = sqr(initVel) / maxTurnAcc;
  r2 = sqr(vel) / maxTurnAcc;
  h2 = h + halfPi;
  c2.x = x + cos(h2) * r2;
  c2.y = y + sin(h2) * r2;
  c1.x = 0.0;
  c1.y = - r1;
  dcc = hypot(c2.x - c1.x, c2.y - c1.y);
  result = (dcc >= (r1 + r2));
  if (not result)
  { // test if a slower exit speed may solve the problem
    // get distance from c1 to exit point
    dce = hypot(x, y - c1.y);
    // find minim radius of r2
    r2 = sqr(MIN_SPEED) / maxTurnAcc;
    result = (dce - r1) > r2;
    if (result)
    { // a solution is possible if there is time to break.
      // find angle to tangent point - direct towards exit
      // is worst case
      a = atan2(y - c1.y, x);
      // traveled distance along first arc - at maximum
      //d = r1 * (halfPi - a);
      // get largest second arc (in worst case)
      r2 = (dce - r1) / 2.0;
      //v2 = sqrt(r2 * maxTurnAcc);
      // get needed acceleration (negative is decreased speed)
/*    This is not the point to give up
      a = (sqr(v2) - sqr(initVel))/(2.0 * d);
      // true if no need to break too much
      result = (a > -MAX_BREAK_ACC);*/
    }
    if (result)
    { // there is a new c2
      c2.x = x + cos(h2) * r2;
      c2.y = y + sin(h2) * r2;
      dcc = hypot(c2.x - c1.x, c2.y - c1.y);
    }
  }
  if (not result)
  { // give a hint in r1 to what is needed
    // try a solution with r2 half of r1
    r1 = hypot(x, y) / 3.0;
    r2 = r1 / 2.0;
    c1.y = -r1;
    c2.x = x + cos(h2) * r2;
    c2.y = y + sin(h2) * r2;
    dcc = hypot(c2.x - c1.x, c2.y - c1.y);
    result = true;
    // no point in giving up here
//    result = (r1 > sqr(MIN_SPEED)/maxTurnAcc);
  }
  if (result)
  { // circles are not touching, so seems OK so far
    // get angle from x-axis to c2 to c1
    a = atan2(c1.y - c2.y, c1.x - c2.x);
    // get angle of tangent point on c1 over
    // center of c1 to center of c2, using
    // line from c1 through tangent point extended by r2 (total r1+r2) and
    // right angle from here to center to circle 2.
    b = asin((r1 + r2) / dcc);
    // angle from c1 to tangent point
    c1.h = a - b - halfPi;
    // first tangent point
    t1.x = cos(c1.h) * r1;
    t1.y = sin(c1.h) * r1 + c1.y;
    // now the angle from c2 to tangent point t2
    c2.h = c1.h + M_PI;
    // then to tangent point 2, just r1 from c2 in direction c
    t2.x = cos(c2.h) * r2 + c2.x;
    t2.y = sin(c2.h) * r2 + c2.y;
    // distance of straight part
    dtt = hypot(t2.x - t1.x, t2.y - t1.y);
    //
    if (logFp != NULL)
    {
      fprintf(logFp, "--- %.4fa, %.4fb, (%.3fx,%.3fy)c1, (%.3fx,%.3fy)t1, %.3fdtt, (%.3fx,%.3fy)t2, (%fx,%fy)c2\n",
              a, b, c1.x, c1.y, t1.x, t1.y, dtt, t2.x, t2.y, c2.x, c2.y);
    }
    //
    // return found values
    if (radius1 != NULL)
      *radius1 = r1;
    if (arc1 != NULL)
    {
      *arc1 = halfPi - c1.h;
      while (*arc1 < 0.0)
        *arc1 += 2.0 * M_PI;
      while (*arc1 > 2.0 * M_PI)
        *arc1 -= 2.0 * M_PI;
    }
    if (dist != NULL)
      *dist = dtt;
    if (radius2 != NULL)
      *radius2 = r2;
    if (arc2 != NULL)
    {
      *arc2 = h - halfPi - c2.h;
      while (*arc2 < 0.0)
        *arc2 += 2.0 * M_PI;
      while (*arc2 > 2.0 * M_PI)
        *arc2 -= 2.0 * M_PI;
    }
  }
  //
  return result;
}

/////////////////////////////////////////////////

bool UPose2pose::get2ViaBreakRightLineLeft(float_t initVel,
                                           float_t maxAcc, float_t maxTurnAcc, float_t minTurnRad,
                                           float_t * initialBreakDist, float_t * toVel,
                                           float_t * radius1, float_t * arc1,
                                           float_t * dist,
                                           float_t * radius2, float_t * arc2,
                                           float_t * finalBreak)
{ // we are in 0,0 and want to this pose (x, y, h)
  UPose c1; // center of first turn
  UPose c2; // center of second turn
  UPose t1; // first tangent point
  UPose t2; // second tangent point
  float_t r1, r2, r; // arc radius
  float_t dcc; // distance center to center
  float_t dtt; // distance from tangent point to tangent point
  bool result = false;
  //float_t h2;
  const float_t halfPi = M_PI / 2.0;
  float_t a, b, dr;
  const float_t MIN_SPEED = 0.2; /* m/sec */
  float_t x2, y2, rMaxV, rMax, rMin, v, d1, d2;
  float_t cosh2, sinh2, sinh0, cosh0;
  const float_t RADIUS_SURPLUS_MARGIN = 0.05; // 5cm away from optimal is OK
  int i;
  //
  v = fmaxf(initVel, vel);
  // needed turn radius is based on maximum speed
  rMaxV = sqr(v) / maxTurnAcc;
  // but not less  than the capable turn radius
  rMax = fmaxf(minTurnRad * 3.0, rMaxV);
  //h2 = h + halfPi;
  cosh0 = cos(h);
  sinh0 = sin(h);
  cosh2 = -sinh0;
  sinh2 = cosh0;
  // minimum is based on minimum speed and minimum turn radius
  rMin = fmaxf(minTurnRad, sqr(MIN_SPEED) / maxTurnAcc);
  r1 = rMax;
  r2 = rMin;
  // rMax -= 1e-5;
  // rMin += 1e-5;
  r = r1;
  c1.x = 0;
  i = 0;
  while (not result and i < 12)
  { // test if a slower exit speed may solve the problem
    // but not higher than target velocity or start velocity
    v = fmin(sqrt(r * maxTurnAcc), fmax(initVel, vel));
    // needed break (or acc) distance
    if (rMaxV > rMax * 0.8 and initVel > vel)
      // safer to break on a straight line then
      d1 = sqr(v - initVel)/(2.0 * maxAcc);
    else
      // no need to have straight break distance
      d1 = 0.0;
    if (v > initVel and dist == NULL)
    { // initial acceleration distance is not desired
      d1 = 0.0;
    }
    x2 = x - d1;
    if (v > vel)
    { // allow a final break distance (x2,y2) is start of final break
      d2 = sqr(v - vel)/(2.0 * maxAcc);
      x2 = x2 - d2 * cosh0;
      y2 = y - d2 * sinh0;
    }
    else
    { // no final break
      y2 = y;
      d2 = 0;
    }
    // center of first arc is on y-axis
    c1.y = - r;
    // center of second arc is in direction h2 from x2, y2
    c2.x = x2 + cosh2 * r;
    c2.y = y2 + sinh2 * r;
    // center to center distance
    dcc = hypot(c2.x - c1.x, c2.y - c1.y);
    // surplus radius margin
    dr = dcc - 2.0 * r;
    if ((dr > RADIUS_SURPLUS_MARGIN) and (r < rMax))
      // is possible, but can be better increase r2
      r2 = r;
    else if (dr > 1e-5)
    { // close and on the safe side - stop
      r1 = r;
      r2 = r;
      result = true;
      break;
    }
    else if (r > rMin)
      // not good decrease r1
      r1 = r;
    else
      // too small - no solution
      break;
    // try between the new limits
    r = (r1 + r2) / 2.0;
    i++;
  }
  if (result)
  { // circles are not touching, so seems OK so far
    // get angle from x-axis to c2 to c1
    a = atan2(c1.y - c2.y, c1.x - c2.x);
    // get angle of tangent point on c1 over
    // center of c1 to center of c2, using
    // line from c1 through tangent point extended by r2 (total r1+r2) and
    // right angle from here to center to circle 2.
    b = asin((r1 + r2) / dcc);
    // angle from c1 to tangent point
    c1.h = a - b - halfPi;
    // first tangent point
    t1.x = cos(c1.h) * r1;
    t1.y = sin(c1.h) * r1 + c1.y;
    // now the angle from c2 to tangent point t2
    c2.h = c1.h + M_PI;
    // then to tangent point 2, just r1 from c2 in direction c
    t2.x = cos(c2.h) * r2 + c2.x;
    t2.y = sin(c2.h) * r2 + c2.y;
    // distance of straight part
    dtt = hypot(t2.x - t1.x, t2.y - t1.y);
    //
    //
    // return found values
    if (radius1 != NULL)
      *radius1 = r1;
    if (arc1 != NULL)
      *arc1 = limitTo2Pi(halfPi - c1.h);
    if (dist != NULL)
      *dist = dtt;
    if (radius2 != NULL)
      *radius2 = r2;
    if (arc2 != NULL)
      *arc2 = limitTo2Pi(h - halfPi - c2.h);
    if (initialBreakDist != NULL)
      *initialBreakDist = d1;
    if (toVel != NULL)
      *toVel = v;
    if (finalBreak != NULL)
      *finalBreak = d2;
  }
  //
  return result;
}

///////////////////////////////////////////////////

bool UPose2pose::get2ViaBreakRightLineRight(float_t initVel,
           float_t maxAcc, float_t maxTurnAcc,
           float_t minTurnRad,
           float_t * initialBreakDist,
           float_t * toVel, // returned result velocity
           float_t * radius1, float_t * arc1,
           float_t * dist,
           float_t * radius2, float_t * arc2,
           float_t * finalBreak)
{ // we are in 0,0 and want to this pose (x, y, h) at 'vel' m/s
  UPose c1; // center of first turn
  UPose c2; // center of second turn
  UPose t1; // first tangent point
  UPose t2; // second tangent point
  float_t r1, r2, r; // arc radius
  //float_t dcc; // distance center to center
  float_t dtt; // distance from tangent point to tangent point
  bool result = false;
  //float_t h2;
  const float_t halfPi = M_PI / 2.0;
  float_t a, b, dr;
  const float_t MIN_SPEED = 0.2; /* m/sec */
  float_t x2, y2, rMax, rMaxV, rMin, v, d1, d2;
  float_t cosh2, sinh2, sinh0, cosh0;
  const float_t RADIUS_SURPLUS_MARGIN = 0.05; // 5cm away from optimal is OK
  float_t dc1x2, dc2x1;
  int i;
  //
  v = fmaxf(initVel, vel);
  rMaxV = sqr(v) / maxTurnAcc;
  // preferred turn radius at current speed
  rMax = fmaxf(minTurnRad * 3.0, rMaxV);
  //h2 = h - halfPi;
  cosh0 = cos(h);
  sinh0 = sin(h);
  cosh2 = sinh0;
  sinh2 = -cosh0;
  // min speed allowed turn radius - sharpest turn possible
  rMin = fmaxf(minTurnRad, sqr(MIN_SPEED) / maxTurnAcc);
  r1 = rMax;
  r2 = rMin;
  rMax -= 1e-5;
  rMin += 1e-5;
  r = r1;
  c1.x = 0;
  i = 0;
  while (not result and i < 12)
  { // test if a slower exit speed may solve the problem
    // but not higher than target velocity or start velocity
    v = fmin(sqrt(r * maxTurnAcc), fmax(initVel, vel));
    // needed initial break distance?
    if (rMaxV > rMax * 0.8 and initVel > v)
      // safer to break on a straight line then
      d1 = sqr(v - initVel)/(2.0 * maxAcc);
    else
      // no need to have straight break-acc distance
      d1 = 0.0;
    x2 = x - d1;
    // need final break?
    if (v > vel)
    { // allow a final break distance
      d2 = sqr(v - vel)/(2.0 * maxAcc);
      x2 = x2 - d2 * cosh0;
      y2 = y - d2 * sinh0;
    }
    else
    { // no final break
      y2 = y;
      d2 = 0;
    }
    // center of first arc is on y-axis
    c1.y = - r;
    // center of second arc is in direction h2 from x2, y2
    c2.x = x2 + cosh2 * r;
    c2.y = y2 + sinh2 * r;
    // center to center distance
    dc1x2 = hypot(c1.x - x2, c1.y - y2);
    dc2x1 = hypot(c2.x, c2.y);
    // surplus radius margin
    dr = fminf(dc1x2 - r, dc2x1 - r);
    // get tangent angle
    a = atan2(c1.y - c2.y, c1.x - c2.x);
    // Get arc-2 angle, if this is slightly negative (should turn almost 360 deg), then
    // there is a chance to get a good solution if radius is smaller
    b = limitToPi(a - h);
    if ((dr > RADIUS_SURPLUS_MARGIN) and (r < rMax) and (b > 0.1))
    { // is possible, but can be better increase r2
      r2 = r;
    }
    else if (dr > 1e-5 or (b <= 0.1 and b > 0.01))
    { // close and on the safe side - use this r
      r1 = r;
      r2 = r;
      result = true;
      break;
    }
    else if (r > rMin)
      // not good decrease r1
      r1 = r;
    else
      // no solution
      break;
    // try between the new limits
    r = (r1 + r2) / 2.0;
    i++;
  }
  if (result)
  { // circles are not touching, so seems OK so far
    // get angle from x-axis to c2 to c1
    a = atan2(c1.y - c2.y, c1.x - c2.x);
    // center center distance
    //dcc = hypot(c1.x - c2.x, c1.y - c2.y);
    // get angle of tangent point on c1 over
    // center of c1 to center of c2, using
    // line from c1 through tangent point extended by r2 (total r1+r2) and
    // right angle from here to center to circle 2.
    //b = asin((r1 - r2) / dcc);
    // angle from c1 to tangent point
    //c1.h = a - b - halfPi;
    c1.h = a - halfPi;
    // first tangent point
    t1.x = cos(c1.h) * r1;
    t1.y = sin(c1.h) * r1 + c1.y;
    // now the angle from c2 to tangent point t2
    c2.h = c1.h;
    // then to tangent point 2, just r1 from c2 in direction c
    t2.x = cos(c2.h) * r2 + c2.x;
    t2.y = sin(c2.h) * r2 + c2.y;
    // distance of straight part
    dtt = hypot(t2.x - t1.x, t2.y - t1.y);
    //
    //
    // return found values
    if (radius1 != NULL)
      *radius1 = r1;
    if (arc1 != NULL)
    {
      *arc1 = limitTo2Pi(halfPi - c1.h);
      // minor angle (0.0005 deg) is zero
      if (*arc1 > (2.0 * M_PI - 1e-5))
        *arc1 = 0.0;
    }
    if (dist != NULL)
      *dist = dtt;
    if (radius2 != NULL)
      *radius2 = r2;
    if (arc2 != NULL)
    {
      *arc2 = limitTo2Pi(c2.h - (h + halfPi));
      // minor angle (0.0005 deg) is zero
      if (*arc2 > (2.0 * M_PI - 1e-5))
        *arc2 = 0.0;
    }
    if (initialBreakDist != NULL)
      *initialBreakDist = d1;
    if (toVel != NULL)
      *toVel = v;
    if (finalBreak != NULL)
      *finalBreak = d2;
  }
  //
  return result;
}

///////////////////////////////////////////////////

bool UPose2pose::get2RightLineRight(float_t initVel, float_t maxAcc, float_t maxTurnAcc,
                                   float_t * radius1, float_t * arc1,
                                   float_t * dist,
                                   float_t * radius2, float_t * arc2,
                                   FILE * logFp)
{ // we are in 0,0 and want to this pose
  // max speed of first angle is start speed,
  // max speed of exit arc is end speed.
  UPose c1; // center of first turn
  UPose c2; // center of second turn
  UPose t1; // first tangent point
  UPose t2; // second tangent point
  float_t r1, r2; // arc radius
  float_t dc1e; // distance from first center to exit point
  float_t dc2s; // distance from center 2 to start point
  float_t dtt; // distance from tangent point to tangent point
  bool result = false;
  float_t h2;
  const float_t halfPi = M_PI / 2.0;
  float_t a, b, d;
  //const float_t MIN_SPEED = 0.2; // m/sec
  float_t dcc; // ceter-center distance
  // a = angle of line from c1 to c2
  //
  r1 = sqr(initVel) / maxTurnAcc;
  r2 = sqr(vel) / maxTurnAcc;
  h2 = h - halfPi;
  c2.x = x + cos(h2) * r2;
  c2.y = y + sin(h2) * r2;
  c1.x = 0.0;
  c1.y = - r1;
  dc1e = hypot(c1.x - x, c1.y - y);
  dc2s = hypot(c2.x, c2.y);
  if (dc2s <= r2)
  { // a slower exit speed will be needed, as origo is inside arc
    // The line from exit to origo has a mid-normal, that
    // crosses the exit turn center, then line in direction h2
    // crosses center too, find cross.
    // distance to exit is
    d = hypot(x, y) / 2.0;
    // angle between line at exit is
    a = h2 - atan2(-y, -x);
    // from exit to cross (hypotonuse) is turn radius
    r2 = d / cos(a) - 0.02;
    // get new center
    c2.x = x + cos(h2) * r2;
    c2.y = y + sin(h2) * r2;
    // and center - center distance
    //dcc = hypot(c2.x - c1.x, c2.y - c1.y);
    //
    // result = r2 >= sqr(MIN_SPEED)/maxTurnAcc;
  }
  if (dc1e < r1)
  { // r1 is too big, find a usable value
    // solve triangle from 0.0, to new r1, to midpoint from 0.0 to exit
    // angle to exit is the same as angle at r1 in triangle
    a = atan2(y, x);
    // distance to half-exit point
    d = hypot(y, x) / 2.0;
    // sin(a) = d/r1.x
    r1 = fabs(d/sin(a)) - 0.02;
    // new center value
    c1.y = - r1;
  }
  result = (r2 >= 0.0) and (r1 >= 0.0);
  if (result)
  { // circles cross or are separated
    // get angle og c1 to c2 line
    a = atan2(c1.y - c2.y, c1.x - c2.x);
    // get centre centre distance
    dcc = hypot(c2.x - c1.x, c2.y - c1.y);
    // get angle of tangent point on c1 over
    // center of c1 to center of c2, using
    // line from c1 through tangent point extended by r2 (total r1+r2) and
    // right angle from here to center to circle 2.
    b = asin((r1 - r2) / dcc);
    // angle from c1 to tangent point
    c1.h = limitToPi(a - b - halfPi);
    // first tangent point
    t1.x = cos(c1.h) * r1;
    t1.y = sin(c1.h) * r1 + c1.y;
    // now the angle from c2 to tangent point t2
    c2.h = c1.h;
    // then to tangent point 2, just r1 from c2 in direction c
    t2.x = cos(c2.h) * r2 + c2.x;
    t2.y = sin(c2.h) * r2 + c2.y;
    // distance of straight part
    dtt = hypot(t2.x - t1.x, t2.y - t1.y);
    //
    // log calculation - if logfile exist
    if (logFp != NULL)
    {
      fprintf(logFp, "--- %.4fa, %.4fb, (%.3fx,%.3fy)c1, (%.3fx,%.3fy)t1, %.3fdtt, (%.3fx,%.3fy)t2, (%fx,%fy)c2\n",
              a, b, c1.x, c1.y, t1.x, t1.y, dtt, t2.x, t2.y, c2.x, c2.y);
    }
/*    fprintf(stdout, "--- (%.2fx,%.2fy)c1, (%.2fx,%.2fy)t1, %.2fdtt,\n"
        "--- (%.3fx,%.3fy)c2, (%.2fx,%.2fy)t2 (RR)\n",
        c1.x, c1.y, t1.x, t1.y, dtt, c2.x, c2.y, t2.x, t2.y);*/
    //
    // return found values
    if (radius1 != NULL)
      *radius1 = r1;
    if (arc1 != NULL)
    {
      *arc1 = halfPi - c1.h;
      while (*arc1 < 0.0)
        *arc1 += 2.0 * M_PI;
      while (*arc1 > 2.0 * M_PI)
        *arc1 -= 2.0 * M_PI;
      // minor angle (0.0005 deg) is zero
      if (*arc1 > (2.0 * M_PI - 1e-5))
        *arc1 = 0.0;
    }
    if (dist != NULL)
      *dist = dtt;
    if (radius2 != NULL)
      *radius2 = r2;
    if (arc2 != NULL)
    {
      *arc2 = c2.h - (h + halfPi);
      while (*arc2 < 0.0)
        *arc2 += 2.0 * M_PI;
      while (*arc2 > 2.0 * M_PI)
        *arc2 -= 2.0 * M_PI;
      // minor angle (0.0005 deg) is zero
      if (*arc2 > (2.0 * M_PI - 1e-5))
        *arc2 = 0.0;
    }
  }
  //
  return result;
}

///////////////////////////////////////////////////////////

bool UPose2pose::get2LeftLineLeft(float_t initVel, float_t maxAcc, float_t maxTurnAcc,
                                    float_t * radius1, float_t * arc1,
                                    float_t * dist,
                                    float_t * radius2, float_t * arc2,
                                    FILE * logFp)
{
  UPose2pose pm(x, -y, -h, vel);
  return pm.get2RightLineRight(initVel, maxAcc, maxTurnAcc,
                            radius1, arc1, dist, radius2, arc2, logFp);
}

///////////////////////////////////////////////////////////

bool UPose2pose::get2ViaBreakLeftLineLeft(float_t initVel,
                                  float_t maxAcc, float_t maxTurnAcc, float_t minTurnRad,
                                  float_t * breakDist, float_t * toVel,
                                  float_t * radius1, float_t * arc1,
                                  float_t * dist,
                                  float_t * radius2, float_t * arc2, float_t * finalBreak)
{
  UPose2pose pm(x, -y, -h, vel);
  return pm.get2ViaBreakRightLineRight(initVel, maxAcc, maxTurnAcc, minTurnRad,
                               breakDist, toVel,
                               radius1, arc1,
                               dist,
                               radius2, arc2,
                               finalBreak);
}

///////////////////////////////////////////////////////////

bool UPose2pose::get2LeftLineRight(float_t initVel, float_t maxAcc, float_t maxTurnAcc,
                                  float_t * radius1, float_t * arc1,
                                  float_t * dist,
                                  float_t * radius2, float_t * arc2,
                                  FILE * logFp)
{
  UPose2pose pm(x, -y, -h, vel);
  return pm.get2RightLineLeft(initVel, maxAcc, maxTurnAcc,
                            radius1, arc1, dist, radius2, arc2, logFp);
}

//////////////////////////////////////////////////

bool UPose2pose::get2ViaBreakLeftLineRight(float_t initVel,
                                  float_t maxAcc, float_t maxTurnAcc, float_t minTurnRad,
                                  float_t * breakDist, float_t * toVel,
                                  float_t * radius1, float_t * arc1,
                                  float_t * dist,
                                  float_t * radius2, float_t * arc2, float_t * finalBreak)
{
  UPose2pose pm(x, -y, -h, vel);
  return pm.get2ViaBreakRightLineLeft(initVel, maxAcc, maxTurnAcc, minTurnRad,
                              breakDist, toVel,
                              radius1, arc1, dist, radius2, arc2, finalBreak);
}

//////////////////////////////////////////////////

bool UPose2pose::get2hereALA(int * manType,
                          float_t initVel, float_t maxAcc, float_t maxTurnAcc,
                          float_t * radius1,
                          float_t * arc1,
                          float_t * dist,
                          float_t * radius2,
                          float_t * arc2, FILE * logFp)
{
  bool result = true;
  //float_t v1 = initVel; //, v2;
  const int MSL = 4;
  bool isOK;
  int i;
  int best = -1;
  float_t minArc = M_PI * 3.0;
  float_t r1, r2, a1, a2, d, asum;
  //const float_t MIN_VEL = 0.2;
//   UPosition p3, p4; // destination crosses x-axis (using dest heading)
  //
  // limit minimum velocity
  //v1 = fmaxf(MIN_VEL, initVel);
  // try all 4 combinations, and select the shortest in time.
  for (i = 0; i < MSL; i++)
  {
    switch (i)
    {
      case 0:
        isOK = get2LeftLineLeft(initVel, maxAcc, maxTurnAcc, &r1, &a1, &d, &r2, &a2, logFp);
        break;
      case 1:
        isOK = get2LeftLineRight(initVel, maxAcc, maxTurnAcc, &r1, &a1, &d, &r2, &a2, logFp);
        break;
      case 2:
        isOK = get2RightLineLeft(initVel, maxAcc, maxTurnAcc, &r1, &a1, &d, &r2, &a2, logFp);
        break;
      case 3:
        isOK = get2RightLineRight(initVel, maxAcc, maxTurnAcc, &r1, &a1, &d, &r2, &a2, logFp);
        break;
      default:
        isOK = false;
        break;
    }
    if (isOK)
    {
      // find best score (based on angles only)
      asum = (a1 + a2);
      // angles above 180 deg count float_t
      if (a1 > M_PI)
        asum += a1;
      if (a2 > M_PI)
        asum += a2;
      if ((a1*r1 + a2*r2 + d) < minArc)
      {
        best = i;
        minArc = a1*r1 + a2*r2 + d;
        if (radius1 != NULL)
          *radius1 = r1;
        if (radius2 != NULL)
          *radius2 = r2;
        if (arc1 != NULL)
          *arc1 = a1;
        if (arc2 != NULL)
          *arc2 = a2;
        if (dist != NULL)
          *dist = d;
      }
      if (logFp != NULL)
      {
        fprintf(logFp, "using %d, %.2fm, (a1=%.3f, d=%.2fm, r2=%.2fm, a2=%.3f)\n",
              i, a1 * r1 + d + a2 * r2,
              a1, d, r2, a2);
      }
      printf("# --- using %d, dist=%.2fm, (r1=%5.3f, a1=%5.1f, d=%5.2fm, r2=%5.2fm, a2=%5.1f)\n",
             i, a1 * r1 + d + a2 * r2,
             r1, a1, d, r2, a2);
    }
  }
  result = best >= 0;
  if (result)
  {
    switch (best)
    {
      case 0: // start and ends in left turns
        break;
      case 1: // ends in a right turn
        *arc2 *= -1.0;
        break;
      case 2: // starts with a right turn
        *arc1 *= -1.0;
      case 3: // starts and ends with right turns
        *arc1 *= 1.0;
        *arc2 *= 1.0;
      default:
        break;
    }
  }
  if (manType != NULL)
    *manType = best;
  //
  return result;
}

//////////////////////////////////////////////////

bool UPose2pose::get2hereLALA(int * manType,
            float_t initVel, float_t maxAcc,
            float_t maxTurnAcc, float_t minTurnRad,
            float_t * breakDist, float_t * breakToVel,
            float_t * radius1,
            float_t * arc1,
            float_t * dist,
            float_t * radius2,
            float_t * arc2,
            float_t * finalBreak)
{
  bool result = true;
  //float_t v1 = initVel; //, v2;
  const int MSL = 4;
  bool isOK;
  int i;
  int best = -1;
  float_t minArc = 20.0;
  float_t r1, r2, a1, a2, d, asum, b, v, bFinal;
  //const float_t MIN_VEL = 0.2;
//   UPosition p3, p4; // destination crosses x-axis (using dest heading)
  //
  // limit minimum velocity
  //v1 = fmaxf(MIN_VEL, initVel);
  // try all 4 combinations, and select the shortest in time.
  for (i = 0; i < MSL; i++)
  { // desired end velocity is in poseV (vel)
    v = initVel; // calculated end velocity
    b = 0.0;
    switch (i)
    {
      case 0:
        isOK = get2ViaBreakLeftLineLeft(initVel, maxAcc,
                   maxTurnAcc, minTurnRad,
                   &b, &v, &r1, &a1, &d,
                   &r2, &a2, &bFinal);
        break;
      case 1:
        isOK = get2ViaBreakLeftLineRight(initVel, maxAcc,
                   maxTurnAcc, minTurnRad,
                   &b, &v, &r1, &a1, &d,
                   &r2, &a2, &bFinal);
        break;
      case 2:
        isOK = get2ViaBreakRightLineLeft(initVel, maxAcc,
                      maxTurnAcc, minTurnRad,
                      &b, &v, &r1, &a1, &d,
                      &r2, &a2, &bFinal);
        break;
      case 3:
        isOK = get2ViaBreakRightLineRight(initVel, maxAcc,
                      maxTurnAcc, minTurnRad,
                      &b, &v, &r1, &a1, &d,
                      &r2, &a2, &bFinal);
        break;
      default:
        isOK = false;
        break;
    }
    if (isOK)
    {
      // find best score (based on angles only)
      asum = (a1 + a2);
      // angles above 180 deg count float_t
      if (a1 > M_PI)
        asum += a1;
      if (a2 > M_PI)
        asum += a2;
      if (asum < minArc)
      {
        best = i;
        minArc = asum;
        if (radius1 != NULL)
          *radius1 = r1;
        if (radius2 != NULL)
          *radius2 = r2;
        if (arc1 != NULL)
          *arc1 = a1;
        if (arc2 != NULL)
          *arc2 = a2;
        if (dist != NULL)
          *dist = d;
        if (breakDist != NULL)
          *breakDist = b;
        if (breakToVel != NULL)
          *breakToVel = v;
        if (finalBreak != NULL)
          *finalBreak = bFinal;
      }
  /*        if (logFp != NULL)
      {
        fprintf(logFp, "using %d, %.2fm, (a1=%.3f, d=%.2fm, r2=%.2fm, a2=%.3f)\n",
                i, a1 * r1 + d + a2 * r2,
                a1, d, r2, a2);
      }*/
    }
  }
  result = best >= 0;
  if (manType != NULL)
    *manType = best;
  //
  return result;
}


////////////////////////////////////////////////////////////

UPose UPose2pose::get2line(float_t turnRad, float_t * turn1, float_t * direct, float_t * turn2)
{ // turn radius is defined from driveon parameters angle gain and distance gain
  // angle gain gA gives turn angle left new angle is to the left.
  // distance gain gD gives turn angle left, if to the right og line,
  // but no more than a turn angle from a 90 deg turn.
  // float_t turnRad = M_PI/2.0 * gA / gD;
  U2Dseg line; // exit line description
  U2Dpos c1, c2;
  bool startRight;
  UPose result;
  float_t d1, d2, t1, t2; //, tu1, tu2l, tu1r, tu2r, dil, dir;
  //
  if (fabs(h) < 1e-7)
  {
    h = 0.0;
    startRight = false;
  }
  else
  {
    line.setFromPose(x, y, h, 1.0);
    // start left turn centre
    c1.x = 0.0;
    c1.y = turnRad;
    // start right turn centre
    c2.x = 0.0;
    c2.y = - turnRad;
    // get distance to line from both
    d1 = fabs(line.distanceSigned(c1.x, c1.y));
    d2 = fabs(line.distanceSigned(c2.x, c2.y));
    //printf("Distance to %.2f,%.2f,%.4f left,right (d1=%.3f, d2=%.3f,  radius %.2fm)\n",
    //       x, y, h, d1, d2, turnRad);
    if (d1 > turnRad or d2 > turnRad)
    { // at large angles, the destination count
  /*    if (fabs(h) >= M_PI / 2.0)
        startRight = y < 0.0;
      else*/
        // at small angles the closest center count
        startRight = d2 < d1;
    }
    else
    { // the closest point in the line is more forward
      // for the start turn
      t1 = line.getPositionOnLine(c1);
      t2 = line.getPositionOnLine(c2);
      //printf("Position left,right (t1=%.3g, t2=%.3g) radius %.2fm\n",
      //       t1, t2, turnRad);
      startRight = t2 > t1;
    }
  }
  //printf("so start right (%s)\n", bool2str(startRight));
  //
  for (int i = 0; i < 2; i++)
  {
    if (startRight)
    { // mirror coordinate system
      y = -y;
      h = -h;
    }
    result = get2lineStartLeft(turnRad, turn1, direct, turn2);
    if (startRight)
    { // reverse back the result
      y = -y;
      h = -h;
      result.y = -result.y;
      result.h = -result.h;
      *turn1 = -*turn1;
      *turn2 = -*turn2;
    }
    if (fabs(*turn1) < 6.0 and fabs(*turn2) < 6.0)
      break;
    printf("*** UPose2pose::get2line mistake!  %.5fx,%.5fy,%.8fh by %.4frad,%.2fm,%.4frad\n", x,y,h, *turn1, *direct, *turn2);
    startRight = not startRight;
  }
/*  printf("p2p to %.2f,%.2f,%.4f by %.4frad,%.2fm,%.4frad\n",
         x,y,h, *turn1, *direct, *turn2);*/
  return result;
}

//////////////////////////////////////////////////

UPose UPose2pose::get2lineStartLeft(float_t turnRad,
                                    float_t * turn1,
                                    float_t * direct,
                                    float_t * turn2)
{
  UPose result; // pose at end of turn
  float_t d2l; // distance from turn centre to destination line
  U2Dseg dSeg; // destination line
  float_t hca; // half return angle
  float_t hrd; // half return distance, when crossing destination line
  float_t hrdd; // half return distance in destination direction
  float_t t;
  U2Dpos nPos; // new position when line is reached
  float_t hp = h; // positive destination heading
  // set line parameters from destination pose
  if (hp < 0.0)
    hp += M_PI;
  /// @todo error if h < ~ -M_PI/2 
  dSeg.setFromPose(x, y, hp); 
  // distance from circle centre (0,turnRad) perpendicular to line
  d2l = dSeg.distanceSigned(0.0, turnRad);
  // get also position on destination line, where
  // manoeuvre ends - if far from line
  t = dSeg.getPositionOnLine(0.0, turnRad);
  // if distance is > turn radius, then surplus distance for straight part
  if (d2l >= turnRad)
  { // first turn gets to straight part
    if (hp > M_PI / 2.0)
    {
      *turn1 = hp - M_PI / 2.0;
      // distance is the remainimg part less the last 90deg turn
      *direct = d2l - turnRad;
      // second turn is 90deg
      if (h >= 0.0)
        *turn2 = M_PI / 2.0;
      else
      { // going the other way
        *turn2 = -M_PI / 2.0;
        // finish further away
        t -= 2.0 * turnRad;
      }
    }
    else
    { // less than 90 deg turn remainimg, so
      // turn to half remainin angle
      *turn1 = hp / 2.0;
      *turn2 = *turn1;
      // straight part of distance to c1's point on target line
      // from end of first turn
      // h is tarhet heading, r is turn radius,
      // d2l is distance to target line from first turn centre
      // b := from centre c1 at half-angle to cross with target line
      // b := d2l / cos(h/2);
      // alpha is angle between line b and target line
      // alpha := Pi/2 - h - h/2;
      // y is extension of direct tangent line befor crossing target line
      // y := r * tan(h/4);
      // b-r is part of b line outside first turning circle
      // x is tangent side opposite angle alpha (crossing b at right angle)
      // x := (b - r) * tan(alpha);
      // direct := x - y;
      *direct = (d2l / cos(h/2.0) - turnRad) * tan(M_PI/2.0 - h/2.0) - turnRad * tan(h/4.0);
    }
  }
  else if (d2l <= -turnRad)
  { // destination line is (far) behind robot
    if (h > - M_PI / 2.0)
    { // first turn gets to straight part facing destination line
      *turn1 = fabs(h) - M_PI / 2.0 + M_PI;
      // second turn is 90deg
      *turn2 = -M_PI / 2.0;
      // end position advances by 2 x turn rad
      t += 2.0 * turnRad;
    }
    else
    { // heading is almost back, so turn about 90 deg
      // 90 deg clockwise, or 1.5 Pi positive, as h angle is negative
      *turn1 = h - M_PI / 2.0 + 2.0 * M_PI;
      // second turn is 90deg
      *turn2 = M_PI / 2.0;
      // end position is OK
    }
    // distance is the remainimg part
    *direct = fabs(d2l) - turnRad;
  }
  else
  { // too close for a straight part
    // and heading is positive
    *direct = 0.0;
    // correct sign is needed for distance to line
    if (h < 0.0)
    {
      dSeg.setFromPose(x, y, h);
      d2l = -d2l;
      t = -t;
    }
    // initial turn crosses destination line, so
    // so turn is to destination heading plus a bit.
    // the bit must bring the robot half the way back to the line
    hrd = (turnRad - d2l) / 2.0;
    // angle to turn passed the top point
    hca = acos((turnRad - hrd) / turnRad);
    // first turn (must be positive)
    *turn1 = limitTo2Pi(h + hca);
    // turn 2 is in the other direction
    *turn2 = -hca;
    // distance in destination direction of last turn
    hrdd = turnRad * sin(hca);
    // end pose is closest point plus the last 2 small turns
    t += hrdd * 2.0;
  }
  nPos = dSeg.getPositionOnLine(t);
  result.x = nPos.x;
  result.y = nPos.y;
  // destination heading remains
  result.h = h;
  //
  return result;
}


