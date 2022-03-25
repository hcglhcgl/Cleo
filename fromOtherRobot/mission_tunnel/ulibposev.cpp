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
#include "ulibposev.h"

UPoseV::UPoseV()
 : UPose()
{
  vel = 0.0;
}

UPoseV::UPoseV(float_t x, float_t y, float_t h, float_t v)
  : UPose(x, y, h)
{
  vel = v;
}

////////////////////////////////////

UPoseV::~UPoseV()
{
}

////////////////////////////////////

void UPoseV::fprint(FILE * fd, const char * prestring)
{ // prints the 2D position and heading only
  // assumes indoor coordinate system in meters
  fprintf(fd, "%s (x,y)=(%.2f,%.2f)m, %.3frad, %.1fdeg, %.2fm/s\n",
         prestring, x, y, h, getHeadingDeg(), getVel());
}

////////////////////////////////////

void UPoseV::snprint(const char * prestring, char * buff, const int buffCnt)
{ // prints the 2D position and heading only
  // assumes indoor coordinate system in meters
  snprintf(buff, buffCnt, "%s (x,y)=(%.2f,%.2f)m, %.3frad, %.1fdeg, %.2fm/s\n",
          prestring, x, y, h, getHeadingDeg(), getVel());
}

//////////////////////////////////////

UPoseV UPoseV::operator-(UPoseV pRef)
{
  UPoseV result;
  //
  result = subCed(pRef);
  result.setVel(vel - pRef.getVel());
  //
  return result;
}

//////////////////////////////////////

UPoseV UPoseV::operator+(UPoseV delta)
{
  UPoseV result;
  //
  result = addCed(delta);
  result.setVel(vel + delta.getVel());
  //
  return result;
}

//////////////////////////////////

UPoseV UPoseV::operator+(UPose delta)
{
  UPoseV result;
  result = addCed(delta);
  return result;
}
