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
#ifndef UPOSE2POSE_H
#define UPOSE2POSE_H

#include "ulibposev.h"

/**
Functions to get from one pose to another

@author Christian Andersen
*/
class UPose2pose : public UPoseV
{
public:
  /**
  Constructor */
  UPose2pose();
  /**
  * Constructor with initial value
  * \param ix is destination x position (forward m)
  * \param iy is destination y position (left m)
  * \param ih is destination heading relative to current heading (radians)
  * \param iv is desired end velocity */
  UPose2pose(float_t ix, float_t iy, float_t ih, float_t iv);
  /**
  Destructor */
  ~UPose2pose();
  /** manoeuvre straight distance (m) */
  float straightDist = 0;
  /** manoeuvre first turn radius (m) */
  float radius1 = 0;
  /** manoeuvre last turn radius (m) */
  float radius2 = 0;
  /** turn angle for initial turn (radians) - positive left (CCV) */
  float turnArc1 = 0;
  /** turn angle for final turn (radians) - positive left (CCV) */
  float turnArc2 = 0;
  /** initial distance to break to first turn - to avoid exceed turn acceleration (m) */
  float initialBreak = 0;
  /** final distance to break to get to final velocity (m) */
  float finalBreak = 0;
  /** straight velocity - that satisfy the acceleration limits (m/s) */
  float straightVel = 0;
  /** turn mode 
   * 0 = left straight left,
   * 1 = left straight right,
   * 2 = right straight left,
   * 3 = right straight right */
  int mode = 0;
  /**
   * calculated movement distance in an ALA (angle-line-angle) manoeuvre */
  inline float movementDistance()
  {
    return straightDist + radius1*turnArc1 + radius2*turnArc2;
  }
  /**
  Assign from base pose */
  inline UPose2pose operator= (UPose source)
  {
    x = source.x;
    y = source.y;
    h = source.h;
    return *this;
  };
  /**
  Assign from base pose */
  inline UPose2pose operator= (UPoseV source)
  {
    x = source.x;
    y = source.y;
    h = source.h;
    vel = source.vel;
    return *this;
  };
  /**
   * Do the manoeuvre calculations
   * \param acc is the maximum linear and turn acceleration [m/s^2],
   * \param maxVel is the desired linear velocity - if acceleration allows
   * \param minimumTurnRadius default is 5cm 
   * \retruns true if manoeuver is possible */
  bool calculateALA(float maxVel, float acc = 1.0, float minimumTurnRadius = 0.05);
  /**
   * print calculated manoeuvre to console */
  void printMan();
  
protected:
  /**
  Calculate a pose-to-pose manoeuvre starting
  in a right turn, then a straight line and then a left turn.
  If not possible, or not possible with current speed and
  acc limits, then return false.
  The end velocity is in the end poseV, the initial velocity is in
  'initVel'. on exit, the 5 result values are set (if not NULL or
  a solution is not found).
  Returns true if successful. */
  bool get2RightLineLeft(float_t initVel, float_t maxAcc, float_t maxTurnAcc,
                    float_t * radius1, float_t * arc1,
                    float_t * dist,
                    float_t * radius2, float_t * arc2,
                    FILE * logFp);
  /**
  Get the needed manoeuvre to get to this position, given this initial
  velocity. The solution consists of 2 arcs and a straight line in-between.
  There may be an initial straight line to break to an allowed speed for the turn. */
  bool get2ViaBreakRightLineLeft(float_t initVel,
                                float_t maxAcc, float_t maxTurnAcc, float_t minTurnRad,
                                float_t * initialBreakDist, float_t * toVel,
                                float_t * radius1, float_t * arc1,
                                float_t * dist,
                                float_t * radius2, float_t * arc2,
                                float_t * finalBreak);
  /**
  Calculate a pose-to-pose manoeuvre starting
  in a right turn, then a straight line and then a right turn.
  If not possible, or not possible with current speed and
  acc limits, then return false.
  The desired end velocity is in the end poseV, the initial velocity is in
  'initVel'. on exit, the 5 result values are set (if not NULL or
  a solution is not found).
  Returns true if successful. */
  bool get2RightLineRight(float_t initVel, float_t maxAcc, float_t maxTurnAcc,
                    float_t * radius1, float_t * arc1,
                    float_t * dist,
                    float_t * radius2, float_t * arc2,
                    FILE * logFp);
  /**
  Calculate a pose-to-pose manoeuvre starting
  in a right turn, then a straight line and then a right turn.
  An initial break or acceleration for distance 'initialBreakDist',
  may be needed (to turn velocity 'toVel'.
  A final break may be needed too for 'finalBreak' distance.
  The end velocity may be lower than specified! (but not faster)
  If not possible, or not possible with current speed and
  acc limits, then return false.
  The desired end velocity is in the end poseV, the initial velocity is in
  'initVel'. on exit, the 8 result values are set (if not NULL or
  no solution found).
  Returns true if successful. */
  bool get2ViaBreakRightLineRight(float_t initVel,
                    float_t maxAcc, float_t maxTurnAcc, float_t minTurnRad,
                    float_t * initialBreakDist, float_t * toVel,
                    float_t * radius1, float_t * arc1,
                    float_t * dist,
                    float_t * radius2, float_t * arc2,
                    float_t * finalBreak);
  /**
  Same as get2RightLineRight(...), just with mirrored values */
  bool get2LeftLineLeft(float_t initVel, float_t maxAcc, float_t maxTurnAcc,
                    float_t * radius1, float_t * arc1,
                    float_t * dist,
                    float_t * radius2, float_t * arc2,
                    FILE * logFp);
  /**
  Same as get2ViaBreakRightLineRight(...), just with mirrored values */
  bool get2ViaBreakLeftLineLeft(float_t initVel,
                    float_t maxAcc, float_t maxTurnAcc, float_t minTurnRad,
                    float_t * breakDist, float_t * toVel,
                    float_t * radius1, float_t * arc1,
                    float_t * dist,
                    float_t * radius2, float_t * arc2, float_t * finalBreak);
  /**
  Same as get2RightLineLeft(...), just with mirrored values */
  bool get2LeftLineRight(float_t initVel, float_t maxAcc, float_t maxTurnAcc,
                        float_t * radius1, float_t * arc1,
                        float_t * dist,
                        float_t * radius2, float_t * arc2,
                        FILE * logFp);
  bool get2ViaBreakLeftLineRight(float_t initVel, float_t maxAcc, float_t maxTurnAcc, float_t minTurnRad,
                        float_t * breakDist, float_t * toVel,
                        float_t * radius1, float_t * arc1,
                        float_t * dist,
                        float_t * radius2, float_t * arc2,
                        float_t * finalBreak);
  /**
  Get the best solution to destination pose.
  Uses at maximum two arcs and one line inbetween.
  Starts at 0,0 with heading 0 and speed 'initVel', and gets to
  this.x .y .h, aiming for the end velocity this.vel.
  If not possible, then ends in a smaller turnradius then
  needed for a this.vel speed.
  If 'logFp' != NULL, then some calculations may be written in logfile.
  Returns true if a solution is found.
  Fails if initial velocity is too high. */
  bool get2hereALA(int * manType,
                    float_t initVel, float_t maxAcc, float_t maxTurnAcc,
                    float_t * radius1,
                    float_t * arc1,
                    float_t * dist,
                    float_t * radius2,
                    float_t * arc2, FILE * logFp);
  /**
  Get the best solution to destination pose.
  Uses at maximum one initial break or accelerate line, two arcs and one line inbetween.
  Starts at 0,0 with heading 0 and speed 'initVel', and gets to
  this.x .y .h, aiming for the end velocity this.vel.
  If not possible, then ends in a smaller turnradius than
  needed for a this.vel speed.
  Returns true if a solution is found.
  Fails if initial velocity is too high. */
  bool get2hereLALA(int * manType,
                    float_t initVel, float_t maxAcc, float_t maxTurnAcc, float_t minTurnRad,
                    float_t * breakDist, float_t * breakToVel,
                    float_t * radius1,
                    float_t * arc1,
                    float_t * dist,
                    float_t * radius2,
                    float_t * arc2, 
                    float_t * finalBreak);
  /**
   * Get a solution to get to the line defined by this pose, estimates driveon behavoiur
   * This function covers situations that will start with a left turn.
   * \param turnRad is the assumed turn radius
   * \param turn1 is the initial turn - positive is to the left
   * \param direct is the (potential) straight part
   * \param turn2 is the end turn - positive is left
   * \returns the pose (on the line), where the manoeuvre is (assumed to be) finished.  */
  UPose get2lineStartLeft(float_t turnRad, 
                          float_t * turn1, 
                          float_t * direct, 
                          float_t * turn2);
  /**
   * Get a solution to get to the line defined by this pose, estimates drive behavoiur.
   * The solution is intended for avoid obstacle collision collision, and to be implemented as one 'driveon' command.
   * The difference to real drive is untested, and will probably works best for
   * an ackermann steered robot, that sets a turning limit not much narrower than the calculated turn radius.
   * Turn radius will be calculated as 'pi/2 * gA / gD' (for gA=2 and gD=0.75 turn radius is about 4m)
   * \param turnRad used turn radius
   * \param turn1 is the initial turn - positive is to the left
   * \param direct is the (potential) straight part
   * \param turn2 is the end turn - positive is left
   * \returns the pose (on the line), where the manoeuvre is (assumed to be) finished.  */
  UPose get2line(float_t turnRad, 
                 float_t * turn1, 
                 float_t * direct, 
                 float_t * turn2);

};

#endif
