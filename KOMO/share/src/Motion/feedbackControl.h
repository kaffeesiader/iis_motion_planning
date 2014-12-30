/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


#pragma once

#include "motion.h"
#include "taskMap_default.h"

/**
 * @file
 * With the feedback control we can define motions for operation space control.
 *
 * We simply define a set of motions via PDtasks/ConstraintForceTask and run
 * them.
 */


//===========================================================================
/**
 * A PDtask defines a motion in operational space.
 */
struct PDtask{
  TaskMap& map;
  MT::String name;
  bool active;
  double prec;

  /// @{ @name Immediate (next step) desired target reference
  arr y_ref; ///< position reference
  arr v_ref; ///< velocity reference
  /// @}

  /// @{ @name Parameters of the PD controller or attractor dynamics
  double Pgain, Dgain;
  /// @}

  /// @{ @name The observations when LAST getDesiredAcceleration was called
  /// Use carefully! (in online mode only)
  arr y, v;
  /// @}

  PDtask(TaskMap* map) : map(*map), active(true), prec(0.), Pgain(0.), Dgain(0.) {}

  void setTarget(const arr& yref, const arr& vref=NoArr);
  void setGains(double Pgain, double Dgain);
  void setGainsAsNatural(double decayTime, double dampingRatio); ///< the decayTime is the to decay to 10% of the initial offset/error

  arr getDesiredAcceleration(const arr& y, const arr& ydot);
};

//===========================================================================

struct ConstraintForceTask{
  TaskMap& map;
  MT::String name;
  bool active;

  double desiredForce;
  PDtask desiredApproach;

  ConstraintForceTask(TaskMap* m):map(*m), active(true), desiredForce(0.), desiredApproach(m){}

  void updateConstraintControl(const arr& g, const double& lambda_desired);
};

//===========================================================================

/**
 * FeedbackMotionControl contains all individual motions/PDtasks.
 */
struct FeedbackMotionControl : MotionProblem {
  MT::Array<PDtask*> tasks;
  MT::Array<ConstraintForceTask*> forceTasks;
  PDtask qitselfPD;

  FeedbackMotionControl(ors::KinematicWorld& _world, bool useSwift=true);

  /// @{ @name adding tasks
  PDtask* addPDTask(const char* name, double decayTime, double dampingRatio, TaskMap *map);
  PDtask* addPDTask(const char* name,
                    double decayTime, double dampingRatio,
                    DefaultTaskMapType type,
                    const char* iShapeName=NULL, const ors::Vector& ivec=NoVector,
                    const char* jShapeName=NULL, const ors::Vector& jvec=NoVector,
                    const arr& params=NoArr);
  ConstraintForceTask* addConstraintForceTask(const char* name, TaskMap *map);
  /// @}

  void getCostCoeffs(arr& c, arr& J); ///< the general (`big') task vector and its Jacobian
  arr getDesiredConstraintForces(); ///< J^T lambda^*
  arr operationalSpaceControl();
  void updateConstraintControllers();
  void reportCurrentState();
};

//===========================================================================
