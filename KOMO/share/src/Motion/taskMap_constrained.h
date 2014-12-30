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


#ifndef _MT_taskMap_constrained_h
#define _MT_taskMap_constrained_h

#include "motion.h"

//===========================================================================

struct CollisionConstraint:public TaskMap {
  double margin;

  CollisionConstraint(double _margin=.1):margin(_margin){ constraint=true; }

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

//===========================================================================

struct LimitsConstraint:public TaskMap {
  double margin;
  arr limits;

  LimitsConstraint():margin(.05){ constraint=true; }

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

//===========================================================================

struct PairCollisionConstraint:public TaskMap {
  int i;       ///< which shapes does it refer to?
  int j;       ///< which shapes does it refer to?
  double margin;

  PairCollisionConstraint(const ors::KinematicWorld& G, const char* iShapeName, const char* jShapeName):
    i(G.getShapeByName(iShapeName)->index),
    j(G.getShapeByName(jShapeName)->index),
    margin(.02)
  {
    constraint=true;
  }

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

//===========================================================================

struct PlaneConstraint:public TaskMap {
  int i;       ///< which shapes does it refer to?
  arr planeParams;  ///< parameters of the variable (e.g., liner coefficients, limits, etc)

  PlaneConstraint(const ors::KinematicWorld& G, const char* iShapeName, const arr& _planeParams):
    i(G.getShapeByName(iShapeName)->index), planeParams(_planeParams){ constraint=true; }

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

//===========================================================================

//this is NOT a constraint -- it turns a constraint into stickiness
struct ConstraintStickiness:public TaskMap {
  TaskMap& map;
  ConstraintStickiness(TaskMap& _map):map(_map){ constraint=false; }

  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G);
  virtual uint dim_phi(const ors::KinematicWorld& G){ return 1; }
};

//===========================================================================

#endif
