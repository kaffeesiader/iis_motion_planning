#pragma once

#include "motion.h"

struct TransitionTaskMap:TaskMap {
  double velCoeff, accCoeff; ///< coefficients to blend between velocity and acceleration penalization
  arr H_rate_diag; ///< duration of single step
  TransitionTaskMap(const ors::KinematicWorld& G);
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau);
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G){ HALT("can only be of higher order"); }
  virtual uint dim_phi(const ors::KinematicWorld& G){ return G.getJointStateDimension(); }
};
