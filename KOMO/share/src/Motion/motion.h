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


#ifndef _MT_motion_h
#define _MT_motion_h

#include <Ors/ors.h>
#include <Optim/optimization.h>

/* Notes
  -- transition models: kinematic, non-holonomic (vel = B u), pseudo dynamic, non-hol dynamic (acc = B u), real dynamic
  -- transition costs: vel^2 *tau, acc^2 * tau, u^2 * tau
  */


//===========================================================================
//
// defines only a map (task space), not yet the costs in this space
//

struct TaskMap {
  bool constraint;  ///< whether this is a hard constraint (implementing a constraint function g)
  uint order;       ///< 0=position, 1=vel, etc
  //Actually, the right way would be to give phi a list of $k+1$ graphs -- and retrieve the velocities/accs from that...
  virtual void phi(arr& y, arr& J, const ors::KinematicWorld& G) = 0;
  virtual void phi(arr& y, arr& J, const WorldL& G, double tau);
  virtual uint dim_phi(const ors::KinematicWorld& G) = 0; //the dimensionality of $y$

  TaskMap():constraint(false),order(0) {}
  virtual ~TaskMap() {};
};


//===========================================================================
//
// the costs in a task space
//

struct TaskCost {
  TaskMap& map;
  MT::String name;
  bool active;
  arr target, prec;  ///< target & precision over a whole trajectory
  double threshold;  ///< threshold for feasibility checks (e.g. in RRTs)
  uint dim_phi(const ors::KinematicWorld& G, uint t){ if(!active || prec.N<=t || !prec(t)) return 0; return map.dim_phi(G); }

  TaskCost(TaskMap* m):map(*m), active(true){}

  enum TaskCostInterpolationType { atTimeOnly, tillTime, fromTime };
  void setCostSpecs(uint fromTime, uint toTime,
                    const arr& _target=ARR(0.),
                    double _prec=1.);

};


//===========================================================================
//
// a motion problem description
//

/// This class allows you to DESCRIBE a motion planning problem, nothing more
struct MotionProblem { //TODO: rename MotionPlanningProblem
  //engines to compute things
  ors::KinematicWorld& world;
  bool useSwift;
  
  //******* the following three sections are parameters that define the problem

  //-- task cost descriptions
  MT::Array<TaskCost*> taskCosts;
  bool makeContactsAttractive;
  
  //-- transition cost descriptions //TODO: should become a task map just like any other
  enum TransitionType { none=-1, kinematic=0, pseudoDynamic=1, realDynamic=2 };
  TransitionType transitionType;
  arr H_rate_diag; ///< cost rate
  uint T; ///< number of time steps
  double tau; ///< duration of single step
  
  //-- start constraints
  arr x0, v0; ///< fixed start state and velocity [[TODO: remove this and replace by prefix only (redundant...)]]
  arr prefix; ///< a set of states PRECEEDING x[0] (having 'negative' time indices) and which influence the control cost on x[0]. NOTE: x[0] is subject to optimization. DEFAULT: constantly equals x0
  arr postfix; ///< fixing the set of statex x[T-k]...x[T]
  //TODO: add methods to properly set the prefix given x0,v0?

  //-- stationary parameters
  arr z0; ///< an initialization of the stationary parameters of the motion problem

  //-- return values of an optimizer
  MT::Array<arr> phiMatrix;
  arr dualMatrix;

  MotionProblem(ors::KinematicWorld& _world, bool useSwift=true);
  
  MotionProblem& operator=(const MotionProblem& other);

  void loadTransitionParameters(); ///< loads transition parameters from cfgFile //TODO: do in constructor of TransitionCost

  //-- setting costs in a task space
  TaskCost* addTask(const char* name, TaskMap *map);
  enum TaskCostInterpolationType { constant, finalOnly, final_restConst, early_restConst, final_restLinInterpolated };
  void setInterpolatingCosts(TaskCost *c,
                             TaskCostInterpolationType inType,
                             const arr& y_finalTarget, double y_finalPrec, const arr& y_midTarget=NoArr, double y_midPrec=-1., double earlyFraction=-1.);

  //-- cost infos
  uint dim_phi(const ors::KinematicWorld& G, uint t);
  uint dim_g(const ors::KinematicWorld& G, uint t);
  uint dim_psi();
//  bool getTaskCosts(arr& phi, arr& J_x, arr& J_v, uint t); ///< the general (`big') task vector and its Jacobian
  bool getTaskCosts2(arr& phi, arr& J, uint t, const WorldL& G, double tau); ///< the general (`big') task vector and its Jacobian
  void costReport(bool gnuplt=true); ///< also computes the costMatrix
  
  void setState(const arr& x, const arr& v=NoArr);
  void activateAllTaskCosts(bool activate=true);

  //-- helpers
  arr getInitialization();
};


//===========================================================================
//
// transforming a motion problem description into an optimization problem
//

struct MotionProblemFunction:KOrderMarkovFunction {
  MotionProblem& MP;
  WorldL configurations;

  MotionProblemFunction(MotionProblem& _P):MP(_P) { MT::Array<ors::KinematicWorld*>::memMove=true; };
  
  //KOrderMarkovFunction definitions
  virtual void phi_t(arr& phi, arr& J, uint t, const arr& x_bar);
  //functions to get the parameters $T$, $k$ and $n$ of the $k$-order Markov Process
  virtual uint get_T() { return MP.T; }
  virtual uint get_k() { return 2; }
  virtual uint dim_x() { return MP.x0.N; }
  virtual uint dim_z() { return MP.z0.N; }
  virtual uint dim_phi(uint t){ return MP.dim_psi() + MP.dim_phi(MP.world, t); } //transitions plus costs (latter include constraints)
  virtual uint dim_g(uint t){ return MP.dim_g(MP.world, t); }
  virtual arr get_prefix(); //the history states x(-k),..,x(-1)
  virtual arr get_postfix();
};


//===========================================================================
//
// transforming a motion problem description into an end-pose optimization problem only
//

struct MotionProblem_EndPoseFunction:VectorFunction {
  MotionProblem& MP;

  MotionProblem_EndPoseFunction(MotionProblem& _P):MP(_P) {};

  //VectorFunction definitions
  virtual void fv(arr& phi, arr& J, const arr& x);
};


//===========================================================================
//
// basic helpers
//

void sineProfile(arr& q, const arr& q0, const arr& qT,uint T);
arr reverseTrajectory(const arr& q);
void getVel(arr& v, const arr& q, double tau);
void getAcc(arr& a, const arr& q, double tau);


#endif
