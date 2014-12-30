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

#include "feedbackControl.h"

//===========================================================================

void PDtask::setTarget(const arr& yref, const arr& vref){
  y_ref = yref;
  if(&vref) v_ref=vref; else v_ref.resizeAs(y_ref).setZero();
}

void PDtask::setGains(double pgain, double dgain) {
  active=true;
  Pgain=pgain;
  Dgain=dgain;
  if(!prec) prec=100.;
}

void PDtask::setGainsAsNatural(double decayTime, double dampingRatio) {
  active=true;
  double lambda = -decayTime*dampingRatio/log(.1);
  Pgain = MT::sqr(1./lambda);
  Dgain = 2.*dampingRatio/lambda;
  if(!prec) prec=100.;
}

arr PDtask::getDesiredAcceleration(const arr& y, const arr& ydot){
  if(!y_ref.N) y_ref.resizeAs(y).setZero();
  if(!v_ref.N) v_ref.resizeAs(ydot).setZero();
  this->y = y;
  this->v = ydot;
//  cout <<" TASK " <<name <<":  \tPterm=(" <<Pgain <<'*' <<length(y_ref-y) <<")  \tDterm=(" <<Dgain <<'*' <<length(v_ref-ydot) <<')' <<endl;
  return Pgain*(y_ref-y) + Dgain*(v_ref-ydot);
}

//===========================================================================

void ConstraintForceTask::updateConstraintControl(const arr& _g, const double& lambda_desired){
  CHECK(_g.N==1, "can handle only 1D constraints so far");
  double g=_g(0);
  CHECK(lambda_desired>=0., "lambda must be positive or zero");

  if(g<0 && lambda_desired>0.){ //steer towards constraint
    desiredApproach.y_ref=ARR(.05); //set goal to overshoot!
    desiredApproach.setGainsAsNatural(.3, 1.);
    desiredApproach.prec=1e4;
  }

  if(g>-1e-2 && lambda_desired>0.){ //stay in constraint -> constrain dynamics
    desiredApproach.y_ref=ARR(0.);
    desiredApproach.setGainsAsNatural(.05, .7);
    desiredApproach.prec=1e6;
  }

  if(g>-0.02 && lambda_desired==0.){ //release constraint -> softly push out
    desiredApproach.y_ref=ARR(-0.04);
    desiredApproach.setGainsAsNatural(.3, 1.);
    desiredApproach.prec=1e4;
  }

  if(g<=-0.02 && lambda_desired==0.){ //stay out of contact -> constrain dynamics
    desiredApproach.active=false;
  }
}

//===========================================================================

FeedbackMotionControl::FeedbackMotionControl(ors::KinematicWorld& _world, bool useSwift)
  : MotionProblem(_world, useSwift), qitselfPD(NULL) {
  loadTransitionParameters();
  qitselfPD.name="qitselfPD";
  qitselfPD.setGains(0.,10.);
  qitselfPD.prec=1.;
}

PDtask* FeedbackMotionControl::addPDTask(const char* name, double decayTime, double dampingRatio, TaskMap *map){
  PDtask *t = new PDtask(map);
  t->name=name;
  tasks.append(t);
  t->setGainsAsNatural(decayTime, dampingRatio);
  return t;
}

PDtask* FeedbackMotionControl::addPDTask(const char* name,
                                         double decayTime, double dampingRatio,
                                         DefaultTaskMapType type,
                                         const char* iShapeName, const ors::Vector& ivec,
                                         const char* jShapeName, const ors::Vector& jvec,
                                         const arr& params){
  PDtask *t = addPDTask(name, decayTime, dampingRatio,
                        new DefaultTaskMap(type, world, iShapeName, ivec, jShapeName, jvec, params));
  return t;
}

ConstraintForceTask* FeedbackMotionControl::addConstraintForceTask(const char* name, TaskMap *map){
  ConstraintForceTask *t = new ConstraintForceTask(map);
  t->name=name;
  t->desiredApproach.name=STRING(name <<"_PD");
  t->desiredApproach.active=false;
  forceTasks.append(t);
  tasks.append(&t->desiredApproach);
  return t;
}

void FeedbackMotionControl::getCostCoeffs(arr& c, arr& J){
  c.clear();
  if(&J) J.clear();
  arr y, J_y, yddot_des;
  for(PDtask* t: tasks) {
    if(t->active) {
      t->map.phi(y, J_y, world);
      yddot_des = t->getDesiredAcceleration(y, J_y*world.qdot);
      c.append(::sqrt(t->prec)*(yddot_des /*-Jdot*qdot*/));
      if(&J) J.append(::sqrt(t->prec)*J_y);
    }
  }
  if(&J) J.reshape(c.N, world.q.N);
}

void FeedbackMotionControl::reportCurrentState(){
  for(PDtask* t: tasks) {
    cout <<"Task " <<t->name;
    if(t->active) {
      cout <<": \ty_ref=" <<t->y_ref <<" \ty=" <<t->y
          <<" \tPterm=(" <<t->Pgain <<'*' <<length(t->y_ref-t->y) <<")  \tDterm=(" <<t->Dgain <<'*' <<length(t->v_ref-t->v) <<')' <<endl;
    }else{
      cout <<" -- inactive" <<endl;
    }
  }
}

void FeedbackMotionControl::updateConstraintControllers(){
  arr y;
  for(ConstraintForceTask* t: forceTasks){
    if(t->active){
      t->map.phi(y, NoArr, world);
      t->updateConstraintControl(y, t->desiredForce);
    }
  }
}

arr FeedbackMotionControl::getDesiredConstraintForces(){
  arr Jl(world.q.N, 1);
  Jl.setZero();
  arr y, J_y;
  for(ConstraintForceTask* t: forceTasks){
    if(t->active) {
      t->map.phi(y, J_y, world);
      CHECK(y.N==1," can only handle 1D constraints for now");
      Jl += ~J_y * t->desiredForce;
    }
  }
  Jl.reshape(Jl.N);
  return Jl;
}

arr FeedbackMotionControl::operationalSpaceControl(){
  arr c, J;
  getCostCoeffs(c, J);
  if(!c.N && !qitselfPD.active) return zeros(world.q.N,1).reshape(world.q.N);
  arr H = diag(H_rate_diag);
  arr A = H;
  arr a(H.d0); a.setZero();
  if(qitselfPD.active){
    a += H_rate_diag % qitselfPD.getDesiredAcceleration(world.q, world.qdot);
  }
  if(c.N){
    A += comp_At_A(J);
    a += comp_At_x(J, c);
  }
  arr q_ddot = inverse_SymPosDef(A) * a;
  return q_ddot;
}
