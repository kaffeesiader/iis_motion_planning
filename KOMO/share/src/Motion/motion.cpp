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



#include "motion.h"
#include "taskMap_default.h"
#include <Gui/opengl.h>
#include <Ors/ors_swift.h>
#include <climits>

//===========================================================================

void TaskMap::phi(arr& y, arr& J, const WorldL& G, double tau){
  CHECK(G.N>=order+1,"I need at least " <<order+1 <<" configurations to evaluate");
  uint k=order;
  if(k==0){// basic case: order=0
    arr J_bar;
    phi(y, (&J?J_bar:NoArr), *G.last());
    if(&J){
      J = zeros(G.N, y.N, J_bar.d1);
      J[G.N-1]() = J_bar;
      arr tmp(J);
      tensorPermutation(J, tmp, TUP(1,0,2));
      J.reshape(y.N, G.N*J_bar.d1);
    }
    return;
  }
  arrA y_bar, J_bar;
  double tau2=tau*tau, tau3=tau2*tau;
  y_bar.resize(k+1);
  J_bar.resize(k+1);
  //-- read out the task variable from the k+1 configurations
  for(uint i=0;i<=k;i++)
    phi(y_bar(i), (&J?J_bar(i):NoArr), *G(G.N-1-i));
  if(k==1)  y = (y_bar(1)-y_bar(0))/tau; //penalize velocity
  if(k==2)  y = (y_bar(2)-2.*y_bar(1)+y_bar(0))/tau2; //penalize acceleration
  if(k==3)  y = (y_bar(3)-3.*y_bar(2)+3.*y_bar(1)-y_bar(0))/tau3; //penalize jerk
  if(&J) {
    J = zeros(G.N, y.N, J_bar(0).d1);
    if(k==1){ J[G.N-1-1]() = J_bar(1);  J[G.N-1-0]() = -J_bar(0);  J/=tau; }
    if(k==2){ J[G.N-1-2]() = J_bar(2);  J[G.N-1-1]() = -2.*J_bar(1);  J[G.N-1-0]() = J_bar(0);  J/=tau2; }
    if(k==3){ J[G.N-1-3]() = J_bar(3);  J[G.N-1-2]() = -3.*J_bar(2);  J[G.N-1-1]() = +3.*J_bar(1);  J[G.N-1-0]() = -1.*J_bar(0);  J/=tau3; }
    arr tmp(J);
    tensorPermutation(J, tmp, TUP(1,0,2));
    J.reshape(y.N, G.N*J_bar(0).d1);
  }
}

//===========================================================================

void TaskCost::setCostSpecs(uint fromTime,
                            uint toTime,
                            const arr& _target,
                            double _prec){
  if(&_target) target = _target; else target = {0.};
  CHECK(toTime>=fromTime,"");
  prec.resize(toTime+1).setZero();
  for(uint t=fromTime;t<=toTime;t++) prec(t) = _prec;
}

//===========================================================================

MotionProblem::MotionProblem(ors::KinematicWorld& _world, bool useSwift)
    : world(_world) , useSwift(useSwift), transitionType(none), T(0), tau(0.)
{
  if(useSwift) {
    makeConvexHulls(world.shapes);
    world.swift().setCutoff(2.*MT::getParameter<double>("swiftCutoff", 0.11));
  }
  world.getJointState(x0, v0);
  if(!v0.N){ v0.resizeAs(x0).setZero(); world.setJointState(x0, v0); }
  double duration = MT::getParameter<double>("duration");
  T = MT::getParameter<uint>("timeSteps");
  tau = duration/T;
}

MotionProblem& MotionProblem::operator=(const MotionProblem& other) {
  world = const_cast<ors::KinematicWorld&>(other.world);
  useSwift = other.useSwift;
  taskCosts = other.taskCosts;
  transitionType = other.transitionType;
  H_rate_diag = other.H_rate_diag;
  T = other.T;
  tau = other.tau;
  x0 = other.x0;
  v0 = other.v0;
  prefix = other.prefix;
  phiMatrix = other.phiMatrix;
  dualMatrix = other.dualMatrix;
  return *this;
}

void MotionProblem::loadTransitionParameters() {
  //transition type
  transitionType = (TransitionType)MT::getParameter<int>("transitionType");
  
  //time and steps
  double duration = MT::getParameter<double>("duration");
  T = MT::getParameter<uint>("timeSteps");
  tau = duration/T;
  
  //transition cost metric
  arr W_diag;
  if(MT::checkParameter<arr>("Wdiag")) {
    W_diag = MT::getParameter<arr>("Wdiag");
  } else {
    W_diag = world.naturalQmetric();
  }
  H_rate_diag = MT::getParameter<double>("Hrate")*W_diag;
}

TaskCost* MotionProblem::addTask(const char* name, TaskMap *m){
  TaskCost *t = new TaskCost(m);
  t->name=name;
  taskCosts.append(t);
  return t;
}

void MotionProblem::setInterpolatingCosts(
  TaskCost *c,
  TaskCostInterpolationType inType,
  const arr& y_finalTarget, double y_finalPrec, const arr& y_midTarget, double y_midPrec, double earlyFraction) {
  uint m=c->map.dim_phi(world);
  setState(x0,v0);
  arr y0;
  c->map.phi(y0, NoArr, world);
  arr midTarget=zeros(m),finTarget=zeros(m);
  if(&y_finalTarget){ if(y_finalTarget.N==1) finTarget = y_finalTarget(0); else finTarget=y_finalTarget; }
  if(&y_midTarget){   if(y_midTarget.N==1)   midTarget = y_midTarget(0);   else midTarget=y_midTarget; }
  switch(inType) {
    case constant: {
      c->target = replicate(finTarget, T+1);
      c->prec.resize(T+1) = y_finalPrec;
    } break;
    case finalOnly: {
      c->target.resize(T+1, m).setZero();
      c->target[T]() = finTarget;
      c->prec.resize(T+1).setZero();
      c->prec(T) = y_finalPrec;
    } break;
    case final_restConst: {
      c->target = replicate(midTarget, T+1);
      c->target[T]() = finTarget;
      c->prec.resize(T+1) = y_midPrec<=0. ? 0. : y_midPrec;
      c->prec(T) = y_finalPrec;
    } break;
    case final_restLinInterpolated: {
      c->target.resize(T+1, m).setZero();
      for(uint t=0; t<=T; t++) {
        double a = (double)t/T;
        c->target[t]() = ((double)1.-a)*y0 + a*finTarget;
      }
      c->prec.resize(T+1) = y_midPrec<0. ? y_finalPrec : y_midPrec;
      c->prec(T) = y_finalPrec;
    } break;
  case early_restConst: {
    uint t;
    CHECK(earlyFraction>=0. && earlyFraction<=1.,"");
    uint Tearly=earlyFraction*T;
    c->target.resize(T+1, m).setZero();
    for(t=0; t<Tearly; t++) c->target[t]() = midTarget;
    for(t=Tearly; t<=T; t++) c->target[t]() = finTarget;
    c->prec.resize(T+1).setZero();
    for(t=0; t<Tearly; t++) c->prec(t) = y_midPrec<=0. ? 0. : y_midPrec;
    for(t=Tearly; t<=T; t++) c->prec(t) = y_finalPrec;
  } break;
  }
}

void MotionProblem::setState(const arr& q, const arr& v) {
  world.setJointState(q, v);
  if(useSwift) world.stepSwift();
  if(transitionType == realDynamic) {
    NIY;
    //requires computation of the real dynamics, i.e. of M and F
  }
}


uint MotionProblem::dim_phi(const ors::KinematicWorld &G, uint t) {
  uint m=0;
  for(TaskCost *c: taskCosts) {
    m += c->dim_phi(G, t); //counts also constraints
  }
  return m;
}

uint MotionProblem::dim_g(const ors::KinematicWorld &G, uint t) {
  uint m=0;
  for(TaskCost *c: taskCosts) {
    if(c->active && c->map.constraint)  m += c->map.dim_phi(G);
  }
  return m;
}


//bool MotionProblem::getTaskCosts(arr& phi, arr& J_x, arr& J_v, uint t) {
//  phi.clear();
//  bool feasible = true;
//  if(&J_x) J_x.clear();
//  if(&J_v) J_v.clear();
//  arr y,J;
//  //-- append task costs
//  for(TaskCost *c: taskCosts) if(c->active && c->prec(t)){
//    if(!c->map.constraint) {
//      c->map.phi(y, J, world);
//      if(absMax(y)>1e10)  MT_MSG("WARNING y=" <<y);
//      CHECK(c->prec.N>t && c->target.N>t, "active task costs "<< c->name <<" have no targets defined");
//      CHECK(c->map.order==0 || c->map.order==1,"");
//      if(c->map.order==0) { //pose costs
//        phi.append(sqrt(c->prec(t))*(y - c->target[t]));
//        if(&J_x) J_x.append(sqrt(c->prec(t))*J);
//        if(&J_v) J_v.append(0.*J);
//      }
//      if(c->map.order==1) { //velocity costs
//        phi.append(sqrt(c->prec(t))*(J*world.qdot - c->target[t]));
//        if(&J_x) J_x.append(0.*J);
//        if(&J_v) J_v.append(sqrt(c->prec(t))*J);
//      }
//    }
//    if(phi.N && phi.last() > c->threshold) feasible = false; //TOTAL hack: last(). Please use constraints, or a separate routine
//  }
//  //-- append constraints
//  for(TaskCost *c: taskCosts) if(c->active && c->prec(t)){
//    if(c->map.constraint) {
//      c->map.phi(y, J, world);
//      phi.append(y);
//      if(phi.last() > c->threshold) feasible = false;
//      if(&J_x) J_x.append(J);
//      if(&J_v) J_v.append(0.*J);
//    }
//  }
//  if(&J_x) J_x.reshape(phi.N, world.q.N);
//  if(&J_v) J_v.reshape(phi.N, world.q.N);

//  CHECK(phi.N == dim_phi(world, t),"");

//  return feasible;
//}

bool MotionProblem::getTaskCosts2(arr& phi, arr& J, uint t, const WorldL &G, double tau) {
  phi.clear();
  if(&J) J.clear();
  arr y, Jy;
  bool constraintsHold=true;
  //-- append task costs
  for(TaskCost *c: taskCosts) if(c->active && c->prec.N>t && c->prec(t)){
    if(!c->map.constraint) {
      c->map.phi(y, (&J?Jy:NoArr), G, tau);
      if(absMax(y)>1e10) MT_MSG("WARNING y=" <<y);
      if(c->target.N==1) y -= c->target(0);
      else if(c->target.nd==1) y -= c->target;
      else y -= c->target[t];
      phi.append(sqrt(c->prec(t))*y);
      if(&J) J.append(sqrt(c->prec(t))*Jy);
    }
  }
  //-- append constraints
  for(TaskCost *c: taskCosts) if(c->active && c->prec.N>t && c->prec(t)){
    if(c->map.constraint) {
      c->map.phi(y, (&J?Jy:NoArr), G, tau);
      phi.append(y);
      if(&J) J.append(Jy);
      if(max(y)>0.) constraintsHold=false;
    }
  }
  if(&J) J.reshape(phi.N, G.N*G.last()->getJointStateDimension());

  CHECK(phi.N == dim_phi(*G.last(), t),"");
  return constraintsHold;
}

uint MotionProblem::dim_psi() {
  if(transitionType==none) return 0;
  return x0.N;
}

void MotionProblem::activateAllTaskCosts(bool active) {
  for(TaskCost *c: taskCosts) c->active=active;
}

void MotionProblem::costReport(bool gnuplt) {
  cout <<"*** MotionProblem -- CostReport" <<endl;
  cout <<"Size of cost matrix:" <<phiMatrix.getDim() <<endl;
  uint T=phiMatrix.d0-1;

  arr plotData(T+1,taskCosts.N+1); plotData.setZero();
  double totalT=0., a;
  cout <<" * transition costs:" <<endl;
  if(dim_psi()){
    for(uint t=0;t<=T;t++){
      totalT += a = sumOfSqr(phiMatrix(t).sub(0,dim_psi()-1));
      plotData(t,0) = a;
    }
  }
  cout <<"\t total=" <<totalT <<endl;

  //-- collect all task costs and constraints
  arr taskC(taskCosts.N); taskC.setZero();
  arr taskG(taskCosts.N); taskG.setZero();
  for(uint t=0; t<=T; t++){
    uint m=dim_psi();
    for(uint i=0; i<taskCosts.N; i++) {
      TaskCost *c = taskCosts(i);
      uint d=c->dim_phi(world, t);
      if(d && !c->map.constraint){
        taskC(i) += a = sumOfSqr(phiMatrix(t).sub(m,m+d-1));
        plotData(t,i+1) = a;
        m += d;
      }
      if(d && c->map.constraint){
        double gpos=0.;
        for(uint j=0;j<d;j++){
          double g=phiMatrix(t)(m+j);
          if(g>0.) gpos+=g;
        }
        taskG(i) += gpos;
        plotData(t,i+1) = gpos;
        m += d;
      }
    }
    CHECK(m == phiMatrix(t).N, "");
  }

  cout <<" * task costs:" <<endl;
  double totalC=0., totalG=0.;
  for(uint i=0; i<taskCosts.N; i++) {
    TaskCost *c = taskCosts(i);
    cout <<"\t '" <<c->name <<"' order=" <<c->map.order <<" con=" <<c->map.constraint;
    cout <<" \tcosts=" <<taskC(i) <<" \tconstraints=" <<taskG(i) <<endl;
    totalC += taskC(i);
    totalG += taskG(i);
  }

  cout <<"\t total trans       = " <<totalT <<endl;
  cout <<"\t total task        = " <<totalC <<endl;
  cout <<"\t total constraints = " <<totalG <<endl;
  cout <<"\t total task+trans  = " <<totalC+totalT <<endl;


  //-- write a nice gnuplot file
  ofstream fil("z.costReport");
  //first line: legend
  fil <<"trans[" <<dim_psi() <<"] ";
  for(auto c:taskCosts){
    uint d=c->map.dim_phi(world);
    fil <<c->name <<'[' <<d <<"] ";
  }
  for(auto c:taskCosts){
    if(c->map.constraint && dualMatrix.N){
      fil <<c->name <<"_dual";
    }
  }
  fil <<endl;
  //rest: just the matrix?
  if(!dualMatrix.N){
    plotData.write(fil,NULL,NULL,"  ");
  }else{
    dualMatrix.reshape(T+1, dualMatrix.N/(T+1));
    catCol(plotData, dualMatrix).write(fil,NULL,NULL,"  ");
  }
  fil.close();

  ofstream fil2("z.costReport.plt");
  fil2 <<"set key autotitle columnheader" <<endl;
  fil2 <<"set title 'costReport ( plotting sqrt(costs) )'" <<endl;
  fil2 <<"plot 'z.costReport' u 0:1 w l \\" <<endl;
  uint i=1;
  for(uint tmp=0;tmp<taskCosts.N;tmp++){ i++; fil2 <<"  ,'' u 0:"<<i<<" w l \\" <<endl;  }
  if(dualMatrix.N) for(uint tmp=0;tmp<taskCosts.N;tmp++){  i++; fil2 <<"  ,'' u 0:"<<i<<" w l \\" <<endl;  }
  fil2 <<endl;
  fil2.close();

  if(gnuplt) gnuplot("load 'z.costReport.plt'");
}

arr MotionProblem::getInitialization(){
  return replicate(x0, T+1);
}

//===========================================================================

arr MotionProblemFunction::get_prefix() {
  if(!MP.prefix.N){
    MP.prefix.resize(get_k(), dim_x());
    for(uint i=0; i<MP.prefix.d0; i++) MP.prefix[i]() = MP.x0;
  }
  CHECK(MP.prefix.d0==get_k() && MP.prefix.d1==dim_x(), "the prefix you set has wrong dimension");
  return MP.prefix;
}

arr MotionProblemFunction::get_postfix() {
  if(!MP.postfix.N) return arr();
  CHECK(MP.postfix.d0==get_k() && MP.postfix.d1==dim_x(), "the postfix you set has wrong dimension");
  return MP.postfix;
}

#if 0
void MotionProblemFunction::phi_t(arr& phi, arr& J, uint t, const arr& x_bar, const arr& z, const arr& J_z) {
  uint T=get_T(), n=dim_x(), k=get_k();

  //assert some dimensions
  CHECK(x_bar.d0==k+1,"");
  CHECK(x_bar.d1==n,"");
  CHECK(t<=T,"");

  double tau=MP.tau;
  double tau2=tau*tau, tau3=tau2*tau;
  
  //-- transition costs
  arr h = sqrt(MP.H_rate_diag)*sqrt(tau);
  if(k==1)  phi = (x_bar[1]-x_bar[0])/tau; //penalize velocity
  if(k==2)  phi = (x_bar[2]-2.*x_bar[1]+x_bar[0])/tau2; //penalize acceleration
  if(k==3)  phi = (x_bar[3]-3.*x_bar[2]+3.*x_bar[1]-x_bar[0])/tau3; //penalize jerk
  phi = h % phi;

  if(&J) {
    J.resize(phi.N, k+1, n);
    J.setZero();
    for(uint i=0;i<n;i++){
      if(k==1){ J(i,1,i) = 1.;  J(i,0,i) = -1.; }
      if(k==2){ J(i,2,i) = 1.;  J(i,1,i) = -2.;  J(i,0,i) = 1.; }
      if(k==3){ J(i,3,i) = 1.;  J(i,2,i) = -3.;  J(i,1,i) = +3.;  J(i,0,i) = -1.; }
    }
    if(k==1) J/=tau;
    if(k==2) J/=tau2;
    if(k==3) J/=tau3;
    J.reshape(phi.N, (k+1)*n);
    for(uint i=0; i<n; i++) J[i]() *= h(i);
  }
  
  if(&J) CHECK(J.d0==phi.N,"");

  //-- task cost (which are taken w.r.t. x_bar[k])
  arr _phi, J_x, J_v;
  if(k>0) MP.setState(x_bar[k], (x_bar[k]-x_bar[k-1])/tau);
  else    MP.setState(x_bar[k], NoArr); //don't set velocities
  MP.getTaskCosts(_phi, (&J?J_x:NoArr), (&J?J_v:NoArr), t);
  phi.append(_phi);
  if(&J && _phi.N) {
    arr Japp(_phi.N, (k+1)*n);
    Japp.setZero();
    Japp.setMatrixBlock(J_x + (1./tau)*J_v, 0,  k*n   ); //w.r.t. x_bar[k]
    Japp.setMatrixBlock(     (-1./tau)*J_v, 0, (k-1)*n); //w.r.t. x_bar[k-1]
    J.append(Japp);
  }
  
  if(&J) CHECK(J.d0==phi.N,"");
  
  //store in CostMatrix
  if(!MP.phiMatrix.N) MP.phiMatrix.resize(get_T()+1);
  MP.phiMatrix(t) = phi;
}
#else

void MotionProblemFunction::phi_t(arr& phi, arr& J, uint t, const arr& x_bar) {
  uint T=get_T(), n=dim_x()+dim_z(), k=get_k();

  //assert some dimensions
  CHECK(x_bar.d0==k+1,"");
  CHECK(x_bar.d1==n,"");
  CHECK(t<=T,"");

  //-- manage configurations
  if(configurations.N!=k+1 || t==0){
    listDelete(configurations);
    for(uint i=0;i<=k;i++) configurations.append(new ors::KinematicWorld())->copy(MP.world, true);
  }
  //find matches
  if(!MP.world.operators.N){ //this efficiency gain only works without operators yet...
    uintA match(k+1); match=UINT_MAX;
    boolA used(k+1); used=false;
    uintA unused;
    for(uint i=0;i<=k;i++) for(uint j=0;j<=k;j++){
      if(!used(j) && x_bar[i]==configurations(j)->q){ //we've found a match
        match(i)=j;
        used(j)=true;
        j=k;
      }
    }
    for(uint i=0;i<=k;i++) if(!used(i)) unused.append(i);
    for(uint i=0;i<=k;i++) if(match(i)==UINT_MAX) match(i)=unused.popFirst();
    configurations.permute(match);
  }
  //apply potential graph operators
  for(ors::GraphOperator *op:MP.world.operators){
    for(uint i=0;i<=k;i++){
      if(t+i>=k && op->timeOfApplication==t-k+i){
        op->apply(*configurations(i));
      }
    }
  }
  //set states
  for(uint i=0;i<=k;i++){
    if(x_bar[i]!=configurations(i)->q){
      configurations(i)->setJointState(x_bar[i]);
      if(MP.useSwift) configurations(i)->stepSwift();
    }
  }

  //-- transition costs
  if(MP.transitionType!=MotionProblem::none){
    double tau=MP.tau, tau2=tau*tau, tau3=tau2*tau;
    arr h = sqrt(MP.H_rate_diag)*sqrt(tau);
    if(k==1)  phi = (x_bar[1]-x_bar[0])/tau; //penalize velocity
    if(k==2)  phi = (x_bar[2]-2.*x_bar[1]+x_bar[0])/tau2; //penalize acceleration
    if(k==3)  phi = (x_bar[3]-3.*x_bar[2]+3.*x_bar[1]-x_bar[0])/tau3; //penalize jerk
    phi = h % phi;

    if(&J) {
      J.resize(phi.N, k+1, n);
      J.setZero();
      for(uint i=0;i<n;i++){
        if(k==1){ J(i,1,i) = 1.;  J(i,0,i) = -1.; }
        if(k==2){ J(i,2,i) = 1.;  J(i,1,i) = -2.;  J(i,0,i) = 1.; }
        if(k==3){ J(i,3,i) = 1.;  J(i,2,i) = -3.;  J(i,1,i) = +3.;  J(i,0,i) = -1.; }
      }
      if(k==1) J/=tau;
      if(k==2) J/=tau2;
      if(k==3) J/=tau3;
      J.reshape(phi.N, (k+1)*n);
      for(uint i=0; i<n; i++) J[i]() *= h(i);
    }

    if(&J) CHECK(J.d0==phi.N,"");
  }

  //-- task cost (which are taken w.r.t. x_bar[k])
  arr _phi, _J;
  MP.getTaskCosts2(_phi, (&J?_J:NoArr), t, configurations, MP.tau);
  phi.append(_phi);
  if(&J) J.append(_J);
//  if(&J_z){
//    for(auto& c:configurations) c->setAgent(1);
//    MP.getTaskCosts2(_phi, J_z, t, configurations, MP.tau);
//    for(auto& c:configurations) c->setAgent(0);
//  }

  if(&J) CHECK(J.d0==phi.N,"");
//  if(&J_z) CHECK(J.d0==phi.N,"");

  //store in CostMatrix
  if(!MP.phiMatrix.N) MP.phiMatrix.resize(get_T()+1);
  MP.phiMatrix(t) = phi;
}
#endif

//===========================================================================

void MotionProblem_EndPoseFunction::fv(arr& phi, arr& J, const arr& x){
  //-- transition costs
  arr h = MP.H_rate_diag;
  if(MP.transitionType==MotionProblem::kinematic){
    h *= MP.tau/double(MP.T);
    h=sqrt(h);
  } else {
    double D = MP.tau*MP.T;
    h *= 16./D/D/D;
    h=sqrt(h);
  }
  phi = h%(x-MP.x0);
  if(&J) J.setDiag(h);

  //-- task costs
  arr _phi, J_x;
  MP.setState(x, zeros(x.N));
  MP.getTaskCosts2(_phi, J_x, MP.T, LIST(MP.world), MP.tau);
  phi.append(_phi);
  if(&J && _phi.N) {
    J.append(J_x);
  }

  if(absMax(phi)>1e10){
    MT_MSG("\nx=" <<x <<"\nphi=" <<phi <<"\nJ=" <<J);
    MP.setState(x, NoArr);
    MP.getTaskCosts2(_phi, J_x, MP.T, LIST(MP.world), MP.tau);
  }

  if(&J) CHECK(J.d0==phi.N,"");

  //store in CostMatrix
  MP.phiMatrix = phi;
}

//===========================================================================

void sineProfile(arr& q, const arr& q0, const arr& qT,uint T){
  q.resize(T+1,q0.N);
  for(uint t=0; t<=T; t++) q[t] = q0 + .5 * (1.-cos(MT_PI*t/T)) * (qT-q0);
}

arr reverseTrajectory(const arr& q){
  uint T=q.d0-1;
  arr r(T+1, q.d1);
  for(uint t=0; t<=T; t++) r[T-t] = q[t];
  return r;
}

void getVel(arr& v, const arr& q, double tau){
  uint T=q.d0-1;
  v.resizeAs(q);
  for(uint t=1; t<T; t++)  v[t] = (q[t+1] - q[t-1])/(2.*tau);
  v[0] = (q[1] - q[0])/tau;
  v[T] = (q[T] - q[T-1])/tau;
}

void getAcc(arr& a, const arr& q, double tau){
  uint T=q.d0-1;
  a.resizeAs(q);
  for(uint t=1; t<T; t++)  a[t] = (q[t+1] + q[t-1] - 2.*q[t])/(tau*tau);
  a[0] = a[1]/2.;
  a[T] = a[T-1]/2.;
}
