#include "taskMap_transition.h"

TransitionTaskMap::TransitionTaskMap(const ors::KinematicWorld& G){
  velCoeff = MT::getParameter<double>("Motion/TaskMapTransition/vecCoeff",.0);
  accCoeff = MT::getParameter<double>("Motion/TaskMapTransition/accCoeff",1.);

  //transition cost metric
  arr H_diag;
  if(MT::checkParameter<arr>("Hdiag")) {
    H_diag = MT::getParameter<arr>("Hdiag");
  } else {
    H_diag = G.naturalQmetric();
  }
  H_rate_diag = MT::getParameter<double>("Hrate")*H_diag;
}

void TransitionTaskMap::phi(arr& y, arr& J, const WorldL& G, double tau){
  if(G.last()->q_agent!=0){ //we're referring to a graph set to non-zero agent!
    uint n=G.last()->getJointStateDimension();
    // CHECK(n!=H_rate_diag.N,"just checking..."); POSSIBLY A BUG!!!
    CHECK(n==H_rate_diag.N,"just checking...");
    y.resize(n).setZero();
    if(&J) J.resize(y.N, order+1, n).setZero();
    return;
  }

  //-- transition costs
  double tau2=tau*tau;
  arr h = sqrt(H_rate_diag)*sqrt(tau);
  if(order==1) velCoeff = 1.;
  if(order>=1) y = velCoeff*(G(G.N-1-1)->q - G(G.N-1-0)->q)/tau; //penalize velocity
  if(order>=2) y += accCoeff*(G(G.N-1-2)->q - 2.*G(G.N-1-1)->q + G(G.N-1-0)->q)/tau2; //penalize acceleration
  if(order>=3) NIY; //  y = (x_bar[3]-3.*x_bar[2]+3.*x_bar[1]-x_bar[0])/tau3; //penalize jerk
  y = h % y;

  if(&J) {
    uint n = G.last()->q.N;
    J.resize(y.N, G.N, n).setZero();
    for(uint i=0;i<n;i++){
      if(order>=1){ J(i,G.N-1-1,i) += velCoeff/tau;  J(i,G.N-1-0,i) += -velCoeff/tau; }
      if(order>=2){ J(i,G.N-1-2,i) += accCoeff/tau2;  J(i,G.N-1-1,i) += -2.*accCoeff/tau2;  J(i,G.N-1-0,i) += accCoeff/tau2; }
//      if(order>=3){ J(i,3,i) = 1.;  J(i,2,i) = -3.;  J(i,1,i) = +3.;  J(i,0,i) = -1.; }
    }
    J.reshape(y.N, G.N*n);
    for(uint i=0; i<n; i++) J[i]() *= h(i);
  }
}
