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

#include "taskMap_constrained.h"

//===========================================================================

void CollisionConstraint::phi(arr& y, arr& J, const ors::KinematicWorld& G){
  G.kinematicsProxyCost(y, J, margin, false);
  y -= .5;
}

//===========================================================================

void LimitsConstraint::phi(arr& y, arr& J, const ors::KinematicWorld& G){
  if(!limits.N) limits = G.getLimits();
  G.kinematicsLimitsCost(y, J, limits, margin);
  y -= .5;
}

//===========================================================================

void PairCollisionConstraint::phi(arr& y, arr& J, const ors::KinematicWorld& G){
  y.resize(1) = -1.;
  if(&J) J.resize(1,G.q.N).setZero();
  for(ors::Proxy *p: G.proxies){
    if((p->a==i && p->b==j) || (p->a==j && p->b==i)){
      G.kinematicsProxyConstraint(y, J, p, margin, false);
      break;
    }
  }
}

//===========================================================================

void PlaneConstraint::phi(arr& y, arr& J, const ors::KinematicWorld& G){
  ors::Body *body_i = G.shapes(i)->body;
  ors::Vector vec_i = G.shapes(i)->rel.pos;

  arr y_eff, J_eff;
  G.kinematicsPos(y_eff, (&J?J_eff:NoArr), body_i, &vec_i);

  y_eff.append(1.); //homogeneous coordinates
  if(&J) J_eff.append(zeros(1,J_eff.d1));

  y.resize(1);
  y(0) = scalarProduct(y_eff, planeParams);
  if(&J) J = ~planeParams * J_eff;
}

//===========================================================================

void ConstraintStickiness::phi(arr& y, arr& J, const ors::KinematicWorld& G){
  map.phi(y, J, G);
  for(uint j=0;j<y.N;j++) y(j) = -y(j)+.1;
  if(&J) for(uint j=0;j<J.d0;j++) J[j]() *= -1.;
}
