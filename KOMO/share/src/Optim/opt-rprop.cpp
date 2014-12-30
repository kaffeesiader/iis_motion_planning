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

#include "opt-rprop.h"

/// minimizes \f$f(x)\f$ using its gradient only
uint optRprop(arr& x, ScalarFunction& f, OptOptions o) {
  return Rprop().loop(x, f, o.fmin_return, o.stopTolerance, o.initStep, o.stopEvals, o.verbose);
}

//===========================================================================
//
// Rprop
//

int _sgn(double x) { if(x > 0) return 1; if(x < 0) return -1; return 0; }
double _mymin(double x, double y) { return x < y ? x : y; }
double _mymax(double x, double y) { return x > y ? x : y; }

struct sRprop {
  double incr;
  double decr;
  double dMax;
  double dMin;
  double rMax;
  double delta0;
  arr lastGrad; // last error gradient
  arr stepSize; // last update
  bool step(arr& w, const arr& grad, uint *singleI);
};

Rprop::Rprop() {
  s = new sRprop;
  s->incr   = 1.2;
  s->decr   = .33;
  s->dMax = 50.;
  s->dMin = 1e-6;
  s->rMax = 0.;
  s->delta0 = 1.;
}

void Rprop::init(double initialStepSize, double minStepSize, double maxStepSize) {
  s->stepSize.resize(0);
  s->lastGrad.resize(0);
  s->delta0 = initialStepSize;
  s->dMin = minStepSize;
  s->dMax = maxStepSize;
}

bool sRprop::step(arr& w, const arr& grad, uint *singleI) {
  if(!stepSize.N) { //initialize
    stepSize.resize(w.N);
    lastGrad.resize(w.N);
    lastGrad.setZero();
    stepSize = delta0;
  }
  CHECK(grad.N==stepSize.N, "Rprop: gradient dimensionality changed!");
  CHECK(w.N==stepSize.N   , "Rprop: parameter dimensionality changed!");

  uint i=0, I=w.N;
  if(singleI) { i=*(singleI); I=i+1; }
  for(; i<I; i++) {
    if(grad.elem(i) * lastGrad(i) > 0) { //same direction as last time
      if(rMax) dMax=fabs(rMax*w.elem(i));
      stepSize(i) = _mymin(dMax, incr * stepSize(i)); //increase step size
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction
      lastGrad(i) = grad.elem(i);                    //memorize gradient
    } else if(grad.elem(i) * lastGrad(i) < 0) { //change of direction
      stepSize(i) = _mymax(dMin, decr * stepSize(i)); //decrease step size
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction (undo half the step)
      lastGrad(i) = 0;                               //memorize to continue below next time
    } else {                              //after change of direcion
      w.elem(i) += stepSize(i) * -_sgn(grad.elem(i)); //step in right direction
      lastGrad(i) = grad.elem(i);                    //memorize gradient
    }
  }

  return stepSize.max() < incr*dMin;
}

bool Rprop::step(arr& x, ScalarFunction& f) {
  arr grad;
  f.fs(grad, NoArr, x);
  return s->step(x, grad, NULL);
}

//----- the rprop wrapped with stopping criteria
uint Rprop::loop(arr& _x,
                 ScalarFunction& f,
                 double *fmin_return,
                 double stoppingTolerance,
                 double initialStepSize,
                 uint maxEvals,
                 uint verbose) {

  if(!s->stepSize.N) init(initialStepSize);
  arr x, J(_x.N), x_min, J_min;
  double fx, fx_min=0;
  uint rejects=0, small_steps=0;
  x=_x;

  if(verbose>1) cout <<"*** optRprop: starting point x=" <<x <<endl;
  ofstream fil;
  if(verbose>0) fil.open("z.opt");

  uint evals=0;
  double diff=0.;
  for(;;) {
    //checkGradient(p, x, stoppingTolerance);
    //compute value and gradient at x
    fx = f.fs(J, NoArr, x);  evals++;

    if(verbose>0) fil <<evals <<' ' <<eval_cost <<' ' << fx <<' ' <<diff <<' ' <<x <<endl;
    if(verbose>1) cout <<"optRprop " <<evals <<' ' <<eval_cost <<" \tf(x)=" <<fx <<" \tdiff=" <<diff <<" \tx=" <<x <<endl;

    //infeasible point! undo the previous step
    if(fx!=fx) { //is NAN
      if(!evals) HALT("can't start Rprop with unfeasible point");
      s->stepSize*=(double).1;
      s->lastGrad=(double)0.;
      x=x_min;
      fx=fx_min;
      J=J_min;
      rejects=0;
    }

    //update best-so-far
    if(evals<=1) { fx_min= fx; x_min=x; }
    if(fx<=fx_min) {
      x_min=x;
      fx_min=fx;
      J_min=J;
      rejects=0;
    } else {
      rejects++;
      if(rejects>10) {
        s->stepSize*=(double).1;
        s->lastGrad=(double)0.;
        x=x_min;
        fx=fx_min;
        J=J_min;
        rejects=0;
      }
    }

    //update x
    s->step(x, J, NULL);

    //check stopping criterion based on step-length in x
    diff=maxDiff(x, x_min);

    if(diff<stoppingTolerance) { small_steps++; } else { small_steps=0; }
    if(small_steps>3)  break;
    if(evals>maxEvals) break;
  }
  if(verbose>0) fil.close();
  if(verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", NULL, true);
  if(fmin_return) *fmin_return= fx_min;
  _x=x_min;
  return evals;
}
