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

#include "optimization.h"

extern const char* MethodName[];

//==============================================================================
//
// UnconstrainedProblem
//
// we define an unconstraint optimization problem from a constrained one
// that can include penalties, log barriers, and augmented lagrangian terms
//

struct UnconstrainedProblem : ScalarFunction{
  /** The VectorFunction F describes the cost function f(x) as well as the constraints g(x)
      concatenated to one vector:
      phi(0) = cost,   phi(1,..,phi.N-1) = constraints */
  ConstrainedProblem &P;
  //-- parameters of the unconstrained meta function F
  double muLB;       ///< log barrier weight
  double mu;         ///< squared penalty weight
  arr lambda;        ///< lagrange multiplier in augmented lagrangian

  //-- buffers to avoid recomputing gradients
  arr x, df_x, Hf_x, g_x, Jg_x;
  double f_x;

  UnconstrainedProblem(ConstrainedProblem &_P):P(_P), muLB(0.), mu(0.) {}

  virtual double fs(arr& dL, arr& HL, const arr& x); ///< the unconstrained meta function F

  void aulaUpdate(double lambdaStepsize=1., arr &x_reeval=NoArr);
  void anyTimeAulaUpdate(double lambdaStepsize=1., double muInc=1., double *L_x=NULL, arr &dL_x=NoArr, arr &HL_x=NoArr);
};


//==============================================================================
//
// PhaseOneProblem
//
// we define a constraint optimization problem that corresponds
// to the phase one problem of another constraint problem
//

struct PhaseOneProblem:ConstrainedProblem{
  ConstrainedProblem &f;

  PhaseOneProblem(ConstrainedProblem &_f):f(_f) {}

  virtual double fc(arr& df, arr& Hf, arr& g, arr& Jg, const arr& x);
  virtual uint dim_x(){ return f.dim_x()+1; }
  virtual uint dim_g(){ return f.dim_g()+1; }
};


//==============================================================================
//
// Solvers
//

uint optConstrained(arr& x, arr &dual, ConstrainedProblem& P, OptOptions opt=NOOPT);


//==============================================================================
//
// evaluating
//

inline void evaluateConstrainedProblem(const arr& x, ConstrainedProblem& P, std::ostream& os){
  arr g;
  double f = P.fc(NoArr, NoArr, g, NoArr, x);
  os <<"f=" <<f <<" compl="<<sum(elemWiseMax(g,zeros(g.N,1))) <<endl;
}


