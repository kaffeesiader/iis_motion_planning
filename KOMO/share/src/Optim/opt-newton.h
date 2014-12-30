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

int optNewton(arr& x, ScalarFunction& f, OptOptions opt=NOOPT);

struct OptNewton{
  arr& x;
  ScalarFunction& f;
  OptOptions o;
  arr *additionalRegularizer;

  enum StopCriterion { stopNone=0, stopCrit1, stopCrit2, stopCritEvals };
  double fx;
  arr gx, Hx;
  double alpha, lambda;
  uint it, evals;
  bool x_changed;
  ofstream fil;

  OptNewton(arr& x, ScalarFunction& f, OptOptions o=NOOPT);
  ~OptNewton();
  StopCriterion step();
  StopCriterion run();
  void reinit();
};
