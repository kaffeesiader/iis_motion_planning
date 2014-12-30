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

#include <Core/array.h>

/// Wrapper for CMA stochastic optimization
struct SearchCMA{
  struct sSearchCMA *s;

  SearchCMA();
  ~SearchCMA();

  /// D=problem dimension, lambda=#samples taken in each iteration, mu \approx lambda/3 specifies the selection size, lo and hi specify an initialization range
  void init(uint D, int mu=-1, int lambda=-1, double lo=-1., double hi=1.);

  /// instead of lo and hi, explicitly give a start point and standard deviation around this point
  void init(uint D, int mu, int lambda, const arr &startPoint, double _startDev);

  /// first call: generate initial random samples
  /// further calls: costs needs to contain the cost function values for all elements in samples; returns a new set of samples
  void step(arr& samples, arr& costs);
};
