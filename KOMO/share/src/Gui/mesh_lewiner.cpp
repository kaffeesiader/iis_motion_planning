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


#include "mesh.h"

#ifdef MT_extern_Lewiner
#  include <extern/Lewiner/MarchingCubes.h>


void ors::Mesh::setImplicitSurface(ScalarFunction& f, double lo, double hi, uint res) {
  MarchingCubes mc(res, res, res);
  mc.init_all() ;
  
  //compute data
  uint k=0, j=0, i=0;
  float x=lo, y=lo, z=lo;
  for(k=0; k<res; k++) {
    z = lo+k*(hi-lo)/res;
    for(j=0; j<res; j++) {
      y = lo+j*(hi-lo)/res;
      for(i=0; i<res; i++) {
        x = lo+i*(hi-lo)/res;
        mc.set_data(f.fs(NoArr, NoArr, ARR(x, y, z)), i, j, k) ;
      }
    }
  }
  
  mc.run();
  mc.clean_temps();
  
  //convert to Mesh
  clear();
  V.resize(mc.nverts(), 3);
  T.resize(mc.ntrigs(), 3);
  for(i=0; i<V.d0; i++) {
    V(i, 0)=lo+mc.vert(i)->x*(hi-lo)/res;
    V(i, 1)=lo+mc.vert(i)->y*(hi-lo)/res;
    V(i, 2)=lo+mc.vert(i)->z*(hi-lo)/res;
  }
  for(i=0; i<T.d0; i++) {
    T(i, 0)=mc.trig(i)->v1;
    T(i, 1)=mc.trig(i)->v2;
    T(i, 2)=mc.trig(i)->v3;
  }
}

#else //extern_Lewiner
void ors::Mesh::setImplicitSurface(ScalarFunction& f, double lo, double hi, uint res) {
  NICO
}
#endif
/** @} */
