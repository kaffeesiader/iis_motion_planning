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




/**
 * @file
 * @ingroup group_geo
 */
/**
 * @ingroup group_geo
 * @{
 */


#include "mesh.h"
#include "opengl.h"

//global options
bool orsDrawJoints=true, orsDrawShapes=true, orsDrawBodies=true, orsDrawProxies=true;
bool orsDrawMeshes=true, orsDrawZlines=false;
double orsDrawAlpha=1.00;
uint orsDrawLimit=0;

#ifdef MT_GL
#  include <GL/gl.h>
#  include <GL/glu.h>

extern void glDrawRect(float, float, float, float, float, float,
                       float, float, float, float, float, float);

extern void glDrawText(const char* txt, float x, float y, float z);

#endif
/** @} */
