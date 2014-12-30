/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
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


void glDrawFloor(float, float, float, float){ NICO }
void glGrabImage(MT::Array<unsigned char>&){ NICO }
void glStandardLight(void*){ NICO }
void glDrawAxes(double){ NICO }
void glDrawPointCloud(MT::Array<double>&, MT::Array<double>&) { NICO }
void glDrawSphere(float) { NICO }
void glDrawCappedCylinder(float, float) { NICO }
void glDrawText(char const*, float, float, float){ NICO }
void glDrawDiamond(float, float, float){ NICO }
void glDrawDisk(float){ NICO }
void glDrawBox(float, float, float){ NICO }
void glDrawCylinder(float, float, bool){ NICO }

//void OpenGL::watchImage(MT::Array<unsigned char> const&, bool, float){}

struct sOpenGL {
  sOpenGL(OpenGL *gl, const char* title, int w, int h, int posx, int posy){
    MT_MSG("creating dummy OpenGL object");
    gl->width=1;
    gl->height=1;
  }
  sOpenGL(OpenGL *gl, void *container){
    MT_MSG("creating dummy OpenGL object");
    gl->width=1;
    gl->height=1;
  }
  ors::Vector downVec, downPos, downFoc;
  ors::Quaternion downRot;
};

void OpenGL::postRedrawEvent(bool){}
void OpenGL::processEvents(){}
void OpenGL::enterEventLoop(){}
void OpenGL::exitEventLoop(){}
void OpenGL::resize(int w,int h){}

void initGlEngine(){}
