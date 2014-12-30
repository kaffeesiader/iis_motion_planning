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
 * @ingroup group_ors
 */
/**
 * @ingroup group_ors
 * @{
 */


#include "ors.h"
#include <Gui/opengl.h>
#include <iomanip>

//global options
bool orsDrawJoints=true, orsDrawShapes=true, orsDrawBodies=true, orsDrawProxies=true, orsDrawMarkers=true;
bool orsDrawMeshes=true, orsDrawZlines=false;
bool orsDrawBodyNames=false;
double orsDrawAlpha=1.00;
uint orsDrawLimit=0;

#ifdef MT_GL
#  include <GL/gl.h>
#  include <GL/glu.h>

extern void glDrawRect(float, float, float, float, float, float,
                       float, float, float, float, float, float);

extern void glDrawText(const char* txt, float x, float y, float z);

//void glColor(float *rgb);//{ glColor(rgb[0], rgb[1], rgb[2], 1.); }

#ifndef MT_ORS_ONLY_BASICS

/**
 * @brief Bind ors to OpenGL.
 * Afterwards OpenGL can show the ors graph.
 *
 * @param graph the ors graph.
 * @param gl OpenGL which shows the ors graph.
 */
void bindOrsToOpenGL(ors::KinematicWorld& graph, OpenGL& gl) {
  gl.add(glStandardScene, 0);
  gl.add(ors::glDrawGraph, &graph);
  gl.setClearColors(1., 1., 1., 1.);

  ors::Body* glCamera = graph.getBodyByName("glCamera");
  if(glCamera) {
    *(gl.camera.X) = glCamera->X;
  } else {
    gl.camera.setPosition(10., -15., 8.);
    gl.camera.focus(0, 0, 1.);
    gl.camera.upright();
  }
  gl.update();
}
#endif

#ifndef MT_ORS_ONLY_BASICS

/// static GL routine to draw a ors::KinematicWorld
void ors::glDrawGraph(void *classP) {
  ((ors::KinematicWorld*)classP)->glDraw();
}

void glDrawShape(ors::Shape *s) {
  //set name (for OpenGL selection)
  glPushName((s->index <<2) | 1);
  glColor(s->color[0], s->color[1], s->color[2], orsDrawAlpha);

  double scale=.33*(s->size[0]+s->size[1]+s->size[2] + 2.*s->size[3]); //some scale
  if(!scale) scale=1.;
  scale*=.3;

  double GLmatrix[16];
  s->X.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);

  if(!orsDrawShapes) {
    glDrawAxes(scale);
    glColor(0, 0, .5);
    glDrawSphere(.1*scale);
  }
  if(orsDrawShapes) {
    switch(s->type) {
      case ors::noneST: break;
      case ors::boxST:
        if(orsDrawMeshes && s->mesh.V.N) s->mesh.glDraw();
        else glDrawBox(s->size[0], s->size[1], s->size[2]);
        break;
      case ors::sphereST:
        if(orsDrawMeshes && s->mesh.V.N) s->mesh.glDraw();
        else glDrawSphere(s->size[3]);
        break;
      case ors::cylinderST:
        if(orsDrawMeshes && s->mesh.V.N) s->mesh.glDraw();
        else glDrawCylinder(s->size[3], s->size[2]);
        break;
      case ors::cappedCylinderST:
        if(orsDrawMeshes && s->mesh.V.N) s->mesh.glDraw();
        else glDrawCappedCylinder(s->size[3], s->size[2]);
        break;
      case ors::markerST:
        if(orsDrawMarkers){
          glDrawDiamond(s->size[0]/5., s->size[0]/5., s->size[0]/5.); glDrawAxes(s->size[0]);
        }
        break;
      case ors::meshST:
        CHECK(s->mesh.V.N, "mesh needs to be loaded to draw mesh object");
        s->mesh.glDraw();
        break;
      case ors::pointCloudST:
        CHECK(s->mesh.V.N, "mesh needs to be loaded to draw point cloud object");
        glDrawPointCloud(s->mesh.V, NoArr);
        break;
      default: HALT("can't draw that geom yet");
    }
  }
  if(orsDrawZlines) {
    glColor(0, .7, 0);
    glBegin(GL_LINES);
    glVertex3d(0., 0., 0.);
    glVertex3d(0., 0., -s->X.pos.z);
    glEnd();
  }

  glColor(1,1,1);
  if(orsDrawBodyNames && s->body) glDrawText(s->body->name, 0, 0, 0);

  glPopName();
}

/// GL routine to draw a ors::KinematicWorld
void ors::KinematicWorld::glDraw() {
  uint i=0;
  ors::Transformation f;
  double GLmatrix[16];

  glPushMatrix();

  //bodies
  if(orsDrawBodies) for(Shape *s: shapes) {
    glDrawShape(s);
    i++;
    if(orsDrawLimit && i>=orsDrawLimit) break;
  }

  //joints
  if(orsDrawJoints) for(Joint *e: joints) {
    //set name (for OpenGL selection)
    glPushName((e->index <<2) | 2);

    double s=e->A.pos.length()+e->B.pos.length(); //some scale
    s*=.25;

    //from body to joint
    f=e->from->X;
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glColor(1, 1, 0);
    //glDrawSphere(.1*s);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(e->A.pos.x, e->A.pos.y, e->A.pos.z);
    glEnd();

    //joint frame A
    f.appendTransformation(e->A);
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDrawAxes(s);
    glColor(1, 0, 0);
    glRotatef(90, 0, 1, 0);  glDrawCylinder(.05*s, .3*s);  glRotatef(-90, 0, 1, 0);

    //joint frame B
    f.appendTransformation(e->Q);
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDrawAxes(s);

    //from joint to body
    glColor(1, 0, 1);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(e->B.pos.x, e->B.pos.y, e->B.pos.z);
    glEnd();
    glTranslatef(e->B.pos.x, e->B.pos.y, e->B.pos.z);
    //glDrawSphere(.1*s);

    glPopName();
    i++;
    if(orsDrawLimit && i>=orsDrawLimit) break;
  }

  //proxies
  if(orsDrawProxies) for(Proxy *proxy: proxies) {
    glLoadIdentity();
    if(!proxy->colorCode) glColor(.75,.75,.75);
    else glColor(proxy->colorCode);
    glBegin(GL_LINES);
    glVertex3dv(proxy->posA.p());
    glVertex3dv(proxy->posB.p());
    glEnd();
    ors::Transformation f;
    f.pos=proxy->posA;
    f.rot.setDiff(ors::Vector(0, 0, 1), proxy->posA-proxy->posB);
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDisable(GL_CULL_FACE);
    glDrawDisk(.02);
    glEnable(GL_CULL_FACE);

    f.pos=proxy->posB;
    f.getAffineMatrixGL(GLmatrix);
    glLoadMatrixd(GLmatrix);
    glDrawDisk(.02);
  }

  glPopMatrix();
}

void displayState(const arr& x, ors::KinematicWorld& G, const char *tag){
  G.setJointState(x);
  G.gl().watch(tag);
}

void displayTrajectory(const arr& _x, int steps, ors::KinematicWorld& G, const char *tag, double delay, uint dim_z) {
 uint k, t, T=_x.d0-1;
  if(!steps) return;
  uint num;
  if(steps==1 || steps==-1) num=T; else num=steps;
  for(k=0; k<=(uint)num; k++) {
    t = k*T/num;
    G.setJointState(_x[t]);
    G.gl().update(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
    if(delay) MT::wait(delay);
  }
  if(steps==1)
    G.gl().watch(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
}

/* please don't remove yet: code for displaying edges might be useful...

void glDrawOdeWorld(void *classP){
  _glDrawOdeWorld((dWorldID)classP);
}

void _glDrawOdeWorld(dWorldID world)
{
  glStandardLight();
  glColor(3);
  glDrawFloor(4);
  uint i;
  Color c;
  dVector3 vec, vec2;
  dBodyID b;
  dGeomID g, gg;
  dJointID j;
  dReal a, al, ah, r, len;
  glPushName(0);
  int t;

  //bodies
  for(i=0, b=world->firstbody;b;b=(dxBody*)b->next){
    i++;
    glPushName(i);

    //if(b->userdata){ glDrawBody(b->userdata); }
    c.setIndex(i); glColor(c.r, c.g, c.b);
    glShadeModel(GL_FLAT);

    //bodies
    for(g=b->geom;g;g=dGeomGetBodyNext(g)){
      if(dGeomGetClass(g)==dGeomTransformClass){
  ((dxGeomTransform*)g)->computeFinalTx();
        glTransform(((dxGeomTransform*)g)->final_pos, ((dxGeomTransform*)g)->final_R);
  gg=dGeomTransformGetGeom(g);
      }else{
  glTransform(g->pos, g->R);
  gg=g;
      }
      b = dGeomGetBody(gg);
      // set the color of the body, 4. Mar 06 (hh)
      c.r = ((Body*)b->userdata)->cr;
      c.g = ((Body*)b->userdata)->cg;
      c.b = ((Body*)b->userdata)->cb;
      glColor(c.r, c.g, c.b);

      switch(dGeomGetClass(gg))
  {
  case dSphereClass:
    glDrawSphere(dGeomSphereGetRadius(gg));
    break;
  case dBoxClass:
    dGeomBoxGetLengths(gg, vec);
    glDrawBox(vec[0], vec[1], vec[2]);
    break;
  case dCCylinderClass: // 6. Mar 06 (hh)
    dGeomCCylinderGetParams(gg, &r, &len);
    glDrawCappedCylinder(r, len);
    break;
  default: HALT("can't draw that geom yet");
  }
      glPopMatrix();
    }

    // removed shadows,  4. Mar 06 (hh)

    // joints

      dxJointNode *n;
      for(n=b->firstjoint;n;n=n->next){
      j=n->joint;
      t=dJointGetType(j);
      if(t==dJointTypeHinge){
      dJointGetHingeAnchor(j, vec);
      a=dJointGetHingeAngle(j);
      al=dJointGetHingeParam(j, dParamLoStop);
      ah=dJointGetHingeParam(j, dParamHiStop);
      glPushMatrix();
      glTranslatef(vec[0], vec[1], vec[2]);
      dJointGetHingeAxis(j, vec);
      glBegin(GL_LINES);
      glColor3f(1, 0, 0);
      glVertex3f(0, 0, 0);
      glVertex3f(LEN*vec[0], LEN*vec[1], LEN*vec[2]);
      glEnd();
      //glDrawText(STRING(al <<'<' <<a <<'<' <<ah), LEN*vec[0], LEN*vec[1], LEN*vec[2]);
      glPopMatrix();
      }
      if(t==dJointTypeAMotor){
  glPushMatrix();
  glTranslatef(b->pos[0], b->pos[1], b->pos[2]);
  dJointGetAMotorAxis(j, 0, vec);
  glBegin(GL_LINES);
  glColor3f(1, 1, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(LEN*vec[0], LEN*vec[1], LEN*vec[2]);
  glEnd();
  glPopMatrix();
      }
      if(t==dJointTypeBall){
  dJointGetBallAnchor(j, vec);
  dJointGetBallAnchor2(j, vec2);
  glPushMatrix();
  glTranslatef(vec[0], vec[1], vec[2]);
  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  glVertex3f(-.05, 0, 0);
  glVertex3f(.05, 0, 0);
  glVertex3f(0, -.05, 0);
  glVertex3f(0, .05, 0);
  glVertex3f(0, 0, -.05);
  glVertex3f(0, 0, .05);
  glEnd();
  glPopMatrix();
  glPushMatrix();
  glTranslatef(vec2[0], vec2[1], vec2[2]);
  glBegin(GL_LINES);
  glColor3f(1, 0, 0);
  glVertex3f(-.05, 0, 0);
  glVertex3f(.05, 0, 0);
  glVertex3f(0, -.05, 0);
  glVertex3f(0, .05, 0);
  glVertex3f(0, 0, -.05);
  glVertex3f(0, 0, .05);
  glEnd();
  glPopMatrix();
      }
    }
      glPopName();
  }
  glPopName();
}
*/

void animateConfiguration(ors::KinematicWorld& C, Inotify *ino) {
  arr x, x0;
  uint t, i;
  C.getJointState(x0);
  C.gl().pressedkey=0;
  for(i=x0.N; i--;) {
    x=x0;
    for(t=0; t<20; t++) {
      if(C.gl().pressedkey==13 || C.gl().pressedkey==27) return;
      if(ino && ino->pollForModification()) return;
      x(i)=x0(i) + .5*sin(MT_2PI*t/20);
      C.setJointState(x);
      C.gl().update(STRING("joint = " <<i), false, false, true);
      MT::wait(0.01);
    }
  }
  C.setJointState(x0);
}


ors::Body *movingBody=NULL;
ors::Vector selpos;
double seld, selx, sely, selz;

struct EditConfigurationHoverCall:OpenGL::GLHoverCall {
  ors::KinematicWorld *ors;
  EditConfigurationHoverCall(ors::KinematicWorld& _ors) { ors=&_ors; }
  bool hoverCallback(OpenGL& gl) {
    if(!movingBody) return false;
    if(!movingBody) {
      ors::Joint *j=NULL;
      ors::Shape *s=NULL;
      gl.Select();
      OpenGL::GLSelect *top=gl.topSelection;
      if(!top) return false;
      uint i=top->name;
      //cout <<"HOVER call: id = 0x" <<std::hex <<gl.topSelection->name <<endl;
      if((i&3)==1) s=ors->shapes(i>>2);
      if((i&3)==2) j=ors->joints(i>>2);
      gl.text.clear();
      if(s) {
        gl.text <<"shape selection: body=" <<s->body->name <<" X=" <<s->body->X <<" ats=" <<endl;
        listWrite(s->ats, gl.text, "\n");
      }
      if(j) {
        gl.text
            <<"edge selection: " <<j->from->name <<' ' <<j->to->name
            <<"\nA=" <<j->A <<"\nQ=" <<j->Q <<"\nB=" <<j->B <<endl;
        listWrite(j->ats, gl.text, "\n");
      }
    } else {
      //gl.Select();
      //double x=0, y=0, z=seld;
      //double x=(double)gl.mouseposx/gl.width(), y=(double)gl.mouseposy/gl.height(), z=seld;
      double x=gl.mouseposx, y=gl.mouseposy, z=seld;
      gl.unproject(x, y, z, true);
      cout <<"x=" <<x <<" y=" <<y <<" z=" <<z <<" d=" <<seld <<endl;
      movingBody->X.pos = selpos + ARR(x-selx, y-sely, z-selz);
    }
    return true;
  }
};

struct EditConfigurationKeyCall:OpenGL::GLKeyCall {
  ors::KinematicWorld &ors;
  bool &exit;
  EditConfigurationKeyCall(ors::KinematicWorld& _ors, bool& _exit): ors(_ors), exit(_exit){}
  bool keyCallback(OpenGL& gl) {
    if(gl.pressedkey==' '){ //grab a body
      if(movingBody) { movingBody=NULL; return true; }
      ors::Joint *j=NULL;
      ors::Shape *s=NULL;
      gl.Select();
      OpenGL::GLSelect *top=gl.topSelection;
      if(!top) { cout <<"No object below mouse!" <<endl;  return false; }
      uint i=top->name;
      //cout <<"HOVER call: id = 0x" <<std::hex <<gl.topSelection->name <<endl;
      if((i&3)==1) s=ors.shapes(i>>2);
      if((i&3)==2) j=ors.joints(i>>2);
      if(s) {
        cout <<"selected shape " <<s->name <<" of body " <<s->body->name <<endl;
        selx=top->x;
        sely=top->y;
        selz=top->z;
        seld=top->dmin;
        cout <<"x=" <<selx <<" y=" <<sely <<" z=" <<selz <<" d=" <<seld <<endl;
        selpos = s->body->X.pos;
        movingBody=s->body;
      }
      if(j) {
        cout <<"selected joint " <<j->index <<" connecting " <<j->from->name <<"--" <<j->to->name <<endl;
      }
      return true;
    }else switch(gl.pressedkey) {
      case '1':  orsDrawBodies^=1;  break;
      case '2':  orsDrawShapes^=1;  break;
      case '3':  orsDrawJoints^=1;  break;
      case '4':  orsDrawProxies^=1;  break;
      case '5':  gl.reportSelects^=1;  break;
      case '6':  gl.reportEvents^=1;  break;
      case '7':  ors.writePlyFile("z.ply");  break;
      case 'j':  gl.camera.X->pos += gl.camera.X->rot*ors::Vector(0, 0, .1);  break;
      case 'k':  gl.camera.X->pos -= gl.camera.X->rot*ors::Vector(0, 0, .1);  break;
      case 'i':  gl.camera.X->pos += gl.camera.X->rot*ors::Vector(0, .1, 0);  break;
      case ',':  gl.camera.X->pos -= gl.camera.X->rot*ors::Vector(0, .1, 0);  break;
      case 'l':  gl.camera.X->pos += gl.camera.X->rot*ors::Vector(.1, .0, 0);  break;
      case 'h':  gl.camera.X->pos -= gl.camera.X->rot*ors::Vector(.1, 0, 0);  break;
      case 'a':  gl.camera.focus(
          (gl.camera.X->rot*(*gl.camera.foc - gl.camera.X->pos)
           ^ gl.camera.X->rot*ors::Vector(1, 0, 0)) * .001
          + *gl.camera.foc);
        break;
      case 's':  gl.camera.X->pos +=
          (
            gl.camera.X->rot*(*gl.camera.foc - gl.camera.X->pos)
            ^(gl.camera.X->rot * ors::Vector(1., 0, 0))
          ) * .01;
        break;
      case 'q' :
        cout <<"EXITING" <<endl;
        exit=true;
        break;
    }
    gl.postRedrawEvent(true);
    return true;
  }
};

void editConfiguration(const char* filename, ors::KinematicWorld& C) {
//  gl.exitkeys="1234567890qhjklias, "; //TODO: move the key handling to the keyCall!
  bool exit=false;
  C.gl().addHoverCall(new EditConfigurationHoverCall(C));
  C.gl().addKeyCall(new EditConfigurationKeyCall(C,exit));
  Inotify ino(filename);
  for(;!exit;) {
    cout <<"reloading `" <<filename <<"' ... " <<std::endl;
    try {
      MT::lineCount=1;
      C.gl().lock.writeLock();
      C <<FILE(filename);
      C.gl().lock.unlock();
    } catch(const char* msg) {
      cout <<"line " <<MT::lineCount <<": " <<msg <<" -- please check the file and press ENTER" <<endl;
      C.gl().watch();
      continue;
    }
    C.gl().update();
    cout <<"animating.." <<endl;
    //while(ino.pollForModification());
    animateConfiguration(C, &ino);
    cout <<"watching..." <<endl;
#if 0
    ino.waitForModification();
#else
    C.gl().watch();
#endif
if(!MT::getInteractivity()){
    exit=true;
}
  }
}


#if 0 //MT_ODE
void testSim(const char* filename, ors::KinematicWorld *C, Ode *ode) {
  C.gl().watch();
  uint t, T=200;
  arr x, v;
  createOde(*C, *ode);
  ors->getJointState(x, v);
  for(t=0; t<T; t++) {
    ode->step();

    importStateFromOde(*C, *ode);
    ors->setJointState(x, v);
    ors->calcBodyFramesFromJoints();
    exportStateToOde(*C, *ode);

    C.gl().text.clear() <<"time " <<t;
    C.gl().timedupdate(10);
  }
}
#endif
#endif

#else ///MT_GL
#ifndef MT_ORS_ONLY_BASICS
void bindOrsToOpenGL(ors::KinematicWorld&, OpenGL&) { NICO };
void ors::KinematicWorld::glDraw() { NICO }
void ors::glDrawGraph(void *classP) { NICO }
void editConfiguration(const char* orsfile, ors::KinematicWorld& C) { NICO }
void animateConfiguration(ors::KinematicWorld& C, OpenGL& gl) { NICO }
void glTransform(const ors::Transformation&) { NICO }
void displayTrajectory(const arr&, int, ors::KinematicWorld&, const char*, double) { NICO }
void displayState(const arr&, ors::KinematicWorld&, const char*) { NICO }
#endif
#endif
/** @} */
