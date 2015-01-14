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
 * @addtogroup group_ors
 * @{
 */


#undef abs
#include <algorithm>
#include <sstream>
#include <climits>
#include "ors.h"
#include "ors_swift.h"
#include "ors_physx.h"
#include "ors_ode.h"
#include <Gui/opengl.h>
#include <Algo/algos.h>

#ifndef MT_ORS_ONLY_BASICS
#  include <Core/registry.h>
//#  include <Gui/plot.h>
#endif
#ifdef MT_extern_ply
#  include <extern/ply/ply.h>
#endif

#define ORS_NO_DYNAMICS_IN_FRAMES

#define SL_DEBUG_LEVEL 1
#define SL_DEBUG(l, x) if(l<=SL_DEBUG_LEVEL) x;

#define Qstate

void lib_ors(){ cout <<"force loading lib/ors" <<endl; }

#define LEN .2

#ifndef MT_ORS_ONLY_BASICS

uint ors::KinematicWorld::setJointStateCount = 0;

//===========================================================================
//
// contants
//

ors::Body& NoBody = *((ors::Body*)NULL);
ors::Shape& NoShape = *((ors::Shape*)NULL);
ors::Joint& NoJoint = *((ors::Joint*)NULL);
ors::KinematicWorld& NoGraph = *((ors::KinematicWorld*)NULL);

//===========================================================================
//
// Body implementations
//

//ors::Body::Body() { reset(); }

//ors::Body::Body(const Body& b) { reset(); *this=b; }

ors::Body::Body(KinematicWorld& _world, const Body* copyBody):world(_world) {
  reset();
  index=world.bodies.N;
  world.bodies.append(this);
  if(copyBody) *this=*copyBody;
}

ors::Body::~Body() {
  reset();
  while(inLinks.N) delete inLinks.last();
  while(outLinks.N) delete outLinks.last();
  while(shapes.N) delete shapes.last();
  world.bodies.removeValue(this);
  listReindex(world.bodies);
}

void ors::Body::reset() {
  listDelete(ats);
  X.setZero();
  type=dynamicBT;
  shapes.memMove=true;
  com.setZero();
  mass = 0.;
  inertia.setZero();
}

void ors::Body::parseAts(KinematicWorld& G) {
  //interpret some of the attributes
  arr x;
  MT::String str;
  ats.getValue<Transformation>(X, "X");
  ats.getValue<Transformation>(X, "pose");
  
  //mass properties
  double d;
  if(ats.getValue<double>(d, "mass")) {
    mass=d;
    inertia.setId();
    inertia *= .2*d;
  }

  type=dynamicBT;
  if(ats.getValue<bool>("fixed"))      type=staticBT;
  if(ats.getValue<bool>("static"))     type=staticBT;
  if(ats.getValue<bool>("kinematic"))  type=kinematicBT;
  
  // SHAPE handling
  Item* item;
  // a mesh which consists of multiple convex sub meshes creates multiple
  // shapes that belong to the same body
  item = ats.getItem("meshes");
  if(item){
    MT::FileToken *file = item->getValue<MT::FileToken>();
    CHECK(file,"somethings wrong");

    // if mesh is not .obj we only have one shape
    if(!file->name.endsWith("obj")) {
      new Shape(G, *this);
    }else{  // if .obj file create Shape for all submeshes
      auto subMeshPositions = getSubMeshPositions(file->name);
      for(uint i=0;i<subMeshPositions.d0;i++){
        auto parsing_pos = subMeshPositions[i];
        Shape *s = new Shape(G, *this);
        s->mesh.parsing_pos_start = parsing_pos(0);
        s->mesh.parsing_pos_end = parsing_pos(1);
        s->mesh.readObjFile(file->getIs());
        s->mesh.makeConvexHull();
        s->type=meshST;
      }
    }
  }

  // add shape if there is no shape exists yet
  if(ats.getItem("type") && !shapes.N){
    Shape *s = new Shape(G, *this);
    s->name = name;
  }

  // copy body attributes to shapes 
  for(Shape *s:shapes) { s->ats=ats;  s->parseAts(); }
  listDelete(ats);
}

void ors::Body::write(std::ostream& os) const {
  if(!X.isZero()) os <<"pose=<T " <<X <<" > ";
  if(mass) os <<"mass=" <<mass <<' ';
  if(type!=dynamicBT) os <<"type=" <<(int)type <<' ';
//  uint i; Item *a;
//  for_list(Type,  a,  ats)
//      if(a->keys(0)!="X" && a->keys(0)!="pose") os <<*a <<' ';
}

void ors::Body::read(std::istream& is) {
  reset();
  ats.read(is);
  if(!is.good()) HALT("body '" <<name <<"' read error: in ");
  parseAts(NoGraph);
}

namespace ors {
std::ostream& operator<<(std::ostream& os, const Body& x) { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Shape& x) { x.write(os); return os; }
std::ostream& operator<<(std::ostream& os, const Joint& x) { x.write(os); return os; }
}

//===========================================================================
//
// Shape implementations
//

ors::Shape::Shape(KinematicWorld &_world, Body& b, const Shape *copyShape, bool referenceMeshOnCopy): world(_world), /*ibody(UINT_MAX),*/ body(NULL) {
  reset();
  CHECK(&world,"you need at least a world to attach this shape to!");
  index=world.shapes.N;
  world.shapes.append(this);
  if(&b){
    body = &b;
    b.shapes.append(this);
  }
  if(copyShape) copy(*copyShape, referenceMeshOnCopy);
}

ors::Shape::~Shape() {
  reset();
  if(body){
    body->shapes.removeValue(this);
    listReindex(body->shapes);
  }
  world.shapes.removeValue(this);
  listReindex(world.shapes);
}

void ors::Shape::copy(const Shape& s, bool referenceMeshOnCopy){
  name=s.name; X=s.X; rel=s.rel; type=s.type;
  memmove(size, s.size, 4*sizeof(double)); memmove(color, s.color, 3*sizeof(double));
  if(!referenceMeshOnCopy){
    mesh=s.mesh;
  }else{
    mesh.V.referTo(s.mesh.V);
    mesh.T.referTo(s.mesh.T);
    mesh.C.referTo(s.mesh.C);
    mesh.Vn.referTo(s.mesh.Vn);
  }
  mesh_radius=s.mesh_radius; cont=s.cont;
  ats=s.ats;
}

void ors::Shape::parseAts() {
  double d;
  arr x;
  MT::String str;
  MT::FileToken *fil;
  ats.getValue<Transformation>(rel, "rel");
  if(ats.getValue<arr>(x, "size"))          { CHECK(x.N==4,"size=[] needs 4 entries"); memmove(size, x.p, 4*sizeof(double)); }
  if(ats.getValue<arr>(x, "color"))         { CHECK(x.N==3,"color=[] needs 3 entries"); memmove(color, x.p, 3*sizeof(double)); }
  if(ats.getValue<double>(d, "type"))       { type=(ShapeType)(int)d;}
  if(ats.getValue<bool>("contact"))         { cont=true; }
  fil=ats.getValue<MT::FileToken>("mesh");  if(fil) { mesh.read(fil->getIs(), fil->name.getLastN(3).p); }
  if(ats.getValue<double>(d, "meshscale"))  { mesh.scale(d); }

  //create mesh for basic shapes
  switch(type) {
    case ors::noneST: HALT("shapes should have a type - somehow wrong initialization..."); break;
    case ors::boxST:
      mesh.setBox();
      mesh.scale(size[0], size[1], size[2]);
      break;
    case ors::sphereST:
      mesh.setSphere();
      mesh.scale(size[3], size[3], size[3]);
      break;
    case ors::cylinderST:
      CHECK(size[3]>1e-10,"");
      mesh.setCylinder(size[3], size[2]);
      break;
    case ors::cappedCylinderST:
      CHECK(size[3]>1e-10,"");
      mesh.setCappedCylinder(size[3], size[2]);
      break;
    case ors::markerST:
      break;
    case ors::meshST:
    case ors::pointCloudST:
      CHECK(mesh.V.N, "mesh needs to be loaded to draw mesh object");
      break;
  }

  //center the mesh:
  if(mesh.V.N){
    Vector c = mesh.center();
    if(!ats.getValue<bool>("rel_includes_mesh_center")){
      rel.addRelativeTranslation(c);
      ats.append<bool>("rel_includes_mesh_center",new bool(true));
    }
    mesh_radius = mesh.getRadius();
  }

  //add inertia to the body
  if(body) {
    Matrix I;
    double mass=-1.;
    switch(type) {
      case sphereST:   inertiaSphere(I.p(), mass, 1000., size[3]);  break;
      case boxST:      inertiaBox(I.p(), mass, 1000., size[0], size[1], size[2]);  break;
      case cappedCylinderST:
      case cylinderST: inertiaCylinder(I.p(), mass, 1000., size[2], size[3]);  break;
      case noneST:
      default: ;
    }
    if(mass>0.){
      body->mass += mass;
      body->inertia += I;
    }
  }
}

void ors::Shape::reset() {
  type=noneST;
  size[0]=size[1]=size[2]=size[3]=1.;
  color[0]=color[1]=color[2]=.8;
  listDelete(ats);
  rel.setZero();
  mesh.V.clear();
  mesh_radius=0.;
  cont=false;
}

void ors::Shape::write(std::ostream& os) const {
  os <<"type=" <<type <<' ';
  if(!rel.isZero()) os <<"rel=<T " <<rel <<" > ";
  for_list(Item, a, ats)
  if(a->keys(0)!="rel" && a->keys(0)!="type") os <<*a <<' ';
}

void ors::Shape::read(std::istream& is) {
  reset();
  ats.read(is);
  if(!is.good()) HALT("shape read error");
  parseAts();
}

uintA stringListToShapeIndices(const MT::Array<const char*>& names, const MT::Array<ors::Shape*>& shapes) {
  uintA I(names.N);
  for(uint i=0; i<names.N; i++) {
    ors::Shape *s = listFindByName(shapes, names(i));
    if(!s) HALT("shape name '"<<names(i)<<"' doesn't exist");
    I(i) = s->index;
  }
  return I;
}

void makeConvexHulls(ShapeL& shapes){
  for(ors::Shape *s: shapes) s->mesh.makeConvexHull();
}


//===========================================================================
//
// Joint implementations
//

bool always_unlocked(void*) { return false; }

//ors::Joint::Joint(KinematicWorld& G)
//  : world(G), index(0), qIndex(UINT_MAX), ifrom(0), ito(0), from(NULL), to(NULL), mimic(NULL), agent(0), locked_func(always_unlocked), locked_data(NULL), H(1.) {
//  reset();
//  index=G.joints.N;
//  G.joints.append(this);
//}

//ors::Joint::Joint(KinematicWorld& G, const Joint& j)
//  : world(G), index(0), qIndex(UINT_MAX), ifrom(0), ito(0), from(NULL), to(NULL), mimic(NULL), agent(0), locked_func(always_unlocked), locked_data(NULL), H(1.) { reset(); *this=j; }

ors::Joint::Joint(KinematicWorld& G, Body *f, Body *t, const Joint* copyJoint)
  : world(G), index(0), qIndex(UINT_MAX), /*ifrom(f->index), ito(t->index),*/ from(f), to(t), mimic(NULL), agent(0), locked_func(always_unlocked), locked_data(NULL), H(1.) {
  reset();
  if(copyJoint) *this=*copyJoint;
  index=G.joints.N;
  G.joints.append(this);
  f->outLinks.append(this);
  t-> inLinks.append(this);
  G.qdim.clear();
}

ors::Joint::~Joint() {
  reset();
  world.checkConsistency();
  if(from){ from->outLinks.removeValue(this); listReindex(from->outLinks); }
  if(to){   to->inLinks.removeValue(this); listReindex(to->inLinks); }
  world.joints.removeValue(this);
  listReindex(world.joints);
  world.qdim.clear();
}
void ors::Joint::reset() { 
  listDelete(ats); A.setZero(); B.setZero(); Q.setZero(); X.setZero(); axis.setZero(); limits.clear(); H=1.; type=JT_none; 
  locked_func=always_unlocked; locked_data=NULL;
}

void ors::Joint::parseAts() {
  //interpret some of the attributes
  double d;
  ats.getValue<Transformation>(A, "A");
  ats.getValue<Transformation>(A, "from");
  if(ats.getValue<bool>("BinvA")) B.setInverse(A);
  ats.getValue<Transformation>(B, "B");
  ats.getValue<Transformation>(B, "to");
  ats.getValue<Transformation>(Q, "Q");
  ats.getValue<Transformation>(X, "X");
  ats.getValue<double>(H, "ctrl_H");
  if(ats.getValue<double>(d, "type")) type=(JointType)(int)d; else type=JT_hingeX;
  if(type==JT_fixed && !Q.isZero()){ A.appendTransformation(Q); Q.setZero(); }
  if(ats.getValue<double>(d, "q")){
    if(type==JT_hingeX) Q.addRelativeRotationRad(d, 1., 0., 0.);
    if(type==JT_fixed)  A.addRelativeRotationRad(d, 1., 0., 0.);
    if(type==JT_transX) Q.addRelativeTranslation(d, 0., 0.);
  }
  if(ats.getValue<double>(d, "agent")) agent=(uint)d;
  if(ats.getValue<bool>("fixed")) agent=UINT_MAX;
  //axis
  arr axis;
  ats.getValue<arr>(axis, "axis");
  if(axis.N) {
    CHECK(axis.N==3,"");
    Vector ax(axis);
    Transformation f;
    f.setZero();
    f.rot.setDiff(Vector_x, ax);
    A = A * f;
    B = -f * B;
  }
  //limit
  arr ctrl_limits;
  ats.getValue<arr>(limits, "limits");
  if(limits.N){
    CHECK(limits.N==2*qDim(), "parsed limits have wrong dimension");
  }
  ats.getValue<arr>(ctrl_limits, "ctrl_limits");
  if(ctrl_limits.N){
    if(!limits.N) limits.resizeAs(ctrl_limits).setZero();
    CHECK(limits.N==ctrl_limits.N, "parsed ctrl_limits have wrong dimension");
    limits.append(ctrl_limits);
  }
  //coupled to another joint requires post-processing by the Graph::read!!
  if(ats.getValue<MT::String>("mimic")) mimic=(Joint*)1;
}

uint ors::Joint::qDim() {
  if(type>=JT_hingeX && type<=JT_transZ) return 1;
  if(type==JT_transXY) return 2;
  if(type==JT_transXYPhi) return 3;
  if(type==JT_trans3) return 3;
  if(type==JT_universal) return 2;
  if(type==JT_quatBall) return 4;
  if(type==JT_glue || type==JT_fixed) return 0;
  HALT("shouldn't be here");
  return 0;
}

void ors::Joint::write(std::ostream& os) const {
  os <<"type=" <<type <<' ';
  if(!A.isZero()) os <<"from=<T " <<A <<" > ";
  if(!B.isZero()) os <<"to=<T " <<B <<" > ";
  if(!Q.isZero()) os <<"Q=<T " <<Q <<" > ";
  for_list(Item, a, ats)
  if(a->keys(0)!="A" && a->keys(0)!="from"
      && a->keys(0)!="axis" //because this was subsumed in A during read
      && a->keys(0)!="B" && a->keys(0)!="to"
      && a->keys(0)!="Q" && a->keys(0)!="q"
      && a->keys(0)!="type") os <<*a <<' ';
}

void ors::Joint::read(std::istream& is) {
  reset();
  ats.read(is);
  if(!is.good()) HALT("joint (" <<from->name <<' ' <<to->name <<") read read error");
  parseAts();
}


ors::Proxy::Proxy() {
  colorCode = 0;
}

//===========================================================================
//
// Graph implementations
//

namespace ors{
struct sKinematicWorld{
  OpenGL *gl;
  SwiftInterface *swift;
  PhysXInterface *physx;
  OdeInterface *ode;
  bool swiftIsReference;
  sKinematicWorld():gl(NULL), swift(NULL), physx(NULL), ode(NULL), swiftIsReference(false) {}
  ~sKinematicWorld(){
    if(gl) delete gl;
    if(swift && !swiftIsReference) delete swift;
    if(physx) delete physx;
    if(ode) delete ode;
  }
};
}

ors::KinematicWorld::KinematicWorld():s(NULL),q_agent(0),isLinkTree(false) {
  bodies.memMove=joints.memMove=shapes.memMove=proxies.memMove=true;
  s=new sKinematicWorld;
}

ors::KinematicWorld::KinematicWorld(const ors::KinematicWorld& other):s(NULL),q_agent(0),isLinkTree(false)  {
  bodies.memMove=joints.memMove=shapes.memMove=proxies.memMove=true;
  s=new sKinematicWorld;
  *this = other;
}

ors::KinematicWorld::KinematicWorld(const char* filename):s(NULL),q_agent(0),isLinkTree(false)  {
  bodies.memMove=joints.memMove=shapes.memMove=proxies.memMove=true;
  s=new sKinematicWorld;
  init(filename);
}
ors::KinematicWorld::~KinematicWorld() {
  clear();
  delete s;
  s=NULL;
}

void ors::KinematicWorld::init(const char* filename) {
  *this <<FILE(filename);
  calc_q_from_Q();
}

void ors::KinematicWorld::clear() {
  qdim.clear();
  q.clear();
  qdot.clear();
  listDelete(proxies); checkConsistency();
  while(shapes.N){ delete shapes.last(); checkConsistency(); }
  while(joints.N){ delete joints.last(); checkConsistency();}
  while(bodies.N){ delete bodies.last(); checkConsistency();}
  isLinkTree=false;
}

void ors::KinematicWorld::copy(const ors::KinematicWorld& G, bool referenceMeshesAndSwiftOnCopy) {
  q = G.q;
  qdot = G.qdot;
  q_agent = G.q_agent;
  isLinkTree = G.isLinkTree;
#if 1
  listCopy(proxies, G.proxies);
  for(Body *b:G.bodies) new Body(*this, b);
  for(Shape *s:G.shapes) new Shape(*this, (s->body?*bodies(s->body->index):NoBody), s, referenceMeshesAndSwiftOnCopy);
  for(Joint *j:G.joints){
    Joint *jj=
        new Joint(*this, bodies(j->from->index), bodies(j->to->index), j);
    if(j->mimic) jj->mimic = joints(j->mimic->index);
  }
  if(referenceMeshesAndSwiftOnCopy){
    s->swift = G.s->swift;
    s->swiftIsReference=true;
  }
#else
  listCopy(proxies, G.proxies);
  listCopy(joints, G.joints);
  for(Joint *j: joints) if(j->mimic){
    MT::String jointName;
    bool good = j->ats.getValue<MT::String>(jointName, "mimic");
    CHECK(good, "something is wrong");
    j->mimic = listFindByName(G.joints, jointName);
    if(!j->mimic) HALT("The joint '" <<*j <<"' is declared coupled to '" <<jointName <<"' -- but that doesn't exist!");
    j->type = j->mimic->type;
  }
  listCopy(shapes, G.shapes);
  listCopy(bodies, G.bodies);
  graphMakeLists(bodies, joints);
  for_list(Body,  b,  bodies) b->shapes.clear();
  for_list(Shape,  s,  shapes) {
    b=bodies(s->ibody);
    s->body=b;
    b->shapes.append(s);
  }
#endif
}

/** @brief transforms (e.g., translates or rotates) the joints coordinate system):
  `adds' the transformation f to A and its inverse to B */
void ors::KinematicWorld::transformJoint(ors::Joint *e, const ors::Transformation &f) {
  e->A = e->A * f;
  e->B = -f * e->B;
}

void ors::KinematicWorld::makeLinkTree() {
  for(Joint *j: joints) {
    for(Shape *s: j->to->shapes)  s->rel = j->B * s->rel;
    for(Joint *j2: j->to->outLinks) j2->A = j->B * j2->A;
    j->B.setZero();
  }
  isLinkTree=true;
}

/** @brief KINEMATICS: given the (absolute) frames of root nodes and the relative frames
    on the edges, this calculates the absolute frames of all other nodes (propagating forward
    through trees and testing consistency of loops). */
void ors::KinematicWorld::calc_fwdPropagateFrames() {
  ors::Transformation f;
  BodyL todoBodies = bodies;
  for(Body *b: todoBodies) {
#if 0 //this does not work if not topsorted!!
    CHECK(n->inLinks.N<=1,"loopy geometry - body '" <<n->name <<"' has more than 1 input link");
    for_list(Joint,  e,  n->inLinks) {
      f = e->from->X;
      f.appendTransformation(e->A);
      e->X = f;
      if(e->type==JT_hingeX || e->type==JT_transX)  e->X.rot.getX(e->axis);
      if(e->type==JT_hingeY || e->type==JT_transY)  e->X.rot.getY(e->axis);
      if(e->type==JT_hingeZ || e->type==JT_transZ)  e->X.rot.getZ(e->axis);
      if(e->type==JT_transXYPhi)  e->X.rot.getZ(e->axis);
      f.appendTransformation(e->Q);
      if(!isLinkTree) f.appendTransformation(e->B);
      n->X=f;
    }
#else
    for(Joint *j:b->outLinks){ //this has no bailout for loopy graphs!
      f = b->X;
      f.appendTransformation(j->A);
      j->X = f;
      if(j->type==JT_hingeX || j->type==JT_transX)  j->X.rot.getX(j->axis);
      if(j->type==JT_hingeY || j->type==JT_transY)  j->X.rot.getY(j->axis);
      if(j->type==JT_hingeZ || j->type==JT_transZ)  j->X.rot.getZ(j->axis);
      if(j->type==JT_transXYPhi)  j->X.rot.getZ(j->axis);
      f.appendTransformation(j->Q);
      if(!isLinkTree) f.appendTransformation(j->B);
      j->to->X=f;
      todoBodies.setAppend(j->to);
    }
#endif
  }
  calc_fwdPropagateShapeFrames();
}

void ors::KinematicWorld::calc_fwdPropagateShapeFrames() {
  for(Shape *s: shapes) {
    if(s->body){
      s->X = s->body->X;
      s->X.appendTransformation(s->rel);
    }else{
      s->X = s->rel;
    }
  }
}

void ors::KinematicWorld::calc_missingAB_from_BodyAndJointFrames() {
  for(Joint *e: joints) {
    if(!e->X.isZero() && e->A.isZero() && e->B.isZero()) {
      e->A.setDifference(e->from->X, e->X);
      e->B.setDifference(e->X, e->to->X);
    }
  }
}

/** @brief given the absolute frames of all nodes and the two rigid (relative)
    frames A & B of each edge, this calculates the dynamic (relative) joint
    frame X for each edge (which includes joint transformation and errors) */
void ors::KinematicWorld::calc_Q_from_BodyFrames() {
  for_list(Joint,  e,  joints) {
    ors::Transformation A(e->from->X), B(e->to->X);
    A.appendTransformation(e->A);
    B.appendInvTransformation(e->B);
    e->Q.setDifference(A, B);
  }
}

/** @brief in all edge frames: remove any displacements, velocities and non-x rotations.
    After this, edges and nodes are not coherent anymore. You might want to call
    calcBodyFramesFromJoints() */
void ors::KinematicWorld::clearJointErrors() {
  ors::Vector xaxis(1, 0, 0);
  for_list(Joint,  e,  joints) {
    e->Q.pos.setZero();
    e->Q.vel.setZero();
    e->Q.rot.alignWith(xaxis);
    e->Q.angvel.makeColinear(xaxis);
  }
}

arr ors::KinematicWorld::naturalQmetric(double power) const {
#if 0
  if(!q.N) getJointStateDimension();
  arr Wdiag(q.N);
  Wdiag=1.;
  return Wdiag;
#else
  //compute generic q-metric depending on tree depth
  arr BM(bodies.N);
  BM=1.;
  for(uint i=BM.N; i--;) {
    for(uint j=0; j<bodies(i)->outLinks.N; j++) {
      BM(i) = MT::MAX(BM(bodies(i)->outLinks(j)->to->index)+1., BM(i));
//      BM(i) += BM(bodies(i)->outLinks(j)->to->index);
    }
  }
  if(!q.N) getJointStateDimension();
  arr Wdiag(q.N);
  for(Joint *j: joints) {
    for(uint i=0; i<j->qDim(); i++) {
      if(j->agent==q_agent) Wdiag(j->qIndex+i) = ::pow(BM(j->to->index), power);
    }
  }
  return Wdiag;
#endif
}

/** @brief revert the topological orientation of a joint (edge),
   e.g., when choosing another body as root of a tree */
void ors::KinematicWorld::revertJoint(ors::Joint *j) {
  cout <<"reverting edge (" <<j->from->name <<' ' <<j->to->name <<")" <<endl;
  //revert
  j->from->outLinks.removeValue(j);
  j->to->inLinks.removeValue(j);
  Body *b=j->from; j->from=j->to; j->to=b;
  j->from->outLinks.append(j);
  j->to->inLinks.append(j);
  listReindex(j->from->outLinks);
  listReindex(j->from->inLinks);
  checkConsistency();

  ors::Transformation f;
  f=j->A;
  j->A.setInverse(j->B);
  j->B.setInverse(f);
  f=j->Q;
  j->Q.setInverse(f);
}

/** @brief re-orient all joints (edges) such that n becomes
  the root of the configuration */
void ors::KinematicWorld::reconfigureRoot(Body *root) {
  MT::Array<Body*> list, list2;
  Body **m,**mstop;
  list.append(root);
  uintA level(bodies.N);
  level=0;
  int i=0;
  
  while(list.N>0) {
    i++;
    list2.clear();
    mstop=list.p+list.N;
    for(m=list.p; m!=mstop; m++) {
      level((*m)->index)=i;
      for_list(Joint,  e,  (*m)->inLinks) {
        if(!level(e->from->index)) { revertJoint(e); e_COUNT--; }
      }
      for(Joint *e: (*m)->outLinks) list2.append(e->to);
    }
    list=list2;
  }
  
  graphTopsort(bodies, joints);
}

/** @brief returns the joint (actuator) dimensionality */
uint ors::KinematicWorld::getJointStateDimension(int agent) const {
  if(agent==-1) agent=q_agent;
  CHECK(agent!=UINT_MAX,"");
  while(qdim.N<=(uint)agent) ((KinematicWorld*)this)->qdim.append(UINT_MAX);
  if(qdim(agent)==UINT_MAX){
    uint qd=0;
    for(Joint *j: joints) if(j->agent==(uint)agent){
      CHECK(j->type!=JT_none,"joint type is uninitialized");
      if(!j->mimic){
        j->qIndex = qd;
        qd += j->qDim();
      }else{
        j->qIndex = j->mimic->qIndex;
      }
    }
    ((KinematicWorld*)this)->qdim(agent) = qd; //hack to work around const declaration
  }
  return qdim(agent);
}

/** @brief returns the vector of joint limts */
arr ors::KinematicWorld::getLimits() const {
  uint N=getJointStateDimension();
  arr limits(N,2);
  limits.setZero();
  for(Joint *j: joints) if(j->agent==q_agent && j->limits.N){
    uint i=j->qIndex;
    uint d=j->qDim();
    for(uint k=0;k<d;k++){//in case joint has multiple dimensions
      limits(i+k,0)=j->limits(0); //lo
      limits(i+k,1)=j->limits(1); //up
    }
  }
//  cout <<"limits=" <<limits <<endl;
  return limits;
}

void ors::KinematicWorld::zeroGaugeJoints() {
  Joint *e;
  ors::Vector w;
  for_list(Body,  n,  bodies) if(n->type!=staticBT) {
    e=n->inLinks(0);
    if(e) {
      w=e->Q.rot / e->Q.angvel; e->Q.angvel.setZero();
      e->A.appendTransformation(e->Q);
      e->Q.setZero();
      e->Q.angvel=w;
    }
  }
}

void ors::KinematicWorld::calc_q_from_Q(bool calcVels) {
//  ors::Quaternion rot;
  
  uint N=getJointStateDimension();
  q.resize(N);
  qdot.resize(N);

  uint n=0;
  for(Joint *j: joints) if(j->agent==q_agent){
    if(j->mimic) continue; //don't count dependent joints
    CHECK(j->qIndex==n,"joint indexing is inconsistent");
    switch(j->type) {
      case JT_hingeX:
      case JT_hingeY:
      case JT_hingeZ: {
        //angle
        ors::Vector rotv;
        j->Q.rot.getRad(q(n), rotv);
        if(q(n)>MT_PI) q(n)-=MT_2PI;
        if(j->type==JT_hingeX && rotv*Vector_x<0.) q(n)=-q(n);
        if(j->type==JT_hingeY && rotv*Vector_y<0.) q(n)=-q(n);
        if(j->type==JT_hingeZ && rotv*Vector_z<0.) q(n)=-q(n);
        //velocity
        if(calcVels){
          qdot(n)=j->Q.angvel.length();
          if(j->type==JT_hingeX && j->Q.angvel*Vector_x<0.) qdot(n)=-qdot(n);
          if(j->type==JT_hingeY && j->Q.angvel*Vector_y<0.) qdot(n)=-qdot(n);
          if(j->type==JT_hingeZ && j->Q.angvel*Vector_z<0.) qdot(n)=-qdot(n);
        }
        n++;
      } break;

      case JT_universal: {
        //angle
        if(fabs(j->Q.rot.w)>1e-15) {
          q(n) = 2.0 * atan(j->Q.rot.x/j->Q.rot.w);
          q(n+1) = 2.0 * atan(j->Q.rot.y/j->Q.rot.w);
        } else {
          q(n) = MT_PI;
          q(n+1) = MT_PI;
        }
        
        if(calcVels) NIY; // velocity: need to fix
        n+=2;
      } break;

      case JT_quatBall: {
        q(n+0)=j->Q.rot.w;
        q(n+1)=j->Q.rot.x;
        q(n+2)=j->Q.rot.y;
        q(n+3)=j->Q.rot.z;
        if(calcVels) NIY;  // velocity: need to fix
        n+=4;
      } break;

      case JT_transX: {
        q(n)=j->Q.pos.x;
        if(calcVels) qdot(n)=j->Q.vel.x;
        n++;
      } break;
      case JT_transY: {
        q(n)=j->Q.pos.y;
        if(calcVels) qdot(n)=j->Q.vel.y;
        n++;
      } break;
      case JT_transZ: {
        q(n)=j->Q.pos.z;
        if(calcVels) qdot(n)=j->Q.vel.z;
        n++;
      } break;
      case JT_transXY: {
        q(n)=j->Q.pos.x;  q(n+1)=j->Q.pos.y;
        if(calcVels){  qdot(n)=j->Q.vel.x;  qdot(n+1)=j->Q.vel.y;  }
        n+=2;
      } break;
      case JT_transXYPhi: {
        q(n)=j->Q.pos.x;
        q(n+1)=j->Q.pos.y;
        ors::Vector rotv;
        j->Q.rot.getRad(q(n+2), rotv);
        if(q(n+2)>MT_PI) q(n+2)-=MT_2PI;
        if(rotv*Vector_z<0.) q(n+2)=-q(n+2);
        if(calcVels){
          qdot(n)=j->Q.vel.x;
          qdot(n+1)=j->Q.vel.y;
          qdot(n+2)=j->Q.angvel.length();
          if(j->Q.angvel*Vector_z<0.) qdot(n)=-qdot(n);
        }
        n+=3;
      } break;
      case JT_trans3: {
        q(n)=j->Q.pos.x;
        q(n+1)=j->Q.pos.y;
        q(n+2)=j->Q.pos.z;
        if(calcVels) {
          qdot(n)=j->Q.vel.x;
          qdot(n+1)=j->Q.vel.y;
          qdot(n+2)=j->Q.vel.z;
        }
        n+=3;
      } break;
      case JT_glue:
      case JT_fixed:
        break;
      default: NIY;
    }
  }
  CHECK(n==N,"");
}

void ors::KinematicWorld::calc_Q_from_q(bool calcVels){
  uint n=0;
  for(Joint *j: joints) if(j->agent==q_agent){
    if(j->mimic){
      j->Q=j->mimic->Q;
    }else{
      CHECK(j->qIndex==n,"joint indexing is inconsistent");
      switch(j->type) {
        case JT_hingeX: {
          j->Q.rot.setRadX(q(n));
          if(calcVels){  j->Q.angvel.set(qdot(n) ,0., 0.);  j->Q.zeroVels=false;  }
          n++;
        } break;

        case JT_hingeY: {
          j->Q.rot.setRadY(q(n));
          if(calcVels){  j->Q.angvel.set(0., qdot(n) ,0.);  j->Q.zeroVels=false;  }
          n++;
        } break;

        case JT_hingeZ: {
          j->Q.rot.setRadZ(q(n));
          if(calcVels){  j->Q.angvel.set(0., 0., qdot(n));  j->Q.zeroVels=false;  }
          n++;
        } break;

        case JT_universal:{
          ors::Quaternion rot1, rot2;
          rot1.setRadX(q(n));
          rot2.setRadY(q(n+1));
          j->Q.rot = rot1*rot2;
          if(calcVels) NIY;
          n+=2;
        } break;

        case JT_quatBall:{
          j->Q.rot.set(q.p+n);
          j->Q.rot.normalize();
          j->Q.rot.isZero=false; //TODO: WHY???? (gradient check fails without!)
          if(calcVels) NIY;
          n+=4;
        } break;

        case JT_transX: {
          j->Q.pos = q(n)*Vector_x;
          if(calcVels){ j->Q.vel.set(qdot(n), 0., 0.); j->Q.zeroVels=false; }
          n++;
        } break;

        case JT_transY: {
          j->Q.pos = q(n)*Vector_y;
          if(calcVels){ j->Q.vel.set(0., qdot(n), 0.); j->Q.zeroVels=false; }
          n++;
        } break;

        case JT_transZ: {
          j->Q.pos = q(n)*Vector_z;
          if(calcVels){ j->Q.vel.set(0., 0., qdot(n)); j->Q.zeroVels=false; }
          n++;
        } break;

        case JT_transXY: {
          j->Q.pos.set(q(n), q(n+1), 0.);
          if(calcVels){ j->Q.vel.set(qdot(n), qdot(n+1), 0.); j->Q.zeroVels=false; }
          n+=2;
        } break;

        case JT_trans3: {
          j->Q.pos.set(q(n), q(n+1), q(n+2));
          if(calcVels){ j->Q.vel.set(qdot(n), qdot(n+1), qdot(n+2)); j->Q.zeroVels=false; }
          n+=3;
        } break;

        case JT_transXYPhi: {
          j->Q.pos.set(q(n), q(n+1), 0.);
          j->Q.rot.setRadZ(q(n+2));
          if(calcVels){
            j->Q.vel.set(qdot(n), qdot(n+1), 0.);  j->Q.zeroVels=false;
            j->Q.angvel.set(0., 0., qdot(n+2));  j->Q.zeroVels=false;
          }
          n+=3;
        } break;

        case JT_glue:
        case JT_fixed:
		  // commented to allow setting joint positions of fixed joints...
		  // j->Q.setZero();
		  // j->Q.zeroVels=true;
          break;
        default: NIY;
      }
    }
  }

  CHECK(n==q.N,"");
}


/** @brief sets the joint state vectors separated in positions and
  velocities */
void ors::KinematicWorld::setJointState(const arr& _q, const arr& _qdot, bool calcVels) {
  setJointStateCount++; //global counter

  uint N=getJointStateDimension();
  CHECK(_q.N==N && (!(&_qdot) || _qdot.N==N), "wrong joint state dimensionalities");
  if(&_q!=&q) q=_q;
  if(&_qdot){ if(&_qdot!=&qdot) qdot=_qdot; }else qdot.clear();

  calc_Q_from_q(calcVels);

  calc_fwdPropagateFrames();
}

void ors::KinematicWorld::setAgent(uint agent, bool calcVels){
  if(agent==q_agent) return; //nothing to do
  q_agent = agent;
  calc_q_from_Q(calcVels);
}



//===========================================================================
//
// core: kinematics and dynamics
//

/** @brief return the jacobian \f$J = \frac{\partial\phi_i(q)}{\partial q}\f$ of the position
  of the i-th body (3 x n tensor)*/
void ors::KinematicWorld::kinematicsPos(arr& y, arr& J, Body *b, ors::Vector *rel) const {
  //get position
  ors::Vector pos_world = b->X.pos;
  if(rel) pos_world += b->X.rot*(*rel);
  if(&y) y = ARRAY(pos_world); //return the output
  if(!&J) return; //do not return the Jacobian

  //get Jacobian
  uint N=getJointStateDimension();
  J.resize(3, N).setZero();
  if(b->inLinks.N) { //body a has no inLinks -> zero jacobian
    Joint *j=b->inLinks(0);
    while(j) { //loop backward down the kinematic tree
      uint j_idx=j->qIndex;
      if(j->agent==q_agent && j_idx>=N) CHECK(j->type==JT_glue || j->type==JT_fixed, "");
      if(j->agent==q_agent && j_idx<N){
        if(j->type==JT_hingeX || j->type==JT_hingeY || j->type==JT_hingeZ) {
          ors::Vector tmp = j->axis ^ (pos_world-j->X.pos);
          J(0, j_idx) += tmp.x;
          J(1, j_idx) += tmp.y;
          J(2, j_idx) += tmp.z;
        }
        else if(j->type==JT_transX || j->type==JT_transY || j->type==JT_transZ) {
          J(0, j_idx) += j->axis.x;
          J(1, j_idx) += j->axis.y;
          J(2, j_idx) += j->axis.z;
        }
        else if(j->type==JT_transXY) {
          if(j->mimic) NIY;
          arr R(3,3); j->X.rot.getMatrix(R.p);
          J.setMatrixBlock(R.sub(0,-1,0,1), 0, j_idx);
        }
        else if(j->type==JT_transXYPhi) {
          if(j->mimic) NIY;
          arr R(3,3); j->X.rot.getMatrix(R.p);
          J.setMatrixBlock(R.sub(0,-1,0,1), 0, j_idx);
          ors::Vector tmp = j->axis ^ (pos_world-(j->X.pos + j->Q.pos));
          J(0, j_idx+2) += tmp.x;
          J(1, j_idx+2) += tmp.y;
          J(2, j_idx+2) += tmp.z;
        }
        else if(j->type==JT_trans3) {
          if(j->mimic) NIY;
          arr R(3,3); j->X.rot.getMatrix(R.p);
          J.setMatrixBlock(R, 0, j_idx);
        }
        else if(j->type==JT_quatBall) {
          ors::Quaternion e;
          ors::Vector axis, tmp;
          for(uint i=0;i<4;i++){
            if(i==0) e.set(1.,0.,0.,0.);
            if(i==1) e.set(0.,1.,0.,0.);
            if(i==2) e.set(0.,0.,1.,0.);
            if(i==3) e.set(0.,0.,0.,1.);//TODO: the following could be simplified/compressed/made more efficient
            e = e / j->Q.rot;
            axis.set(e.x, e.y, e.z);
            axis = j->X.rot*axis;
            axis *= -2.;
            axis /= sqrt(sumOfSqr(q.subRange(j->qIndex,j->qIndex+3))); //account for the potential non-normalization of q
            tmp = axis ^ (pos_world-j->X.pos);
            J(0, j_idx+i) += tmp.x;
            J(1, j_idx+i) += tmp.y;
            J(2, j_idx+i) += tmp.z;
          }
        }
      }
      if(!j->from->inLinks.N) break;
      j=j->from->inLinks(0);
    }
  }
}

/** @brief return the Hessian \f$H = \frac{\partial^2\phi_i(q)}{\partial q\partial q}\f$ of the position
  of the i-th body (3 x n x n tensor) */
void ors::KinematicWorld::hessianPos(arr& H, Body *b, ors::Vector *rel) const {
  HALT("this is buggy: a sign error: see examples/Ors/ors testKinematics");
  Joint *j1, *j2;
  uint j1_idx, j2_idx;
  ors::Vector tmp, pos_a;
  
  uint N=getJointStateDimension();
  
  //initialize Jacobian
  H.resize(3, N, N);
  H.setZero();
  
  //get reference frame
  pos_a = b->X.pos;
  if(rel) pos_a += b->X.rot*(*rel);
  
  if(b->inLinks.N) {
    j1=b->inLinks(0);
    while(j1) {
      CHECK(j1->agent==q_agent,"NIY");
      j1_idx=j1->qIndex;

      j2=j1;
      while(j2) {
        CHECK(j2->agent==q_agent,"NIY");
        j2_idx=j2->qIndex;

        if(j1->type>=JT_hingeX && j1->type<=JT_hingeZ && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //both are hinges
          tmp = j2->axis ^ (j1->axis ^ (pos_a-j1->X.pos));
          H(0, j1_idx, j2_idx) = H(0, j2_idx, j1_idx) = tmp.x;
          H(1, j1_idx, j2_idx) = H(1, j2_idx, j1_idx) = tmp.y;
          H(2, j1_idx, j2_idx) = H(2, j2_idx, j1_idx) = tmp.z;
        }
        else if(j1->type>=JT_transX && j1->type<=JT_transZ && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans, j=hinge
          tmp = j1->axis ^ j2->axis;
          H(0, j1_idx, j2_idx) = H(0, j2_idx, j1_idx) = tmp.x;
          H(1, j1_idx, j2_idx) = H(1, j2_idx, j1_idx) = tmp.y;
          H(2, j1_idx, j2_idx) = H(2, j2_idx, j1_idx) = tmp.z;
        }
        else if(j1->type==JT_transXY && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans3, j=hinge
          NIY;
        }
        else if(j1->type==JT_transXYPhi && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans3, j=hinge
          NIY;
        }
        else if(j1->type==JT_trans3 && j2->type>=JT_hingeX && j2->type<=JT_hingeZ) { //i=trans3, j=hinge
          Matrix R,A;
          j1->X.rot.getMatrix(R.p());
          A.setSkew(j2->axis);
          R = R*A;
          H(0, j1_idx  , j2_idx) = H(0, j2_idx  , j1_idx) = R.m00;
          H(1, j1_idx  , j2_idx) = H(1, j2_idx  , j1_idx) = R.m10;
          H(2, j1_idx  , j2_idx) = H(2, j2_idx  , j1_idx) = R.m20;
          H(0, j1_idx+1, j2_idx) = H(0, j2_idx, j1_idx+1) = R.m01;
          H(1, j1_idx+1, j2_idx) = H(1, j2_idx, j1_idx+1) = R.m11;
          H(2, j1_idx+1, j2_idx) = H(2, j2_idx, j1_idx+1) = R.m21;
          H(0, j1_idx+2, j2_idx) = H(0, j2_idx, j1_idx+2) = R.m02;
          H(1, j1_idx+2, j2_idx) = H(1, j2_idx, j1_idx+2) = R.m12;
          H(2, j1_idx+2, j2_idx) = H(2, j2_idx, j1_idx+2) = R.m22;
        }
        else if(j1->type>=JT_hingeX && j1->type<=JT_hingeZ && j2->type>=JT_transX && j2->type<=JT_trans3) { //i=hinge, j=trans
          //nothing! Hessian is zero (ej is closer to root than ei)
        }
        else NIY;

        if(!j2->from->inLinks.N) break;
        j2=j2->from->inLinks(0);
      }
      if(!j1->from->inLinks.N) break;
      j1=j1->from->inLinks(0);
    }
  }
}

/* takes the joint state x and returns the jacobian dz of
   the position of the ith body (w.r.t. all joints) -> 2D array */
/// Jacobian of the i-th body's z-orientation vector
void ors::KinematicWorld::kinematicsVec(arr& y, arr& J, Body *b, ors::Vector *vec) const {
  //get the vectoreference frame
  ors::Vector vec_world;
  if(vec) vec_world = b->X.rot*(*vec);
  else    b->X.rot.getZ(vec_world);
  if(&y) y = ARRAY(vec_world); //return the vec
  if(!&J) return; //do not return a Jacobian

  //get Jacobian
  uint N = getJointStateDimension();
  J.resize(3, N).setZero();
  if(b->inLinks.N) {
    Joint *j=b->inLinks(0);
    while(j) { //loop backward down the kinematic tree
      uint j_idx=j->qIndex;
      if(j->agent==q_agent && j_idx>=N) CHECK(j->type==JT_glue || j->type==JT_fixed, "");
      if(j->agent==q_agent && j_idx<N){
        if((j->type>=JT_hingeX && j->type<=JT_hingeZ) || j->type==JT_transXYPhi) {
          if(j->type==JT_transXYPhi) j_idx += 2; //refer to the phi only
          ors::Vector tmp = j->axis ^ vec_world;
          J(0, j_idx) += tmp.x;
          J(1, j_idx) += tmp.y;
          J(2, j_idx) += tmp.z;
        }
        else if(j->type==JT_quatBall) {
          ors::Quaternion e;
          ors::Vector axis, tmp;
          for(uint i=0;i<4;i++){
            if(i==0) e.set(1.,0.,0.,0.);
            if(i==1) e.set(0.,1.,0.,0.);
            if(i==2) e.set(0.,0.,1.,0.);
            if(i==3) e.set(0.,0.,0.,1.);//TODO: the following could be simplified/compressed/made more efficient
            e = e / j->Q.rot;
            axis.set(e.x, e.y, e.z);
            axis *= -2.;
            axis /= sqrt(sumOfSqr(q.subRange(j->qIndex,j->qIndex+3))); //account for the potential non-normalization of q
            axis = j->X.rot*axis;
            tmp = axis ^ vec_world;
            J(0, j_idx+i) += tmp.x;
            J(1, j_idx+i) += tmp.y;
            J(2, j_idx+i) += tmp.z;
          }
        }
        //all other joints: J=0 !!
      }
      if(!j->from->inLinks.N) break;
      j=j->from->inLinks(0);
    }
  }
}

/* takes the joint state x and returns the jacobian dz of
   the position of the ith body (w.r.t. all joints) -> 2D array */
/// Jacobian of the i-th body's z-orientation vector
void ors::KinematicWorld::kinematicsQuat(arr& y, arr& J, Body *b) const {
  Joint *j;
  uint j_idx;

  uint N=getJointStateDimension();

  //get reference frame
  ors::Quaternion rot_b = b->X.rot;

  if(&y) y = ARRAY(rot_b); //return the vec
  if(!&J) return; //do not return a Jacobian

  //initialize Jacobian
  J.resize(4, N);
  J.setZero();

  if(b->inLinks.N) {
    j=b->inLinks(0);
    while(j) { //loop backward down the kinematic tree
      j_idx=j->qIndex;
      if(j->agent==q_agent && j_idx>=N) CHECK(j->type==JT_glue || j->type==JT_fixed, "");
      if(j->agent==q_agent && j_idx<N){
        if((j->type>=JT_hingeX && j->type<=JT_hingeZ) || j->type==JT_transXYPhi) {
          if(j->type==JT_transXYPhi) j_idx += 2; //refer to the phi only
          ors::Quaternion tmp(0., 0.5*j->axis.x, 0.5*j->axis.y, 0.5*j->axis.z ); //this is unnormalized!!
          tmp = tmp * rot_b;
          J(0, j_idx) += tmp.w;
          J(1, j_idx) += tmp.x;
          J(2, j_idx) += tmp.y;
          J(3, j_idx) += tmp.z;
        }
        else if(j->type==JT_quatBall) {
          ors::Quaternion e;
          ors::Vector axis, tmp;
          for(uint i=0;i<4;i++){
            if(i==0) e.set(1.,0.,0.,0.);
            if(i==1) e.set(0.,1.,0.,0.);
            if(i==2) e.set(0.,0.,1.,0.);
            if(i==3) e.set(0.,0.,0.,1.);//TODO: the following could be simplified/compressed/made more efficient
            e = e / j->Q.rot;
            axis.set(e.x, e.y, e.z);
            axis *= -2.;
            axis /= sqrt(sumOfSqr(q.subRange(j->qIndex,j->qIndex+3))); //account for the potential non-normalization of q
            axis = j->X.rot*axis;
            ors::Quaternion tmp(0., 0.5*axis.x, 0.5*axis.y, 0.5*axis.z ); //this is unnormalized!!
            tmp = tmp * rot_b;
            J(0, j_idx+i) += tmp.w;
            J(1, j_idx+i) += tmp.x;
            J(2, j_idx+i) += tmp.y;
            J(3, j_idx+i) += tmp.z;
          }
        }
        //all other joints: J=0 !!
      }
      if(!j->from->inLinks.N) break;
      j=j->from->inLinks(0);
    }
  }
}

void ors::KinematicWorld::jacobianR(arr& J, Body *b) const {
  Joint *j;
  uint j_idx;
  
  uint N=getJointStateDimension();
  
  J.resize(3, N).setZero();
  
  if(b->inLinks.N) {
    j=b->inLinks(0);
    while(j) {
      j_idx=j->qIndex;
      if(j->agent==q_agent && j_idx>=N) CHECK(j->type==JT_glue || j->type==JT_fixed, "");
      if(j->agent==q_agent && j_idx<N){
        if((j->type>=JT_hingeX && j->type<=JT_hingeZ) || j->type==JT_transXYPhi) {
          J(0, j_idx) = j->axis.x;
          J(1, j_idx) = j->axis.y;
          J(2, j_idx) = j->axis.z;
        }
        else if(j->type==JT_quatBall) {
          ors::Quaternion e;
          ors::Vector axis;
          for(uint i=0;i<4;i++){
            if(i==0) e.set(1.,0.,0.,0.);
            if(i==1) e.set(0.,1.,0.,0.);
            if(i==2) e.set(0.,0.,1.,0.);
            if(i==3) e.set(0.,0.,0.,1.);//TODO: the following could be simplified/compressed/made more efficient
            e = e / j->Q.rot;
            axis.set(e.x, e.y, e.z);
            axis = j->X.rot*axis;
            axis *= -2.;
            axis /= sqrt(sumOfSqr(q.subRange(j->qIndex,j->qIndex+3))); //account for the potential non-normalization of q
            J(0, j_idx+i) += axis.x;
            J(1, j_idx+i) += axis.y;
            J(2, j_idx+i) += axis.z;
          }
        }
        //all other joints: J=0 !!
      }
      if(!j->from->inLinks.N) break;
      j=j->from->inLinks(0);
    }
  }
}

/** @brief return the configuration's inertia tensor $M$ (n x n tensor)*/
void ors::KinematicWorld::inertia(arr& M) {
  uint j1_idx, j2_idx;
  ors::Transformation Xa, Xi, Xj;
  Joint *j1, *j2;
  ors::Vector vi, vj, ti, tj;
  double tmp;
  
  uint N=getJointStateDimension();
  
  //initialize Jacobian
  M.resize(N, N);
  M.setZero();
  
  for(Body *a: bodies) {
    //get reference frame
    Xa = a->X;
    
    j1=a->inLinks(0);
    while(j1) {
      j1_idx=j1->qIndex;
      
      Xi = j1->from->X;
      Xi.appendTransformation(j1->A);
      Xi.rot.getX(ti);
      
      vi = ti ^(Xa.pos-Xi.pos);
      
      j2=j1;
      while(j2) {
        j2_idx=j2->qIndex;
        
        Xj = j2->from->X;
        Xj.appendTransformation(j2->A);
        Xj.rot.getX(tj);
        
        vj = tj ^(Xa.pos-Xj.pos);
        
        tmp = a->mass * (vi*vj);
        //tmp += scalarProduct(a->a.inertia, ti, tj);
        
        M(j1_idx, j2_idx) += tmp;
        
        if(!j2->from->inLinks.N) break;
        j2=j2->from->inLinks(0);
      }
      if(!j1->from->inLinks.N) break;
      j1=j1->from->inLinks(0);
    }
  }
  //symmetric: fill in other half
  for(j1_idx=0; j1_idx<N; j1_idx++) for(j2_idx=0; j2_idx<j1_idx; j2_idx++) M(j2_idx, j1_idx) = M(j1_idx, j2_idx);
}

void ors::KinematicWorld::equationOfMotion(arr& M, arr& F, bool gravity) {
  static ors::LinkTree tree; //TODO: HACK!!
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  if(gravity){
    clearForces();
    gravityToForces();
  }
  ors::equationOfMotion(M, F, tree, qdot);
  F *= -1.;
}

/** @brief return the joint accelerations \f$\ddot q\f$ given the
  joint torques \f$\tau\f$ (computed via Featherstone's Articulated Body Algorithm in O(n)) */
void ors::KinematicWorld::fwdDynamics(arr& qdd, const arr& qd, const arr& tau) {
  static ors::LinkTree tree;
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  //cout <<tree <<endl;
  //ors::fwdDynamics_aba_1D(qdd, tree, qd, tau);
  //ors::fwdDynamics_aba_nD(qdd, tree, qd, tau);
  ors::fwdDynamics_MF(qdd, tree, qd, tau);
}

/** @brief return the necessary joint torques \f$\tau\f$ to achieve joint accelerations
  \f$\ddot q\f$ (computed via the Recursive Newton-Euler Algorithm in O(n)) */
void ors::KinematicWorld::inverseDynamics(arr& tau, const arr& qd, const arr& qdd) {
  static ors::LinkTree tree;
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  ors::invDynamics(tau, tree, qd, qdd);
}

/*void ors::KinematicWorld::impulsePropagation(arr& qd1, const arr& qd0){
  static MT::Array<Featherstone::Link> tree;
  if(!tree.N) GraphToTree(tree, *this);
  else updateGraphToTree(tree, *this);
  mimickImpulsePropagation(tree);
  Featherstone::RF_abd(qdd, tree, qd, tau);
}*/

/// [prelim] some heuristic measure for the joint errors
double ors::KinematicWorld::getJointErrors() const {
  double err=0.0;
  for_list(Joint, e, joints) err+=e->Q.pos.lengthSqr();
  return ::sqrt(err);
}

/** @brief checks if all names of the bodies are disjoint */
bool ors::KinematicWorld::checkUniqueNames() const {
  for_list(Body,  n,  bodies) for(Body *b: bodies) {
    if(n==b) break;
    if(n->name==b->name) return false;
  }
  return true;
}

/** @brief checks if all names of the bodies are disjoint */
void ors::KinematicWorld::setShapeNames() {
  for(Body *b: bodies){
    uint i=0;
    for(Shape *s:b->shapes){
      if(!s->name.N){ s->name = b->name; s->name <<'_' <<i; }
      i++;
    }
  }
}

/// find body with specific name
ors::Body* ors::KinematicWorld::getBodyByName(const char* name) const {
  for(Body *b: bodies) if(b->name==name) return b;
  if(strcmp("glCamera", name)!=0)
    MT_MSG("cannot find Body named '" <<name <<"' in Graph");
  return 0;
}

/// find shape with specific name
ors::Shape* ors::KinematicWorld::getShapeByName(const char* name) const {
  for(Shape *s: shapes) if(s->name==name) return s;
  MT_MSG("cannot find Shape named '" <<name <<"' in Graph");
  return NULL;
}

/// find shape with specific name
ors::Joint* ors::KinematicWorld::getJointByName(const char* name) const {
  for(Joint *j: joints) if(j->name==name) return j;
  MT_MSG("cannot find Joint named '" <<name <<"' in Graph");
  return NULL;
}

/// find joint connecting two bodies
ors::Joint* ors::KinematicWorld::getJointByBodies(const Body* from, const Body* to) const {
  for(Joint *j: to->inLinks) if(j->from==from) return j;
  return NULL;
}

/// find joint connecting two bodies with specific names
ors::Joint* ors::KinematicWorld::getJointByBodyNames(const char* from, const char* to) const {
  for_list(Body, f, bodies) if(f->name==from) break;
  for_list(Body, t, bodies) if(t->name==to) break;
  if(!f || !t) return 0;
  return graphGetEdge<Body, Joint>(f, t);
}

ShapeL ors::KinematicWorld::getShapesByAgent(const uint agent) const {
  ShapeL agent_shapes;
  for(ors::Joint *j : joints) {
    if(j->agent==agent) {
      ShapeL tmp;
      tmp.append(j->from->shapes);
      tmp.append(j->to->shapes);
      for(ors::Shape* s : tmp) {
        if (!agent_shapes.contains(s)) agent_shapes.append(s);
      }
    } 
  }  
  return agent_shapes;
}

uintA ors::KinematicWorld::getShapeIdxByAgent(const uint agent) const {
  uintA agent_shape_idx;
  ShapeL agent_shapes = getShapesByAgent(agent);
  for(ors::Shape* s : agent_shapes)
    agent_shape_idx.append(s->index);
  return agent_shape_idx;
}

/** @brief creates uniques names by prefixing the node-index-number to each name */
void ors::KinematicWorld::prefixNames() {
  for_list(Body, n, bodies) n->name=STRING(n->index<< n->name);
}

/// return a OpenGL extension
OpenGL& ors::KinematicWorld::gl(){
  if(!s->gl){
    s->gl = new OpenGL;
    bindOrsToOpenGL(*this,*s->gl);
  }
  return *s->gl;
}

/// return a Swift extension
SwiftInterface& ors::KinematicWorld::swift(){
  if(!s->swift) s->swift = new SwiftInterface(*this);
  return *s->swift;
}

/// reset the current Swift extension (inserted by Martin Griesser for uibk usage)
void ors::KinematicWorld::resetSwift(){
	if(s->swift) {
		delete s->swift;
		s->swift=NULL;
	}
}

/// return a PhysX extension
PhysXInterface& ors::KinematicWorld::physx(){
  if(!s->physx){
    s->physx = new PhysXInterface(*this);
    s->physx->setArticulatedBodiesKinematic();
  }
  return *s->physx;
}

/// return a ODE extension
OdeInterface& ors::KinematicWorld::ode(){
  if(!s->ode) s->ode = new OdeInterface(*this);
  return *s->ode;
}

void ors::KinematicWorld::watch(bool pause, const char* txt){
  if(pause) gl().watch(txt);
  else gl().update(txt);
}

void ors::KinematicWorld::stepSwift(){
  swift().step(*this, false);
}

void ors::KinematicWorld::stepPhysx(double tau){
  physx().step(tau);
}

void ors::KinematicWorld::stepOde(double tau){
#ifdef MT_ODE
  ode().setMotorVel(qdot, 100.);
  ode().step(tau);
  ode().importStateFromOde();
#endif
}

void ors::KinematicWorld::stepDynamics(const arr& Bu_control, double tau, double dynamicNoise){

  struct DiffEqn:VectorFunction{
    ors::KinematicWorld &S;
    const arr& Bu;
    DiffEqn(ors::KinematicWorld& _S, const arr& _Bu):S(_S), Bu(_Bu){}
    void fv(arr& y, arr& J, const arr& x){
      S.setJointState(x[0], x[1]);
      arr M,Minv,F;
      S.equationOfMotion(M, F);
      inverse_SymPosDef(Minv, M);
      y = Minv * (Bu - F);
    }
  } eqn(*this, Bu_control);

#if 0
  arr M,Minv,F;
  getDynamics(M, F);
  inverse_SymPosDef(Minv,M);

  //noisy Euler integration (Runge-Kutte4 would be much more precise...)
  qddot = Minv * (u_control - F);
  if(dynamicNoise) rndGauss(qddot, dynamicNoise, true);
  q    += tau * qdot;
  qdot += tau * qddot;
  arr x1=cat(s->q, s->qdot).reshape(2,s->q.N);
#else
  arr x1;
  rk4_2ndOrder(x1, cat(q, qdot).reshape(2,q.N), eqn, tau);
  if(dynamicNoise) rndGauss(x1[1](), ::sqrt(tau)*dynamicNoise, true);
#endif

  setJointState(x1[0], x1[1]);
}

/** @brief prototype for \c operator<< */
void ors::KinematicWorld::write(std::ostream& os) const {
  for(Body *b: bodies) {
    os <<"body " <<b->name <<" { ";
    b->write(os);  os <<" }\n";
  }
  os <<std::endl;
  for(Shape *s: shapes) {
    os <<"shape ";
    if(s->name.N) os <<s->name <<' ';
    os <<"(" <<(s->body?(char*)s->body->name:"") <<"){ ";
    s->write(os);  os <<" }\n";
  }
  os <<std::endl;
  for(Joint *j: joints) {
    os <<"joint ";
    if (j->name.N) os <<j->name <<' ';
    os <<"(" <<j->from->name <<' ' <<j->to->name <<"){ ";
    j->write(os);  os <<" }\n";
  }
}

#define DEBUG(x) //x

/** @brief prototype for \c operator>> */
void ors::KinematicWorld::read(std::istream& is) {
  KeyValueGraph G;
  
  G.read(is);
//  cout <<"***KVG" <<G <<endl;
  
  clear();
  
  ItemL bs = G.getItems("body");
  for_list(Item,  it,  bs) {
    CHECK(it->keys(0)=="body","");
    CHECK(it->getValueType()==typeid(KeyValueGraph), "bodies must have value KeyValueGraph");
    
    Body *b=new Body(*this);
    if(it->keys.N>1) b->name=it->keys(1);
    b->ats = *it->getValue<KeyValueGraph>();
    b->parseAts(*this);
  }
  
  ItemL ss = G.getItems("shape");
  for(Item *it: ss) {
    CHECK(it->keys(0)=="shape","");
    CHECK(it->parents.N<=1,"shapes must have no or one parent");
    CHECK(it->getValueType()==typeid(KeyValueGraph),"shape must have value KeyValueGraph");
    
    Shape *s;
    if(it->parents.N==1){
      Body *b = listFindByName(bodies, it->parents(0)->keys(1));
      CHECK(b,"");
      s=new Shape(*this, *b);
    }else{
      s=new Shape(*this, NoBody);
    }
    if(it->keys.N>1) s->name=it->keys(1);
    s->ats = *it->getValue<KeyValueGraph>();
    s->parseAts();
  }
  
  uint nCoupledJoints=0;
  ItemL js = G.getItems("joint");
  for(Item *it: js) {
    CHECK(it->keys(0)=="joint","");
    CHECK(it->parents.N==2,"joints must have two parents");
    CHECK(it->getValueType()==typeid(KeyValueGraph),"joints must have value KeyValueGraph");
    
    Body *from=listFindByName(bodies, it->parents(0)->keys(1));
    Body *to=listFindByName(bodies, it->parents(1)->keys(1));
    CHECK(from,"JOINT: from '" <<it->parents(0)->keys(1) <<"' does not exist ["<<*it <<"]");
    CHECK(to,"JOINT: to '" <<it->parents(1)->keys(1) <<"' does not exist ["<<*it <<"]");
    Joint *j=new Joint(*this, from, to);
    if(it->keys.N>1) j->name=it->keys(1);
    j->ats = *it->getValue<KeyValueGraph>();
    j->parseAts();

    //if the joint is coupled to another:
    if(j->mimic) nCoupledJoints++;
  }

  if(nCoupledJoints){
    for(Joint *j: joints) if(j->mimic){
      MT::String jointName;
      bool good = j->ats.getValue<MT::String>(jointName, "mimic");
      CHECK(good, "something is wrong");
      j->mimic = listFindByName(joints, jointName);
      if(!j->mimic) HALT("The joint '" <<*j <<"' is declared coupled to '" <<jointName <<"' -- but that doesn't exist!");
      j->type = j->mimic->type;
    }
  }

  //-- clean up the graph
  checkConsistency();
  topSort();
  //makeLinkTree();
  calc_missingAB_from_BodyAndJointFrames();
  getJointStateDimension();
  calc_q_from_Q();
  calc_fwdPropagateFrames();
}

void ors::KinematicWorld::writePlyFile(const char* filename) const {
  ofstream os;
  MT::open(os, filename);
  uint nT=0,nV=0;
  uint j;
  ors::Mesh *m;
  for(Shape *s: shapes) { nV += s->mesh.V.d0; nT += s->mesh.T.d0; }
  
  os <<"\
ply\n\
format ascii 1.0\n\
element vertex " <<nV <<"\n\
property float x\n\
property float y\n\
property float z\n\
property uchar red\n\
property uchar green\n\
property uchar blue\n\
element face " <<nT <<"\n\
property list uchar int vertex_index\n\
end_header\n";

  uint k=0;
  ors::Transformation t;
  ors::Vector v;
  for_list(Shape, s, shapes) {
    m = &s->mesh;
    t = s->X;
    if(m->C.d0!=m->V.d0) {
      m->C.resizeAs(m->V);
      for(j=0; j<m->C.d0; j++) { m->C(j, 0)=s->color[0]; m->C(j, 1)=s->color[1]; m->C(j, 2)=s->color[2]; }
    }
    for(j=0; j<m->V.d0; j++) {
      v.set(m->V(j, 0), m->V(j, 1), m->V(j, 2));
      v = t*v;
      os <<' ' <<v.x <<' ' <<v.y <<' ' <<v.z
         <<' ' <<int(255.f*m->C(j, 0)) <<' ' <<int(255.f*m->C(j, 1)) <<' ' <<int(255.f*m->C(j, 2)) <<endl;
    }
    k+=j;
  }
  uint offset=0;
  for(Shape *s: shapes) {
    m=&s->mesh;
    for(j=0; j<m->T.d0; j++) {
      os <<"3 " <<offset+m->T(j, 0) <<' ' <<offset+m->T(j, 1) <<' ' <<offset+m->T(j, 2) <<endl;
    }
    offset+=m->V.d0;
  }
}

/// dump the list of current proximities on the screen
void ors::KinematicWorld::reportProxies(std::ostream *os) {
  (*os) <<"Proximity report: #" <<proxies.N <<endl;
  for_list(Proxy, p, proxies) {
    ors::Shape *a = shapes(p->a);
    ors::Shape *b = shapes(p->b);
    (*os)
        <<p_COUNT <<" ("
        <<a <<':' <<a->body->name <<")-("
        <<b <<':' <<b->body->name
        <<") d=" <<p->d
        <<" |A-B|=" <<(p->posB-p->posA).length()
        <<" cenD=" <<p->cenD
        <<" d^2=" <<(p->posB-p->posA).lengthSqr()
        <<" normal=" <<p->normal
        <<" posA=" <<p->posA
        <<" posB=" <<p->posB
        <<endl;
  }
}

bool ProxySortComp(const ors::Proxy *a, const ors::Proxy *b) {
  return (a->a < b->a) || (a->a==b->a && a->b<b->b) || (a->a==b->a && a->b==b->b && a->d < b->d);
}

void ors::KinematicWorld::glueBodies(Body *f, Body *t) {
  Joint *j = new Joint(*this, f, t);
  j->A.setDifference(f->X, t->X);
  j->A.vel.setZero();
  j->A.angvel.setZero();
  j->type=JT_fixed;
  j->Q.setZero();
  j->B.setZero();
  isLinkTree=false;
}


/// clear all forces currently stored at bodies
void ors::KinematicWorld::clearForces() {
  for_list(Body,  n,  bodies) {
    n->force.setZero();
    n->torque.setZero();
  }
}

/// apply a force on body n 
void ors::KinematicWorld::addForce(ors::Vector force, ors::Body *n) {
  n->force += force;
  if (!s->physx) {
    NIY;
  }
  else {
    s->physx->addForce(force, n);
  }
  //n->torque += (pos - n->X.p) ^ force;
}

/// apply a force on body n at position pos (in world coordinates)
void ors::KinematicWorld::addForce(ors::Vector force, ors::Body *n, ors::Vector pos) {
  n->force += force;
  if (!s->physx) {
    NIY;
  }
  else {
    s->physx->addForce(force, n, pos);
  }
  //n->torque += (pos - n->X.p) ^ force;
}

void ors::KinematicWorld::frictionToForces(double coeff) {
  HALT("never do this: add it directly in the equations...");
  ors::Vector a;
  ors::Transformation X;
  double v;
  for_list(Joint,  e,  joints) {
    X = e->from->X;
    X.appendTransformation(e->A);
    X.rot.getX(a);//rotation axis
    
    v=e->Q.angvel.length();
    if(e->Q.angvel*Vector_x<0.) v=-v;
    
    e->from->torque -= (coeff*v)*a;
    e->to->torque   += (coeff*v)*a;
  }
}

void ors::KinematicWorld::gravityToForces() {
  ors::Vector g(0, 0, -9.81);
  for_list(Body,  n,  bodies) n->force += n->mass * g;
}

/// compute forces from the current contacts
void ors::KinematicWorld::contactsToForces(double hook, double damp) {
  ors::Vector trans, transvel, force;
  uint i;
  int a, b;
  for(i=0; i<proxies.N; i++) if(proxies(i)->d<0.) {
      a=proxies(i)->a; b=proxies(i)->b;
      
      //if(!i || proxies(i-1).a!=a || proxies(i-1).b!=b) continue; //no old reference sticking-frame
      //trans = proxies(i)->rel.p - proxies(i-1).rel.p; //translation relative to sticking-frame
      trans    = proxies(i)->posB-proxies(i)->posA;
      //transvel = proxies(i)->velB-proxies(i)->velA;
      //d=trans.length();
      
      force.setZero();
      force += (hook) * trans; //*(1.+ hook*hook*d*d)
      //force += damp * transvel;
      SL_DEBUG(1, cout <<"applying force: [" <<a <<':' <<b <<"] " <<force <<endl);
      
      if(a!=-1) addForce(force, shapes(a)->body, proxies(i)->posA);
      if(b!=-1) addForce(-force, shapes(b)->body, proxies(i)->posB);
    }
}

void ors::KinematicWorld::kinematicsProxyDist(arr& y, arr& J, Proxy *p, double margin, bool useCenterDist, bool addValues) const {
  ors::Shape *a = shapes(p->a);
  ors::Shape *b = shapes(p->b);

  y.resize(1);
  if(&J) J.resize(1, getJointStateDimension());
  if(!addValues){ y.setZero();  if(&J) J.setZero(); }

//  //costs
//  if(a->type==ors::sphereST && b->type==ors::sphereST){
//    ors::Vector diff=a->X.pos-b->X.pos;
//    double d = diff.length() - a->size[3] - b->size[3];
//    y(0) = d;
//    if(&J){
//      arr Jpos;
//      arr normal = ARRAY(diff)/diff.length(); normal.reshape(1, 3);
//      kinematicsPos(NoArr, Jpos, a->body);  J += (normal*Jpos);
//      kinematicsPos(NoArr, Jpos, b->body);  J -= (normal*Jpos);
//    }
//    return;
//  }
  y(0) = p->d;
  if(&J){
    arr Jpos;
    ors::Vector arel, brel;
    if(p->d>0.) { //we have a gradient on pos only when outside
      arel=a->X.rot/(p->posA-a->X.pos);
      brel=b->X.rot/(p->posB-b->X.pos);
      CHECK(p->normal.isNormalized(), "proxy normal is not normalized");
      arr normal; normal.referTo(&p->normal.x, 3); normal.reshape(1, 3);
      kinematicsPos(NoArr, Jpos, a->body, &arel);  J += (normal*Jpos);
      kinematicsPos(NoArr, Jpos, b->body, &brel);  J -= (normal*Jpos);
    }
  }
}

void ors::KinematicWorld::kinematicsProxyCost(arr& y, arr& J, Proxy *p, double margin, bool useCenterDist, bool addValues) const {
  ors::Shape *a = shapes(p->a);
  ors::Shape *b = shapes(p->b);
  CHECK(a->mesh_radius>0.,"");
  CHECK(b->mesh_radius>0.,"");

  y.resize(1);
  if(&J) J.resize(1, getJointStateDimension());
  if(!addValues){ y.setZero();  if(&J) J.setZero(); }

  //costs
  if(a->type==ors::sphereST && b->type==ors::sphereST){
    ors::Vector diff=a->X.pos-b->X.pos;
    double d = diff.length() - a->size[3] - b->size[3];
    y(0) = 1. - d/margin;
    if(&J){
      arr Jpos;
      arr normal = ARRAY(diff)/diff.length(); normal.reshape(1, 3);
      kinematicsPos(NoArr, Jpos, a->body);  J -= 1./margin*(normal*Jpos);
      kinematicsPos(NoArr, Jpos, b->body);  J += 1./margin*(normal*Jpos);
    }
    return;
  }
  double ab_radius = margin + 10.*(a->mesh_radius+b->mesh_radius);
  CHECK(p->d<(1.+1e-6)*margin, "something's really wierd here!");
  CHECK(p->cenD<(1.+1e-6)*ab_radius, "something's really wierd here! You disproved the triangle inequality :-)");
  double d1 = 1.-p->d/margin;
  double d2 = 1.-p->cenD/ab_radius;
  if(d2<0.) d2=0.;
  if(!useCenterDist) d2=1.;
  y(0) += d1*d2;
 
  //Jacobian
  if(&J){
    arr Jpos;
    ors::Vector arel, brel;
    if(p->d>0.) { //we have a gradient on pos only when outside
      arel=a->X.rot/(p->posA-a->X.pos);
      brel=b->X.rot/(p->posB-b->X.pos);
      CHECK(p->normal.isNormalized(), "proxy normal is not normalized");
      arr normal; normal.referTo(&p->normal.x, 3); normal.reshape(1, 3);
          
      kinematicsPos(NoArr, Jpos, a->body, &arel);  J -= d2/margin*(normal*Jpos);
      kinematicsPos(NoArr, Jpos, b->body, &brel);  J += d2/margin*(normal*Jpos);
    }
        
    if(useCenterDist && d2>0.){
      arel=a->X.rot/(p->cenA-a->X.pos);
      brel=b->X.rot/(p->cenB-b->X.pos);
      CHECK(p->cenN.isNormalized(), "proxy normal is not normalized");
      arr normal; normal.referTo(&p->cenN.x, 3); normal.reshape(1, 3);
        
      kinematicsPos(NoArr, Jpos, a->body, &arel);  J -= d1/ab_radius*(normal*Jpos);
      kinematicsPos(NoArr, Jpos, b->body, &brel);  J += d1/ab_radius*(normal*Jpos);
    }
  }
}

/// measure (=scalar kinematics) for the contact cost summed over all bodies
void ors::KinematicWorld::kinematicsProxyCost(arr &y, arr& J, double margin, bool useCenterDist) const {
  y.resize(1).setZero();
  if(&J) J.resize(1, getJointStateDimension()).setZero();
  for(uint i=0; i<proxies.N; i++) if(proxies(i)->d<margin) {
    kinematicsProxyCost(y, J, proxies(i), margin, useCenterDist, true);
  }
}

void ors::KinematicWorld::kinematicsProxyConstraint(arr& g, arr& J, Proxy *p, double margin, bool addValues) const {
  g.resize(1);
  if(&J) J.resize(1, getJointStateDimension());
  if(!addValues){ g.setZero();  if(&J) J.setZero(); }

  g(0) += margin - p->d;

  //Jacobian
  if(&J){
    arr Jpos, normal;
    ors::Vector arel,brel;
    ors::Shape *a = shapes(p->a);
    ors::Shape *b = shapes(p->b);
    if(p->d>0.) { //we have a gradient on pos only when outside
      arel=a->X.rot/(p->posA-a->X.pos);
      brel=b->X.rot/(p->posB-b->X.pos);
      CHECK(p->normal.isNormalized(), "proxy normal is not normalized");
      normal.referTo(&p->normal.x, 3);
    } else { //otherwise take gradient w.r.t. centers...
      arel=a->X.rot/(p->cenA-a->X.pos);
      brel=b->X.rot/(p->cenB-b->X.pos);
      CHECK(p->cenN.isNormalized(), "proxy normal is not normalized");
      normal.referTo(&p->cenN.x, 3);
    }
    normal.reshape(1, 3);

    kinematicsPos(NoArr, Jpos, a->body, &arel);  J -= (normal*Jpos);
    kinematicsPos(NoArr, Jpos, b->body, &brel);  J += (normal*Jpos);
  }
}

void ors::KinematicWorld::kinematicsContactConstraints(arr& y, arr &J) const {
  J.clear();
  ors::Vector normal;
  uint i, con=0;
  Shape *a, *b;
  arr Jpos, dnormal, grad(1, q.N);

  y.clear();
  for(i=0; i<proxies.N; i++) y.append(proxies(i)->d);

  if(!&J) return; //do not return the Jacobian

  ors::Vector arel, brel;
  for(i=0; i<proxies.N; i++) {
    a=shapes(proxies(i)->a); b=shapes(proxies(i)->b);
    
    arel.setZero();  arel=a->X.rot/(proxies(i)->posA-a->X.pos);
    brel.setZero();  brel=b->X.rot/(proxies(i)->posB-b->X.pos);
    
    CHECK(proxies(i)->normal.isNormalized(), "proxy normal is not normalized");
    dnormal.referTo(proxies(i)->normal.p(), 3); dnormal.reshape(1, 3);
    grad.setZero();
    kinematicsPos(NoArr, Jpos, a->body, &arel); grad += dnormal*Jpos; //moving a long normal b->a increases distance
    kinematicsPos(NoArr, Jpos, b->body, &brel); grad -= dnormal*Jpos; //moving b long normal b->a decreases distance
    J.append(grad);
    con++;
  }
  J.reshape(con, q.N);
}

void ors::KinematicWorld::kinematicsLimitsCost(arr &y, arr &J, const arr& limits, double margin) const {
  y.resize(1).setZero();
  if(&J) J.resize(1, getJointStateDimension()).setZero();
  double d;
  for(uint i=0; i<q.N; i++) if(limits(i,1)>limits(i,0)){ //only consider proper limits (non-zero interval)
    double m = margin*(limits(i,1)-limits(i,0));
    d = limits(i, 0) + m - q(i); //lo
    if(d>0.) {  y(0) += d/m;  if(&J) J(0, i)-=1./m;  }
    d = q(i) - limits(i, 1) + m; //up
    if(d>0.) {  y(0) += d/m;  if(&J) J(0, i)+=1./m;  }
  }
}

/// center of mass of the whole configuration (3 vector)
double ors::KinematicWorld::getCenterOfMass(arr& x_) const {
  double M=0.;
  ors::Vector x;
  x.setZero();
  for_list(Body,  n,  bodies) {
    M+=n->mass;
    x+=n->mass*n->X.pos;
  }
  x/=M;
  x_ = ARRAY(x);
  return M;
}

/// gradient (Jacobian) of the COM w.r.t. q (3 x n tensor)
void ors::KinematicWorld::getComGradient(arr &grad) const {
  double M=0.;
  arr J(3, getJointStateDimension());
  grad.resizeAs(J); grad.setZero();
  for_list(Body, n, bodies) {
    M += n->mass;
    kinematicsPos(NoArr, J, n);
    grad += n->mass * J;
  }
  grad/=M;
}

ors::Proxy* ors::KinematicWorld::getContact(uint a, uint b) const {
  uint i;
  for(i=0; i<proxies.N; i++) if(proxies(i)->d<0.) {
      if(proxies(i)->a==(int)a && proxies(i)->b==(int)b) return proxies(i);
      if(proxies(i)->a==(int)b && proxies(i)->b==(int)a) return proxies(i);
    }
  return NULL;
}

/** @brief */
double ors::KinematicWorld::getEnergy() const {
  double m, v, E;
  ors::Matrix I;
  ors::Vector w;
  
  E=0.;
  for_list(Body, n, bodies) {
    m=n->mass;
    ors::Quaternion &rot = n->X.rot;
    I=(rot).getMatrix() * n->inertia * (-rot).getMatrix();
    v=n->X.vel.length();
    w=n->X.angvel;
    E += .5*m*v*v;
    E += 9.81 * m * (n->X*n->com).z;
    E += .5*(w*(I*w));
  }
  
  return E;
}

void ors::KinematicWorld::removeUselessBodies() {
  //-- remove bodies and their in-joints
  for_list_rev(Body, b, bodies) if(!b->shapes.N && !b->outLinks.N) {
    cout <<" -- removing useless body " <<b->name <<" with in-joints ( ";
    delete b;
  }
  //-- reindex
  listReindex(bodies);
  listReindex(joints);
//  for_list(Joint, j, joints) j->index=j_COUNT;  j->ifrom = j->from->index;  j->ito = j->to->index;  }
//  for(Shape *s: shapes) s->ibody = s->body->index;
  //-- clear all previous index related things
  qdim.clear();
  q.clear();
  qdot.clear();
  proxies.clear();
}

bool ors::KinematicWorld::checkConsistency(){
  for(Body *b: bodies){
    CHECK(&b->world, "");
    CHECK(&b->world==this,"");
    CHECK(b==bodies(b->index),"");
    for(Joint *j: b->outLinks) CHECK(j->from==b,"");
    for(Joint *j: b->inLinks)  CHECK(j->to==b,"");
    for(Shape *s: b->shapes)   CHECK(s->body==b,"");
  }
  for(Joint *j: joints){
    CHECK(&j->world && j->from && j->to, "");
    CHECK(&j->world==this,"");
    CHECK(j==joints(j->index),"");
    CHECK(j->from->outLinks.findValue(j)>=0,"");
    CHECK(j->to->inLinks.findValue(j)>=0,"");
  }
  for(Shape *s: shapes){
    CHECK(&s->world, "");
    CHECK(&s->world==this,"");
    CHECK(s==shapes(s->index),"");
    if(s->body) CHECK(s->body->shapes.findValue(s)>=0,"");
  }
  return true;
}

void ors::KinematicWorld::meldFixedJoints() {
  checkConsistency();
  for(Joint *j: joints) if(j->type==JT_fixed) {
    cout <<" -- melding fixed joint " <<j->name <<" (" <<j->from->name <<' ' <<j->to->name <<" )" <<endl;
    Body *a = j->from;
    Body *b = j->to;
    Transformation bridge = j->A * j->Q * j->B;
    //reassociate shapes with a
    for(Shape *s: b->shapes) {
      s->body=a;
      s->rel = bridge * s->rel;
      a->shapes.append(s);
    }
    b->shapes.clear();
    //joints from b-to-c now become joints a-to-c
    for(Joint *jj: b->outLinks) {
      jj->from=a;
      jj->A = bridge * jj->A;
      a->outLinks.append(jj);
    }
    b->outLinks.clear();
    //reassociate mass
    a->mass += b->mass;
    a->inertia += b->inertia;
    b->mass = 0.;
  }
  checkConsistency();
  //-- remove fixed joints and reindex
  for_list_rev(Joint, jj, joints) if(jj->type==JT_fixed) delete jj;
  listReindex(joints);
  //for_list(Joint, j, joints) { j->index=j_COUNT;  j->ifrom = j->from->index;  j->ito = j->to->index;  }
  checkConsistency();
}

//===========================================================================

ors::GraphOperator::GraphOperator():
  symbol(none), timeOfApplication(UINT_MAX), fromId(UINT_MAX), toId(UINT_MAX){
}

void ors::GraphOperator::apply(KinematicWorld& G){
  Body *from=G.bodies(fromId), *to=G.bodies(toId);
  if(symbol==deleteJoint){
    Joint *j = G.getJointByBodies(from, to);
    CHECK(j,"can't find joint between '"<<from->name <<"--" <<to->name <<"' Deleted before?");
    delete j;
    return;
  }
  if(symbol==addRigid){
    Joint *j = new Joint(G, from, to);
//    j->A.setDifference(from->X, to->X);
    j->type=JT_fixed;
//    j->agent=1;
    G.isLinkTree=false;
    return;
  }
  HALT("shouldn't be here!");
}


//===========================================================================
//
// helper routines -- in a classical C interface
//

#endif

#undef LEN

double forceClosureFromProxies(ors::KinematicWorld& ORS, uint bodyIndex, double distanceThreshold, double mu, double torqueWeights) {
  ors::Vector c, cn;
  arr C, Cn;
  for_list(ors::Proxy, p, ORS.proxies){
    int body_a = ORS.shapes(p->a)->body?ORS.shapes(p->a)->body->index:-1;
    int body_b = ORS.shapes(p->b)->body?ORS.shapes(p->b)->body->index:-1;
    if(p->d<distanceThreshold && (body_a==(int)bodyIndex || body_b==(int)bodyIndex)) {
      if(body_a==(int)bodyIndex) {
        c = p->posA;
        cn=-p->normal;
      } else {
        c = p->posB;
        cn= p->normal;
      }
      C.append(ARRAY(c));
      Cn.append(ARRAY(cn));
    }
  }
  C .reshape(C.N/3, 3);
  Cn.reshape(C.N/3, 3);
  double fc=forceClosure(C, Cn, ORS.bodies(bodyIndex)->X.pos, mu, torqueWeights, NULL);
  return fc;
}


//===========================================================================
//-- template instantiations

#include <Core/util_t.h>
template void MT::Parameter<ors::Vector>::initialize();

#ifndef  MT_ORS_ONLY_BASICS
template MT::Array<ors::Shape*>::Array(uint);
template ors::Shape* listFindByName(const MT::Array<ors::Shape*>&,const char*);

#include <Core/array_t.h>
template MT::Array<ors::Joint*>::Array();
#endif
/** @} */
