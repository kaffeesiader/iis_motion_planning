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


/// @file
/// @ingroup group_ors
/// @addtogroup group_ors
/// @{

#ifndef MT_ors_actionInterface_h
#define MT_ors_actionInterface_h

#include <Core/array.h>


// huepfen der bloecke, falls sie zb runterfallen
#define ODE_COLL_BOUNCE 0.0
//usually .2!! stiffness (time-scale of contact reaction) umso groesser, desto mehr Fehlerkorrektur; muss zwischen 0.1 und 0.5 sein (ungefaehr)
#define ODE_COLL_ERP 0.4
// 0.3
//softness // umso groesser, desto breiter, desto weicher, desto weniger Fehlerkorrektur; zwischen 10e-10 und 10e5
#define ODE_COLL_CFM 10e-4
//1e-2
//alternative: dInfinity;
#define ODE_FRICTION 0.02

#define DROP_TARGET_NOISE 0.11
#define SEC_ACTION_ABORT 500


typedef unsigned int uint;
namespace ors {
struct KinematicWorld;
}
struct OdeInterface;
struct SwiftInterface;
struct OpenGL;

class ActionInterface {
public:
  ActionInterface();
  ~ActionInterface();
  
  //initialization
  void loadConfiguration(const char* ors_filename);
  void startOde(double ode_coll_bounce = ODE_COLL_BOUNCE, double ode_coll_erp = ODE_COLL_ERP, double ode_coll_cfm = ODE_COLL_CFM, double ode_friction = ODE_FRICTION);
  void startSwift();
  //void startSchunk();
  void startIBDS();
  void simulate(uint t);
  
  //shutdown
  void shutdownAll();
  
  //etc
  void watch();
  
  //posture control
  void relaxPosition();   ///< move into a relaxed position
  void moveTo(const char *man_id, const arr& target);      ///< move into a relaxed position
  
  //object manipulation
  //void catchObject(const char *man_id, const char *obj_id);  ///< catch obj_id with the body part man_id
  //void catchObject(const char *obj_id); ///< catch obj with finger of right hand
  //void catchObject(uint obj_id); ///< catch obj with finger of right hand
  void dropObjectAbove(const char *obj_id, const char *rel_id);    ///< drop the body part obj_id above the object rel_id
  void dropObjectAbove(uint obj_id, uint rel_id);
  void dropObjectAbove(uint rel_id); ///< drop obj hold in finger of right hand above the object rel_id
  void grab(const char *man_id, const char *obj_id);
  void grab(uint ID);
  void grab(const char* obj);
  
  //state information
  bool partOfBody(uint id);     ///< check if id is ``part of the body'' (i.e., also grasped)
  uint getCatched(uint man_id); ///< get id of the object catched by man_id
  uint getCatched(); ///< get id of object catched with finger of right hand
#ifndef SWIG //SWIG can't handle arrays and lists yet..
  void getObjects(uintA& objects); ///< return list all objects
  void getManipulableObjects(uintA& objects); ///< return list of manipulable objects
  void getObservableObjects(uintA& objs); ///< objects which are not on the bottom
  void getObjectsAbove(uintA& list, const char *obj_id); ///< return list of objects above and in contact with id
  void getObjectsAbove(uintA& list, const uint obj_id);
  //  void getObjectsBelow(uintA& list, const char *obj_id); ///< return list of objects below and in contact with id
  //  void getObjectsBelow(uintA& list, const uint obj_id);
#endif
  bool onBottom(uint id);
  uint getTableID();
  bool inContact(uint a, uint b);  ///< check if a and b are in contact
  void writeAllContacts(uint id);
  bool isUpright(uint id);        ///< check if id is upright
  uint convertObjectName2ID(const char* name); ///< returns the graph-index of the object named "name"
  const char* convertObjectID2name(uint ID); ///< returns the name of the object with index "ID" in graph
  
  bool freePosition(double x, double y, double radius);
  double highestPosition(double x, double y, double radius, uint id_ignored);
  
  // object attributes
  int getType(uint id);
  // Box: shape[0]=x, shape[1]=y, shape[2]=z, shape[3]=nix
  double* getShape(uint id);
  double* getColor(uint id);
  double* getPosition(uint id);
  
  //void printAboveBelowInfos();
  void printObjectInfo();
  
  OpenGL *gl;
//private:
  ors::KinematicWorld *C;
  OdeInterface *ode; // HIER PARAMETER verstellen
  SwiftInterface *swift;
  TaskVariableList TVs;
  //SchunkModule *schunk;
  //IBDSModule *ibds;
  uint noObjects;
  uint Tabort; //abortion time when attractors fail
  
  void indicateFailure();
};

#endif
/// @}
