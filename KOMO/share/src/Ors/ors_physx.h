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

#ifndef MT_ors_physx_h
#define MT_ors_physx_h
#include "ors.h"

namespace physx {
  class PxMaterial;
}

/**
 * @defgroup ors_interface_physx Interface to PhysX
 * @ingroup ors_interfaces
 * @{
 */
struct PhysXInterface {
  ors::KinematicWorld& world;
  struct sPhysXInterface *s;
  
  PhysXInterface(ors::KinematicWorld& _world);
  ~PhysXInterface();
  
  void step(double tau=1./60.);
  
  void pushToPhysx();
  void pullFromPhysx();

  void setArticulatedBodiesKinematic(uint agent=0);
  void ShutdownPhysX();

  void glDraw();

  void addForce(ors::Vector& force, ors::Body* b);
  void addForce(ors::Vector& force, ors::Body* b, ors::Vector& pos);
};

void glPhysXInterface(void *classP);


void bindOrsToPhysX(ors::KinematicWorld& graph, OpenGL& gl, PhysXInterface& physx);

#endif
/// @}
