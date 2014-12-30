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

#include "ors.h"

//===========================================================================
/**
 * @defgroup ors_interface_SWIFT SWIFT Interface.
 * @{
 */

class SWIFT_Scene;

/// contains all information necessary to communicate with swift
struct SwiftInterface {
  SWIFT_Scene *scene;
  intA INDEXswift2shape, INDEXshape2swift;
  double cutoff;

  SwiftInterface(const ors::KinematicWorld& world, double _cutoff=.2);
  ~SwiftInterface();

  void setCutoff(double _cutoff){ cutoff=_cutoff; }

  void step(ors::KinematicWorld& world, bool dumpReport=false);
  void pushToSwift(const ors::KinematicWorld& world);
  void pullFromSwift(ors::KinematicWorld& world, bool dumpReport);

  void reinitShape(const ors::Shape *s);
//  void close();
  void deactivate(ors::Shape *s1, ors::Shape *s2);
  void deactivate(const MT::Array<ors::Shape*>& shapes);
  void deactivate(const MT::Array<ors::Body*>& bodies);
  void initActivations(const ors::KinematicWorld& world, uint parentLevelsToDeactivate=3);
  void swiftQueryExactDistance();
};
/** @} */
