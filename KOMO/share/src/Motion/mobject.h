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

#ifndef MOBJECT_H
#define MOBJECT_H

#include <Core/util.h>
#include <Core/array.h>
#include <Ors/ors.h>

/* M(oving)Objects represents an object in the scene and moves or rotates the object
 */

struct MObject {
  enum ObjectType {OBSTACLE, GOAL};

  ors::KinematicWorld *world;
  MT::String name;
  ObjectType objectType;
  double stepLength;

  arr position;
  arr orientation;
  arr direction;
  arr prediction;
  arr positionHistory; // save positions of object
  arr orientationHistory;

  MObject(ors::KinematicWorld *_world,MT::String _name, ObjectType _objectType, double _stepLength=0.001, const arr& _direction = ARRAY(1.,0.,0.));
  ~MObject();

  void predict(uint _T); // predict _T time steps ahead
  void move();  // simulate object for one time step along direction
  void move(const arr& _offset);  // simulate object for one time step
  void rotate(const arr& _offset);

  // Getter
  void setPosition(const arr& _position);
  void setOrientation(const arr& _orientation);
};

#endif // MOBJECT_H
