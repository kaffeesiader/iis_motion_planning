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

#include "mobject.h"

MObject::MObject(ors::KinematicWorld *_world,MT::String _name, ObjectType _objectType, double _stepLength, const arr &_direction) {
  world=_world;
  name = _name;
  objectType = _objectType;
  direction = _direction;
  stepLength = _stepLength;
  cout << "Loaded MObject: " << name << endl;

  world->kinematicsPos(position,NoArr, world->getBodyByName(name));
  positionHistory.append(~position);

  world->kinematicsVec(orientation,NoArr, world->getBodyByName(name));
  orientationHistory.append(~orientation);
}

MObject::~MObject() {

}


void MObject::predict(uint _T) {
  prediction = position + (direction*stepLength)*double(_T);
}

void MObject::setPosition(const arr& _position) {
  world->kinematicsPos(position,NoArr, world->getBodyByName(name));
  positionHistory.append(~position);
  world->getBodyByName(name)->X.pos = _position;
  position = _position;
}

void MObject::setOrientation(const arr& _orientation) {
  positionHistory.append(~orientation);
  orientation = _orientation;
}

void MObject::move() {
  world->kinematicsPos(position,NoArr, world->getBodyByName(name));
  positionHistory.append(~position);
  position = position + (direction*stepLength);
  world->getBodyByName(name)->X.pos = position;
}

void MObject::move(const arr& _offset) {
  world->kinematicsPos(position,NoArr, world->getBodyByName(name));
  positionHistory.append(~position);
  position = position + _offset;
  world->getBodyByName(name)->X.pos = position;
}

void MObject::rotate(const arr& _offset) {
  orientationHistory.append(~orientation);
  orientation = orientation + _offset;
  world->getBodyByName(name)->X.rot.setDeg( world->getBodyByName(name)->X.rot.getDeg()+0.7,_offset);
}

