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


#ifndef MT_MotionPlanner_h
#define MT_MotionPlanner_h

#include "motion.h"

void threeStepGraspHeuristic(arr& q, MotionProblem& M, uint shapeId, uint verbose);
void setGraspGoals_Schunk(MotionProblem& M, uint T, uint shapeId, uint side, uint phase);
void setGraspGoals_PR2(MotionProblem& M, uint T, uint shapeId, uint side, uint phase);
void setPlaceGoals(MotionProblem& M, uint T, uint shapeId, int belowToShapeId, const arr& locationTo);
void setHomingGoals(MotionProblem& M, uint T);

double keyframeOptimizer(arr& x, MotionProblem& M, bool x_is_initialized, uint verbose);
void interpolate_trajectory(arr &q, const arr& q0, const arr& qT, uint T);

#endif
