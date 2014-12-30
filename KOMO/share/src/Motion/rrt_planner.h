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

#ifndef _HEADER_GUARD_RRT_PLANNER_H_
#define _HEADER_GUARD_RRT_PLANNER_H_

#include <Core/array.h>

struct MotionProblem;
struct OpenGL;

namespace ors { 
  struct KinematicWorld;
  struct RRTPlanner {
    private:
      struct sRRTPlanner *s;
    public:
      KinematicWorld *G;                 ///< the graph to plan in
      MotionProblem& problem;   ///< the MotionProblem gives the feasibility test for new states

      arr joint_max, joint_min; ///< in which range are the joints allowed (boundaries for the sample space)

      RRTPlanner(ors::KinematicWorld* G, MotionProblem &problem, double stepsize, bool verbose = false);

      arr getTrajectoryTo(const arr& target, int max_iter=0); ///< returns the trajectory created by the RRT
  };
}




#endif // _HEADER_GUARD_RRT_PLANNER_H_


