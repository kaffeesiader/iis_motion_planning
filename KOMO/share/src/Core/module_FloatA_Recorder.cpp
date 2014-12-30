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

#include "module_FloatA_Recorder.h"

REGISTER_MODULE(FloatA_Recorder)

FloatA_Recorder::FloatA_Recorder():Module("FloatA_Recorder"){
}

void FloatA_Recorder::open(){
  file.open(STRING("z." <<x.name <<'.' <<MT::getNowString() <<".dat"));
}


void FloatA_Recorder::close(){
  file.close();
}

void FloatA_Recorder::step(){
  uint rev = x.readAccess();
  floatA X = x();
  double time = x.var->revisionTime();
  x.deAccess();
  MT::String tag;
  tag.resize(30, false);
  sprintf(tag.p, "%6i %13.6f", rev, time);

  file <<tag <<' ' <<X <<endl;
}
