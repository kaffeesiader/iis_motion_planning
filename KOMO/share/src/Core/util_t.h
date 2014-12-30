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


#ifndef MT_util_t_cpp
#define MT_util_t_cpp

#include "util.h"
#include <map>
#include <string>
#include <sstream>
#include <string.h>
#include <iomanip>
#ifndef MT_MSVC
#  include <unistd.h>
#endif

namespace MT {
extern std::ifstream cfgFile;
extern bool cfgFileOpen;
extern Mutex cfgFileMutex;
}

namespace MT {
/** @brief a standard method to save an object into a file. The same as
  std::ofstream file; MT::open(file, filename); file <<x;
  file.close(); */
template<class T> void save(const T& x, const char *filename) {
  std::ofstream file;
  open(file, filename);
  file <<x;
  file.close();
}

/** @brief a standard method to load object from a file. The same as
std::ifstream file; MT::open(file, filename); file >>x;
file.close(); */
template<class T> void load(T& x, const char *filename, bool change_directory) {
#ifdef MT_MSVC
  if(change_directory) MT_MSG("can't handle change_directory with MSVC");
  change_directory = false;
#endif
  if(!change_directory) {
    std::ifstream file;
    open(file, filename);
    file >>x;
    file.close();
  } else {
    FILE(filename) >>x;
  }
}

/** @brief Search for a command line option \c -tag and, if found, pipe the
 next command line option into \c value by the
 \c operator>>(istream&, type&). Returns false on failure. */
template<class T>
bool getFromCmdLine(T& x, const char *tag) {
  char *opt=getCmdLineArgument(tag);
  if(!opt) return false;
  std::istringstream s(opt);
  s >>x;
  if(s.fail()) HALT("error when reading parameter from command line: " <<tag);
  return true;
}


/** @brief Search the first occurence of a sequence '\c tag:'
in the config file (opened automatically) and, if found, pipes
it in \c value. Returns false if parameter is not found. */
template<class T>
bool getFromCfgFile(T& x, const char *tag) {
  cfgFileMutex.lock();
  if(!cfgFileOpen) openConfigFile();
  cfgFile.clear();
  cfgFile.seekg(std::ios::beg);
  if(!cfgFile.good()) { cfgFileMutex.unlock(); return false; }
  unsigned n=strlen(tag);
  char *buf=new char [n+2]; memset(buf, 0, n+2);
  while(cfgFile.good()) {
    memmove(buf, buf+1, n);
    buf[n]=cfgFile.get();
    if(buf[n]==' ' || buf[n]=='\t' || buf[n]==':' || buf[n]=='=') { buf[n]=0; if(!strcmp(tag, buf)) break; buf[n]=':'; }
  };
  delete[] buf;
  
  if(!cfgFile.good()) { cfgFileMutex.unlock(); return false; }
  
  skip(cfgFile, " :=\n\r\t");
  cfgFile >>x;
  
  if(cfgFile.fail()) HALT("error when reading parameter " <<tag);
  cfgFileMutex.unlock();
  return true;
}

template<class T> //von Tim Rackowski
struct ParameterMap {
  static std::map<std::string,T> m;
};

template<class T> std::map<std::string,T> ParameterMap<T>::m;

template<class T>
void putParameter(const char* tag, const T& x) {
  ParameterMap<T>::map[tag] = x;
}

template <class T>
bool getFromMap(T& x, const char* tag) {
  typename std::map<std::string,T>::const_iterator p = ParameterMap<T>::m.find(tag);
  if(p == ParameterMap<T>::m.end())
    return false;
  x = p->second;
  return true;
}

template<class T>
bool getParameterBase(T& x, const char *tag, bool hasDefault, const T* Default) {
  log() <<std::setw(20) <<tag <<" = " <<std::setw(5);
  log().flush();
  
  if(getFromMap<T>(x, tag)) {
    log() <<x <<" [" <<typeid(x).name() <<"] (map!)" <<std::endl;
    return true;
  }
  
  if(getFromCmdLine(x, tag)) {
    log() <<x <<" [" <<typeid(x).name() <<"] (cmd line!)" <<std::endl;
    return true;
  }
  
  if(getFromCfgFile(x, tag)) {
    log() <<x <<" [" <<typeid(x).name() <<"]" <<std::endl;
    return true;
  }
  
  if(hasDefault) {
    if(Default) {
      x=*Default;
      log() <<x <<" [" <<typeid(x).name() <<"] (default!)" <<std::endl;
    }
    return false;
  }
  
  HALT("could not initialize parameter `" <<tag
       <<"': parameter has no default;\n     either use command option `-"
       <<tag <<" ...' or specify `"
       <<tag <<"= ...' in the config file (which might be `MT.cfg')");
}

template<class T> T getParameter(const char *tag) {
  T x;
  getParameterBase<T>(x, tag, false, (T*)NULL);
  return x;
}
template<class T> T getParameter(const char *tag, const T& Default) {
  T x;
  getParameterBase<T>(x, tag, true, &Default);
  return x;
}
template<class T> void getParameter(T& x, const char *tag, const T& Default) {
  getParameterBase<T>(x, tag, true, &Default);
}
template<class T> void getParameter(T& x, const char *tag) {
  getParameterBase(x, tag, false, (T*)NULL);
}
template<class T> bool checkParameter(const char *tag) {
  T x;
  return getParameterBase(x, tag, true, (T*)NULL);
}
template<class T> void Parameter<T>::initialize() {
  if(!initialized) {
    getParameterBase(value, tag, hasDefault, &Default);
    initialized = true;
  }
}
}

#endif
