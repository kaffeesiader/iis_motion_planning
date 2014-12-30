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


#ifndef MT_mesh_h
#define MT_mesh_h

#include <Core/array.h>
#include <Core/geo.h>

/// @file
/// @ingroup group_geo
/// @addtogroup group_geo
/// @{

namespace ors {

//===========================================================================
/// a mesh (arrays of vertices, triangles, colors & normals)
struct Mesh {
  arr V;                ///< vertices
  arr Vn;               ///< triangle normals
  arr C;                ///< vertex colors
  
  uintA T;              ///< triangles (faces)
  arr   Tn;             ///< vertex normals

  long parsing_pos_start;
  long parsing_pos_end;
  
  Mesh();
  
  /// @name set or create
  void clear();
  void setBox();
  void setTetrahedron();
  void setOctahedron();
  void setDodecahedron();
  void setSphere(uint fineness=3);
  void setHalfSphere(uint fineness=3);
  void setCylinder(double r, double l, uint fineness=3);
  void setCappedCylinder(double r, double l, uint fineness=3);
  void setGrid(uint X, uint Y);
  void setImplicitSurface(ScalarFunction& f, double lo=-10., double hi=+10., uint res=100);
  void setRandom(uint vertices=10);
  
  /// @name transform and modify
  void subDevide();
  void scale(double f);
  void scale(double sx, double sy, double sz);
  void translate(double dx, double dy, double dz);
  Vector center();
  void box();
  void addMesh(const ors::Mesh& mesh2);
  void makeConvexHull();
  
  /// @name internal computations & cleanup
  void computeNormals();
  void deleteUnusedVertices();
  void fuseNearVertices(double tol=1e-5);
  void clean();
  void flipFaces();
  Vector getMeanVertex();
  double getRadius();

  //[preliminary]]
  void skin(uint i);
  
  /// @name IO
  void write(std::ostream&) const; ///< only writes generic info
  void read(std::istream&, const char* fileExtension);
  void readFile(const char* filename);
  void readTriFile(std::istream& is);
  void readObjFile(std::istream& is);
  void readOffFile(std::istream& is);
  void readPlyFile(std::istream& is);
  void readStlFile(std::istream& is);
  void writeTriFile(const char* filename);
  void writeOffFile(const char* filename);
  void writePLY(const char *fn, bool bin);
  void readPLY(const char *fn);
  void glDraw();
};



//===========================================================================
//
// operators
//

stdOutPipe(Mesh);
uintA getSubMeshPositions(const char* filename);



//===========================================================================
//
// OpenGL static draw functions
//

void glDrawMesh(void *classP);


//===========================================================================
//
// C-style functions
//

void inertiaSphere(double *Inertia, double& mass, double density, double radius);
void inertiaBox(double *Inertia, double& mass, double density, double dx, double dy, double dz);
void inertiaCylinder(double *Inertia, double& mass, double density, double height, double radius);

} //END of namespace

/// @} end of group_geo


//===========================================================================
/**
 * @defgroup ors_interface_qhull QHULL Interface.
 * @{
 */
void plotQhullState(uint D);
extern int QHULL_DEBUG_LEVEL;
const char* qhullVersion();

double distanceToConvexHull(const arr &X,        //points
                            const arr &y,        //query point
                            arr *projectedPoint, //return query point projected on closest facet
                            uintA *faceVertices, //return indices of vertices of closest facet
                            bool freeqhull);     //free allocated qhull engine after request [true]

double distanceToConvexHullGradient(arr& dDdX,       //gradient (or same dim as X)
                                    const arr &X,    //points
                                    const arr &y,    //query point
                                    bool freeqhull); //free allocated qhull engine after request [true]

double forceClosure(const arr& X,  //contact points (size Nx3)
                    const arr& Xn, //contact normals (size Nx3)
                    const ors::Vector& center, //object center
                    double mu=.5,     //friction coefficient
                    double discountTorques=1.,   //friction coefficient
                    arr *dFdX=NULL);    //optional: also compute gradient


void getTriangulatedHull(uintA& T, arr& V);

void getDelaunayEdges(uintA& E, const arr& V);
/** @} */


//===========================================================================
//
// GJK interface
//

double GJK_distance(ors::Mesh& mesh1, ors::Mesh& mesh2,
                    ors::Transformation& t1, ors::Transformation& t2,
                    ors::Vector& p1, ors::Vector& p2);


#endif
