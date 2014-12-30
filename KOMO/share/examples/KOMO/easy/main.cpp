#include <Ors/ors.h>
#include <Motion/komo.h>

//===========================================================================

void TEST(Easy){
  ors::KinematicWorld G("test.ors");
  arr x = moveTo(G, *G.getShapeByName("endeff"), *G.getShapeByName("target"));
  for(uint i=0;i<1;i++) displayTrajectory(x, 1, G, "planned trajectory", 0.01);
}

//===========================================================================

void TEST(EasyPR2){
  //NOTE: this uses a 25-DOF whole-body-motion model of the PR2
  ors::KinematicWorld G("model.kvg");
  makeConvexHulls(G.shapes);
  for(ors::Shape *s:G.shapes) s->cont=true;
  arr x = moveTo(G, *G.getShapeByName("endeff"), *G.getShapeByName("target"));
  for(uint i=0;i<1;i++) displayTrajectory(x, 1, G, "planned trajectory", 0.01);
}

//===========================================================================

void TEST(EasyAlign){
  ors::KinematicWorld G("test.ors");
  arr x = moveTo(G, *G.getShapeByName("endeff"), *G.getShapeByName("target"), 7); //aligns all 3 axes
  for(uint i=0;i<1;i++) displayTrajectory(x, 1, G, "planned trajectory", 0.01);
}

//===========================================================================

void TEST(EasyAlign2){
  ors::KinematicWorld G("test.ors");
  ors::Shape *s = G.getShapeByName("target");
  s->rel.addRelativeRotationDeg(90,1,0,0);
  arr x = moveTo(G, *G.getShapeByName("endeff"), *s, 7); //aligns all 3 axes
  for(uint i=0;i<1;i++) displayTrajectory(x, 1, G, "planned trajectory", 0.01);
}

//===========================================================================

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

//  testEasy();
//  testEasyAlign();
//  testEasyAlign2();
  testEasyPR2();

  return 0;
}


