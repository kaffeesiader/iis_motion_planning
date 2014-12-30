#include <Core/util_t.h>
#include <Gui/opengl.h>

#include <Motion/motionHeuristics.h>
#include <Motion/pr2_heuristics.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Ors/ors_swift.h>

void test_Loading_submeshes()
{
	ors::Mesh mesh;
	mesh.readObjFile(FILE("chair_back_decomposed.obj"));
	OpenGL gl;
	gl.add(ors::glDrawMesh, &mesh);
	gl.watch();
}

void AssembleChair(uint check_constraints=0){
	cout <<"\n= sequential motion planning=\n" <<endl;
	int show_steps = -1;  // output mode
	bool show_costs = true; // plot optimization cost functions
	arr positions;  // constraints
	positions.resize(5,7);
	orsDrawProxies = false;
	arr x, xT;
	ifstream out3("constraints.txt"); positions.readRaw(out3); out3.close();
	//setup the problem
	MT::Array<const char*> targets = {"leg1","leg2","leg3","leg4","chair_back"};
	ors::KinematicWorld G("ikea2.kvg.txt");
	makeConvexHulls(G.shapes);
	arr initial = G.getJointState();
	//-- randomize the initial positions of the objects
	for (uint i=0;i<5;i++) {
		G.getBodyByName(targets(i))->X.addRelativeRotationDeg((rand() % 60)-30,0,1,0);
		G.getBodyByName(targets(i))->X.addRelativeTranslation(0.1,0,-0.2);
	}
	G.calc_fwdPropagateShapeFrames();
	G.watch(true);
	// motion problem
	MotionProblem MP(G);
	MP.loadTransitionParameters();
	MP.H_rate_diag = pr2_reasonable_W(G);

	arr finalpos; ors::Vector current;
	ors::Quaternion original; ors::Quaternion orientation;
	original.set(sqrt(0.5),-sqrt(0.5),0,0);

	if (check_constraints>0) //assemble chair by magic
		for (uint i=0;i<5;i++)  {
			current.set(positions(i,0),positions(i,1),positions(i,2));
			finalpos = ARRAY(G.getShapeByName("chair_sitting_main")->X*(original*current));
			G.getBodyByName(targets(i))->X.pos = finalpos;
			orientation.set(positions(i,3),positions(i,4),positions(i,5),positions(i,6));
			G.getBodyByName(targets(i))->X.rot =G.getShapeByName("chair_sitting_main")->X.rot*orientation;
			G.calc_fwdPropagateShapeFrames();
			G.watch(true);
		}
	else
		for (uint i=0;i<5;i++)  {
			//-- grasp the target
			threeStepGraspHeuristic(xT, MP, G.getShapeByName(targets(i))->index, 2);
			MotionProblemFunction MF(MP);
			sineProfile(x, MP.x0, xT, MP.T);
			// Newton optimization procedure
			optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, damping=1e-0, stopTolerance=1e-2, maxStep=.5));
			if (show_costs){
				MP.costReport();
				gnuplot("load 'z.costReport.plt'", false, true);
			}
			displayTrajectory(x, -1, G, "planned trajectory");

			//-- attach the target to the wrist
			G.glueBodies(G.getBodyByName("l_wrist_roll_link"), G.getBodyByName(targets(i)));
			G.swift().initActivations(G);
			listDelete(MF.configurations);

			//-- setup new motion problem
			MP.prefix.clear();
			listDelete(MP.taskCosts);
			MP.x0 = x[MP.T-1];

			//-- offsets due to differences in mesh/body coordinates  from constraints file
			if (i<4) current.set(positions(i,0),positions(i,1)-0.16,positions(i,2));
			else current.set(positions(i,0),positions(i,1)-0.06,positions(i,2));
			// different tasks
			finalpos = ARRAY(G.getShapeByName("chair_sitting_main")->X*(original*current));
			TaskCost *c;
			// desired position of the target from constraints file
			c = MP.addTask("position", new DefaultTaskMap(posTMT, G, targets(i), ors::Vector(0, 0,0)));
			c->setCostSpecs(MP.T, MP.T, finalpos, 1e3);

			c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, G));
			c->map.order=1; //make this a velocity variable!
			c->setCostSpecs(MP.T, MP.T, {0.}, 1e1);

			c = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, .04));
			c->setCostSpecs(0, MP.T, {0.}, 5e-1);

			c = MP.addTask("homing", new DefaultTaskMap(qItselfTMT,G));
			c->setCostSpecs(0, MP.T, {0.}, 1e-3);

			// desired orientation of the target from constraints file
			orientation.set(positions(i,3),positions(i,4),positions(i,5),positions(i,6));
			c = MP.addTask("orientation", new DefaultTaskMap(quatTMT, G, targets(i), ors::Vector(0, 0, 0)));
			c->setCostSpecs(MP.T, MP.T, ARRAY(G.getShapeByName("chair_sitting_main")->X.rot*orientation), 1e3);

			//initialize trajectory
			for(uint t=0;t<=MP.T;t++) x[t]() = MP.x0;

			//-- optimize
			optNewton(x, Convert(MF), OPT(verbose=2, stopIters=20, maxStep=1., stepInc=2.));
			if (show_costs) MP.costReport();

			displayTrajectory(x, show_steps, G, "planned trajectory", .01);

			//-- deattach the object
			delete G.joints.last();
			G.swift().initActivations(G);

			//-- setup new motion problem - go back to the initial state
			MP.prefix.clear();
			listDelete(MP.taskCosts);
			MP.x0 = x[MP.T-1];
			//! go to the initial state

			c = MP.addTask("q_vel", new DefaultTaskMap(qItselfTMT, G));
			c->map.order=1; //make this a velocity variable!
			c->setCostSpecs(MP.T, MP.T, {0.}, 1e1);

			c = MP.addTask("robot_itself", new DefaultTaskMap(qItselfTMT, G));
			c->setCostSpecs(MP.T, MP.T, initial, 1e1);

			c = MP.addTask("collision", new ProxyTaskMap(allPTMT, {0}, .04));
			c->setCostSpecs(0, MP.T, {0.}, 2e-1);

			//initialize trajectory
			for(uint t=0;t<=MP.T;t++) x[t]() = MP.x0;

			//-- optimize
			optNewton(x, Convert(MF), OPT(verbose=2, stopIters=20, maxStep=1., stepInc=2.));
			if (show_costs) MP.costReport();

			displayTrajectory(x, show_steps, G, "planned trajectory", .001);
			MP.prefix.clear();
			listDelete(MP.taskCosts);
			MP.x0 = x[MP.T-1];

		}
	G.watch(true);

}
//===========================================================================

int main(int argc,char **argv){
	MT::initCmdLine(argc,argv);
	AssembleChair(0);
	return 0;
}
