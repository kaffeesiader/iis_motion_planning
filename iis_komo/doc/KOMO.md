# KOMO motion optimization framework

This document gives an overview about Marc Toussaint's motion optimization framework KOMO, along with some detailed usage instructions. General information about KOMO can be found in [this paper](http://arxiv.org/pdf/1407.0414v1.pdf). As there is no detailed documentation about the framework available, most of the information within this document was acquired by reading the KOMO source code, analyzing the `ikea_chair` example and of course by trial and error. Therefore not everything explained within this document claims absolute correctness as it only reflects my personal opinion about how things can be used.

## KOMO overview

The KOMO framework allows to plan complex robot motions by translating a specified motion problem into a general-case mathematical optimization problem and then finding the best solution for it.

#### MotionProblem
#### TaskMap
#### KinematicWorld

## Basic data structures

#### `arr` (= `MT::Array<double>`)
Simple array container to store arbitrary-dimensional double arrays. It provides various vector and matrix operations like addition, subtraction, (scalar or vector/matrix) multiplication, division, inner product, outer product, transpose, ...
The definition can be found within `KOMO/share/src/Core/array.h` header file.

A robot configuration within a KinematicWorld instance is represented as `arr` where each value represents a joint position. Trajectories are represented as two-dimensional `arr` structures (arrays of configurations).

#### `ors::Vector`
A three dimensional vector. Provides common vector functions like normalization, angle calculation and of course operations like addition, subtraction,...

#### `ors::Quaternion`
Represents a unit-quaternion and provides common quaternion functionality.

#### `ors::Transformation`
Represents a transformation in Cartesian space, composed from a `ors::Vector` (pos) and a `ors::Quaternion` (rot).

#### `ors::Shape`


#### `ors::Body`

#### `ors::Joint`

#### `ors::KinematicWorld`

## Motion optimization
This document only describes the practical usage of the KOMO framework. Please see the [KOMO paper](http://arxiv.org/pdf/1407.0414v1.pdf) for exact definitions of the used terminology.

### Data structures

#### MotionProblem

#### TaskMap

#### TaskCost

### Defining MotionProblems
The first step is to create an instance of type `MotionProblem`, based on a `KinematicWorld` instance that represents the robot and it's environment and force KOMO to load the configured transition parameters from the configuration file `config/MT.cfg`:

```cpp
// load robot description from file and create
// an instance of KinematicWorld    
ors::KinematicWorld w("../data/iis_robot.kvg")

//-- set up the MotionProblem and load transition params
MotionProblem MP(w);
MP.loadTransitionParameters();
```

after that, the `MotionProblem` needs to be constrained. This is done by adding one or more TaskMaps and/or constraints to the `MotionProblem`. The following code snippet adds a constraint that enforces KOMO to stay within the configured joint limits on all time slices:

```cpp
// Constraint to enforce joint limits on all time slices
LimitsConstraint *lc = new LimitsConstraint();
lc->margin = 0.005;
TaskCost c = MP.addTask("Joint_limits", lc);
c->setCostSpecs(0, MP.T, {0.}, 1e2);
```

On the first two lines an instance of `LimitsConstraint` is created, using a margin of 0.005 radiants. The call to the function `MotionProblem::addTask` adds the constraint to the `MotionProblem` and returns a pointer to a `TaskCost` instance associated with our constraint. The function `TaskCost::setCostSpecs` is then used to further specify on which time slices the constraint should be enforced, a desired target value (`{.0}` in our case) and the desired precission value (`1e2`). In the given example, the `LimitsConstraint` is enforced on all time slices (`0 to MP.T`). Each TaskMap/Constraint can be enforced on one or more time slices. Constraints like collision checking or joint limits need to be enforced during the whole trajectory. Other constraints like target end effector position/orientation need to be enforced only on the last time slice as they form the trajectory goal. But an orientation constraint could also be applied to the whole trajectory, for example to keep the end end effector in an upright position during the movement. It is also possible to define waypoints, by placing position constraints on certain time slices of the MotionProblem.

An arbitrary number of `TaskMaps` can be added to each `MotionProblem` instance. The precision parameters influence, how much impact a specific `TaskMap` instance has to the overall optimization process as it controls the amount of penalization in case of constraint violations.

### Available constraints

The KOMO framework provides various different types of TaskMaps and Constraints, ready to be used do specify the MotionProblem. 

### Cartesian space goals
Goals in Cartesian space are defined by placing a marker reference frame at the desired position and then force KOMO to align any end effector reference frame with the target frame. The target reference frame is usually a shape of type `Marker` that is defined within the robot description file and therefore part of the planning scene. During planning requests, this shape needs to be placed at the desired position/orientation:

```cpp
ors::Shape *target = w.getShapeByName("target");

ors::Vector pos(X, Y, Z);
ors::Quaternion quat(W, X, Y, Z);

// set target position...
target->rel.pos = pos;

// ...and orientation
target->rel.rot = quat;

// force KOMO to propagate the shape frames
w.calc_fwnPropagateShapeFrames();
```

#### End effector position
End effector position constraints are defined by adding an instance of `DefaultTaskMap`. The constructor takes the `TaskMapType` which is `posTMT` and the indices of the end effector and the target marker frames:

```cpp
int eef_idx = w.getShapeByName("right_sdh_tip_link")->index;
int target_idx = w.getShapeByName("target")->index;

TaskCost *c;
// TaskMap for end effector position
c = MP.addTask("EEF_position", new DefaultTaskMap(posTMT, eef_idx, NoVector, target_idx, NoVector));
c->setCostSpecs(MP.T, MP.T, {0.}, 1e4);
```

In the example above adds a position constraint for the end effector link `right_sdh_tip_link` to the `MotionProblem`. The last line specifies, that the constraint should only apply on the last time slice. The end effector link is also a marker reference frame, configured within `data/iis_robot.kvg`. There are various different marker frames available and additional ones can easily be configured within that file.

#### End effector alignment
The `DefaultTaskMap` of type `vecAlignTMT` allows to align an axis of the end effector reference frame with anonther axis of the target reference frame:

```cpp
ors::Vector axis(1,0,0); // X-Axis

c = MP.addTask("align_x", new DefaultTaskMap(vecAlignTMT, eef_idx, axis, target_idx, axis));
c->setCostSpecs(MP.T, MP.T, {1.}, _alignmentPrecision);
```

#### End effector orientation

#### End effector velocity

#### Joint state

#### Joint limits

#### Collisions

#### TransitionTaskMap

### Running the optimization process
After adding all constraints it is necessary to set the desired start configuration of our `MotionProblem`, initialize the trajectory and trigger the optimization process:

```cpp
// set the initial state to current state
MP.x0 = w.getJointState();

// initialize the trajectory
arr x = replicate(MP.x0, MP.T+1);
rndGauss(x, .01, true); //don't initialize at a singular config

//-- create the Optimization problem (of type kOrderMarkov)
MotionProblemFunction MF(MP);

// run the optimization
optConstrained(x, NoArr, Convert(MF), OPT(verbose=0, stopIters=100, maxStep=.5, stepInc=2., allowOverstep=false));

// report the costs
MP.costReport(true);
```

First, the initial state of the `MotionProblem` is set to the current state of our `KinematicWorld` instance. The trajectory is then initialized by replicating the initial state for the amount of time slices T (+1). After that, the `MotionProblem` is converted into a kOrderMarkov optimization problem and the optimization process is started. The `verbose` parameter specifies the amount of console output during the optimization process (0-2). I was not able to figure out the meaning of all the other parameters. The last line triggers KOMO to create a cost report. If the parameter `gnuPlot` is set to true, the reported costs are also shown in a plot window. Otherwise the costs are only printed on the console output and stored in the file `z.costReport` in the current working directory.