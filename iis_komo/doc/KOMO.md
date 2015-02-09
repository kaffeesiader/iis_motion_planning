# KOMO motion optimization framework

This document gives an overview about Marc Toussaint's motion optimization framework KOMO, along with some detailed usage instructions. General information about KOMO can be found in [this paper](http://arxiv.org/pdf/1407.0414v1.pdf). As there is no detailed documentation about the framework available, most of the information within this document was acquired by reading the KOMO source code, analyzing the `ikea_chair` example and of course by trial and error. Therefore not everything explained within this document claims absolute correctness as it only reflects my personal opinion about how things can be used.

## KOMO overview

The KOMO framework allows to plan complex robot motions by translating a specified motion problem into a general-case mathematical optimization problem and then finding the best solution for it. The following sections describe the most important data structures:

## Basic data structures

#### `arr` (= `MT::Array<double>`)
Simple array container to store arbitrary-dimensional double arrays. It provides various vector and matrix operations like addition, subtraction, (scalar or vector/matrix) multiplication, division, inner product, outer product, transpose, ...
The definition can be found within `KOMO/share/src/Core/array.h` header file.

A robot configuration within a KinematicWorld instance is represented as `arr` where each value represents a joint position. Trajectories are represented as two-dimensional `arr` structures (arrays of configurations).

Usage:
```cpp
arr a(3);              // create a one dimensional arr of size 3
arr b(2,3);            // create a two dimensional arr of size 2*3
arr c = {1,2,3};       // create and initialize a one dimensional arr

a(0) = 1.2;            // value assignment
b(0,1) = 1.3;          // value assignment

double value = b(0,1); // retrieving single value
arr d = b[0]           // results in a one dimensional arr

arr e = a + c    // addition of two arr instances (dimensions must match)
...              // various operators are defined (+,-,*,/,...)
```

#### `ors::Vector`
A three dimensional vector. Provides common vector functions like normalization, angle calculation and of course operations like addition, subtraction,...

Usage:
```cpp
ors::vector a(1,2,3);
ors::vector b;
b(0) = 2;                  // same as b.x = 2;
b(1) = 3;                  // same as b.y = 3;
b(2) = 4;                  // same as b.z = 4;

ors::vector c = b - a;     // various operators are defined
double angle = b.angle(a); // angle betw. a and b
a.normalize();             // normalization
...                        // various other vector operations
```

#### `ors::Quaternion`
Represents a unit-quaternion and provides common quaternion functionality.

#### `ors::Transformation`
Represents a transformation in Cartesian space, composed from a `ors::Vector` (pos) and a `ors::Quaternion` (rot).

Please have a look into `KOMO/share/src/Core/geo.h` header file for more information about vectors, quaternions and transformations.

##Environment representation
This section describes the most important data types that are used to represent the planning environment. The textual description of the environment is done in ORS format, which is explained later on.

#### `ors::Body`
Represents a rigid body within a `KinematicWorld` instance. Each body has a defined mass and inertial matrix and is composed from one or more shapes. Bodies are identified by their unique `name`. The `index` parameter determines the index of the body within the list of all bodies of the `ors::KinematicWorld` instance.

#### `ors::Shape`
Represents a geometric shape. Shapes can be primitives like cylinders, boxes, spheres, cones... but also complex meshes or just simple marker reference frames. Each shape is associated to a body. The shape type is determined by the `type` parameter of a `ors::Shape` instance. Possible values are:

| ShapeType            | ID   | Description   |
| ---------------------| ---- | ------------- |
| `noneST`             | -1   | never used... |
| `boxST`              |  0   | Box           |
| `sphereST`           |  1   | Sphere        |
| `cappedCylinderST`   |  2   | never used... |
| `meshST`             |  3   | Complex mesh  |
| `cylinderST`         |  4   | Cylinder      |
| `markerST`           |  5   | Marker        |
| `pointCloudST`       |  6   | never used... |

Each shape is identified by its unique name. There are two transformations associated to each shape - one determines the transformation relative to the containing body (parameter `rel`) and the other one seems to be an absolute transformation relative to the world reference frame (parameter `X`). The `index` parameter determines the index of the shape within the list of all shapes of the `ors::KinematicWorld` instance. The boolean parameter `cont` specifies whether contacts with this shape should be registered or not. Setting this parameter to `false` excludes the underlying shape from collision checking. Other shape parameters are `size`, `color` and `body`, which is a pointer to the containing `ors::Body` instance.

#### `ors::Joint`
Represents a robot joint. Joints are connections between bodies. Each joint has a unique name and a clearly defined joint type. There are several possible types of joints but I only used revolute joints and fixed joints for our model.

| JointType            | ID   | Description   |
| ---------------------| ---- | ------------- |
| `JT_none`            | -1   | never used... |
| `JT_hingeX`          |  0   | revolute joint (default) |
| `JT_hingeY`          |  1   | never used... |
| `JT_hingeZ`          |  2   | never used... |
| `JT_transX`          |  3   | never used... |
| `JT_transY`          |  4   | never used... |
| `JT_transZ`          |  5   | never used... |
| `JT_transXY`         |  6   | never used... |
| `JT_trans3`          |  7   | never used... |
| `JT_transXYPhi`      |  8   | never used... |
| `JT_universal`       |  9   | never used... |
| `JT_fixed`           | 10   | Fixed connection |
| `JT_quatBall`        | 11   | never used... |
| `JT_glue`            | 12   | used to temporarily connect two bodies |

Joints can also be identified by their unique name. The `index` parameter determines the index of the joint within the list of all joints of the `ors::KinematicWorld` instance. The `qIndex` parameter represents the index of the joint within the `arr` instance that represents the robot's current configuration within the `KinematicWorld` (more on that later). There are several other parameters available - please have a look into the `Ors/ors.h` header file.

#### `ors::Proxy`
A data structure to store proximity information (when two shapes become close) as return value from external collision libs (Swift, ODE, ...). Each proxy contains the indices of the two shapes and information about distance, contact normal vectors and closest points on surface of both shapes. The parameters are well described in the above mentioned header file.

#### `ors::KinematicWorld`
Data structure that represents the robot and its environment. A `KinematicWorld` instance is composed from lists of bodies, shapes, joints and proxies and provides several functions to access specific instances of those types. The parameters are also described within the `Ors/ors.h` header file.

## Usage of the `ors::KinematicWorld` data structure
This section describes how some common tasks can be performed, using the KOMO framework. The provided code snippets assume variable `w` to be an instance of `ors::KinematicWorld`. This instance can be created based on the ors model, by providing the file path of the description file **relative** to the current working directory:

```cpp
ors::KinematicWorld w("../data/iis_robot.kvg");
```

The above code snippet loads the robot description from the file `iis_komo/data/iis_robot.kvg` and creates a new instance of our `KinematicWorld`. It is assumed that the current working directory is `iis_komo/tmp`.

#### Working with Agents
Agents are the KOMO representation of move groups. A move group can be seen as a group of joints that belong to a component of the robot (left arm, right gripper,...). Each joint of the robot can be associated to an agent (usually done within the ORS description). This concept allows to divide the model into various components and do planning only for a subset of joints that belong to the selected agent. Unfortunately this feature of KOMO is not well tested and therefore still buggy. Maybe this can be fixed in future releases of the framework. I only used this functionality to disable the hand joints of the model by setting the agent of those joints to value 1 (within `move_groups.ors`) and do planning for agent 0.

#### Modifying joint positions
The current configuration of the robot model within the `KinematicWorld` data structure can be modified in different ways. The most common approach is to pass an `arr` data structure, containing the current joint positions to the `setJointState` method:

```cpp
// create an arr instance with correct dimensionality
arr state(w.getJointStateDimension());
// find the qIndex of each single joint and set the desired position
state(w.getJointByName("right_arm_0_joint")->qIndex) = 0.0;
state(w.getJointByName("right_arm_1_joint")->qIndex) = 1.0;
... // repeat for all joints in the model
state(w.getJointByName("left_arm_5_joint")->qIndex) = 2.0;
state(w.getJointByName("left_arm_6_joint")->qIndex) = 1.5;

// pass the new state to our KinematicWorld instance
w.setJointState(state);
// force KOMO to recalculate all the transformations based on the new state
w.calc_fwdPropagateFrames();
```

The above code snippet creates an `arr` instance with the expected dimensions. This results in a one dimensional array. The size of the array depends on the number of active (non fixed) joints of the currently active agent within the model. The call to `w.getJointByName(JOINT_NAME)` results in a pointer to the `Joint` data structure with the given name. The `qIndex` property of the joint denotes the correct index within our `state` array. After setting all target positions, the state array is passed to the `KinematicWorld` instance. The function call `w.calc_fwdPropagateFrames()` forces KOMO to recompute all transformations, based on the new state.

Another approach is to set the joint position directly within the `Joint` data structure:

```cpp
double new_pos = 0.12;
ors::Joint *jnt = w.getJointByName("right_arm_0_joint");
jnt->Q.rot.setRad(new_pos, 1, 0, 0);
... // repeat for other joints as well 

// force KOMO to recalculate all the transformations based on the new state
w.calc_fwdPropagateFrames();
// force the KinematicWorld to update its internal q data structure
// to the new values (necessary for consistence)
w.calc_q_from_Q();
```

The above code snippet finds the joint by its name and returns a pointer to it. Then, the joints transformation is modified by directly modifying the underlying quaternion. After that it is also necessary to force KOMO to update all transformations based on the new settings. The last function call is necessary because the KinematicWorld data structure also holds an instance parameter called `q`, which also contains the current configuration. The statement `w.calc_q_from_Q()` forces KOMO to update this array, based on the new settings.

#### Retrieving and interpreting joint states

The current joint state can easily be accessed as follows:
```cpp
arr state = w.getJointState();
```

The `state` variable now holds a one dimensional array with all joint positions. The following code snippet shows, how the positions of specific joints can be  extracted from this array:

```cpp
int qIdx = w.getJointByName("right_arm_0_joint")->qIndex;
double jnt0Pos = state(qIdx);
```

#### Retrieving collision information
To do collision checking, it is necessary to first set the `KinematicWorld` to the desired joint state, as already explained above. After that, the underlying physics engine has to be triggered to perform a 'step', which simply means to compute proximity/collision information (but also velocities and accelerations if necessary). The collision checking results are then stored in a list of `ors::Proxy` data structures within the `KinematicWorld` instance:

```cpp
// set the desired joint state
w.setJointState(state);
// trigger physics computation (using swift engine)
w.swift().step(w);
// iterate over list of resulting proxies
for(ors::Proxy *p : w.proxies) {
    if(p->d <= MIN_DISTANCE) {
        // do whatever you want with that info :)
    }
}
```

Each `ors::Proxy` stores proximity information, concerning two close-up shapes. The physics engine only considers shapes that are not part of the same body and where the corresponding `cont` property is set to true. Adjacent shapes seem also to be excluded from collision checking (shapes that are always in collision, because their bodies are connected via joints) but I was not able to figure out the exact rules that are applied. The following code snippet shows how the proximity information can be interpreted:

```cpp
// iterate over list of resulting proxies
for(ors::Proxy *p : w.proxies) {
    ors::Shape *shapeA = w.shapes(p->a); // pointer to shape A
    ors::Shape *shapeB = w.shapes(p->b); // pointer to shape B
    double distance = p->d;              // distance
    ors::Vector normal = p->normal       // contact normal
    ... // other possible parameters in 'Ors/ors.h'
}
```

The above code iterates over all `Proxy` instances in the list of proxies. The data structure contains (among others) the indices of both involved shapes, the distance between the shapes and the normal vector pointing from shape B to shape A. 

#### Modifying shape poses
Sometimes it is necessary to modify the position/orientation of a specific shape within the `KinematicWorld`, for example to place the target reference frame before planning. This can be done as follows:

```cpp
// define position and rotation
ors::Vector goal_pos(X,Y,Z);
ors::Quaternion goal_quat(W,X,Y,Z);
// get pointer to desired shape
ors::Shape *target = w.getShapeByName("target");
// set new position and orientation
target->rel.pos = goal_pos;
target->rel.rot = goal_rot;
// force KOMO to update the transformations
w.calc_fwdPropagateShapeFrames();
```

#### FK calculation
The `KinematicWorld` instance can also be used to compute forward kinematics problems. This can be done by setting the joint positions to the desired values and then check the resulting transformation of the desired reference frame:
```cpp
// set the desired state
w.setJointState(state);
// force KOMO to compute all the transformations
w.calc_fwdPropagateShapeFrames();
// retrieve the desired shape (reference frame)
ors::Shape *s = w.getShapeByName("right_sdh_palm_link");
// read the resulting transformation
ors::Transformation trans = s->X;
ors::Vector position = trans.pos;
ors::Quaternion orientation = trans.rot;
```

#### Validating joint limits
The `KinematicWorld` allows to set the joints to arbitrary values, without checking if the provided position lies within the configured limits or not. Joint limits are enforced during motion planning only if a corresponding constraint is added to the `MotionProblem` and even then it is possible that the limits are violated. Therefore it is sometimes necessary to validate if a robot configuration violates joint limits. This can be done as follows:
```cpp
arr config(w.getJointStateDimension());
... // fill config with the desired joint positions

// retrieve the limits as configured in ors description
arr limits = w.getLimits(); // n*2 array where n is joint state dimension
// iterate over all joints and check positions
for(int i = 0; i < config.d0; ++i) {
    // extract upper and lower limits for joint i
    double upper = limits(i, 1);
    double lower = limits(i, 0);
    // ensure that there are configured limits, i.e. upper-lower > 0
    // otherwise skip this joint
    if(upper == lower)
        continue;

    // get the joint position
    double position = config(i);
    // check if limits are violated
    if(position < lower || position > upper) {
        ... // we have a violation detected!
    }
}
```

The above code snippet assumes that the configuration to check is stored in the `config` array. The call to `w.getLimits()` returns a n*2 array where n is the number of active joints. If no limits for given joint are configured then upper and lower limits are set to zero. Therefore it is always necessary to check, whether upper and lower limits defer. All values in the given configuration are validated against upper and lower limits.

#### Displaying robot states and trajectories
Each `KinematicWorld` instance has an associated OpenGL window, that can be used to visualize robot states or trajectories. The window gets created during the first call to one of the display methods. The following example shows how to display the current state:
```cpp
bool block = true;
w.watch(block, "Message to display");
```

The above snippet displays the UI window, shows the current configuration and displays the given message. If the parameter `block` is set to true, this call blocks, until the user hits the ENTER key within the window, otherwise, the call returns immediately. There are also two convenience functions to display arbitrary states or trajectories:
```cpp
displayState(state, w, "Message to display");
```

Displays given `arr` instance `state`, using `KinematicWorld` instance `w` and given message in unblocking mode.
```cpp
int steps = 1; // use -1 for unblocking mode
double delay = 0.01; // delay between waypoints (configurations) 
displayTrajectory(traj, steps, w, "Message to display", delay);
```

Displays given trajectory. The `steps` parameter can be used to switch between blocking(1) and non-blocking(-1) mode. The `delay` parameter indicates the amount of time, each single waypoint is displayed and can therefore be used to control the speed of trajectory visualization. The trajectory is assumed to be an array with dimensions NUM_WAYPOINTS*NUM_JOINTS.

## The ORS robot description format
The model description of the two robot arms and hands is spread across various files, located within the `iis_komo/data/iis_robot` folder. The file `iis_komo/data/iis_robot.kvg` brings everything together and the path to this file needs to be passed as constructor argument when creating an instance of `KinematicWorld`. As explained above, the model consists of various bodies, connected by joints and each body is composed from one or more shapes. All those components need to be defined in a special text format, called ORS format.
```
# This is a comment
# Include the contents of another file
%include FILE_NAME
body  NAME                                    { PARAMETER_LIST }
shape NAME (ASSOCIATED_BODY_NAME)             { PARAMETER_LIST }
joint NAME (PARENT_BODY_NAME CHILD_BODY_NAME) { PARAMETER_LIST }

# Extend the definition of a named component
Merge NAME { PARAMETER_LIST }
```

The lines above show the basic syntax of an ORS description file. Lines beginning with a sharp (#) sign are ignored during parsing. An `%include` statement loads the content of another file and allows therefore to spread the model description across various files. A `Merge` statement allows to extend the parameter list of a component that was previously defined (maybe in one of the included files). This was used for example to define the move group assignment (`move_groups.ors`).

Each line describes a single part of the model, beginning with the keyword that identifies the type of the described component (body, shape or joint). The NAME argument is used to uniquely identify each body/shape/joint. For shapes it is necessary to declare the name of the body, the shape belongs to, enclosed between parenthesis. When defining joints, the parenthesis hold the names of the parent and child bodies. The parameter list is written between curly brackets and depends on the type of the described part.
```
{ param_1 param_2 ... param_n }   // parameters are separated by whitspaces (commas are allowed, but not necessary)
{ boolean_arg }         // boolean parameter (consists only of one keyword, e.g. 'fixed')
{ key=value }           // parameter definition as key/value pair, e.g. 'type=0'
{ key='string value' }  // value is a string
{ key=[val_1 val_2 ... val_n]}  // value is a list, e.g. 'color=[.1 .1 .1]'
{ key=<T ... > }        // value is a transformation, e.g. 'rel=<T t(0.1 0 0)>'

```

As can be seen above, parameters can be either of boolean type (only one keyword) or key-value pairs. The value part can be a single value like a number or a string, a list of values (between brackets, separated by whitspace) or a transformation (see next section). 

#### Defining transformations
The final transformation is described as a sequential series of single transformations/rotations in the following syntax:
```
<T t_1 t_2 ... t_n>
```

A transformation value starts with `<T`. Then comes a list, containing one or more transformation steps (`t_1` - `t_n`), separated by whitespace characters. The transformation description is closed with `>`. The transformation steps are applied sequentially. The following steps can be used:
```
t(x y z)     // translation along vector (x y z)>
d(DEG x y z) // rotation of DEG degrees around vector (x y z)
q(w x y z)   // rotation, defined as quaternion
r(RAD x y z) // rotation of RAD radiants aroung vector (x y z)
E(r p y)     // Euler angle rotation (roll pitch yaw) 
```

Here are some example transformation descriptions:
```
<T t(0.1 0.1 0.3)>  // simple translation
<T t(0.1 0.1 0.3) d(90 1 0 0)>  // translation and rotation
<T t(0.1 0.1 0.3) E(3.14 0 0)>  // translation and rotation, using Euler angle notation
<T r(1.57 1 0 0) t(0 0 0.1) d(45 0 1 0)> // rotation, translation, rotation
```

#### Defining bodies
The following text fragment shows an example body definition:
```
body my_body { X=<T t(-0.1 0.1 0) E(1.57 0 0)> mass=1.75 fixed }
```

Known parameters for bodies:

| Parameter name   | Value type     | Description                           |
| ----------------| -------------- | ------------------------------------- |
| `X`             | Transformation | Transformation relative to world reference frame |
| `mass`          | double         | The mass of the body                  |
| `fixed`         | boolean        | Results in a body with type `staticBT`  |
| `kinematic`     | boolean        | Results in a body with type `dynamicBT` |

The above table shows the arguments allowed within the argument list of a body definition section. The boolean arguments `static`, `kinematic` influence the resulting body type, but I was not able to figure out the exact difference between those types.

#### Defining shapes
Shapes are usually defined after the bodies, because each shape belongs to a specific body which has to be predefined. The following text fragment shows an example shape definition:
```
shape my_shape (my_body) { type=4 size=[.1 .1 .02 .061] rel=<T t(0.2 0.0 0) d(90 1 0 0)> color=[.55 .55 .55] contact}
```

The above shape definition creates a shape with name 'my_shape' that belongs to the body 'my_body'.

Known parameters for shapes:

| Parameter name   | Value type     | Description                          |
| ----------------| -------------- | ------------------------------------- |
| `rel`           | Transformation | Transformation relative to the reference frame of the containing body|
| `size`          | [X Y Z ?]      | 4 values                              |
| `type`          | int            | The type of the resulting shape       |
| `color`         | [R G B A]      | The color of the resulting shape      |
| `mesh`          | string         | Relative path to mesh file            |
| `contact`       | boolean        | When set, collision checking for this shape is enabled |

The `rel` parameter defines the transformation with respect to the body reference frame. The `size` parameter expects a list of 4 values but I was not able to figure out, what the 4th value means. Please refer to the `ors::Shape` section above for a list of available shape types. The `mesh` parameter expects the path to the mesh file and is only required for shapes with `type=3`(`meshST`). The `contact` parameter enables collision checking for the resulting shape. Setting this parameter has the same effect as the following code snippet:
```
ors::Shape *s = w.getShapeByName("my_shape");
s->cont = true; // enable collision checking for this shape
```


#### Defining joints
The following text fragment shows an example joint definition:
```
joint left_arm_0_joint (left_arm_base_link left_arm_1_link)  { q=-.35 A=<T t(0 0 0) d(-90 0 1 0)> B=<T t(0.11 0 0)> Q=<T d(0 1 0 0)> limits=[-2.96705972839 2.96705972839]  ctrl_limits=[0.191986217719 204] }

```

Known parameters for joints:

| Parameter name | Value type     | Description                           |
| ---------------| -------------- | ------------------------------------- |
| `A`            | Transformation | Transformation from parent body to joint |
| `B`            | Transformation | Transformation from joint to child body  |
| `Q`            | Transformation | Transformation within the joint, e.g. rotation |
| `X`            | Transformation | joint pose in world coordinates (same as from->X*A) |
| `type`         | int            | The type of the joint, e.g. `type=0` -> `JT_hingeX` |
| `q`            | double         | The initial joint position            |
| `agent`        | int            | Number of move group, this joint belongs to |
| `limits`       | [LOW UPP]  | Two values, describing the lower and upper movement limits of the joint      |
| `ctrl_limits`  | [VEL ACC]      | Two values, describing the motor limits (velocity and acceleration) | 
| `mimic`        | string         | Name of connected joint, e.g. `mimic='left_sdh_knuckle_joint'` |

The `type` parameter specifies the joint type. All available joint types (along with their IDs) are listed in the `ors::Joint` section above. If no type parameter is specifies, the joint will be of type `JT_hingeX` which is a revolute joint that rotates around its x-axis. The parameters `limits` and `ctrl_limits` expect a list with two values. A mimic joint simply copies the position of the connected joint. Mimic joints are used for the second pivoting joint within the hand model (`left_sdh.ors` and `right_sdh.ors`). The `agent` parameter specifies the move group, this joint belongs to. I used `agent=0` for all arm joints and `agent=1` for all hand joints. Therefore all the hand joints are disabled and not considered during motion planning. The agent-assignment is done within `move_groups.ors`.

## Motion optimization
This document only describes the practical usage of the KOMO framework. Please see the [KOMO paper](http://arxiv.org/pdf/1407.0414v1.pdf) for exact definitions of the used terminology.

### Defining MotionProblems
The first step is to create an instance of type `MotionProblem`, based on a `KinematicWorld` instance that represents the robot and its environment and force KOMO to load the configured transition parameters from the configuration file `config/MT.cfg`:

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

On the first two lines an instance of `LimitsConstraint` is created, using a margin of 0.005 radiants. The call to the function `MotionProblem::addTask` adds the constraint to the `MotionProblem` and returns a pointer to a `TaskCost` instance associated with our constraint. The function `TaskCost::setCostSpecs` is then used to further specify on which time slices the constraint should be enforced, a desired target value (`{.0}` in our case) and the desired precision value (`1e2`). In the given example, the `LimitsConstraint` is enforced on all time slices (`0 to MP.T`).

Each TaskMap/Constraint can be enforced on one or more time slices. Constraints like collision checking or joint limits need to be enforced during the whole trajectory. Other constraints like target end effector position/orientation need to be enforced only on the last time slice as they form the trajectory goal. But an orientation constraint could also be applied to the whole trajectory, for example to keep the end end effector in an upright position during the movement. It is also possible to define waypoints, by placing position constraints on certain time slices of the MotionProblem.

An arbitrary number of `TaskMaps` can be added to each `MotionProblem` instance. The precision parameters influence, how much impact a specific `TaskMap` instance has to the overall optimization process as it controls the amount of penalization in case of constraint violations.

### Available TaskMaps/Constraints

The KOMO framework provides various different types of TaskMaps and Constraints, ready to be used do specify the MotionProblem. I wasn't able to figure out the exact difference between those two terms but it seems that they treat a `TaskMap` as something that causes additional costs if a certain condition is met, whereas a `Constraint` describes something that has to be strictly avoided. However, I also faced situations where I used `Constraints` and they have been violated, but there seems to be a difference how they are treated during the optimization process.

### Cartesian space goals
Goals in Cartesian space are defined by placing a marker reference frame at the desired position and then force KOMO to align any end effector reference frame with the target frame. The target reference frame is usually a shape of type `Marker` that is defined within the robot description file and therefore part of the planning scene. During planning requests, this shape needs to be placed at the desired position/orientation:

```cpp
ors::Shape *target = w.getShapeByName("target");

ors::Vector pos(X, Y, Z);
ors::Quaternion quat(W, X, Y, Z);
quat.normalize();

// set target position...
target->rel.pos = pos;

// ...and orientation
target->rel.rot = quat;

// force KOMO to propagate the shape frames
w.calc_fwnPropagateShapeFrames();
```

It is very important that the quaternion is always normalized, otherwise KOMO delivers unpredictable results!

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
The `DefaultTaskMap` of type `vecAlignTMT` allows to align an axis of the end effector reference frame with another axis of the target reference frame:

```cpp
ors::Vector axis(1,0,0); // X-Axis

c = MP.addTask("align_x", new DefaultTaskMap(vecAlignTMT, eef_idx, axis, target_idx, axis));
c->setCostSpecs(MP.T, MP.T, {1.}, _alignmentPrecision); // {0.} make axes orthogonal
```

The code example adds a constraint that aligns the x-axis of the end effector reference frame with the x-axis of the target frame and enforces this constraint on the last time slice. This constraint could also be used as a path constraint, for example to keep the end effector in a certain orientation during the whole trajectory, simply by enforcing it on all time slices (`0 to MP.T`. The target value in the above example is set to `{1.}`, because this constraint optimizes the dot product of the two vectors. A dot product of 1 means both vectors are in parallel and point in the same direction. Setting the target value to `{0.}` would force KOMO to make the two vectors orthogonal. 

#### End effector velocity
KOMO also allows to constrain the velocity of the specified end effector. This is done by using the same constraint as for the end effector position, but setting the order parameter of the `TaskMap` to 1, which indicates that it should work with velocities (2 -> accelerations).

```cpp
c = MP.addTask("EEF_vel", new DefaultTaskMap(posTMT, eef_idx));
c->setCostSpecs(MP.T, MP.T, {0.}, 1e1);
c->map.order=1; //make this a velocity variable!
```

The example adds a constraint that enforces the velocity of the end effector link to be zero on the last time slice. This technique could probably also be used to constrain the velocity of the end effector during the trajectory, but I did not test that.

#### Joint state
Planning for joint space goals is done by constraining for a certain robot configuration at specified time slices. Therefore a `DefaultTaskMap` of type `qItselfTMT` is used:

```cpp
// the goal configuration must contain exactly one value per joint!
arr goal_config(w.getJointStateDimension());
// set the desired values for each single joint at the correct qIndex
goal_config(w.getJointByName("right_arm_0_joint")->qIndex) = 0.0;
... // repeat for all joints
goal_config(w.getJointByName("right_arm_6_joint")->qIndex) = 0.0;

// TaskMap for goal state
c = MP.addTask("Goal_state", new DefaultTaskMap(qItselfTMT));
c->setCostSpecs(MP.T, MP.T, goal_config, 1e5);
```

It is very important that the `arr` that specifies the target configuration has the correct dimensionality and contains a value for each single joint. Otherwise KOMO will crash as a critical assertion is violated. This can be achieved by first creating an `arr` instance with the joint state dimension of our `KinematicWorld`. After that, the desired joint target positions need to be set at the correct index within our `arr`. This index is called the `qIndex` and can be queried by first finding the joint by its name and then read the `qIndex` parameter. Don't confuse that with the `index` parameter, which indicates the joint's index within the list of joints in the `KinematicWorld`!

The same `TaskMap` could be used to constrain the target joint velocities, by simply setting the `order` parameter to 1:

```cpp
// TaskMap for zero velocity at goal
c = MP.addTask("Goal_vel", new DefaultTaskMap(qItselfTMT));
c->setCostSpecs(MP.T, MP.T, {0.}, 1e1);
c->map.order = 1; //make this a velocity variable!
```

The example adds a constraint for all joint velocities to be zero at the last time slice.

#### Joint limits
During trajectory planning it is also necessary to enforce that all the joints stay within some configured limits. This can be achieved on two different ways - either by using a `LimitsConstraint` or using a `DefaultTaskMap` of type `qLimitsTMT`. The advantage on using a `LimitsConstraint` is that it allows to configure a desired margin whereas the `DefaultTaskMap` uses a default margin of 0.1. The following code snippets show the usage of both approaches:

##### LimitsConstraint approach
```cpp
// first approach - LimitsConstraint
LimitsConstraint *lc = new LimitsConstraint();
lc->margin = 0.005;
c = MP.addTask("Joint_limits", lc);
c->setCostSpecs(0, MP.T, {0.}, 1e1);
```

##### TaskMap approach
```cpp
// second approach - DefaultTaskMap
c = MP.addTask("Joint_limits", new DefaultTaskMap(qLimitsTMT));
c->setCostSpecs(0, MP.T, {0.}, 1e1);
```

On both approaches, the joint limits are considered throughout the whole trajectory. The limits are configured within the .ors description files and can be accessed by the function `KinematicWorld::getLimits()`. This returns a 2 times n matrix (`arr`), containing the upper and lower joint limits. If no limits are configured then it is assumed that the joint has no limit. However, limit constraints can be violated, so it is always necessary to check the resulting trajectories for such violations before sending them to the robot. An example, how this can be achieved can be found within `komo_helpers.h` in the `iis_komo` package.

#### Collisions
There exist different approaches to force KOMO to avoid unwanted collisions of the robot with its environment. The easiest way is to add an instance of `CollisionConstraint` to the `MotionProblem`:

```cpp
double margin = 0.02;
c = MP.addTask("Collision", new CollisionConstraint(margin));
c->setCostSpecs(0, MP.T, {0.}, 1e0);
```

The example creates and adds an instance of `CollisionConstraint` to the `MotionProblem` with a 2 cm collision margin. This constraint will penalize robot states where the distance between any two independent shapes within the environment is below 2 cm. This constraint needs to be enforced during the whole trajectory.

An alternative way to add collision constraints is to make use of the `ProxyTaskMap`. The following code snippet shows how to achieve the same behaviour, using a `ProxyTaskMap` of type `allPTMT`:

```cpp
uintA shapes;
margin = 0.02;
c = MP.addTask("Collision", new ProxyTaskMap(allPTMT, shapes, margin));
c->setCostSpecs(0, MP.T, {0.}, 1e0);
```

The constructor of `ProxyTaskMap` expects an instance of `uintA` (`MT::Array<uint>`) as second parameter but this list can be empty because the task map type `allPTMT` does not make use of it. Therefore we simply pass an empty list. The big advantage in using the `ProxyTaskMap` is that its behavior can be configured with the task map type parameter. It allows to check for all collisions, but also for collisions between specific pairs of shapes, or to exclude specific pairs of shapes from collision checking. In those cases the `shapes` parameter needs to be filled with the indices of the desired shapes. Another way to use the `ProxyTaskMap` would be to force KOMO to get in touch with an object. This could be achieved by setting the `target` (`{0.}`) parameter to a value other than zero. Maybe this could be useful when optimizing a pickup task. An example usage can be found in the function `setGraspGoals_Schunk` within `Motion/motionHeuristics.cpp` source code file.

Possible values for the task map type are:

| TaskMapType            | Description (from header file)                     |
| ---------------------- | -------------------------------------------------- |
| `allPTMP`              | phi = sum over all proxies (as is standard)        |
| `listedVsListedPTMT`   | phi = sum over all proxies between listed shapes   |
| `allVersusListedPTMT`  | phi = sum over all proxies between listed shapes   |
| `allExceptListedPTMT`  | as above, but excluding listed shapes              |
| `pairsPTMT`            | sum over proxies of explicitly listed pairs (shapes is n-times-2)|
| `allExceptPairsPTMT`   | sum excluding these pairs                          |
| `vectorPTMT`           | vector of all pair proxies (this is the only case where dim(phi)>1)|

The `ProxyTaskMap` type is defined within `Motion/taskMap_proxy.h` header file
in KOMO source code. I did not test task map types other than `allPTMT` but some of them are used within the `ikea_chair` example.

It is very important to always check the planning outcome for collision constraint violations after the optimization process because it is not guaranteed that the resulting trajectory is collision free. An example, how this is done can be found within `komo_helpers.h` in the `iis_komo` package.

#### TransitionTaskMap
The `TransitionTaskMap` is responsible for penalizing high velocities or accelerations during the trajectory. For example when planning for a Cartesian space goal without using a `TransitionTaskMap`, the resulting trajectory would simply stay in the start state until the last time slice, where the final state is reached immediately. The `TransitionTaskMap` enforces a smooth transition from the start state towards the final state. The usage of the `TransitionTaskMap` is straight forward:

```cpp
// TaskMap for transition costs
c = MP.addTask("Transitions", new TransitionTaskMap(w));
//  c->map.order = 1; // penalize velocities
c->map.order = 2; // penalize accelerations
c->setCostSpecs(0, MP.T, {0.}, 1e0);
```

The `map.order` parameter in the above example specifies whether to penalize high velocities (=1) or high accelerations (=2). If a trajectory is initialized before starting the optimization process, the `TransitionTaskMap` is not really necessary. For example when planning for joint space goals it is possible to initialize the trajectory by simply interpolating between start and goal state before running the optimization process. In that case it is not necessary to use a `TransitionTaskMap`. The following code snippet shows two ways, how a trajectory could be initialized:

```cpp
arr traj;
sineProfile(traj, start_state, goal_state, MP.T);
```

The function `sineProfile` expects an empty `arr` instance, a start state, a goal state and the amount of time slices. I then fills the `arr` with interpolating states between start and goal state.

### Running the optimization process
After adding all constraints it is necessary to set the desired start configuration of our `MotionProblem`, initialize the trajectory and trigger the optimization process:

```cpp
// set the initial state to current state
MP.x0 = w.getJointState();

// initialize the trajectory
arr traj = replicate(MP.x0, MP.T+1);
rndGauss(traj, .01, true); //don't initialize at a singular config

// interpolation initialization (ALTERNATIVE)
// arr traj;
// sineProfile(traj, start_state, goal_state, MP.T);

//-- create the Optimization problem (of type kOrderMarkov)
MotionProblemFunction MF(MP);

// run the optimization
optConstrained(traj, NoArr, Convert(MF), OPT(verbose=0, stopIters=100, maxStep=.5, stepInc=2., allowOverstep=false));

// report the costs
MP.costReport(true);
```

First, the initial state of the `MotionProblem` is set to the current state of our `KinematicWorld` instance. The trajectory is then initialized by replicating the initial state for the amount of time slices T (+1). The initialization can also be done by interpolating between start state and an arbitrary goal state and depends on the nature of the planning problem.

After that, the `MotionProblem` is converted into a kOrderMarkov optimization problem and the optimization process is started. The `verbose` parameter specifies the amount of console output during the optimization process (0-2). I was not able to figure out the meaning of all the other parameters. The last line triggers KOMO to create a cost report. If the parameter `gnuPlot` is set to true, the reported costs are also shown in a plot window. Otherwise the costs are only printed on the console output and stored in the file `z.costReport` in the current working directory. The array `traj` finally holds the resulting trajectory.

### Interpreting the resulting trajectory
The resulting trajectory is stored in a two dimensional `arr` data structure. The first dimension represents the time slice, the second one represents the joint configuration on each time slice:
```cpp
// iterate over all waypoints
for(int i = 0; i < traj.d0 - 1; ++i) {
    // get waypoint on index i
    arr wp = traj[i];   // represents a robot configuration
    // extract joint positions
    double pos0 = wp(w.getJointByName("right_arm_0_joint")->qIndex);
    double pos1 = wp(w.getJointByName("right_arm_1_joint")->qIndex);
    ... // repeat for all joints
}
```