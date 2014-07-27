# 0. Quick Start

##State Representations

This section deals with all the definitions that can be found in `monolithic_pr2_planner/include/monolithic_pr2_planner/StateReps`


These form the basic definitions of the planning domain, the state vector, etc., as described below.

### 0.1. Graph State

**Defined in : `monolithic_pr2_planner/include/monolithic_pr2_planner/StateReps/GraphState.h`**

This defines the underlying graph state that constitutes the planning domain. It relies on a RobotState object as its private member.

To define a GraphState object : 

```c++
// Define a robot_state
RobotState robot_state(base_state, right_arm_state, left_arm_state);
// Create a graph state
GraphState graph_state = GraphState(robot_state);
// Create a graph state pointer
GraphStatePtr graph_state_pointer = boost::make_shared<GraphState>(robot_state);
```

Note that GraphStatePtr is `typedef`-ed to a boost::shared_ptr<GraphState>

> Tip : Any ClassPtr is `typedef`-ed to a boost::shared_ptr<Class>

### 0.2. Robot State

**Defined in : `monolithic_pr2_planner/include/monolithic_pr2_planner/StateReps/RobotState.h`**

This defines the state of the robot that the GraphState is built on.

The state of the robot is defined as 

- 6 DOF pose of the object (as held by the robot, both single or dual arm, based on the mode of planning)
- right free angle
- left free angle
- x, y, z, theta of the base


There are four main components to the RobotState : 

- ContBaseState m_base_state
- RightContArmState m_right_arm
- LeftContArmState m_left_arm
- DiscObjectState m_obj_state


> Tip : All member variables are prefixed with `m_`

To build a robot state, we first need the base state and the arm states. Together, these form the complete definition of the robot's current state. The object state is then implicitly calculated from the defined pose of the arm, based on whether the planning is left arm dominant or right arm dominant.

```c++
// assuming you already have the base and arm states
RobotState robot_state(base_state, right_arm_state, left_arm_state);
// create a pointer to a robot state
RobotPosePtr robot_pose_ptr = boost::make_shared<RobotState>(base_state, right_arm_state, left_arm_state);
```

You can also build a robot state from a base state and an object state. This implicitly calls the `compute_robot_pose()` method, which calls the IK solver for the left and right arm poses. This can be useful to compute start states where you want the robot to be holding an object in a particular pose, but you don't really care about the exact angles that the arms take.

```c++
// assuming you have the base state and an object state that you want the arms to be in
RobotState robot_state(base_state, continuous_object_state);  // Note that this ContObjectState is in the body frame.
```

#### Useful methods in RobotState

This section covers some useful methods that are wrapped in the `RobotState` class.

##### 0.2.1. Getting/setting the Base states

There are two kinds of base states as documented in the [Base States](#03-base-states) section. The member variable of the `RobotState` is a `ContBaseState`.

To get the base state from the `RobotState` : 

```c++
// Get the discrete base state
DiscBaseState disc_base_state = robot_state.base_state();
// Get the continuous base state
ContBaseState cont_base_state = robot_state.getContBaseState();
```

Note that the base states are also implicitly converted between each other. Which means, you can assign a `DiscBaseState` to a `ContBaseState` and vice-versa.

```c++
// Convert a discrete base state to a Continuous base state
// robot_state.base_state() returns a discrete base state, which is converted to a ContBaseState
ContBaseState cont_base_state = robot_state.base_state();

// Convert a continuous base state to a Discrete base state
DiscBaseState disc_base_state = robot_state.getContBaseState();
```

Setting the base state of the `RobotState`:

```c++
// assuming you have a new_base_state (can be continuous or discrete)
robot_state.base_state(new_base_state);
```

##### 0.2.2. Getting/setting the arm states
The ContArmState class is documented in the [Arm States](#04-arm-states) section. To set/get the arm states:

```c++
// Get the arm states
RightContArmState right_cont_arm_state = robot_state.right_arm();
LeftContArmState left_cont_arm_state = robot_state.left_arm();

// Set the arm states
robot_state.right_arm(new_right_cont_arm_state);
robot_state.left_arm(new_left_cont_arm_state);
```

Similarly, the free angles can be obtained and set by: 

```c++
// getting the free angles
unsigned int left_free_angle = robot_state.left_free_angle();
unsigned int right_free_angle = robot_state.right_free_angle();

// setting the free angles
robot_state.left_free_angle(new_left_free_angle);
robot_state.right_free_angle(new_right_free_angle);
```

##### 0.2.3. Debugging RobotState

There are useful routines to print the values in the robot state.

```c++
// Print the state to debug level
robot_state.printToDebug(SEARCH_LOG);

// Print the state to info level
robot_state.printToInfo(SEARCH_LOG);

// Print to file
FILE* debug_file = fopen("/tmp/debug_file.txt", "w");
robot_state.printToFile(debug_file);
```

##### 0.2.4. Setting the planning mode
The `setPlanningMode` is a static method that sets the planning mode to one of 7 supported methods, as defined in `Constants.h` :

- BASE_ONLY : only the base motion primitives are applied
- RIGHT_ARM : single arm planning mode
- LEFT_ARM : single arm planning mode
- DUAL_ARM : dual arm, stationary
- RIGHT_ARM_MOBILE : right arm with base
- LEFT_ARM_MOBILE : left arm with base
- DUAL_ARM_MOBILE : dual arm with base

```c++
RobotState::setPlanningMode(monolithic_pr2_planner::PlanningModes::DUAL_ARM_MOBILE);
```

This needs to be set in the planning_request, and the configuration is done in the Environment's `configureQuerySpecificParams`.

##### 0.2.5. Visualizing the robot state
To publish a `MarkerArray` that visualizes the current state, simply call

```c++
robot_state.visualize();

// to set the hue
robot_state.visualize(hue);  // hue is an int from 0 to 240
```

##### 0.2.6 Getting the object state
To get the `x,y,z,roll,pitch,yaw` of the object state with respect to the map or the body:

```c++
// with respect to the body of the robot
DiscObjectState object_state_in_body_frame = robot_state.getObjectStateRelBody();

// with respect to the map's origin
ContObjectState object_state_in_map_frame = robot_state.getObjectStateRelMap();
```

##### 0.2.7 Computing a new robot pose
The static method `computeRobotPose` takes an object state, a seed robot pose and computes a new robot pose. This is used when applying motion primitives to compute an IK solution for the arms in the new object state.

Note that the object state provided must be in the body frame (`/torso_lift_link`)

```c++
computeRobotPose(object_state_you_want_ik_for, seed_robot_pose, pointer_to_new_robot_pose, free_angle_search_or_not);
```

The computed robot pose is stored in the object pointed to by `pointer_to_new_robot_pose`, which is a `RobotPosePtr` instance.

The `bool free_angle_search` parameter controls whether the free angle can be randomized when computing the IK solution.

##### 0.2.8 Interpolation with the RobotState
There are two routines that deal with interpolating between two RobotState objects.

###### numInterpSteps
This method takes in two `RobotState`s, start and end, and returns an integer which represents the number of steps required to finely interpolate between them. 

This method essentially computes the maximum change that takes place in the rotation of the object states and the distance moved by the base. These are then divided by the respective resolutions in the XYZ space and the RPY space to get the number of steps required. The final number is the maximum of the steps required in either space.

Call it thus:

```c++
int steps_required = RobotState::numInterpSteps(start_robot_state, end_robot_state);
```

###### workspaceInterpolate
This method takes in the start and end `RobotState` objects, and a pointer to a vector of `RobotState`s where the result is stored.

The `numInterpSteps` method is called to decide on the number of intermediate robot poses required for the interpolation.

This method returns at least the start and end poses if they are too close to each other.

### 0.3 Base States

The base state objects represent the `x, y, z, theta` of the base, where `z` is the torso. There are two kinds of base states : `ContBaseState` and `DiscBaseState`, which stand for Continuous Base State and Discrete Base State respectively.

To define a base state, there are multiple options.

```c++
// a continuous base state
ContBaseState cont_base_state(x, y, z, theta);
ContBaseState cont_base_state(vector_of_x_y_z_theta);
ContBaseState cont_base_state(BodyPose_object);
ContBaseState cont_base_state(discrete_base_state_object);

// a discrete base state
DiscBaseState disc_base_state(x, y, z, theta);  // integer values; theta can only be one of the discrete angles
DiscBaseState disc_base_state(cont_base_state);  // converts the continuous values to equivalent discrete ones
```

To get the `x, y, z, theta` values, use the following. They're of type `double` or `int` depending on whether the `base_state` is continuous or discrete, respectively.
```c++
// getting the values
base_state.x();
base_state.y();
base_state.z();
base_state.theta();

// setting them
base_state.x(x_value);
base_state.y(y_value);
base_state.z(z_value);
base_state.theta(theta_value);
```

The `ContBaseState::Interpolate` interpolates between two `ContBaseState` objects. It takes in the number of steps parameter, and returns a vector of size `max(2, num_steps + 1)`.

### 0.4 Arm States

The `ContArmState` class is the base class, from which the `LeftContArmState` and the `RightContArmState` classes are derived. The ArmState classes are based off the `SBPLArmModel` from the `pr2_collision_checker` package.

To create an arm model instance,
```c++
// This example is for the left arm. The right arm works similarly.
LeftContArmState l_arm;
l_arm.setShoulderPan(0.038946287971107774);
l_arm.setShoulderLift(1.2146697069025374);
l_arm.setUpperArmRoll(1.3963556492780154);
l_arm.setElbowFlex(-1.1972269899800325);
l_arm.setForearmRoll(-4.616317135720829);
l_arm.setWristFlex(-0.9887266887318599);
l_arm.setWristRoll(1.1755681069775656);

// or, if you have the angles as a vector of doubles,
LeftContArmState l_arm(l_arm_angles);
```

To get current arm angles in a vector: 
```c++
std::vector <double> l_arm_angles;
l_arm_object.getAngles(&l_arm_angles);  // note: the argument is a pointer to a vector.
```

To get the side of the arm, use the `getArm()` function. Valid values are defined in `Constants.h` under `monolithic_pr2_planner::ArmSide`.

The object offset properties define where the tool frame is with respect to the object's position. This is defined by a `KDL::Frame`. When a call to `getObjectStateRelBody()` is made, first the FK solutino is computed to get the position of the tool frame. From this position, the object's offset is further applied to get the position of the object with respect to the body of the robot (`/torso_lift_link`).

### 0.5 Object States
Similar to the Base States, there are two kinds of object states - the `ContObjectState` and the `DiscObjectState`. These define the 6 DOF pose of the object.

As with the base states, these can be implicitly typecast to each other. That is, a `ContObjectState` object can be assigned to a `DiscObjectState` (and vice-versa) and the conversion takes place implicitly.

To get/set the 6 degrees of freedom, use:
```c++
// get values
obj_state.x()
obj_state.y()
obj_state.z()
obj_state.roll()
obj_state.pitch()
obj_state.yaw()

//set values
obj_state.x(value)
obj_state.y(value)
obj_state.z(value)
obj_state.roll(value)
obj_state.pitch(value)
obj_state.yaw(value)
```

The `ContObjectState::interpolate` function returns a vector of `ContObjectState` objects that give the interpolation from the start state to the end state. The size of the vector is `max(2, num_steps+1)`, where `num_steps` is an argument to the function that controls how many steps the interpolation should take.


### 0.6 Goal State

The GoalState object describes the underspecified goal state for the search. It has useful routines that help determine if the goal has been reached.
To create a goal state,
```c++
// all the tolerances are double and the obj_goal is a DiscObjectState instance
GoalStatePtr goal_state_ptr = boost::make_shared<GoalState>(obj_goal, xyz_tolerance, roll_tolerance, pitch_tolerance, yaw_tolerance);
```

To check if a `GraphState` satisfies the goal:
```c++
goal_state_ptr->isSatisfiedBy(graph_state_ptr);
```

When a `GraphState` satisfies the goal pose, it can be added to the list of potential goals by
```c++
goal_state_ptr->addPotentialSolnState(graph_state_ptr);
```

The full solution state is saved by:
```c++
goal_state_ptr->storeAsSolnState(graph_state_ptr);
```


# 3. Adding new heuristics

All the heuristic-related files go in the `Heuristics` folder in `monolithic_pr2_planner/include` and `monolithic_pr2_planner/src`. The setup is:

```
planner <-----> environment <------> HeuristicMgr ----> Heuristic_0
                                                |-----> Heuristic_1
                                                |-----> ...
                                                |-----> Heuristic_n
```

### 3.1. The Heuristic Manager

The `HeuristicMgr` class serves as the interface between all the heuristics and the `Environment`. The environment has an `m_heur_mgr` member.

Adding a new heuristic is as simple as:
```c++
// add a 3D heuristic (commonly the end effector heuristic)
add3DHeur("end_effector_heuristic", cost_multiplier);

// add a 2D heuristic (commonly the base heuristic)
add2DHeur("base_heuristic", cost_multiplier, radius_around_goal);
```

The `cost_multiplier` for all the heuristics is simply a scaling factor. It serves as a method to scale the underlying grid search units to units comparable to other heuristics.

The Heuristic manager uses a `std::map` to identify each heuristic. This makes it easy to add and query heuristics. To get the value of a heuristic,
```c++
// a map that maps strings to integers - heuristic name -> value pairs
typedef std::unordered_map <std::string, int> stringintmap;
// define an empty map
std::unique_ptr<stringintmap> values;

// get all the values at one go for this graph state
m_heur_mgr->getGoalHeuristic(GraphState_you_want_heuristic_for, values);

// getting the value you want is now a nice query
// use the same name that you used when you added the heuristic
int base_heuristic_value = values->at("base_heuristic");
```

The internal map of the manager stores a `std::string -> int` mapping, where the string represents the name of the heuristic, and the integer value is an index into the `m_heuristics` vector. Therefore, when a new heuristic is added, it is added to the `m_heuristics` vector, and this index is then stored in the map.

### 3.2. The AbstractHeuristic class

All heuristics inherit from the `AbstractHeuristic` class that has some necessary definitions.

#### 3.2.1. setGoal
When a planning request is made and the goal state is known, the heuristic manager updates this information in all the heuristics defined via the `heuristic->setGoal(GoalState)` function. Each heuristic can choose to handle this as per its own design requirements.

#### 3.2.2. update3DHeuristicMap() and update2DHeuristicMap()
These functions are called when the 3D occupancy grid or the 2D NavMap is updated. Depending on the nature of the heuristic, a call to either (or both) of these can trigger an update of the heuristic's map. For instance, the `BFS2DHeuristic` updates the map for the `SBPL2DGridSearch` object upon a call to `update2DHeuristicMap()`.

#### 3.2.3. setCostMultiplier() and getCostMultiplier()
As the names suggest, these functions are used to set and get the cost multiplier for the heuristic. The final value (again, depends on the implementation) is `getCostMultiplier() * computed_value`. This can come in handy when designing inadmissible heuristics.

#### 3.2.4 getGoalHeuristic(GraphStatePtr)
The `HeuristicMgr` makes a call to the `getGoalHeuristic` of each heuristic defined in order to return the final values. Therefore, this functions needs to be necessarily implemented to return the (int) value of the heuristic for the query graph state.

### 3.3. Defining new heuristics

To define your own heuristic, create your own `MyNewHeuristic.h` and `MyNewHeuristic.cpp` files. The class needs to inherit from the `AbstractHeurstic` class:
```c++
// inherit from AbstractHeuristic
class MyNewHeuristic : public virtual AbstractHeuristic, public OccupancyGridUser {
    ...
};

// define a shared pointer type
typedef boost::shared_ptr<MyNewHeuristic> MyNewHeuristicPtr;
```

The `OccupancyGridUser` class gives access to the occupancy grid, from where you can get the resolution and so on.

Depending on what your heuristic does, you can add a helper function to the `HeuristicMgr` that will add the heuristic to the internal map and add it to the vector of heuristics.

```c++
// this function makes adding an instance of MyNewHeuristic a cakewalk
HeuristicMgr::addMyNewHeur(std::string name_you_want_to_give_this, ... optional parameters ...) {
    // create the heuristic
    MyNewHeuristicPtr new_heuristic = boost::make_shared<MyNewHeuristic>();

    // set some parameters (optionally)
    new_heuristic->customFunction( ... );

    // update its map
    new_heuristic->update2DHeuristicMap();

    // add it to the internal vector
    m_heuristics.push_back(new_heuristic);

    // create the mapping to the index where this is stored.
    m_heuristic_map[name_you_want_to_give_this] = static_cast<int>(m_heuristics.size() - 1);
}
```