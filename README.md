# behaviorlib

Software library for programming robot behaviors.

<a href="https://github.com/cvar-upm/behaviorlib/raw/master/doc/ros_node.png">
       <img src="https://github.com/cvar-upm/behaviorlib/raw/master/doc/ros_node.png" width=400>
   </a>

You can find more information about **behaviorlib** in this publication:

- Martin  Molina,  Pablo  Santamaria,  and  Abraham  Carrera (2021). [Programming  robot  behaviors  with  execution  management  functions](https://arxiv.org/abs/2103.06545).  Arxiv  preprint arXiv:2103.06545.

# Description

Behaviors define different actions that an agent can execute such as a drone taking off or a turtle moving in a straight line or in circles. They are implemented using ROS nodes and they usually form part of a behaviors package that groups behaviors with common uses. 

The `behaviorlib` has a class called `BehaviorExecutionManager` and behaviors extend from it since it provides all the methods needed for the configuration and execution of a behavior.

As it is explained below, behaviors make use of some methods. Here, it's be explained how all those methods work together.

First, the constructor is called in the main function and the behavior starts when `start` is called. Then the behavior proceeds to configure itself by using the method `onConfigure`. 

After `onConfigure` is called, the behavior waits until someone invokes the activation service (see [Services and topics](#services-and-topics)) and then it proceeds to execute the method `onActivate` that reads the parameters and creates subscribers and publishers.

After that, the method `onExecute` is called. This method is inside a loop that constantly calls `onExecute` and the *check* methods (`checkSituation`, `checkProgress`, `checkGoal` and `checkProcesses`) to execute the behavior and check if the goal has been achieved or if something went wrong.

If any of the *check* methods does a `setTerminationCause` (see [Protected members](#protected-members)), the deactivation service (see [Services and topics](#services-and-topics)) is called or the behavior reached the timeout, the behavior executes the  `onDeactivate` method to stop the behavior.

Here there is a diagram to sum up all the information above.

[[Files/diagrama.jpg]]


# Methods 
Behaviors extend from the `BehaviorManager` in `behaviorlib` and they use the methods that will be explained below. There is no need to code all of this methods in the behavior but the essential ones are **onConfigure()** and **onActivate()**.

### onConfigure()
This method prepares the execution and reads params and/or files. It should get the ROS node handle and the namespace and, if needed, parameters from ROS parameter server. It also can subscribe here to topics before the activation of the behavior. This method is only executed once and this happens when the node is launched using `roslaunch`.

### onActivate()
This method activates the behavior, reads the arguments and creates subscribers and publishers. It is executed everytime the activation service is called using `rosservice call ... /activate_behavior`.

### onDeactivate()
This method stops the execution of the behavior and shutdowns every publisher and subscriber the behavior uses. It is executed when the behavior is deactivated by calling the deactivation service using  `rosservice call ... /deactivate_behavior` or when a `setTerminationCause()` is made.

### onExecute()
This method develops the execution and sometimes there is no need to code it since the behavior can be executed in the `onActivate()` method. While the behavior is activated, this method its executed continuously without the need of making infinite loops.

### checkSituation()
This method verifies if the behavior can be executed in the current situation. It is called continuously like the method `onExecute()` without the need of an infinite loop. 

### checkGoal()
This method verifies if the behavior has achieved its goal. If the goal is achieved, it sets the termination cause to `GOAL_ACHIEVED` using the function `setTerminationCause()`. This method is called continuously like `onExecute()` without the need of an infinite loop. 

### checkProgress()
This method supervises the progress of the execution. If the progress is wrong, it sets the termination cause to `WRONG_PROGRESS` using the method `setTerminationCause()`. The method is called continuously like `onExecute()` without the need of an infinite loop. 

### checkProcesses()
This method supervises the state of the necessary processes. If there is a process failure, it sets the termination cause to `PROCESS_FAILURE` using the function `setTerminationCause()`. This method is called continuously like `onExecute()` without the need of an infinite loop. 

# Example

### Step 1: ROS workspace and packages
For this tutorial, you we will be using a ROS workspace to create the ROS package (you can check [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) for more information about how all this works). You will also need to install inside the workspace the library `behaviorlib` which has the `BehaviorExecutionManager`. You  can do it by going to your ROS workspace *src* directory and cloning the repository:

```bash
$ cd src
$ git clone https://github.com/cvar-upm/behaviorlib
```

The behavior we are going to make is simple: it moves a turtle from the `turtlesim` package in a straight line. In the tutorial we are going to create a behavior that receives a distance and it's going to make the node with the turtle move that distance. 

Behaviors are ROS nodes and each ROS node is defined in a ROS package. So the first thing we need to do is to create a ROS package inside the workspace:

```bash
$ catkin_create_pkg <name> <dependencies>
```

ROS packages must have a name (which have a specific format). You can call the package as you wish, but when programming behaviors names are usually the name of the behavior (if the package just defines one behavior, for instance `behavior_rotate`) or the name of what the behaviors control (for example, if we were porgramming behaviors for a quadrotor, the name will be `quadrotor_motion`). In our example we will just code one behavior for moving a turtle node but we will create the package as if it were for a behavior collection. It will control how a turtle moves, so we can go with the name `turtle_motion`.

We also have to specify our project dependencies. This depends greatly on what is actually being done but we will need them in this example. The recommendation here would be to start with what you *obviously* need and then edit metadata files when new dependencies are discovered during development. We know we will for sure depend on `roscpp` (we will be publishing topics with ROS in C++), `behavior_execution_manager` (the entire document is about creating a behavior!) and `turtlesim`, so we will start with those. Write the following command to create the package and rememeber to do it inside the *src* folder of the ROS workspace:

```bash
$ catkin_create_pkg turtle_motion roscpp behavior_execution_manager turtlesim
```

We should now have a `turtle_motion` folder. Catkin should have created the next files:

- `package.xml`: This file contains the definition for the Catkin package.
- `CMakeLists.txt`: CMake file for compiling.
- `src`: This directory is intended to hold all the source code for the behavior.
- `include`: For library headers (we will be doing executables, not libraries, so we are not going to actually use this folder, you can feel free to delete it).

The layout used here is the usual of ROS packages, but you can use whatever you what. We will stick to it.

We will program the entire behavior and then edit the metadata files. Another matter of discussion apart from project layout is code layout (i.e. how the source code is divided into files). Some people like having both headers and implementation in the same folder, others separate them into two different folders... This is just a matter of personal preference. We will just stick with the first method, but choose whatever works for you.

We have said that behaviors are just non-abstract classes that inherit from `BehaviorExecutionManager` and implement all its virtual methods so for creating our first behavior we just need to create both, a header and an implementation file for it. Once again we need to decide a name. As with all things above, you are not forced into a specific naming, but it is common to name a behavior with the structure `Behavior<Description>`, where `<Description>` is just a verb and a noun that specifies what the behavior does. In our case our behavior will just make the turtle node move foward, so we will go with `BehaviorMoveFoward`. That said the next thing we need to do is create the behavior:

```bash
$ cd turtle_motion/src
$ touch BehaviorMoveFoward.hpp
$ touch BehaviorMoveFoward.cpp
```

### Step 2: Header and C++ files

Now we can start programming our behavior. It is usual to first declare the header and then continue with the implementation, so we will first edit `BehaviorMoveFoward.hpp`, open it with your favourite editor and write down the following (fear not, everything will be explained in the following section):

```cpp
// You can declare the header guard as you like, just don't forget it.
#ifndef BEHAVIOR_MOVE_FOWARD_HPP
#define BEHAVIOR_MOVE_FOWARD_HPP

#include <string>

// We will need the `BehaviorExecutionManager` declaration.
#include "BehaviorExecutionManager.h"
// We will be publishing and subscribing to topics, so we will need ROS.
#include "ros/ros.h"
// We'll need this type to publish in the 'velocity' topic
#include "geometry_msgs/Twist.h"
// We'll need this type to subscribe the 'pose' topic
#include "turtlesim/Pose.h"

class BehaviorMoveFoward : public BehaviorExecutionManager {
public:
	BehaviorMoveFoward();

	// These are all the methods we need to implement.
	void onConfigure() override;
	void onActivate() override;
	void onDeactivate() override;
	void onExecute() override;
	bool checkSituation() override;
	void checkGoal() override;
	void checkProgress() override;
	void checkProcesses() override;
};

#endif
```

That is how all behaviors start, obviously the header will grow as we implement it, but for now thats enough.

Next we will edit `BehaviorMoveFoward.cpp` and make the real implementation. This one will be a little more guided. First, we will add the include of the header file and some constants we will be using later: 

```cpp
#include "BehaviorMoveFoward.hpp"

// Name of the behavior
const std::string BehaviorMoveFoward::BEHAVIOR_NAME = "behavior_move_foward";
// Base name of the velocity topic
const std::string BehaviorMoveFoward::CMD_VEL_BASE_NAME = "cmd_vel";
// Base name of the pose topic
const std::string BehaviorMoveFoward::POSE_BASE_NAME = "pose";
// Default distance in case none is passed as argument
const double BehaviorMoveFoward::DEFAULT_DISTANCE = 3;
// Velocity of the turtle
const double BehaviorMoveFoward::LINEAR_VELOCITY = 1;
```

We need a main function to start the behavior:

```cpp
// As we said the behavior doesn't come into play until an instance is started.
// It is important to know that behaviors expect ROS to be initialized when they
// are started, so it is the responsability of whoever started the behavior to
// make sure ROS is running.
// When developing behaviors it is common for the main function to initialize
// ROS, perform any needed setup, create the behavior instance and start it.
int main(int argc, char **argv) {
	// For our example we don't really need setup, so we can just initialize
	// ROS, create the behavior instance and start it.
	ros::init(argc, argv, ros::this_node::getName());

	BehaviorMoveFoward().start();

	return 0;
}
```

Then, we will continue implementing the constructor:

```cpp
// Behavior constructors are responsible for setting the behavior name and its
// execution goal.
BehaviorMoveFoward::BehaviorMoveFoward() {
	// The name of the behavior is usually a constant string obtained by
	// snake-casing the behavior class name (in our case `BehaviorMoveFoward`
	// translates into `behavior_move_foward`).
	setName(BEHAVIOR_NAME);

	// The execution goal depends on whether the behavior runs infinitely or
	// until reaching a specific goal. Our behavior will execute until the
	// desired distance is reached. That means its runs until we reach a goal
	// (the displacement), so wit is are goal oriented.
	setExecutionGoal(BehaviorExecutionManager::ExecutionGoals::ACHIEVE_GOAL);
}
```

Once we have that set we can continue with the virutal methods. The usual way of implementing them is in the same order in which they are executed. As in [Indepth description](#indepth-description), the client of the behavior just creates an instance of it and then calls the `start` method (which starts execution and blocks the calling thread for the rest of the process). When the `start` method is called it invokes `onConfigure`, making it the first function called.

```cpp
// Configuration usually implies setting up all external resources that will be
// used during all executions of the behavior. These resources are commonly
// connections to ROS topics, that is, subscribers and publishers.
// Note that you can also set subscribers and publishers on activation in case
// you want the connections to vary between runs around activation parameters
// or any other external input.
void BehaviorMoveFoward::onConfigure() {
	// In our case we know we will need to publish to the `cmd_vel`
	// topic for updating the turtle velocity and to subscribe to the
	// `pose` topic to receive turtle position updates.
	// This means we will create a publisher for `cmd_vel` and a subscriber
	// for `pose`.
	// You should know that you can get a node handle from the `BehaviorExecutionManager`
	// (this is the same as creating a handle with the default constructor).
	// Behaviors usually subscribe to topics inside their namespace (be careful
	// about the behavior namespace, as it is not the same as the ROS node
	// namespace).
	ros::NodeHandle node_handle = getNodeHandle();
	std::string namespace_ = getNamespace();

	cmd_vel_publisher = node_handle.advertise<geometry_msgs::Twist>(namespace_ + '/' + CMD_VEL_BASE_NAME, 1, true);

	pose_subscriber = node_handle.subscribe(namespace_ + '/' + POSE_BASE_NAME, 1, &BehaviorMoveFoward::onPose, this);
}

```

Observe that we stored the return value into variables we never actually declared. We need this publisher and subscriber to be available when we activate the behavior. If we store them as local variables, we would lose them once configuration is done. We need to store the values as member of the class, so make sure to update the header with the declaration:

```cpp
...

private:
	// At construction this will be default initialized to a *null* publisher.
	// As long as we wait until activation to use it we are safe.
	ros::Publisher cmd_vel_publisher;

	// This will also be initialized to an invalid subscriber.
	ros::Subscriber pose_subscriber;
	
...
```

You can also see that to subscribe a topic we had to pass a callback function called `onPose`. We will explain it later but first, let's add it into the header:

```cpp
...
    void onPose(const turtlesim::Pose &pose);
...
```

In the next methods we will be using a variable called `initial_position` that, as the name says, will contain the initial position of the turtle at the moment of the activation. It will be a `Position` so we also need to add the declaration of the *struct* into the header file.

```cpp
...    
    struct Position {
		double x;
		double y;
        
        // We'll use this function later to calculate the distance between two Positions
		double distance(const Position &position) {
			return std::sqrt(std::pow(position.x - x, 2) + std::pow(position.y - y, 2));
		}
	};

	// We'll declare it as std::optional because we'll need to reset the values later
    std::optional<Position> initial_position;
...
```

After configuration, the behavior will just start waiting for activation. Once someone invokes the activation service (see [Services and topics](#services-and-topics)), `onActivate` will be called, making it our next target:

```cpp
// Behavior activation usually involves three steps: setting up pending conections,
// parsing activation parameters and initializing the behavior internal state.
void BehaviorMoveFoward::onActivate() {
	// In our case we have already all conections set, but we need to parse the
	// the activation parameters and set the speed to the one desired.

	// We will have parameters up on activation, so we just extract it.
	// From the BehaviorExecutionManager point of view parameters is just a
	// string, so it is up to the implementation to decide which the format
	// is. The most common path is to just recive a YAML document, but in our
	// case we will just receive a number (for instance "4.5").
	std::string parameters = getParameters();

	try {
		distance = std::stod(parameters);
	} catch(std::exception &exception) {
		distance = DEFAULT_DISTANCE;
	}

	// After creation the std::optional will not contain a value, os this may
	// seem like something redundant. The trick is, a behavior can be activated
	// several times, so even thought this line will not be needed in the
	// initial case, if someone activates the behavior after a previous
	// execution the pose will contain the wrong value, breaking the behavior!
	initial_position.reset();

	displacement = 0;

	past_displacement = 0;
}
```

In this fragment we introduce three new *undeclared* variable: `distance`, `displacement` and `past_displacement`. They will be needed when we calculate where the turtle needs to go and to check if the turtle arrived at the desired destination or if some error ocurred. By adding their declaration to the header, the `BehaviorMoveFowrward.h` file will be finished.

```cpp
...
	// Don't use this before activation!
	double distance;
    double displacement;
	double past_displacement;
...
```

After activation, execution proceeds. This involves calling `onExecute`. Beware that this method is constantly being called while the behavior is active:

```cpp
// Execution usually involves informing external conections about our state
void BehaviorMoveFoward::onExecute() {
	// The only thing we need to do is apply set the velocity for the turtle
	// to move in the desired direction.
	// The geometry_msgs::Twist class has two fields, one for the linear
	// velocity and another one for the angular velocity.
	// The first one is used when translating (i.e. moving) and the second one
	// when rotating.
	// In our case we only what to move, so we will just modify the linear one.
	geometry_msgs::Twist twist;

	// The geometry_msgs::Twist type is generated by ROS when compiling.
	// These generated types just have one default constructor which
	// zero-initializes every field, so we just need to set whatever we want
	// not to be zero. When instantiated the turtle will start at the center
	// facing to the right. In ROS the horizontal axis is that of the X axis,
	// with negative values to the left of the origin and positive values to
	// the right, so we want to apply a positive velocity on the X axis.
	twist.linear.x = LINEAR_VELOCITY;

	// After that we just need to command the turtle to move.
	cmd_vel_publisher.publish(twist);
}
```

The next functions are used to check the exection of the behavior. Not all of them need to have code inside since it depends on what you want the behavior to do. First, we will code `checkSituation`. 

```cpp
// Checking the situation requires us to check if the invariants needed for
// executing the behavior are met. This usually involves checking all
// conections made looking for some error.
// In case something goes badly this function return should return false
// (remember that, all checks comunicate errors by setting the termination
// cause except checkSituation).
// Note that all checks are called before executing.
bool BehaviorMoveFoward::checkSituation() {
	// ROS doen't really have a way to check if a publisher or a subscriber
	// are valid (they will crash the program if something bad happens),
	// so we will just return true always.
	return true;

	// If there was a way to make the check we would go with some member
	// function like "valid" that would look something like this:
	/* {
		if(!resource.valid()) {
			return false;
		}

		return true;
	} */
}
```

Then, we have the method `checkGoal` who checks if the behavior reached the desired goal. If it did, the behavior stops.

```cpp
// The first check will be the goal check (i.e., is the behavior done?).
// Checking the goal usually just involves testing internal state for some
// condition and set the termination cause to BehaviorActivationFinished::GOAL_ACHIEVED.
void BehaviorMoveFoward::checkGoal() {
	// Our example just needs to check if we have reached the desired
	// destination, that is, if the displacement between the inital
	// and current position is the desired distance.
	if(displacement >= distance) {
		setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);

		// Then we reset the velocity!
		cmd_vel_publisher.publish(geometry_msgs::Twist());
	}

	// Note that we are setting the termination cause here becouse we are
	// making a goal-oriented behavior. If we were to just loop infinitely we
	// would just override the function with an empty body (we would need to
	// check no goal).
}
```

We can also check if the progress of the behavior is correct with `checkProgress`:

```cpp
// Check progress just makes sure we are heading in the right direction.
// If we want to notify any error during execution of the behavior this is the
// point.
void BehaviorMoveFoward::checkProgress() {
	// For the example we want ot make sure we are never heading in the wrong
	// direction, that is, if we are getting close to our target point.
	// We just need to ensure the displacement is not getting greater.
	// This will only happen if we have made some error when programming the
	// behavior or when there is some kind of external entity preventing us
	// from progressing (other behavior is moving the turtle for instance).
	if(past_displacement > displacement) {
		// We notify errors setting the termination cause to `WRONG_PROGRESS`.
		setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
		setErrorMessage("The displacement must decrease for the behavior to be cool!");

		// In case we error we reset the velocity of the turtle.
		cmd_vel_publisher.publish(geometry_msgs::Twist());
	}

	// Note that we are checking only of the current_displacement is greater
	// than the past displacement. You may be temped to check if it is greater
	// or equal to it, if it was equals that would mean we are not moving right?
	// Well, sadly, no. First, equality with floating values is broken due to
	// how their are represented internally (the computer stores an aproximation
	// of the value, not the actual value, so two seemingly equal values may
	// turn out to give a false when comparing them due to aproximation error).
	// Second, even if we were lucky and the comparision succeeded we may be
	// giving a false positive: if we do the check before the pose callback is
	// updated (even if the turlte is moving, it may happen that the message
	// is being sent) the displacement would be the same as in the prevous
	// iteration, reaising the error (even though that would be the correct
	// behavior). We could make the check stronger bt adding more fields to the
	// class, but for the sake of simplicity we will leave as it is (which is
	// still a good implementation).
}
```

The last check method we have is `checkProcesses` but it is deprecated and we don't use it anymore so we'll leave it empty.

```cpp
void BehaviorMoveFoward::checkProcesses() {}
```

We will also need to code what happens when the behavior is deactivated and we do that with `onDeactivate()`:

```cpp
// Finally, we have the deactivation callback. Deactivation usually involves
// releasing any resources adquired during activation.
void BehaviorMoveFoward::onDeactivate() {
	// In our case we haven't really aquired anything so we will just leave this
	// empty.

	// If we had done something like opening a connection to a database during
	// activation we would stick something like this here:
	/* {
		connection.close();
	} */
}
```

To finish, we will have to code the callback functions to all the subscribers we subscribed to. In this case is just the function `onPose()`:

```cpp
// The turtle node will be constantly updating us on its pose.
// We just store it as any other value.
void BehaviorMoveFoward::onPose(const turtlesim::Pose &pose) {
	Position position{pose.x, pose.y};

	if(!initial_position) {
		initial_position = position;
	}

	past_displacement = displacement;
	displacement = position.distance(*initial_position);
}
```

The final files should look like this:

**Header file**
```cpp

#ifndef BEHAVIOR_MOVE_FOWARD_HPP
#define BEHAVIOR_MOVE_FOWARD_HPP

#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "BehaviorExecutionManager.h"
#include "turtlesim/Pose.h"

class BehaviorMoveFoward : public BehaviorExecutionManager {
public:
	BehaviorMoveFoward();

	void onConfigure() override;
	void onActivate() override;
	void onExecute() override;
	void checkGoal() override;
	bool checkSituation() override;
	void checkProgress() override;
	void checkProcesses() override;
	void onDeactivate() override;

private:
	ros::Publisher cmd_vel_publisher;
	ros::Subscriber pose_subscriber;

	double distance;
    double displacement;
	double past_displacement;
	
	struct Position {
		double x;
		double y;

		double distance(const Position &position) {
			return std::sqrt(std::pow(position.x - x, 2) + std::pow(position.y - y, 2));
		}
	};

	std::optional<Position> initial_position;
	
	void onPose(const turtlesim::Pose &pose);
};

#endif
```

 **C++ file**
```cpp
 #include "BehaviorMoveFoward.hpp"

const std::string BehaviorMoveFoward::BEHAVIOR_NAME = "behavior_move_foward";
const std::string BehaviorMoveFoward::CMD_VEL_BASE_NAME = "cmd_vel";
const std::string BehaviorMoveFoward::POSE_BASE_NAME = "pose";
const double BehaviorMoveFoward::DEFAULT_DISTANCE = 3;
const double BehaviorMoveFoward::LINEAR_VELOCITY = 1;

BehaviorMoveFoward::BehaviorMoveFoward() {
    setName(BEHAVIOR_NAME);
    setExecutionGoal(BehaviorExecutionManager::ExecutionGoals::ACHIEVE_GOAL);
}

void BehaviorMoveFoward::onConfigure() {
	ros::NodeHandle node_handle = getNodeHandle();
	std::string namespace_ = getNamespace();

	cmd_vel_publisher = node_handle.advertise<geometry_msgs::Twist>(namespace_ + '/' + CMD_VEL_BASE_NAME, 1, true);
	pose_subscriber = node_handle.subscribe(namespace_ + '/' + POSE_BASE_NAME, 1, &BehaviorMoveFoward::onPose, this);
}

void BehaviorMoveFoward::onActivate() {
	std::string parameters = getParameters();

	try {
		distance = std::stod(parameters);
	} catch(std::exception &exception) {
		distance = DEFAULT_DISTANCE;
	}

	initial_position.reset();
	displacement = 0;
	past_displacement = 0;
}

void BehaviorMoveFoward::onExecute() {
	geometry_msgs::Twist twist;
	twist.linear.x = LINEAR_VELOCITY;
	cmd_vel_publisher.publish(twist);
}

bool BehaviorMoveFoward::checkSituation() {
	return true;
}

void BehaviorMoveFoward::checkGoal() {
	if(displacement >= distance) {
		setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
		cmd_vel_publisher.publish(geometry_msgs::Twist());
	}
}

void BehaviorMoveFoward::checkProgress() {
	if(past_displacement > displacement) {
		setTerminationCause(behavior_execution_manager_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
		setErrorMessage("The displacement must decrease for the behavior to be cool!");

		cmd_vel_publisher.publish(geometry_msgs::Twist());
	}
}

void BehaviorMoveFoward::checkProcesses() {}

void BehaviorMoveFoward::onDeactivate() {}

void BehaviorMoveFoward::onPose(const turtlesim::Pose &pose) {
	Position position{pose.x, pose.y};

	if(!initial_position) {
		initial_position = position;
	}

	past_displacement = displacement;

	displacement = position.distance(*initial_position);
}
```
 
 
### Step 3: Edit the files CMakeLists and package.xml
Once we are done programming and before we try the behavior, we need to inform ROS of its presence and compile it. Both of these tasks can be achieved by editing the package metadata files (`package.xml` and `CMakeLists.txt`). 

First, let's inform ROS of our package by opening the `package.xml` file:

```xml
<?xml version="1.0"?>
<package format="2">
  <name>turtle_motion</name>

  <version>1.0.0</version>

  <description>We move turtles!</description>

  <maintainer email="your.name@your.domain">Your Name</maintainer>

  <license>BSD</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>behavior_execution_manager</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>turtlesim</build_depend>

  <build_export_depend>behavior_execution_manager</build_export_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>turtlesim</build_export_depend>

  <exec_depend>behavior_execution_manager</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>turtlesim</exec_depend>
</package>
```

Second, the `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.0.2)

project(turtle_motion)

find_package(catkin REQUIRED COMPONENTS behavior_execution_manager roscpp turtlesim)

catkin_package(CATKIN_DEPENDS behavior_execution_manager roscpp turtlesim)

add_executable(behavior_move_foward src/BehaviorMoveFoward.cpp)

add_dependencies(behavior_move_foward ${catkin_EXPORTED_TARGETS})

target_include_directories(behavior_move_foward PUBLIC ${catkin_INCLUDE_DIRS})

target_link_libraries(behavior_move_foward ${catkin_LIBRARIES})

set_target_properties(behavior_move_foward PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED TRUE)
```

### Step 4: Run the behavior

Once we are done with these we can just compile the project:

```bash
$ catkin_make
```

When thats done, eveything should be set up. Now we just need to try the behavior. First run the ROS master node:

```bash
$ roscore
```

Second, lauch the turtle node (this should open a window):

```bash
$ rosrun turtlesim turtlesim_node
```

Finally we need to execute the behavior. This involves two steps: first we launch¹ the behavior node:

```bash
$ rosparam set empty/namespace turtle1
$ rosrun turtle_motion behavior_move_foward²
```

With that the behavior will be running, but it is still not activated so it is doing nothing (just look at the simulation window, the turtle should be still). What we need to do now is activating the behavior. For that, we need to invoke the activation service. A quick look at the list of availiable services will reveal us the behavior services:

```bash
$ rosservice list

/clear
/empty/get_loggers
/empty/set_logger_level
/kill
/reset
/rosout/get_loggers
/rosout/set_logger_level
/spawn
/turtle1/behavior_behavior_move_foward/activate_behavior
/turtle1/behavior_behavior_move_foward/check_activation
/turtle1/behavior_behavior_move_foward/check_activation_conditions
/turtle1/behavior_behavior_move_foward/deactivate_behavior
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/get_loggers
/turtlesim/set_logger_level
```


Some of these are from the turtle simulator, others are from the master node and the rest are from our behavior. The ones our behavior is responsible of are all the ones with `behavior_behavior_move_foward` in the name. The one we care is `/turtle1/behavior_behavior_move_foward/activate_behavior`, the activation service (see [Services and topics](#services-and-topics). This service takes a `behavior_execution_manager_msgs/ActivateBehavior` message, which has two fields, the arguments and timeout. We just invoke it as any other ROS service:

```bash
$ rosservice call turtle1/behavior_behavior_move_foward/activate_behavior "{arguments: '5', timeout: 10}"
```

That should activate the behavior and the turtle should start moving foward until reaching five units of distance. With this we are done and you will have a little turtle moving around.

¹ Don't forget sourcing the workspace (`$source devel/setup.sh`)!
² Note that the usual aproach for running behaviors is the same as for ROS nodes: sticking everything into a launch file and call `roslauch`. We are using `rosrun` here for simplicity.


# References

## Protected members

Here are explained some of the protected members that behaviors use.

### getNodeHandle

Gets the ROS node handle.

### getNamespace

Gets the namespace of the behavior.

### setName / getName

Sets or gets the name of the behavior.

### getParameters

Gets the parameters of the behavior.

### setExecutionGoal 

Sets or gets the execution goal of the behavior. It can be: `ACHIEVE_GOAL` or `KEEP_RUNNING`.

### setTerminationCause / getTerminationCause

Sets or gets the termination cause of the behavior. It can be: `GOAL_ACHIEVED`, `WRONG_PROGRESS` or `PROCESS_FAILURE`.

### setErrorMessage

Sets and error message to show.

This table shows which of these internal functions can be used in each method. Method in **bold** have to be used there compulsory.

| Method | Getters | Setters
|-|-|-|
| *Constructor* | getNodeHandle, getNamespace | **setName** and **setExecutionGoal** |
| onConfigure | getName *and Above* | *None* |
| onActivate | getParameters *and Above* | *None*  |
| onExecute | *Above* | *None* |
| checkGoal | *Above* | setTerminationCause with GOAL_ACHIEVED (Optional) |
| checkProgress | *Above* | setTerminationCause with WRONG_PROGRESS (Optional) |
| checkProcesses | *Above* | setTerminationCause with PROCESS_FAILURE (Optional) |
| checkSituation | *Above* | *None* |
| onDeactivate | getTerminationCause | *None* |

Note: All services are published after onCofigure.

The setter *setErrorMessage* can be used in any method.

## Services and topics

### Services

The following services are advertised by the `BehaviorExecutionManager`and they have callback functions for whenever they are called.

#### activate_behavior

The `activate_behavior` service calls the `activateServiceCallback` function in the `BehaviorExecutionManager`. This function checks if the behavior was alredy active and if not, it sets the paremeters, the maximum execution time (see [Protected members](#protected-members)) and calls the `onActivate` method of the behavior.

#### deactivate_behavior

The `deactivate_behavior` service calls the `deactivateServiceCallback` function in the `BehaviorExecutionManager`. This function checks if the behavior was alredy deactivated and if not, it sets the termination cause of the behavior to `INTERRUPTED` and publishes it in the `behavior_activation_finished` topic (see [Topics](#topics)). Then it calls to the `onDeactivate` method of the behavior.

#### check_activation_conditions

The `check_activation_conditions` service calls the `checkSituationServiceCallback` function in the `BehaviorExecutionManager`. This function calls the `checkSituation` method of the behavior.

#### check_activation

The `check_activation` service calls the `checkActivation` method in the `BehaviorExecutionManager` which checks if the behavior is activated.

### Topics

#### behavior_activation_finished

This topic is used to publish or get messages with information about the termination cause of a behavior and error messages related to its termination.

## Parameter Server

This is the list of parameters used by the BehaviorExecutionManager. Parameters are read on the behavior configuration¹ using ROS parameter server.

| Name² | Type | Default | Explanation
|-|-|-|-|
| `namespace` | String | `"drone1"` | Namespace attrbiute. See [Protected members](#protected-members) for more information. |
| `behavior_system` | String | `""` | Used to make a more complete namespace if its a package with various behaviors. |
| `frecuency` | Number | 100 | Times per second `onExecute` will be called once the behavior is activated. |
| `activate_behavior_srv` | String | `"activate_behavior"` | Final fragment of the activation service name. See [Services and topics](#services-and-topics) for more information. |
| `deactivate_behavior_srv` | String | `"deactivate_behavior"` | Final fragment of the deactivation service name. See [Services and topics](#services-and-topics) for more information. |
| `check_activation_conditions_srv` | String | `"check_activation_conditions"` | Final fragment of the checkSituation service. See [Services and topics](#services-and-topics) for more information. |
| `check_activation_srv` | String | `"check_activation"` | Final fragment of the checkActivation service. See [Services and topics](#services-and-topics) for more information. |
| `activation_finished_topic` | String | `"behavior_activation_finished"` | Final fragment of the publisher name. See [Services and topics](#services-and-topics) for more information. |


¹ Parameters are read just before invoking  `onConfigure`. Modifying them inside the member function will have no effect.
² Names are read with respect to the ROS namespace the node is in, these names are not absolute.
