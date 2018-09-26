# Gripper Testing

This package contains plugins and models for testing parallel gripper performance in simulation.

## Requirements

Gazebo 8

ROS Kinetic

Gazebo-ROS compatibility packages

Follow the system setup tutorial for [ARIAC 2018](http://wiki.ros.org/ariac/2018/Tutorials/SystemsSetup) (excluding the ARIAC package itself) for a detailed guide.

## Contents

### Gazebo Plugins

The BenchmarkGripper plugin is used to monitor the pose error of a grasped object while the gripper is subject to various user-defined forces. The plugin can be used with any disembodied parallel gripper. The plugin first sets both the gripper and grasped object to be kinematic except for the gripper's fingertips, and can then be moved using various ROS service calls.

The fingertips can be actuated using the GripCommand ROS service. The plugin uses a simplified control scheme: a constant user-defined force is applied through both finger joints with a proportional corrective force used to maintain the symmetrical position of the gripper's fingertips. This scheme was used to better reflect the functionality of actual pneumatic grippers.

The VelocityCommand service disables the kinematic flag of the grasped object and sets the gripper to move at a constant velocity.

The AccelCommand services disables the kinematic flag of the grasped object and applies a constant linear and angular acceleration to the gripper for the specified duration. While the acceleration is being applied, the plugin publishes the grasped object's pose relative to the gripper on a ROS topic.

The ParallelGripper plugin is a simplified version of the BenchmarkGripper plugin intended to be used in full robot simulations. It uses the same control scheme as the previous plugin and outputs the state of the gripper (enabled/disabled) on a ROS topic.

### Models

Example models of 2 grippers and a gear are provided. Both grippers are set up to use the BenchmarkGripper plugin described above. The included world file spawns the gripper and grasped object in free space so that the gripper can be moved in any direction via acceleration commands.
