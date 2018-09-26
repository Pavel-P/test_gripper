#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include "test_gripper/GripCommand.h"
#include "test_gripper/VelocityCommand.h"
#include "test_gripper/AccelCommand.h"
#include <geometry_msgs/PoseStamped.h>

/* Plugin for Simulated Grasp Benchmarking 

This plugin is used by a disembodied parallel gripper to benchmark grasp performance
across a range of linear and angular accelerations.

It provides 3 ROS services:
  1. GripCommand - Enables or disables the gripper
  2. VelocityCommand - Sets the linear and angular velocity of the gripper
  3. AccelCommand - Applies a constant acceleration to the gripper.

The first 2 services are intended to be used for manual testing, while the
last services provides a means of automated benchdisplacementmarking. The final service 
also generates a log of the grasped object's pose relative to the gripper's
frame of reference. */

namespace gazebo
{
  class BenchmarkGripper : public ModelPlugin
  {
    private: physics::JointPtr joint1;
    private: physics::JointPtr joint2;
    private: physics::LinkPtr base_link;
    private: physics::ModelPtr grasped_object;
    
    private: bool grip_enabled;
    private: double grip_force_close;
    private: double grip_force_open;
    private: double grip_kp;

    private: bool accel_command_active;
    private: common::Time prev_time;
    private: common::Time elapsed_time;

    private: ignition::math::Vector3d linear_accel;
    private: ignition::math::Vector3d angular_accel;
    private: double accel_duration;
    
    private: ignition::math::Vector3d linear_velocity;
    private: ignition::math::Vector3d angular_velocity;
    
    private: ros::ServiceServer grip_service;
    private: ros::ServiceServer velocity_service;
    private: ros::ServiceServer accel_service;
    private: ros::Publisher grasp_pose_pub;
    private: std::unique_ptr<ros::NodeHandle> rosnode;

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
          << "unable to load plugin. Load the Gazebo system plugin "
          << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }
      
      //Get SDF Params

      grip_force_close = 100;
      if (_sdf->HasElement("grip_force_close"))
        grip_force_close = _sdf->Get<double>("grip_force_close");
      
      grip_force_open = -0.5 * grip_force_close;
      if (_sdf->HasElement("grip_force_open"))
        grip_force_open = _sdf->Get<double>("grip_force_open");

      grip_kp = 1000;
      if (_sdf->HasElement("grip_kp"))
        grip_kp = _sdf->Get<double>("grip_kp");

      grip_enabled = false;
      if (_sdf->HasElement("enable_grip_on_start"))
        grip_enabled = _sdf->Get<bool>("enable_grip_on_start");

      std::string joint1_name = "joint1";
      if (_sdf->HasElement("joint1"))
        joint1_name = _sdf->Get<std::string>("joint1");
      
      std::string joint2_name = "joint2";
      if (_sdf->HasElement("joint2"))
        joint2_name = _sdf->Get<std::string>("joint2");

      std::string base_link_name = "base_link";
      if (_sdf->HasElement("base_link"))
        base_link_name = _sdf->Get<std::string>("base_link");

      std::string grasped_object_name = "object";
      if (_sdf->HasElement("grasped_object"))
        grasped_object_name = _sdf->Get<std::string>("grasped_object");
      
      std::string robot_namespace = "";
      if (_sdf->HasElement("robot_namespace"))
        robot_namespace = _sdf->Get<std::string>("robot_namespace");

      std::string grip_topic = "test_gripper/grip";
      if (_sdf->HasElement("grip_topic"))
        grip_topic = _sdf->Get<std::string>("grip_topic"); 

      std::string velocity_topic = "test_gripper/velocity";
      if (_sdf->HasElement("velocity_topic"))
        velocity_topic = _sdf->Get<std::string>("velocity_topic");

      std::string accel_topic = "test_gripper/accel";
      if (_sdf->HasElement("accel_topic"))
        accel_topic = _sdf->Get<std::string>("accel_topic");

      std::string grasp_pose_topic = "test_gripper/grasp_pose";
      if (_sdf->HasElement("grasp_pose_topic"))
        grasp_pose_topic = _sdf->Get<std::string>("grasp_pose_topic");


      // Store the pointer to the model
      this->model = _parent;

      //get pointers to each gripper joint
      joint1 = this->model->GetJoint(joint1_name);
      joint2 = this->model->GetJoint(joint2_name);
      if (!joint1 || !joint2) {
        ROS_FATAL_STREAM("Gripper joints not found, check specified joint names.");
      }

      base_link = this->model->GetLink(base_link_name);
      if (!base_link) {
        ROS_FATAL_STREAM("Gripper base link not found, check specified link name.");
      }
      
      //get pointer to grasped object
      grasped_object = this->model->GetWorld()->ModelByName(grasped_object_name);
      if (!grasped_object) {
        ROS_FATAL_STREAM("Grasped object not found, check specified object name.");
      }

      //disable physics for base of gripper
      base_link->SetKinematic(true);
      this->model->SetGravityMode(false);

      //disable physics for grasped object at start of simulation
      for(auto const& link : grasped_object->GetLinks())
        link->SetKinematic(true);

      linear_velocity = ignition::math::Vector3d(0,0,0);
      angular_velocity = ignition::math::Vector3d(0,0,0);

      accel_command_active = false;     

      rosnode.reset(new ros::NodeHandle(robot_namespace));

      grip_service = rosnode->advertiseService(grip_topic, &BenchmarkGripper::OnGripCommand, this);
      velocity_service = rosnode->advertiseService(velocity_topic, &BenchmarkGripper::OnVelocityCommand, this);
      accel_service = rosnode->advertiseService(accel_topic, &BenchmarkGripper::OnAccelCommand, this);

      grasp_pose_pub = rosnode->advertise<geometry_msgs::PoseStamped>(grasp_pose_topic, 1000);
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&BenchmarkGripper::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      if (grip_enabled) {

        //Additional force modifier to ensure symmetrical finger positions
        double force_modifier = (joint1->Position(0) - joint2->Position(0)) * grip_kp;

        joint1->SetForce(0, grip_force_close - force_modifier);
        joint2->SetForce(0, grip_force_close + force_modifier);

      } else {
        joint1->SetForce(0, grip_force_open);
        joint2->SetForce(0, grip_force_open);
      }      
      
      if (accel_command_active) {
        
        PublishGraspPose();

        auto current_time = this->model->GetWorld()->SimTime();
        auto time_step = current_time - prev_time;

        elapsed_time += time_step;
        prev_time = current_time;

        auto linear_accel_step = linear_accel * time_step.Double();
        auto angular_accel_step = angular_accel * time_step.Double();

        // Acceleration represented as 3-piece velocity function
        if (elapsed_time.Double() < accel_duration / 4) {

          linear_velocity += linear_accel_step;
          angular_velocity += angular_accel_step;
          
        } else if (elapsed_time.Double() < accel_duration * 3 / 4) {

          linear_velocity -= linear_accel_step;
          angular_velocity -= angular_accel_step;

        } else if (elapsed_time.Double() < accel_duration) {

          linear_velocity += linear_accel_step;
          angular_velocity += angular_accel_step;

        //additional time for simulation to settle  
        } else if (elapsed_time.Double() < accel_duration + 0.5) {

          linear_velocity = ignition::math::Vector3d(0,0,0);
          angular_velocity = ignition::math::Vector3d(0,0,0);

        }
        else {
            linear_velocity = ignition::math::Vector3d(0,0,0);
            angular_velocity = ignition::math::Vector3d(0,0,0);
            accel_command_active = false;
            
            //Disable physics at end of sweep
            for(auto const& link : grasped_object->GetLinks())
              link->SetKinematic(true);

            grasped_object->SetLinearVel(ignition::math::Vector3d(0,0,0));
            grasped_object->SetAngularVel(ignition::math::Vector3d(0,0,0));

            gzdbg << "Accel Sweep Complete!" << std::endl;
        }
        
      }
      //NOTE: Kinematic links cannot be moved via SetLinearAccel/SetAngularAccel
      this->model->SetLinearVel(linear_velocity);
      this->model->SetAngularVel(angular_velocity);
    }

    public: bool OnGripCommand(test_gripper::GripCommand::Request &_req,
                                test_gripper::GripCommand::Response &_res)
    {
      gzdbg << "Grip command requested: " << (_req.enable ? "Enable" : "Disable") << std::endl;
      grip_enabled = _req.enable;
      _res.success = true;
      return _res.success;
    }

    public: bool OnVelocityCommand(test_gripper::VelocityCommand::Request &_req,
                                    test_gripper::VelocityCommand::Response &_res)
    {
      gzdbg << "Velocity command requested," << std::endl <<
      "Linear: " << std::endl << _req.linear <<
      "Angular: " << std::endl << _req.angular << std::endl;

      linear_velocity = ignition::math::Vector3d(_req.linear.x ,_req.linear.y, _req.linear.z);
      angular_velocity = ignition::math::Vector3d(_req.angular.x, _req.angular.y, _req.angular.z);

      //Re-enable physics once object is grasped and gripper begins moving
      for(auto const& link : grasped_object->GetLinks())
        link->SetKinematic(false);

      _res.success = true;
      return _res.success;
    }

    public: bool OnAccelCommand(test_gripper::AccelCommand::Request &_req,
                                    test_gripper::AccelCommand::Response &_res)
    {
      gzdbg << "Accel command requested," << std::endl <<
      "Accel: " << std::endl << _req.accel <<
      "Duration: " << _req.duration << std::endl;

      linear_accel = ignition::math::Vector3d(_req.accel.linear.x ,_req.accel.linear.y, _req.accel.linear.z);
      angular_accel = ignition::math::Vector3d(_req.accel.angular.x, _req.accel.angular.y, _req.accel.angular.z);
      accel_duration = _req.duration;

      linear_velocity = ignition::math::Vector3d(0,0,0);
      angular_velocity = ignition::math::Vector3d(0,0,0);
      accel_command_active = true;

      prev_time = this->model->GetWorld()->SimTime();
      elapsed_time = common::Time(0);

      //Re-enable physics once object is grasped and gripper begins moving
      for(auto const& link : grasped_object->GetLinks())
        link->SetKinematic(false);

      _res.success = true;
      return _res.success;
    }

    private: void PublishGraspPose()
    {
      auto relative_grasp_pose = grasped_object->WorldPose() - this->model->WorldPose();
        
      geometry_msgs::PoseStamped msg;
      msg.header.stamp.sec = elapsed_time.sec;
      msg.header.stamp.nsec = elapsed_time.nsec;

      msg.pose.position.x = relative_grasp_pose.Pos().X();
      msg.pose.position.y = relative_grasp_pose.Pos().Y();
      msg.pose.position.z = relative_grasp_pose.Pos().Z();
        
      msg.pose.orientation.w = relative_grasp_pose.Rot().W();
      msg.pose.orientation.x = relative_grasp_pose.Rot().X();
      msg.pose.orientation.y = relative_grasp_pose.Rot().Y();
      msg.pose.orientation.z = relative_grasp_pose.Rot().Z();
      
      grasp_pose_pub.publish(msg);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(BenchmarkGripper)
}