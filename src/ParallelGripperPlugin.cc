#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "test_gripper/GripCommand.h"

namespace gazebo
{
  class ParallelGripperPlugin : public ModelPlugin
  {
    private: physics::JointPtr joint1;
    private: physics::JointPtr joint2;
    
    private: bool grip_enabled;
    private: double grip_force_close;
    private: double grip_force_open;
    private: double grip_kp;

    private: ros::ServiceServer grip_service;
    private: ros::Publisher state_pub;
    private: std::unique_ptr<ros::NodeHandle> rosnode;

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      //Get SDF Params

      grip_force_close = 50;
      if (_sdf->HasElement("grip_force_close"))
        grip_force_close = _sdf->Get<double>("grip_force_close");
      
      grip_force_open = -0.5 * grip_force_close;
      if (_sdf->HasElement("grip_force_open"))
        grip_force_open = _sdf->Get<double>("grip_force_open");

      grip_kp = 1000;
      if (_sdf->HasElement("grip_kp"))
        grip_kp = _sdf->Get<double>("grip_kp");

      std::string joint1_name = "joint1";
      if (_sdf->HasElement("joint1"))
        joint1_name = _sdf->Get<std::string>("joint1");
      
      std::string joint2_name = "joint2";
      if (_sdf->HasElement("joint2"))
        joint2_name = _sdf->Get<std::string>("joint2");

      std::string robot_namespace = "";
      if (_sdf->HasElement("robot_namespace"))
        robot_namespace = _sdf->Get<std::string>("robot_namespace");

      std::string control_topic = "gripper/control";
      if (_sdf->HasElement("control_topic"))
        control_topic = _sdf->Get<std::string>("control_topic"); 

      std::string state_topic = "gripper/state";
      if (_sdf->HasElement("state_topic"))
        state_topic = _sdf->Get<std::string>("state_topic");

      // Store the pointer to the model
      this->model = _parent;

      //get pointers to each gripper joint
      joint1 = this->model->GetJoint(joint1_name);
      joint2 = this->model->GetJoint(joint2_name);
      if (!joint1 || !joint2) {
        ROS_FATAL_STREAM("Gripper joints not found, check specified joint names.");
      }

      grip_enabled = false;
      
      rosnode.reset(new ros::NodeHandle(robot_namespace));
      grip_service = rosnode->advertiseService(control_topic, &ParallelGripperPlugin::OnGripCommand, this);
      state_pub = rosnode->advertise<std_msgs::Bool>(state_topic, 1000);
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ParallelGripperPlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      std_msgs::Bool msg;
      msg.data = grip_enabled;
      state_pub.publish(msg);

      if (grip_enabled) {

        //Additional force modifier to ensure symmetrical finger positions
        double force_modifier = (joint1->Position(0) - joint2->Position(0)) * grip_kp;

        joint1->SetForce(0, grip_force_close - force_modifier);
        joint2->SetForce(0, grip_force_close + force_modifier);

      } else {
        joint1->SetForce(0, grip_force_open);
        joint2->SetForce(0, grip_force_open);
      }
    }

    public: bool OnGripCommand(test_gripper::GripCommand::Request &_req,
                                test_gripper::GripCommand::Response &_res)
    {
      gzdbg << "Grip command requested: " << (_req.enable ? "Enable" : "Disable") << std::endl;
      grip_enabled = _req.enable;
      _res.success = true;
      return _res.success;
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ParallelGripperPlugin)
}