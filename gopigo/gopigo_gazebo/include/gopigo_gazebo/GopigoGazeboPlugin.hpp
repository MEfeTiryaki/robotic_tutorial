/*
 Author : M. Efe Tiryaki
 */

#pragma once

// c++
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <atomic>
#include <condition_variable>
#include <vector>
#include <math.h>
// required for std::this_thread
#include <thread>

// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/Element.hh>
// ros
#include <ros/ros.h>
#include <ros/callback_queue.h>
// geometry msgs
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
// std msgs
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>

// sensor msgs
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

#include "gopigo_msgs/GopigoState.h"

// urdf
#include <urdf/model.h>

namespace gazebo {
class GopigoGazeboPlugin : public ModelPlugin
{
 public:
  // Constructor.
  GopigoGazeboPlugin();

  // Destructor.
  virtual ~GopigoGazeboPlugin();

  // Implements Gazebo virtual load function.
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr /*_sdf*/);

  // Overrides Gazebo init function.
  virtual void Init();

  // Overrides Gazebo reset function.
  virtual void Reset();

  virtual void OnUpdate();

 protected:
  // Reads parameters from the parameter server.
  virtual void readParameters(sdf::ElementPtr sdf);

  virtual void initJointStructures() ;

  virtual void initLinkStructure() ;

  // Inits the ROS subscriber.
  virtual void initSubscribers() ;
  // Inits the ROS subscriber.
  virtual void initPublishers() ;

  // Read simulation state.
  virtual void readSimulation();

  // Writes simulation state.
  virtual void writeSimulation();

  // Set Commands to be writen in writeSimulation
  void leftMotorCommandsCallback(const std_msgs::Int8& msg);
  void rightMotorCommandsCallback(const std_msgs::Int8& msg);


  // Ros node
  std::string ns_;
  ros::NodeHandle* nodeHandle_;

  // Ensures gazebo methods are called sequentially
  std::recursive_mutex gazeboMutex_;

  // Name of the robot.
  std::string robotName_;
  // ROS robot description parameter name.
  std::string robotDescriptionParamName_;
  // Robot base link.
  std::string robotBaseLink_;
  // ROS robot description URDF string.
  std::string robotDescriptionUrdfString_;
  // ROS robot description URDF model.
  urdf::Model robotDescriptionUrdfModel_;


  // Pulishers
  ros::Publisher leftMotorAnglePublisher_;
  ros::Publisher rightMotorAnglePublisher_;
  // Publisher names
  std::string leftMotorAnglePublisherName_;
  std::string rightMotorAnglePublisherName_;
  // Publisher queue_size
  int leftMotorAnglePublisherQueueSize_;
  int rightMotorAnglePublisherQueueSize_;
  // Publisher msgs
  std_msgs::Float64 leftMotorAngle_ ;
  std_msgs::Float64 rightMotorAngle_ ;

  // Subscriber
  ros::Subscriber leftMotorCommandSubscriber_;
  ros::Subscriber rightMotorCommandSubscriber_;
  // Subscriber names
  std::string actuatorCommandSubscriberName_;
  // Subscriber queue_size
  int actuatorCommandSubscriberQueueSize_;
  // Subscriber msgs
  std_msgs::Int8 leftMotorCommands_;
  std_msgs::Int8 rightMotorCommands_;

  // Model.
  physics::ModelPtr model_;
  // World update event.
  event::ConnectionPtr updateConnection_;

  // Robot links
  physics::LinkPtr baseLink_;

  // Actuators
  std::vector<std::string> jointNames_;
  std::unordered_map<std::string, int> jointNametoJointId_;

  std::vector<gazebo::physics::JointPtr> jointPtrs_;
  std::vector<int> jointTypes_;
  std::vector<double> jointPositionsReset_;
  std::vector<double> jointPositionsDefault_;


  // Gazebo time step.
  double gazeboTimeStep_ = 0.0;
  // Time step for publishing simulation state.
  double publishingTimeStep_ = 0.0;
  // Simulation time stamp taken at the start of the last updateCb() function call.
  common::Time lastStartUpdateSimTime_;
  // System time stamp taken at the start of the last updateCb() function call.
  std::chrono::time_point<std::chrono::steady_clock> lastStartUpdateSysTime_;
  // Current inter-update simulation time step.
  double updateSimTimeStep_ = 0.0;

};

}
