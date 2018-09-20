/*
 Author : M. Efe Tiryaki
 */


#include "gopigo_gazebo/GopigoGazeboPlugin.hpp"
namespace gazebo {

// Todo : check if we can add gopigo name here
GopigoGazeboPlugin::GopigoGazeboPlugin()
    : nodeHandle_(),
      isEstimatorUsed(false),
      actuatorCommands_()
{
}

GopigoGazeboPlugin::~GopigoGazeboPlugin()
{
}

void GopigoGazeboPlugin::Init()
{
}
void GopigoGazeboPlugin::Reset()
{
}

void GopigoGazeboPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  ns_ = ros::this_node::getNamespace();
  robotName_  = sdf->GetElement("robot_name")->Get<std::string>();

  nodeHandle_ = new ros::NodeHandle("~");

  // Note : check if this is placed correctly
  this->readParameters(sdf);

  // Model
  this->model_ = model;

  // initialize joint structure
  initJointStructures();
  initLinkStructure();
  // initialize ROS pub/sub/services
  initSubscribers();

  // reset simulation variables
  Reset();

  // connect to world updates from Gazebo
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GopigoGazeboPlugin::OnUpdate, this));
}

void GopigoGazeboPlugin::OnUpdate()
{
  writeSimulation();
}

void GopigoGazeboPlugin::readParameters(sdf::ElementPtr sdf)
{
  // Get Frame parameters
  nodeHandle_->getParam(ns_ + "/frame/base/name", frameBase_);
  nodeHandle_->getParam(ns_ + "/frame/odometry/name", frameOdometry_);
  nodeHandle_->getParam(ns_ + "/frame/world/name", frameWorld_);

}

void GopigoGazeboPlugin::initJointStructures()
{
  jointPtrs_ = model_->GetJoints();

  std::cout << "[" << robotName_
            << "::GazeboPlugin::initJointStructures] "<< "Detected Joints are :" << std::endl;

  jointPositionsReset_ = std::vector<double>(jointPtrs_.size());

  // Init the joint structures.
  for (int i = 0; i < jointPtrs_.size(); i++) {
    const auto jointPtr = jointPtrs_[i];
    std::cout << "  - " << jointPtr->GetName() << std::endl;

    // Set joint position to default initial position
    jointPtr->SetPosition(0, 0);
    jointPositionsReset_[i] = 0;
  }
}

void GopigoGazeboPlugin::initLinkStructure()
{
  physics::Link_V links = model_->GetLinks();

  for (int i = 0; i < links.size(); i++) {
    if (links[i]->GetName().find("base") != std::string::npos) {
      baseLink_ = links[i];
      std::cout << "[" << robotName_
                << "::GazeboPlugin::initLinkStructure] "<< "Model contain base_link!" << std::endl;
      return;
    }
  }
}

void GopigoGazeboPlugin::initSubscribers()
{

  // Actuator Command Subscriber
  const std::string subscriberStr = "/"+ robotName_ + actuatorCommandSubscriberName_;
  std::cout << ns_ << " , " << robotName_ << std::endl;

actuatorCommandSubscriber_ = nodeHandle_->subscribe("/"+ns_+"/"+robotName_+"/cmd_vel",
                                                      actuatorCommandSubscriberQueueSize_,
                                                      &GopigoGazeboPlugin::actuatorCommandsCallback,
                                                      this);

  actuatorCommands_.velocity = std::vector<double>(2, 0.0);

}

void GopigoGazeboPlugin::actuatorCommandsCallback(const sensor_msgs::JointState& msg)
{
  actuatorCommands_ = msg;
}

void GopigoGazeboPlugin::writeSimulation()
{
  // TODO : get-rid of 2
  for (int i = 0 ;i<2 ;i++ ) {
    double jointVelocity = actuatorCommands_.velocity[i];
    jointPtrs_[i]->SetVelocity(0, jointVelocity);
  }
}


GZ_REGISTER_MODEL_PLUGIN(GopigoGazeboPlugin)
}
