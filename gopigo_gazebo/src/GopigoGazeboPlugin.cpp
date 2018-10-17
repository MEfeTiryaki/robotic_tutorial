/*
 Author : M. Efe Tiryaki
 */


#include "gopigo_gazebo/GopigoGazeboPlugin.hpp"
namespace gazebo {

// Todo : check if we can add gopigo name here
GopigoGazeboPlugin::GopigoGazeboPlugin()
    : nodeHandle_(),
      leftMotorCommands_(),
      rightMotorCommands_()
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
  initPublishers();

  // reset simulation variables
  Reset();

  // connect to world updates from Gazebo
  this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GopigoGazeboPlugin::OnUpdate, this));
}

void GopigoGazeboPlugin::OnUpdate()
{
  readSimulation();
  writeSimulation();
}

void GopigoGazeboPlugin::readParameters(sdf::ElementPtr sdf)
{
}

void GopigoGazeboPlugin::initJointStructures()
{
  jointPtrs_ = model_->GetJoints();

  //std::cout << "[" << robotName_
  //          << "::GazeboPlugin::initJointStructures] "<< "Detected Joints are :" << std::endl;

  jointPositionsReset_ = std::vector<double>(jointPtrs_.size());

  // Init the joint structures.
  for (int i = 0; i < jointPtrs_.size(); i++) {
    const auto jointPtr = jointPtrs_[i];
    //std::cout << "  - " << jointPtr->GetName() << std::endl;

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
      //std::cout << "[" << robotName_
      //          << "::GazeboPlugin::initLinkStructure] "<< "Model contain base_link!" << std::endl;
      return;
    }
  }
}

void GopigoGazeboPlugin::initSubscribers()
{

  leftMotorCommandSubscriber_ = nodeHandle_->subscribe("/"+ns_+"/"+robotName_+"/motor/pwm/left",
                                                        actuatorCommandSubscriberQueueSize_,
                                                        &GopigoGazeboPlugin::leftMotorCommandsCallback,
                                                        this);
  rightMotorCommandSubscriber_ = nodeHandle_->subscribe("/"+ns_+"/"+robotName_+"/motor/pwm/right",
                                                        actuatorCommandSubscriberQueueSize_,
                                                        &GopigoGazeboPlugin::rightMotorCommandsCallback,
                                                        this);
  leftMotorCommands_.data = 0;
  rightMotorCommands_.data = 0;
}

void GopigoGazeboPlugin::initPublishers()
{
  leftMotorAnglePublisher_ = nodeHandle_->advertise<std_msgs::Float64>(
                  "/"+ns_+"/"+robotName_+"/motor/encoder/left",10);
  rightMotorAnglePublisher_ = nodeHandle_->advertise<std_msgs::Float64>(
                  "/"+ns_+"/"+robotName_+"/motor/encoder/right",10);
}

void GopigoGazeboPlugin::leftMotorCommandsCallback(const std_msgs::Int8& msg)
{
  leftMotorCommands_ = msg;
}
void GopigoGazeboPlugin::rightMotorCommandsCallback(const std_msgs::Int8& msg)
{
  rightMotorCommands_ = msg;
}
void GopigoGazeboPlugin::writeSimulation()
{
  // TODO : get-rid of 2
  double scale = 0.05;
  double leftVelocity = scale * leftMotorCommands_.data;
  double rightVelocity = scale * rightMotorCommands_.data;
  jointPtrs_[0]->SetVelocity(0, leftVelocity);
  jointPtrs_[1]->SetVelocity(0, rightVelocity);
  std::cout << jointPtrs_[0]->GetAngle(0)<<std::endl;
}
void GopigoGazeboPlugin::readSimulation()
{
  double leftAngle = jointPtrs_[0]->GetAngle(0).Radian();
  double rightAngle = jointPtrs_[1]->GetAngle(0).Radian();
  while (leftAngle>M_PI){
    leftAngle -= 2*M_PI;
  }
  while (leftAngle<-M_PI){
    leftAngle += 2*M_PI;
  }
  while (rightAngle>M_PI){
    rightAngle -= 2*M_PI;
  }
  while (rightAngle<-M_PI){
    rightAngle += 2*M_PI;
  }
  leftMotorAnglePublisher_.publish(leftAngle);
  rightMotorAnglePublisher_.publish(rightAngle);
}

GZ_REGISTER_MODEL_PLUGIN(GopigoGazeboPlugin)
}
