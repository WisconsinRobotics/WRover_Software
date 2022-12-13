/**
 * @file ArmControlActionServer.cpp
 * @author Ben Nowotny
 * @brief The exeutable file to run the Arm Control Action Server
 * @date 2022-12-05
 */
#include "XmlRpcValue.h"

#include "ros/ros.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <algorithm>
#include <csignal>
#include <string>
#include <array>
#include <memory>
#include <std_srvs/Trigger.h>
#include "SimpleJoint.hpp"
#include "DifferentialJoint.hpp"
#include "ros/console.h"

using XmlRpc::XmlRpcValue;

/**
 * @brief Refresh rate of ros::Rate
 */
constexpr float CLOCK_RATE = 50;

constexpr double IK_WARN_RATE = 1.0/2;

constexpr double JOINT_SAFETY_MAX_SPEED = 0.3;
constexpr double JOINT_SAFETY_HOLD_SPEED = 0.15;

/**
 * @brief Nessage cache size of publisher
 */
constexpr std::uint32_t MESSAGE_CACHE_SIZE = 1000; 

/**
 * @brief Period between timer callback
 */
constexpr float TIMER_CALLBACK_DURATION = 1.0 / 50.0;

/**
 * @brief Defines space for all Joint references
 */
constexpr int NUM_JOINTS = 5;
std::array<std::unique_ptr<AbstractJoint>, NUM_JOINTS> joints;
// AbstractJoint *joints[numJoints];
/**
 * @brief The Joint State Publisher for MoveIt
 */
ros::Publisher jointStatePublisher;
/**
 * @brief Simplify the SimpleActionServer reference name
 */
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;
/**
 * @brief The service server for enabling IK
 */
ros::ServiceServer enableServiceServer;
/**
 * @brief The status of IK program
 */
std::atomic_bool IKEnabled{true};
/**
 * @brief The service client for disabling IK
 */
ros::ServiceClient enableServiceClient;

/**
 * @brief Perform the given action as interpreted as moving the arm joints to specified positions
 * 
 * @param goal The goal state given
 * @param as The Action Server this is occuring on
 */
void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as) {
  if (!IKEnabled) {
    as->setAborted();
    ROS_WARN_THROTTLE(IK_WARN_RATE, "IK is disabled"); // NOLINT(hicpp-no-array-decay,hicpp,hicpp-vararg,cppcoreguidelines-pro-bounds-array-to-pointer-decay, cppcoreguidelines-pro-type-vararg)
    return;
  }

  int currPoint = 1;

  std::cout << "start exec: " << goal->trajectory.points.size() << std::endl;
  // For each point in the trajectory execution sequence...
  for(const auto& name : goal->trajectory.joint_names){
    std::cout << name << "\t";
  }
  std::cout << std::endl;
  for(const auto &currTargetPosition : goal->trajectory.points){
    for(double pos : currTargetPosition.positions){
      std::cout << std::round(pos*100)/100 << "\t";
    }  
    std::cout << std::endl;
  }
  for(const auto &currTargetPosition : goal->trajectory.points){
    // Track whether or not the current position is done
    bool hasPositionFinished = false;
    // Keep max loop rate at 50 Hz
    ros::Rate loop{CLOCK_RATE};

    const double VELOCITY_MAX = abs(*std::max_element(
        currTargetPosition.velocities.begin(),
        currTargetPosition.velocities.end(),
        [](double a, double b) -> bool{ return abs(a)<abs(b); }
      ));

    ArmMotor *currmotor = NULL; 
    int currItr = 0;
    // std::cout << currPoint << " / " << goal->trajectory.points.size() << std::endl;
    currPoint++;
    for(const auto &joint : joints){
      for(int i = 0; i < joint->getDegreesOfFreedom(); i++){
        double velocity = VELOCITY_MAX == 0.F ? JOINT_SAFETY_HOLD_SPEED : currTargetPosition.velocities[currItr]/VELOCITY_MAX;
        // std::cout << "config setpoint: " << currTargetPosition.positions[currItr] << ":" << velocity << std::endl;
        joint->configSetpoint(i, currTargetPosition.positions[currItr], velocity);
        currItr++;
      }
      joint->exectute();
    }

    // While the current position is not complete yet...
    while(!hasPositionFinished){
      // Assume the current action is done until proven otherwise
      hasPositionFinished = true;
      // Create the Joint State message for the current update cycle

      // For each joint specified in the currTargetPosition...
      for(const auto &joint : joints){

        for(int k = 0; k < joint->getDegreesOfFreedom(); k++){
          // if (joint->getMotor(k)->getMotorState() == MotorState::MOVING) {
          //   std::cout << "Moving" << std::endl;
          // } else if (joint->getMotor(k)->getMotorState() == MotorState::RUN_TO_TARGET) {
          //   std::cout << "Run to target" << std::endl;
          // } else if (joint->getMotor(k)->getMotorState() == MotorState::STALLING) {
          //   std::cout << "Stalling" << std::endl;
          // } else if (joint->getMotor(k)->getMotorState() == MotorState::STOP) {
          //   std::cout << "Stop" << std::endl;
          // } else {
          //   std::cout << "Error" << std::endl;
          // }

          if (joint->getMotor(k)->getMotorState() == MotorState::STALLING) {
            std::cout << "ACS stall detected" << std::endl;
            IKEnabled = false;
            std_srvs::Trigger srv;
            if (enableServiceClient.call(srv)) {
              ROS_WARN("%s", (std::string{"PLACEHOLDER_NAME: "} + srv.response.message).data()); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay, cppcoreguidelines-pro-type-vararg)
            } else {
              ROS_WARN("Error: failed to call service PLACEHOLDER_NAME"); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay, cppcoreguidelines-pro-type-vararg)
            }
          } else {
            hasPositionFinished &= joint->getMotor(k)->getMotorState() == MotorState::STOP;
          }      
          // DEBUGGING OUTPUT: Print each motor's name, radian position, encoder position, and power
          // std::cout<<std::setw(30)<<motors[j]->getEncoderCounts()<<std::endl;
        }

        joint->exectute();

        // if (!joint->exectute()) {
        //   IKEnabled = false;
        //   std_srvs::Trigger srv;
        //   if (enableServiceClient.call(srv)) {
        //     ROS_WARN("%s", (std::string{"PLACEHOLDER_NAME: "} + srv.response.message).data()); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay, cppcoreguidelines-pro-type-vararg)
        //   } else {
        //     ROS_WARN("Error: failed to call service PLACEHOLDER_NAME"); // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay, cppcoreguidelines-pro-type-vararg)
        //   }
        //   return;
        // }
      }
      // Sleep until the next update cycle
      loop.sleep();
    }
  }

  //When all positions have been reached, set the current task as succeeded
  
  as->setSucceeded();
}

/**
 * @brief publishes the arm's position
 */
void publishJointStates(const ros::TimerEvent &event){
  std::vector<std::string> names;
  std::vector<double> positions;
  sensor_msgs::JointState js_msg;

  for(const auto &joint : joints){
    for(int i = 0; i < joint->getDegreesOfFreedom(); i++){
      names.push_back(joint->getMotor(i)->getMotorName());
      positions.push_back(joint->getMotor(i)->getRads());
    }
  }

  js_msg.name = names;
  js_msg.position = positions;
  js_msg.header.stamp = ros::Time::now();
  // Publish the Joint State message
  jointStatePublisher.publish(js_msg);
}

/**
 * @brief The main executable method of the node.  Starts the ROS node and the Action Server for processing Arm Control commands
 * 
 * @param argc The number of program arguments
 * @param argv The given program arguments
 * @return int The status code on exiting the program
 */
auto main(int argc, char** argv) ->int
{
  std::cout << "start main" << std::endl;
  // Initialize the current node as ArmControlSystem
  ros::init(argc, argv, "ArmControlActionServer");
  // Create the NodeHandle to the current ROS node
  ros::NodeHandle n;

  // Initialize the Action Server
  Server server(n, "/arm_controller/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);
  // Start the Action Server
  server.start();
  std::cout << "server started" << std::endl;

  enableServiceServer = n.advertiseService("start_IK", 
    static_cast<boost::function<bool(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&)>>(
      [](std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)->bool{
        IKEnabled = true;
        res.message = "Arm IK Enabled";
        res.success = static_cast<unsigned char>(true);
        return true;
      }
    ));

  enableServiceClient = n.serviceClient<std_srvs::Trigger>("PLACEHOLDER_NAME");
  
  std::cout << "entering ROS spin..." << std::endl;
  // ROS spin for communication with other nodes
  ros::spin();
  // Return 0 on exit (successful exit)
  return 0;
}