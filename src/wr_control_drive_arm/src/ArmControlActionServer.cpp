/**
 * @file ArmControlActionServer.cpp
 * @author Ben Nowotny
 * @brief The exeutable file to run the Arm Control Action Server
 * @date 2022-12-05
 */
#include "DifferentialJointToMotorSpeedConverter.hpp"
#include "DirectJointToMotorSpeedConverter.hpp"
#include "Joint.hpp"
#include "Motor.hpp"
#include "RoboclawChannel.hpp"
#include "SingleEncoderJointPositionMonitor.hpp"
#include "XmlRpcValue.h"
#include "ros/init.h"

#include <actionlib/server/simple_action_server.h>
#include <algorithm>
#include <array>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <csignal>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <unordered_map>

using XmlRpc::XmlRpcValue;

/**
 * @brief Refresh rate of ros::Rate
 */
constexpr float CLOCK_RATE{50};

constexpr double IK_WARN_RATE{1.0 / 2};

constexpr double JOINT_SAFETY_MAX_SPEED{0.5};
constexpr double JOINT_SAFETY_HOLD_SPEED{0.15};

/**
 * @brief Nessage cache size of publisher
 */
constexpr std::uint32_t MESSAGE_CACHE_SIZE{10};

/**
 * @brief Period between timer callback
 */
constexpr float TIMER_CALLBACK_DURATION{1.0 / 50.0};

/**
 * @brief Defines space for all Joint references
 */
std::unordered_map<std::string, std::unique_ptr<Joint>> namedJointMap;

/**
 * @brief Simplify the SimpleActionServer reference name
 */
using Server = actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>;
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
 * @brief Perform the given action as interpreted as moving the arm joints to
 * specified positions
 *
 * @param goal The goal state given
 * @param as The Action Server this is occuring on
 */
void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal,
             Server *server) {
    if (!IKEnabled) {
        server->setAborted();
        ROS_WARN_THROTTLE( // NOLINT(hicpp-no-array-decay,hicpp,hicpp-vararg,cppcoreguidelines-pro-bounds-array-to-pointer-decay, cppcoreguidelines-pro-type-vararg)
            IK_WARN_RATE,
            "IK is disabled");
        return;
    }

    std::cout << "start exec: " << goal->trajectory.points.size() << std::endl;
    // DEBUG: Display every point in the movement sequence
    for (const auto &name : goal->trajectory.joint_names) {
        std::cout << name << "\t";
    }
    std::cout << std::endl;
    for (const auto &currTargetPosition : goal->trajectory.points) {
        for (double pos : currTargetPosition.positions) {
            std::cout << std::round(pos * 100) / 100 << "\t";
        }
        std::cout << std::endl;
    }

    for (const auto &currTargetPosition : goal->trajectory.points) {

        const double VELOCITY_MAX = abs(*std::max_element(
            currTargetPosition.velocities.begin(),
            currTargetPosition.velocities.end(),
            [](double a, double b) -> bool { return abs(a) < abs(b); }));

        for (uint32_t i = 0; i < goal->trajectory.joint_names.size(); ++i) {
            auto jointVelocity{JOINT_SAFETY_HOLD_SPEED};
            if (VELOCITY_MAX != 0)
                jointVelocity = currTargetPosition.velocities.at(i) / VELOCITY_MAX * JOINT_SAFETY_MAX_SPEED;

            namedJointMap.at(goal->trajectory.joint_names.at(i))->setTarget(currTargetPosition.positions.at(i), jointVelocity);
        }
    }

    auto waypointComplete{false};
    ros::Rate updateRate{CLOCK_RATE};

    while (!waypointComplete && ros::ok()) {
        waypointComplete = true;
        for (const auto &[_, joint] : namedJointMap) {
            waypointComplete &= joint->hasReachedTarget();
        }
        updateRate.sleep();
    }
    // When all positions have been reached, set the current task as succeeded

    server->setSucceeded();
}

auto getEncoderConfigFromParams(const XmlRpcValue &params, const std::string &jointName) -> EncoderConfiguration {
    return {.countsPerRotation = static_cast<uint32_t>(static_cast<int32_t>(params[jointName]["counts_per_rotation"])),
            .offset = static_cast<uint32_t>(static_cast<int32_t>(params[jointName]["offset"]))};
}

/**
 * @brief The main executable method of the node.  Starts the ROS node and the
 * Action Server for processing Arm Control commands
 *
 * @param argc The number of program arguments
 * @param argv The given program arguments
 * @return int The status code on exiting the program
 */
auto main(int argc, char **argv) -> int {
    std::cout << "start main" << std::endl;
    // Initialize the current node as ArmControlSystem
    ros::init(argc, argv, "ArmControlActionServer");
    // Create the NodeHandle to the current ROS node
    ros::NodeHandle n;
    ros::NodeHandle pn{"~"};

    XmlRpcValue encParams;
    pn.getParam("encoder_parameters", encParams);

    // Initialize all motors with their MoveIt name, WRoboclaw initialization,
    // and reference to the current node
    std::cout << "init motors" << std::endl;

    using std::literals::string_literals::operator""s;

    const auto turntableMotor{std::make_shared<Motor>("aux0"s, RoboclawChannel::A, n)};
    const auto shoulderMotor{std::make_shared<Motor>("aux0"s, RoboclawChannel::B, n)};
    const auto elbowMotor{std::make_shared<Motor>("aux1"s, RoboclawChannel::A, n)};
    const auto forearmRollMotor{std::make_shared<Motor>("aux1"s, RoboclawChannel::B, n)};
    const auto wristLeftMotor{std::make_shared<Motor>("aux2"s, RoboclawChannel::A, n)};
    const auto wristRightMotor{std::make_shared<Motor>("aux2"s, RoboclawChannel::B, n)};

    SingleEncoderJointPositionMonitor turntablePositionMonitor{
        "aux0"s,
        RoboclawChannel::A,
        getEncoderConfigFromParams(encParams, "turntable"),
        n};
    SingleEncoderJointPositionMonitor shoulderPositionMonitor{
        "aux0"s,
        RoboclawChannel::B,
        getEncoderConfigFromParams(encParams, "shoulder"),
        n};
    SingleEncoderJointPositionMonitor elbowPositionMonitor{
        "aux1"s,
        RoboclawChannel::A,
        getEncoderConfigFromParams(encParams, "elbow"),
        n};
    SingleEncoderJointPositionMonitor forearmRollPositionMonitor{
        "aux1"s,
        RoboclawChannel::B,
        getEncoderConfigFromParams(encParams, "forearmRoll"),
        n};
    SingleEncoderJointPositionMonitor wristPitchPositionMonitor{
        "aux2"s,
        RoboclawChannel::A,
        getEncoderConfigFromParams(encParams, "wristPitch"),
        n};
    SingleEncoderJointPositionMonitor wristRollPositionMonitor{
        "aux2"s,
        RoboclawChannel::B,
        getEncoderConfigFromParams(encParams, "wristRoll"),
        n};

    DirectJointToMotorSpeedConverter turntableSpeedConverter{turntableMotor};
    DirectJointToMotorSpeedConverter shoulderSpeedConverter{shoulderMotor};
    DirectJointToMotorSpeedConverter elbowSpeedConverter{elbowMotor};
    DirectJointToMotorSpeedConverter forearmRollSpeedConverter{forearmRollMotor};
    DifferentialJointToMotorSpeedConverter differentialSpeedConverter{wristLeftMotor, wristRightMotor};

    // Initialize all Joints

    std::cout << "init joints" << std::endl;

    namedJointMap.insert({"turntable_joint", std::make_unique<Joint>(
                                                 "turntable"s,
                                                 turntablePositionMonitor,
                                                 turntableSpeedConverter,
                                                 n)});
    namedJointMap.insert({"shoulder_joint", std::make_unique<Joint>(
                                                "shoulder",
                                                shoulderPositionMonitor,
                                                shoulderSpeedConverter,
                                                n)});
    namedJointMap.insert({"elbowPitch_joint", std::make_unique<Joint>(
                                                  "elbow",
                                                  elbowPositionMonitor,
                                                  elbowSpeedConverter,
                                                  n)});
    namedJointMap.insert({"elbowRoll_joint", std::make_unique<Joint>(
                                                 "forearmRoll",
                                                 forearmRollPositionMonitor,
                                                 forearmRollSpeedConverter,
                                                 n)});
    namedJointMap.insert({"wristPitch_joint", std::make_unique<Joint>(
                                                  "wristPitch",
                                                  wristPitchPositionMonitor,
                                                  [&converter = differentialSpeedConverter](double speed) { converter.setPitchSpeed(speed); },
                                                  n)});
    namedJointMap.insert({"wristRoll_link", std::make_unique<Joint>(
                                                "wristRoll",
                                                wristRollPositionMonitor,
                                                [&converter = differentialSpeedConverter](double speed) { converter.setRollSpeed(speed); },
                                                n)});

    // Initialize the Action Server
    Server server(
        n, "/arm_controller/follow_joint_trajectory",
        [&server](auto goal) { execute(goal, &server); }, false);
    // Start the Action Server
    server.start();
    std::cout << "server started" << std::endl;

    enableServiceServer = n.advertiseService(
        "start_IK",
        static_cast<boost::function<bool(std_srvs::Trigger::Request &,
                                         std_srvs::Trigger::Response &)>>(
            [](std_srvs::Trigger::Request &req,
               std_srvs::Trigger::Response &res) -> bool {
                IKEnabled = true;
                res.message = "Arm IK Enabled";
                res.success = static_cast<unsigned char>(true);
                return true;
            }));

    enableServiceClient =
        n.serviceClient<std_srvs::Trigger>("PLACEHOLDER_NAME");

    std::cout << "entering ROS spin..." << std::endl;
    // ROS spin for communication with other nodes
    ros::spin();
    // Return 0 on exit (successful exit)
    return 0;
}
