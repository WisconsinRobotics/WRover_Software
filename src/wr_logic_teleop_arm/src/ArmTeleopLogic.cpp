#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "boost/function.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <atomic>
#include <cstdint>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

using Std_Bool = const std_msgs::BoolConstPtr&;
using Std_Float32 = const std_msgs::Float32ConstPtr&;

std::atomic_bool actionClean {true};
const tf2::Quaternion WORLD_OFFSET {0, sin(M_PI/4), 0, cos(M_PI/4)};

auto updateTarget(float x_pos, float y_pos, float z_pos, tf2::Quaternion orientation, ros::Publisher &pub) -> void{
    geometry_msgs::PoseStamped p {};
    p.pose.position.x = x_pos;
    p.pose.position.y = y_pos;
    p.pose.position.z = z_pos;
    auto outOrientation = /* tf2::Quaternion(0,sin(M_PI/4), 0, cos(M_PI/4)) * */ orientation;
    p.pose.orientation = tf2::toMsg(outOrientation);
    p.header.frame_id = "turntable";
    pub.publish(p);
    actionClean.store(false);
}

auto main(int argc, char** argv) -> int{
    ros::init(argc, argv, "ArmTeleopLogic");
    ros::AsyncSpinner spin(1);
    spin.start();
    ros::NodeHandle np("~"); 

    constexpr float PLANNING_TIME = 0.05;
    constexpr float CLOCK_RATE = 2;
    constexpr uint32_t MESSAGE_QUEUE_LENGTH = 1000; 
    constexpr float TRIGGER_PRESSED = 0.5;

    constexpr float STEP_X = 0.001;
    constexpr float STEP_Y = 0.001;
    constexpr float STEP_Z = 0.001;

    constexpr float HOME_X = 0.25;
    constexpr float HOME_Y = 0;
    constexpr float HOME_Z = 0.25;

    float x_pos = HOME_X;
    float y_pos = HOME_Y;
    float z_pos = HOME_Z;

    tf2::Quaternion orientation {0, sin(-M_PI/4), 0, cos(-M_PI/4)};
    orientation = WORLD_OFFSET * orientation;

    const tf2::Quaternion SPIN_X {sin(2*M_PI/1000), 0, 0, cos(2*M_PI/1000)};
    const tf2::Quaternion SPIN_Y {0, sin(2*M_PI/1000), 0, cos(2*M_PI/1000)};
    const tf2::Quaternion SPIN_Z {0, 0, sin(2*M_PI/1000), cos(2*M_PI/1000)};

    moveit::planning_interface::MoveGroupInterface move("arm");
    // move.setPlannerId("RRTStar");
    move.setPlanningTime(PLANNING_TIME);
    ros::Rate loop {CLOCK_RATE};

    ros::Publisher nextTarget = np.advertise<geometry_msgs::PoseStamped>("/logic/arm_teleop/next_target", 
        MESSAGE_QUEUE_LENGTH);
        

    ros::Subscriber yAxis = np.subscribe("/xbox_test/axis/pov_y", 
        MESSAGE_QUEUE_LENGTH, 
        static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {
            if(msg->data != 0){
                y_pos += static_cast<bool>(msg->data) ? msg->data > 0 ? STEP_Y : -STEP_Y : 0;
                updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
            }
        }));
    ros::Subscriber xAxis = np.subscribe("/xbox_test/axis/pov_x", 
        MESSAGE_QUEUE_LENGTH, 
        static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {
            if(msg->data != 0){
                x_pos += static_cast<bool>(msg->data) ? msg->data > 0 ? STEP_X : -STEP_X : 0;
                updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
            }
        }));
    ros::Subscriber zUp = np.subscribe("/xbox_test/button/shoulder_l",
        MESSAGE_QUEUE_LENGTH,
        static_cast<boost::function<void(Std_Bool)>>([&](Std_Bool msg) -> void {
            if(msg->data){
                z_pos += msg->data ? STEP_Z : 0;
                updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
            }
        }));
    ros::Subscriber zDown = np.subscribe("/xbox_test/axis/trigger_left",
        MESSAGE_QUEUE_LENGTH,
        static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {
            if(msg->data >= TRIGGER_PRESSED){
                z_pos += msg->data >= TRIGGER_PRESSED ? -STEP_Z : 0;
                updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
            }
        }));
    
    ros::Subscriber roll = np.subscribe("/xbox_test/axis/stick_left_x",
        MESSAGE_QUEUE_LENGTH,
        static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {
            if(abs(msg->data) >= 0.5){
                orientation *= WORLD_OFFSET.inverse() * (msg->data > 0 ? SPIN_X : SPIN_X.inverse()) * WORLD_OFFSET;
                updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
            }
        }));

    ros::Subscriber pitch = np.subscribe("/xbox_test/axis/stick_left_y",
        MESSAGE_QUEUE_LENGTH,
        static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {
            if(abs(msg->data) >= 0.5){
                orientation *= WORLD_OFFSET.inverse() * (msg->data > 0 ? SPIN_Y : SPIN_Y.inverse()) * WORLD_OFFSET;
                updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
            }
        }));

    ros::Subscriber yaw = np.subscribe("/xbox_test/axis/stick_right_x",
        MESSAGE_QUEUE_LENGTH,
        static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {
            if(abs(msg->data) >= 0.5){
                orientation *= WORLD_OFFSET.inverse() * (msg->data > 0 ? SPIN_Z : SPIN_Z.inverse()) * WORLD_OFFSET;
                updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
            }
        }));

    // transform = move.getCurrentState()->getFrameTransform("odom_combined");
    // updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);

    while(ros::ok()){
        geometry_msgs::PoseStamped p {};
        p.pose.position.x = x_pos;
        p.pose.position.y = y_pos;
        p.pose.position.z = z_pos;
        p.pose.orientation = tf2::toMsg(WORLD_OFFSET.inverse() * orientation);
        p.header.frame_id = "odom_combined";
        move.setPoseTarget(p);
        std::vector<geometry_msgs::Pose> waypoints {p.pose};
        moveit_msgs::RobotTrajectory traj;
        double discard = move.computeCartesianPath(waypoints, 0.01, 0.0, traj);
        move.asyncExecute(traj);
        while(!move.getMoveGroupClient().getState().isDone() && actionClean.load()) loop.sleep();
        if(!actionClean.load()) { 
            move.stop();
            actionClean.store(true);
        }
    }

    return 0;
}