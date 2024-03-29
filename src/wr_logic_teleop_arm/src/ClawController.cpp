#include "ClawController.hpp"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"





ClawController::ClawController(ros::NodeHandle& n) : 
    pub{n.advertise<std_msgs::Int16>("/hsi/roboclaw/aux3/cmd/left", 
        MESSAGE_QUEUE_LENGTH)}, 
    openASub{n.subscribe("/hci/arm/gamepad/button/a", 
        MESSAGE_QUEUE_LENGTH, &ClawController::openClaw, this)},
    closeBSub{n.subscribe("/hci/arm/gamepad/button/b", 
        MESSAGE_QUEUE_LENGTH, &ClawController::closeClaw, this)},
    aPressed{false}, 
    bPressed{false} {}

void ClawController::openClaw(const std_msgs::Bool::ConstPtr& msg)
{
    // This should open the claw

    this->aPressed = (msg->data != 0U);
    checkMessage();
}

void ClawController::closeClaw(const std_msgs::Bool::ConstPtr& msg)
{
    // This should close the claw

    this->bPressed = (msg->data != 0U);
    checkMessage();
}

void ClawController::checkMessage()
{
    if ((aPressed || !bPressed) && (!aPressed || bPressed))
    {
        std_msgs::Int16 msgNA;
        msgNA.data = 0;
        pub.publish(msgNA);
    }
    else if (aPressed)
    {
        std_msgs::Int16 msgA;
        msgA.data = openSpeed;
        pub.publish(msgA);
    }
    else
    {
        std_msgs::Int16 msgB;
        msgB.data = closeSpeed;
        pub.publish(msgB);
    }
    
}