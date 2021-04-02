#include <iostream>
#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int16.h"
#include "math.h"
#include <string>

enum MotorState{
    STOP, MOVING, RUN_TO_TARGET
};
class ArmMotor{
    private:
        static unsigned long int const COUNTS_PER_ROTATION;
        static unsigned long int const ENCODER_BOUNDS[2];
        MotorState currState;
        std::string motorName;
        unsigned long int controllerID;
        unsigned long int motorID;
        unsigned long int encoderVal;
        ros::Subscriber encRead;
        ros::Publisher speedPub;
        std_msgs::Int16 *powerMsg;
        static unsigned long int radToEnc(float rad);
        void storeEncoderVals(const std_msgs::UInt32::ConstPtr& msg);
    public:
        ArmMotor();
        ArmMotor(std::string motorName, unsigned long int controllerID, unsigned long int motorID, ros::NodeHandle* n);
        // ~ArmMotor();
        unsigned long int getEncoderCounts();
        // void resetEncoder();
        void runToTarget(unsigned long int targetCounts, float power);
        void runToTarget(unsigned long int targetCounts, float power, bool block);
        void runToTarget(double rads, float power);
        MotorState getMotorState();
        void setPower(float power);
        float getRads();
        std::string getMotorName();
        bool hasReachedTarget(unsigned long int targetCounts);
        bool hasReachedTarget(unsigned long int targetCounts, unsigned long int tolerance);
        float getPower();
};