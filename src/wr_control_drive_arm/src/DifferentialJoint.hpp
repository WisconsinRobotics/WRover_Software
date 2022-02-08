#include "AbstractJoint.hpp"

class DifferentialJoint : public AbstractJoint {
    public:
        ~DifferentialJoint();
        DifferentialJoint(ArmMotor *leftMotor, ArmMotor *rightMotor, ros::NodeHandle *n);

        void getMotorPositions(vector<double> &jointPositions, vector<double> &target);
        void getMotorVelocities(vector<double> &jointVelocities, vector<double> &target);
        void getJointPositions(vector<double> &motorPositions, vector<double> &target);

        void configVelocityHandshake(std::string pitchTopicName, std::string rollTopicName, std::string leftTopicName, std::string rightTopicName);

    private:
        // linear transformations works for position and velocity
        // [0.5 0.5]   [left motor ]    [pitch]
        // [ -1  1 ] * [right motor]  = [roll ]
        // double motorToJointMatrix[2][2] = {{0.5, 0.5}, {-1, 1}};
        double motorToJointMatrix[2][2] = {{1, 0}, {0, 1}};
        // [1 -0.5]   [pitch]    [left motor ]
        // [1  0.5] * [roll ]  = [right motor]
        // double jointToMotorMatrix[2][2] = {{1, 0.5}, {1, -0.5}};
        double jointToMotorMatrix[2][2] = {{1, 0}, {0, 1.0}};

        void handoffPitchOutput(std_msgs::Float64);
        void handoffRollOutput(std_msgs::Float64);
        void handOffAllOutput();
        ros::Subscriber pitchOutputSubscriber;
        ros::Subscriber rollOutputSubscriber;
        ros::Publisher leftOutputPublisher;
        ros::Publisher rightOutputPublisher;
        float cachedPitchOutput = 0.0;
        float cachedRollOutput = 0.0;
        bool hasNewPitchOutput = false;
        bool hasNewRollOutput = false;

        void handoffLeftFeedback(std_msgs::Float64);
        void handoffRightFeedback(std_msgs::Float64);
        void handOffAllFeedback();
        ros::Subscriber leftFeedbackSubscriber;
        ros::Subscriber rightFeedbackSubscriber;
        ros::Publisher pitchFeedbackPublisher;
        ros::Publisher rollFeedbackPublisher;
        float cachedLeftFeedback = 0.0;
        float cachedRightFeedback = 0.0;
        bool hasNewLeftFeedback = false;
        bool hasNewRightFeedback = false;
};