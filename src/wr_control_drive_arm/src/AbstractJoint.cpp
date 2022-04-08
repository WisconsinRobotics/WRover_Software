/**
 * @file AbstractJoint.cpp
 * @author Nichols Underwood
 * @brief ablskjlfkejfs
 * @date 2021-10-25
 */
#include "AbstractJoint.hpp"

AbstractJoint::AbstractJoint(ros::NodeHandle* n, int numMotors){
    this->n = n;
    this->numMotors = numMotors;

    jointPositions.reserve(numMotors);
    jointVelocites.reserve(numMotors);

}

int AbstractJoint::getDegreesOfFreedom(){
    return this->numMotors;
}

ArmMotor* AbstractJoint::getMotor(int motorIndex){
    return this->motors[motorIndex];
}

void AbstractJoint::configSetpoint(int degreeIndex, double position, double velocity){

    this->jointPositions[degreeIndex] = position;
    this->jointVelocites[degreeIndex] = velocity;
}

void AbstractJoint::exectute(){
    vector<double> motorPositions = vector<double>();
    this->getMotorPositions(jointPositions, motorPositions);

    for(int i = 0; i < this->numMotors; i++){
        this->motors[i]->runToTarget(motorPositions[i], 0);
    }
}