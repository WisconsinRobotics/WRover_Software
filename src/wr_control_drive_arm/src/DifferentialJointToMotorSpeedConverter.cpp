#include "DifferentialJointToMotorSpeedConverter.hpp"
#include <mutex>

DifferentialJointToMotorSpeedConverter::DifferentialJointToMotorSpeedConverter(
    std::shared_ptr<Motor> leftMotor,
    std::shared_ptr<Motor> rightMotor)
    : cachedPitchSpeed{0},
      cachedRollSpeed{0},
      leftMotor{std::move(leftMotor)},
      rightMotor{std::move(rightMotor)} {}

void DifferentialJointToMotorSpeedConverter::setPitchSpeed(double speed) {
    cachedPitchSpeed = speed;
    dispatchDifferentialSpeed();
}

void DifferentialJointToMotorSpeedConverter::setRollSpeed(double speed) {
    cachedRollSpeed = speed;
    dispatchDifferentialSpeed();
}

void DifferentialJointToMotorSpeedConverter::dispatchDifferentialSpeed() {
    const std::lock_guard guard{mutex};
    auto m1Speed{cachedPitchSpeed - (cachedRollSpeed * AVERAGE_SCALING_FACTOR)};
    auto m2Speed{cachedPitchSpeed + (cachedRollSpeed * AVERAGE_SCALING_FACTOR)};
    leftMotor->setSpeed(m1Speed);
    rightMotor->setSpeed(m2Speed);
}