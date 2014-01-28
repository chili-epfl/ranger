#include "webots/Motor.hpp"
#include "webots/DifferentialWheels.hpp"

#include "math.h"
#include <limits>       // std::numeric_limits

using namespace webots;

bool Motor::mChanged = true;

Motor::Motor(const std::string &name, const DifferentialWheels *differentialWheels, double minPos, double maxPos) :
  Device(name, differentialWheels)
{
  mTargetPosition = 0.0;
  mVelocity = M_PI; // max speed (rad/s)
  mMaxPos = maxPos;
  mMinPos = minPos;
}

Motor::~Motor() {
}

void Motor::setPosition(double position) {
  if(position != mTargetPosition) {
    mTargetPosition = position;
    if(mTargetPosition > mMaxPos)
      mTargetPosition = mMaxPos;
    if(mTargetPosition < mMinPos)
      mTargetPosition = mMinPos;
    mChanged = true;
  }
}

double Motor::getTargetPosition() const {
  return mTargetPosition;
}

void Motor::setVelocity(double vel) {
  double speed = vel;
  if(speed < 0 || speed > M_PI)
    speed = M_PI;
  
  if(vel != mVelocity) {
    mVelocity = vel;
    mChanged = true;
  }
}

void Motor::enablePosition(int ms) {
}

void Motor::disablePosition() {
}

double Motor::getPosition() {
  return mTargetPosition;
}

int Motor::getType() const {
  return 0;
}
      
bool Motor::hasChanged() {
  return mChanged;
}

void Motor::resetChanged() {
  mChanged = false;
}

double Motor::getMaxVelocity() {
  return mVelocity;
}

double Motor::getMinPosition() {
  return mMinPos;
}

double Motor::getMaxPosition() {
  return mMaxPos;
}
