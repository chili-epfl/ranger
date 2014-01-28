#include "webots/TouchSensor.hpp"
#include "webots/DifferentialWheels.hpp"

using namespace webots;

TouchSensor::TouchSensor(const std::string &name, const DifferentialWheels *differentialWheels, int type) :
  Device(name, differentialWheels)
{
  mValue = 0;
  mType = type;
}

TouchSensor::~TouchSensor() {
}

void TouchSensor::enable(int ms) {
}

void TouchSensor::disable() {
}

double TouchSensor::getValue() const {
  if(mType == BUMPER) {
    if(mValue == 0)
      return false;
    else
      return true;
  }
  else
    return mValue;
}

int TouchSensor::getType() const {
  return mType;
}

void TouchSensor::setValue(const double value) {
  mValue = value;
}
