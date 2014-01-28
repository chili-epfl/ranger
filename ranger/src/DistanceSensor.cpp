#include "webots/DistanceSensor.hpp"
#include "webots/DifferentialWheels.hpp"

using namespace webots;

DistanceSensor::DistanceSensor(const std::string &name, const DifferentialWheels *differentialWheels) :
  Device(name, differentialWheels)
{
  mValue = 0;
}

DistanceSensor::~DistanceSensor() {
}

void DistanceSensor::enable(int ms) {
}

void DistanceSensor::disable() {
}

const double DistanceSensor::getValue() const {
  return mValue;
}

void DistanceSensor::setValue(const double value) {
  mValue = value;
}
