#include "webots/Accelerometer.hpp"
#include "webots/DifferentialWheels.hpp"

using namespace webots;

Accelerometer::Accelerometer(const std::string &name, const DifferentialWheels *differentialWheels) :
  Device(name, differentialWheels)
{
  mValues[0] = 0;
  mValues[1] = 0;
  mValues[2] = 0;
}

Accelerometer::~Accelerometer() {
}

void Accelerometer::enable(int ms) {
}

void Accelerometer::disable() {
}

const double* Accelerometer::getValues() const {
  return mValues;
}

void Accelerometer::setValues(const double x, const double y, const double z) {
  mValues[0] = x;
  mValues[1] = y;
  mValues[2] = z;
}
