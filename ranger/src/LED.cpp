#include "webots/LED.hpp"
#include "webots/DifferentialWheels.hpp"

using namespace webots;


LED::LED(const std::string &name, const DifferentialWheels *differentialWheels) :
  Device(name, differentialWheels)
{
  mColor = 0;
  mChanged = false;
}

LED::~LED() {
}

void LED::set(int value) {
  if(mColor!=value) {
    mColor = value;
    mChanged = true;
  }
}

int LED::get() {
  return mColor;
}

bool LED::hasChanged() {
  return mChanged;
}

void LED::resetChanged() {
  mChanged = false;
}
