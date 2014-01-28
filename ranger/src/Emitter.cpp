#include "webots/Emitter.hpp"
#include "webots/DifferentialWheels.hpp"

#include "dashelinterface.h"

using namespace webots;

Emitter::Emitter(const std::string &name, const DifferentialWheels *differentialWheels, DashelInterface *dashelInterface) :
  Device(name, differentialWheels)
{
  mDashelInterface = dashelInterface;
}

Emitter::~Emitter() {
}

int Emitter::send(const void *data, int size) {
  bool needRefresh = false;
  
  for(int c=0; c<7; ++c) {
    if((c >= size) && (((int *)data)[c] != mValue[c])) {
      needRefresh = true;
      mValue[c] = ((int *)data)[c];
    }
  }
  
  if(needRefresh) {
    const int * args = (const int *)data;
    mDashelInterface->sendEvent("emitterEvent", args, 7);
  }
  
  return 1;
}

int Emitter::getChannel() const {
  return 1;
}

void Emitter::setChannel(int channel) {
}

double Emitter::getRange() const {
  return 6.0; // TODO : arbitrary
}

void Emitter::setRange(double range) {
}

int Emitter::getBufferSize() const {
  return 8;
}
