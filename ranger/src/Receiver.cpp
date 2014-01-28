#include "webots/Receiver.hpp"
#include "webots/DifferentialWheels.hpp"

using namespace webots;

Receiver::Receiver(const std::string &name, const DifferentialWheels *differentialWheels) :
  Device(name, differentialWheels)
{
}

Receiver::~Receiver() {
}

void Receiver::enable(int ms) {
}

void Receiver::disable() {
}

const double* Receiver::getEmitterDirection() const {
   return mPacket[mPacket.size()].getEmitterDirection();
}

double Receiver::getSignalStrength() const {
  return mPacket[mPacket.size()].getSignalStrength();
}

int Receiver::getQueueLength() const {
  return mPacket.size();
}

const void * Receiver::getData() const {
  return mPacket[mPacket.size()].getPacketData();
}

void  Receiver::nextPacket() {
  if(mPacket.size() > 0)
    mPacket.pop_back();
}

int Receiver::getEmitterID() const {
  return mPacket[mPacket.size()].getEmitterID();
}

void Receiver::resetQueue() {
	while(mPacket.size() > 0)
	  mPacket.pop_back();
}

void Receiver::addPacket(int id, int distance, int angle, int * data) {
  Packet newPacket(id, distance, angle, data);
   mPacket.push_front(newPacket);
}
