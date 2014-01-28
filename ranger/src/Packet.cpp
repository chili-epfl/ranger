#include "Packet.hpp"
#include <math.h>

using namespace webots;

Packet::Packet(int id, int distance, int angle, int * data)
{
  mEmitterDirection[0] = sin(M_PI * ((float)angle / 1800));
  mEmitterDirection[1] = 0;
  mEmitterDirection[2] = cos(M_PI * ((float)angle / 1800));
  mSignalStrength = 1.0 / (((float)distance/1000) * ((float)distance/1000));
  mEmitterID = id;
  for(int c=0; c<7; ++c)
    mData[c] = data[c];
}

Packet::~Packet() {
}

const double* Packet::getEmitterDirection() const {
  return mEmitterDirection;
}

double Packet::getSignalStrength() const {
  return mSignalStrength;
}

int Packet::getEmitterID() const {
  return mEmitterID;
}

const void * Packet::getPacketData() const {
  return mData;
}

void Packet::resetPacket() {
  mEmitterDirection[0] = 0;
  mEmitterDirection[1] = 0;
  mEmitterDirection[2] = 0;
  mSignalStrength = 0;
  mEmitterID = -1;
  for(int c=0; c<7; ++c)
    mData[c] = 0;
}
