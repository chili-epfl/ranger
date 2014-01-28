/*******************************************************************************************************/
/* File:         Receiver.hpp                                                                          */
/* Date:         jul 2013                                                                              */
/* Description:  Wrapper of the Receiver Webots API for the Ranger real robot                          */
/* Author:       david.mansolino@epfl.ch                                                               */
/*******************************************************************************************************/

#ifndef RECEIVER_HPP
#define RECEIVER_HPP

#include "DifferentialWheels.hpp"
#include "../Packet.hpp"
#include "Device.hpp"

#include <deque>

namespace webots {
  class Receiver: public Device  {
    public:
                    Receiver(const std::string &name, const DifferentialWheels *differentialWheels);
      virtual      ~Receiver();
      virtual void  enable(int ms);
      virtual void  disable();
      const double* getEmitterDirection() const;
      double        getSignalStrength() const;
      int           getEmitterID() const;
      int           getQueueLength() const;
      const void *  getData() const;
      virtual void  nextPacket();

    private:
      std::deque<Packet> mPacket;
      void          resetQueue();
      void          addPacket(int id, int distance, int angle, int * data);
    
    friend int DifferentialWheels::step(int ms);
  };
}

#endif // RECEIVER_HPP
