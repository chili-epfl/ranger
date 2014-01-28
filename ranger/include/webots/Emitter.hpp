/*******************************************************************************************************/
/* File:         Emitter.hpp                                                                           */
/* Date:         jul 2013                                                                              */
/* Description:  Wrapper of the Emitter Webots API for the Ranger real robot                           */
/* Author:       david.mansolino@epfl.ch                                                               */
/*******************************************************************************************************/

#ifndef EMITTER_HPP
#define EMITTER_HPP

#include "DifferentialWheels.hpp"
#include "Device.hpp"

class DashelInterface;

namespace webots {
  class Emitter: public Device  {
    public:
	  enum {CHANNEL_BROADCAST};
	  
                    Emitter(const std::string &name, const DifferentialWheels *differentialWheels, DashelInterface *dashelInterface);
      virtual      ~Emitter();
      virtual int   send(const void *data, int size);
      int           getChannel() const;
      virtual void  setChannel(int channel);
      double        getRange() const;
      virtual void  setRange(double range);
      int           getBufferSize() const;

    private:
	  int              mValue[8];
	  DashelInterface *mDashelInterface;

    
    friend int DifferentialWheels::step(int ms);
  };
}

#endif // EMITTER_HPP
