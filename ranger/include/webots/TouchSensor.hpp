/*******************************************************************************************************/
/* File:         TouchSensor.hpp                                                                       */
/* Date:         jul 2013                                                                              */
/* Description:  Wrapper of the TouchSensor Webots API for the Ranger real robot                       */
/* Author:       david.mansolino@epfl.ch                                                               */
/*******************************************************************************************************/

#ifndef TOUCHSENSOR_HPP
#define TOUCHSENSOR_HPP

#include "DifferentialWheels.hpp"
#include "Device.hpp"

namespace webots {
  class TouchSensor: public Device  {
    public:
      enum Type     {BUMPER, FORCE, FORCE3D};
      
                    TouchSensor(const std::string &name, const DifferentialWheels *differentialWheels, int type);
      virtual      ~TouchSensor();
      virtual void  enable(int ms);
      virtual void  disable();
      double        getValue() const;
      int           getType() const;

    private:
      double        mValue;
      int           mType;
      void          setValue(const double value);
    
    friend int DifferentialWheels::step(int ms);
  };
}

#endif // TOUCHSENSOR_HPP
