/*******************************************************************************************************/
/* File:         DistanceSensor.hpp                                                                    */
/* Date:         jul 2013                                                                              */
/* Description:  Wrapper of the DistanceSensor Webots API for the Ranger real robot                    */
/* Author:       david.mansolino@epfl.ch                                                               */
/*******************************************************************************************************/

#ifndef DISTANCESENSOR_HPP
#define DISTANCESENSOR_HPP

#include "DifferentialWheels.hpp"
#include "Device.hpp"

namespace webots {
  class DistanceSensor: public Device  {
    public:
                    DistanceSensor(const std::string &name, const DifferentialWheels *differentialWheels);
      virtual      ~DistanceSensor();
      virtual void  enable(int ms);
      virtual void  disable();
      const double  getValue() const;

    private:
      double        mValue;
      void          setValue(const double value);
    
    friend int DifferentialWheels::step(int ms);
  };
}

#endif // DISTANCESENSOR_HPP
