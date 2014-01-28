/*******************************************************************************************************/
/* File:         Accelerometer.hpp                                                                    */
/* Date:         jul 2013                                                                              */
/* Description:  Wrapper of the Accelerometer Webots API for the Ranger real robot                     */
/* Author:       david.mansolino@epfl.ch                                                               */
/*******************************************************************************************************/

#ifndef ACCELEROMETER_HPP
#define ACCELEROMETER_HPP

#include "DifferentialWheels.hpp"
#include "Device.hpp"

namespace webots {
  class Accelerometer: public Device  {
    public:
                    Accelerometer(const std::string &name, const DifferentialWheels *differentialWheels);
      virtual      ~Accelerometer();
      virtual void  enable(int ms);
      virtual void  disable();
      const double* getValues() const;

    private:
      double        mValues[3];
      void          setValues(const double x, const double y, const double z);
    
    friend int DifferentialWheels::step(int ms);
  };
}

#endif // ACCELEROMETER_HPP
