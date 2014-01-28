/*******************************************************************************************************/
/* File:         Motor.hpp                                                                             */
/* Date:         jul 2013                                                                              */
/* Description:  Wrapper of the Motor Webots API for the Ranger real robot                             */
/* Author:       david.mansolino@epfl.ch                                                               */
/*******************************************************************************************************/

#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "DifferentialWheels.hpp"
#include "Device.hpp"

namespace webots {
  class Motor: public Device  {
    public:
                    Motor(const std::string &name, const DifferentialWheels *differentialWheels, double minPos, double maxPos);
      virtual      ~Motor();
      
      void          setPosition(double position);
      double        getTargetPosition() const;
      void          setVelocity(double vel);
      void          enablePosition(int ms);
      void          disablePosition();
      double        getPosition();
      int           getType() const;
      double        getMinPosition();
      double        getMaxPosition();

    private:
    
      double       mTargetPosition;
      double       mVelocity;
      double       mMaxPos;
      double       mMinPos;
      
      double       getMaxVelocity();
      
      static bool  mChanged;
      
      static bool  hasChanged();
      static void  resetChanged();
      
    
    friend int DifferentialWheels::step(int ms);
  };
}

#endif // MOTOR_HPP
