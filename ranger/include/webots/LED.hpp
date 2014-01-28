/*******************************************************************************************************/
/* File:         LED.hpp                                                                               */
/* Date:         jul 2013                                                                              */
/* Description:  Wrapper of the LED Webots API for the Ranger real robot                               */
/* Author:       david.mansolino@epfl.ch                                                               */
/*******************************************************************************************************/

#ifndef LED_HPP
#define LED_HPP

#include "DifferentialWheels.hpp"
#include "Device.hpp"

namespace webots {
  class LED: public Device  {
    public:
                   LED(const std::string &name, const DifferentialWheels *differentialWheels); //Use Robot::getLED() instead
      virtual     ~LED();
      virtual void set(int value);
      int          get(void);

    private:
    
    bool         hasChanged();
    void         resetChanged();
      
    friend int DifferentialWheels::step(int ms);
    
    int  mColor;
    bool mChanged;
  };
}

#endif // LED_HPP
