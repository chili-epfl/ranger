/*******************************************************************************************************/
/* File:         Device.hpp                                                                            */
/* Date:         jul 2013                                                                              */
/* Description:  Wrapper of the Device Webots API for the Ranger real robot                            */
/* Author:       david.mansolino@epfl.ch                                                               */
/*******************************************************************************************************/

#ifndef DEVICE_HPP
#define DEVICE_HPP

#define WB_USING_CPP_API
#include <string>

namespace webots {
  class DifferentialWheels;
  class Device {
    public:
      virtual ~Device() {}
      const std::string &getName() const { return name; }

    protected:
      Device(const std::string &n, const DifferentialWheels *d) : name(n), differentialWheels(d) {}
      const DifferentialWheels *getDifferentialWheels() const { return differentialWheels; }

    private:
      std::string name;
      const DifferentialWheels *differentialWheels;
  };
}

#endif //DEVICE_HPP
