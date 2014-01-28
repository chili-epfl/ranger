// File:          ranger.cpp
// Date:          18.06.2013
// Description:   Ranger C++ controller
// Author:        david.mansolino@epfl.ch

#ifndef RANGERCPP_HPP
#define RANGERCPP_HPP

#include "webots/DifferentialWheels.hpp"
#include <deque>

#define TIME_STEP          25
#define NMOTORS             8
#define NDISTANCESENSORS    5

#define BASE_ID             4
#define ATTRACTOR_ID        20

//#define DEBUG

namespace webots {
  class Motor;
  class TouchSensor;
  class DistanceSensor;
  class Receiver;
  
  
  class Ranger : public DifferentialWheels {
    public:
                                       Ranger();
      virtual                         ~Ranger();

      bool                             gotToChargeStation();
      void                             approachChargingStation();
      void                             goAwayFromStation(double distance);
      bool                             waitSignalFromAttractor(int nbIteration = -1);
      bool                             hasLolette();
      bool                             isAttractorActif(int nbIteration = 5);
      void                             goToAttractor();
      
      void                             printPos(); // debug function
      void                             updateStationPos();
      void                             openEyes();
      void                             closeEyes();
      void                             blinkEyes();
      
    private:
      void                             myStep();
      
      Motor                           *mMotors[NMOTORS];
      DistanceSensor                  *mDistanceSensors[NDISTANCESENSORS];
      TouchSensor                     *mBumper;
      TouchSensor                     *mBalance;
      Receiver                        *mReceiver;
      
      // Charging station
      double                           mStationDistance;
      double                           mStationAngle; // seen from the robot
      double                           mRobotAngle;   // seen from the station
      
      // Beacon(s)
      double                           mAttractorDistance;
      double                           mAttractorAngle; // seen from the robot
      double                           mAttractorUpdateTime;
      
      double                           mRightEncoder;
      double                           mLeftEncoder;
      
      double                           mReferenceTime;
      
      double                           mean(std::deque<double> tab);
      double                           selfRotationAngle2Encoder(double angle) { return angle*350.0; }
  };
};

#endif
