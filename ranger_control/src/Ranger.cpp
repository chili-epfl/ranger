#include "Ranger.hpp"
#include "webots/DifferentialWheels.hpp"
#include "webots/DistanceSensor.hpp"
#include "webots/Accelerometer.hpp"
#include "webots/TouchSensor.hpp"
#include "webots/Receiver.hpp"
#include "webots/Emitter.hpp"
#include "webots/Motor.hpp"
#include "webots/LED.hpp"

#include <stdio.h>
#include <cstdlib>
#include <math.h>

using namespace webots;

static const char *motorNames[NMOTORS] = {
  "RightPupilVert",   "RightPupilHori",  // Right pupil
  "LeftPupilVert",    "LeftPupilHori",   // Left  pupil
  "LeftUpperEyelid",  "LeftLowerEyelid", // Left  eyelid
  "RightUpperEyelid", "RightLowerEyelid" // Right eyelid
};

static const char *DSNames[NDISTANCESENSORS] = {
  "dsBottomRight", "dsBottomLeft",                // Ground DistanceSensors
  "dsFrontRight",  "dsFrontCenter", "dsFrontLeft" // Front DistanceSensors
};

//Constructor
Ranger::Ranger():
    DifferentialWheels()
{
  // Get all the Motors
  for(int i=0; i<NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    mMotors[i]->enablePosition(TIME_STEP); // this is present only for compatibility with simulation
  }
  
  // Get all the distance sensors
  for(int i=0; i<NDISTANCESENSORS; i++) {
    mDistanceSensors[i] = getDistanceSensor(DSNames[i]);
    mDistanceSensors[i]->enable(TIME_STEP);
  }
  
  mReceiver = getReceiver("infraredReceiver");
  mReceiver->enable(TIME_STEP);
  
  // Get the bumper and the balance
  mBumper = getTouchSensor("bumper");
  mBalance = getTouchSensor("balance");
  
  mBumper->enable(TIME_STEP);
  mBalance->enable(TIME_STEP);
  
  mReferenceTime = 0.0;
  mAttractorUpdateTime = 0.0;
  for(int i = 0; i<20; ++i)
    myStep();
  setSpeed(0,0);
}

//Destructor
Ranger::~Ranger() {
}

//Step function
void Ranger::myStep() {
  int ret = step(TIME_STEP);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void Ranger::printPos() {
  // This is a debug function, that can for example be used in order to print the distance and angle
  // between the station and the charging station
  while(1) {
    updateStationPos();
    printf("\rmStationDistance: %.3lf  mStationAngle: %.3lf  mRobotAngle: %.3lf", mStationDistance, mStationAngle, mRobotAngle);
    myStep();
  }
}

void Ranger::updateStationPos() {
  while(mReceiver->getQueueLength() != 0) {
    // Packet from the charging station
    if(mReceiver->getEmitterID() == BASE_ID) {
      mStationAngle = atan2(mReceiver->getEmitterDirection()[0], mReceiver->getEmitterDirection()[2]);
      mStationDistance = 1 / sqrt(mReceiver->getSignalStrength());
      mRobotAngle = -M_PI * ((float)((int *)mReceiver->getData())[0] / 1800) + 2*M_PI;
    }
    // Packet from the beacon
    else if(mReceiver->getEmitterID() == ATTRACTOR_ID) {
      mAttractorAngle = atan2(mReceiver->getEmitterDirection()[0], mReceiver->getEmitterDirection()[2]);
      mAttractorDistance = 1 / sqrt(mReceiver->getSignalStrength());
      mAttractorUpdateTime = getTime();
    }
    mReceiver->nextPacket();
  }
}

bool Ranger::gotToChargeStation() {
  
  enum { FACE_STATION_1, COMPUTE_NEEDED_ROTATION, ALIGN_STATION_EMITTER, FACE_STATION_2, APPROACHING_STATION, REALIGNE_STATION};
  bool reinit[REALIGNE_STATION];
  for(int i=0; i<REALIGNE_STATION; ++i)
    reinit[i] = false;
  
  
  int action = FACE_STATION_1;
  int rotationDirection = 0;
  
  enableEncoders(TIME_STEP);
  
  if(isCharging()) // Already charging
    return true;
  
  while(1) {
    updateStationPos();
    double left_speed = 0;
    double right_speed = 0;
    
    switch(action) {
      case FACE_STATION_1:            // Face the charging station
#ifdef DEBUG
        printf("FACE_STATION_1\n");
#endif
        if(mStationAngle != 0.0 && fabs(mStationAngle-M_PI) > 0.05) {
          if(mStationAngle > 0) { // turn left
            left_speed  = -20;
            right_speed = 20;
          }
          else {          // turn right
            left_speed  = 20;
            right_speed = -20;
          }
          // Face station from front
          if(mStationAngle < 0.2 && mStationAngle > -0.2) { 
            left_speed  = 0;
            right_speed = 0;
            action++;
          }
          // Face station from rear       
          if(((mStationAngle > 3 && mStationAngle < 3.3) || mStationAngle < -3 || (mStationAngle > 6 && mStationAngle < 7)) && fabs(mStationAngle - 3.142) > 0.01) {  // Station Faced
            left_speed  = 0;
            right_speed = 0;
            action++;
          }
        }
        else {
          left_speed  = 20;
          right_speed = -20;
        }
        break;


      case COMPUTE_NEEDED_ROTATION: // determine if approaching station from left or right
#ifdef DEBUG
        printf("COMPUTE_NEEDED_ROTATION\n");
#endif
        if(mRobotAngle > M_PI || mRobotAngle < 0)
          rotationDirection = -1;
        else
          rotationDirection = 1;
        action++;
        break;


      case ALIGN_STATION_EMITTER: {            // do a circle around the station in order to align with its 0 angle
#ifdef DEBUG
        printf("ALIGN_STATION_EMITTER\n");
#endif
        static bool firstRotation = true;
        static double overpassing = false;
        static double referenceTime = getTime();
        static int nbOverpass = 0;
        const double maxTime = 75; // maximum time in second to execute the maneuvre
        double offset = 0;
        double angleOffset = 0;
        double angle = 75;

        if(!reinit[ALIGN_STATION_EMITTER]) {
          firstRotation = true;        
          overpassing = false;
          referenceTime = getTime();
          nbOverpass = 0;
          reinit[ALIGN_STATION_EMITTER] = true;
        }
        
        mRightEncoder = getRightEncoder();
        // Face station from front
        if(mStationAngle < 0.2 && mStationAngle > -0.2) { 
          
          left_speed  = 20 * rotationDirection;
          right_speed = -20 * rotationDirection;
          
          if(mStationAngle > 0)
            angleOffset = (180 * -mStationAngle * rotationDirection) / M_PI;
          else
            angleOffset = (180 * mStationAngle * rotationDirection) / M_PI;
        }
        else {
          left_speed  = -20 * rotationDirection;
          right_speed = 20 * rotationDirection;
        
          if(mStationAngle > 0)
            angleOffset = (180 * (M_PI - mStationAngle) * rotationDirection) / M_PI;
          else
            angleOffset = (180 * (-M_PI - mStationAngle) * rotationDirection) / M_PI;
        }
        
        // Execute a rotation to prepare the circle trajectory
        while(firstRotation && (fabs(getRightEncoder() - mRightEncoder) < selfRotationAngle2Encoder(angle + angleOffset))) {
#ifdef DEBUG
          printf("Rotating: %d / %.0lf\n", abs(getRightEncoder() - mRightEncoder), selfRotationAngle2Encoder(angle + angleOffset));
#endif
          setSpeed(left_speed, right_speed);
          myStep();
        }
        firstRotation = false;
        
        static double initialDistance = 1.2;
        static double distanceCorrection;
        distanceCorrection = mStationDistance;
        if(distanceCorrection > 1.5 * initialDistance)
          distanceCorrection = 1.5 * initialDistance;
        if((mStationDistance - initialDistance) > (0.2 * initialDistance) || (mStationDistance - initialDistance) < -(0.2 * initialDistance))
          offset = 35.0*((distanceCorrection - initialDistance)/initialDistance); // can be tunned up
        else
          offset = 0;
        
        // Execute the circle trajectory until we reached baseStation angle 0
        if(rotationDirection < 0) {
          left_speed  = -(30 + 5.0/mStationDistance + offset); // in order to keep allways the same radius (can be tunned up)
          right_speed = -30;
        }
        else {
          left_speed  = -30;
          right_speed = -(30 + 5.0/mStationDistance + offset); // in order to keep allways the same radius (can be tunned up)
        }
       
        // obstacle collision => problem! Or maneuvre too long
        if(mBumper->getValue() || ((getTime() - referenceTime) > maxTime) || getRightMotorCurrent() > 600 || getLeftMotorCurrent() > 600) { 
          setSpeed(0, 0);
          myStep();
          return false;
        }
        
#ifdef DEBUG
        printf("L: %lf R: %lf\n", left_speed, right_speed);
#endif
        // robot face the charging station, but continue a little bit
        if((mRobotAngle < 0.1 && mRobotAngle > -0.1) || (mRobotAngle > 6.1 && fabs(mRobotAngle - 2*M_PI) > 0.01) || overpassing) { // Angle 0 reached (WARNING: wrong value of 2*PI is somtimes returned)
          overpassing = true;
          nbOverpass++;
#ifdef DEBUG
          printf("\nOVERPASSING\n");
#endif
          if(nbOverpass > 200 * mStationDistance)
            action++;
        }
        
        break; }


      case FACE_STATION_2:
#ifdef DEBUG
        printf("FACE_STATION_2\n");
#endif
        if(rotationDirection < 0) { // turn left
          left_speed  = -20;
          right_speed = 20;
        }
        else {                   // turn right
          left_speed  = 20;
          right_speed = -20;
        }
        
        setSpeed(left_speed, right_speed);
        mRightEncoder = getRightEncoder();
        while(abs(getRightEncoder() - mRightEncoder) < selfRotationAngle2Encoder(75.0)) { // 75 instead of 90
#ifdef DEBUG
          printf("\nRotating: %d / %.0lf", abs(getRightEncoder() - mRightEncoder), selfRotationAngle2Encoder(75.0));
#endif
          setSpeed(left_speed, right_speed);
          myStep();
        }
        
        for(int i = 0; i<500; ++i) { // go straight slowly in order to realign wheels
          setSpeed(-10, -10);
          myStep();
        }
          
        action = REALIGNE_STATION;
        break;


      case APPROACHING_STATION:
        
        static int currentRealignment = 0;
        static const int NBRealigment = 6;
        static const double realigmentDistances[NBRealigment] = {0.6, 0.5, 0.4, 0.3, 0.2, 0.1};
        static double referenceDistance = mStationDistance;
        static double referenceTime = getTime();
        static const double maxTime = 45; // maximum time in second
        
        if(!reinit[APPROACHING_STATION]) {
          currentRealignment = 0;
          referenceDistance = mStationDistance;
          referenceTime = getTime();
          reinit[APPROACHING_STATION] = true;
        }
#ifdef DEBUG
        printf("APPROACHING_STATION DistRef:%.3lf\n", referenceDistance);
#endif
        left_speed  = -20;
        right_speed = -20;
        if(mReferenceTime == 0.0)
          mReferenceTime = getTime();
        // Not realigned from more than 2 second => realigment forced
        if((getTime() - mReferenceTime) > 2) {
          referenceDistance = mStationDistance;
          action++;
          break;
        }
        
        // obstacle collision => station missed!
        if(mBumper->getValue() || (getTime() - referenceTime) > maxTime || getRightMotorCurrent() > 600 || getLeftMotorCurrent() > 600) { 
          setSpeed(0, 0);
          myStep();
          printf("\n\n *** STATION MISSED *** \n\n");
          return false;
        }
        
        // Check if realigned distance reached
        for(int i=0; i<NBRealigment ; ++i) {
          // Check if station is really close
          if(i >= (NBRealigment-1) && mStationDistance < realigmentDistances[NBRealigment-1]) {
		    action++;
            if(currentRealignment < NBRealigment) // first iteration, realign
              currentRealignment++;
            else {
              while(1) { // Second iteration, go straight until obstacle or station collision
                setSpeed(-15, -15);
                myStep();
                if(isCharging()) {  // station collision
                  setSpeed(0, 0);
                  getLED("LED")->set(0x000000);
                  myStep();
                  printf("\n\n *** STATION REACHED *** \n\n");
                  closeEyes();
                  return true;
                }
                else if(mBumper->getValue()  || getRightMotorCurrent() > 600 || getLeftMotorCurrent() > 600) { //obstacle collision
                  setSpeed(0, 0);
                  myStep();
                  printf("\n\n *** STATION MISSED *** \n\n");
                  return false;
                }
              }
            }
          }
          // Check for realigment distance
          else if(currentRealignment < i && mStationDistance < realigmentDistances[i]) {
            referenceDistance = mStationDistance;
            currentRealignment++;
            action++;
            break;
          }
        }
        break;
        
        
      case REALIGNE_STATION:
        static int signChange = 0;
        static int oldAngle;
#ifdef DEBUG
        printf("REALIGNE_STATION SignChange: %d\n", signChange);
#endif
        if(mStationAngle < 0) { // turn left
          left_speed  = 15;
          right_speed = -15;
          if(oldAngle > 0)
            signChange++;
        }
        else {                   // turn right
          left_speed  = -15;
          right_speed = 15;
          if(oldAngle < 0)
            signChange++;
        }
        if((signChange > 8) || ((mStationAngle < -3 || mStationAngle > 3) && fabs(mStationAngle - 3.142) > 0.01) || (fabs(mStationAngle - 2.356) < 0.01) || (fabs(mStationAngle + 2.356) < 0.01)) {  // Station Faced (wrong pi value must be ingored, and +-2.356 is wrong value when station is aligned)
          left_speed  = 0;
          right_speed = 0;
          signChange = 0;
          mReferenceTime = getTime();
          action--;
        }
        oldAngle = mStationAngle;
        break;
        
        
      default:
        printf("OUT OF THE LOOP (action=%d)\n", action);
        break;
    }

#ifdef DEBUG
    printf("mStationDistance: %.3lf  mStationAngle: %.3lf  mRobotAngle: %.3lf\n", mStationDistance, mStationAngle, mRobotAngle);
#endif
    setSpeed(left_speed, right_speed);
    myStep();
  }
}

void Ranger::approachChargingStation() {
  
  enableEncoders(TIME_STEP);      // make sure encoders are enable
  getLED("LED")->set(0x00FF00);   // set led in green
  const int angleSpeedFactor    = 55;
  const int distanceSpeedFactor = 25;
  const int DSSampleNumber      = 15;   // used in order to filter distance sensors
  const int DSThreshold         = 350;  // used in order to filter distance sensors
  const double stationMinimalDistance  = 1.2;  // distance to reach
  const double stationTooSmallDistance = 0.85; // if closer than this, need to go away
  const double breitenbergFactor       = 0.15;
  const bool obstacleAvoidance = false;
  
  bool firstLook = true;
  
  if(isCharging()) // Already charging
    return;
  
  // main loop
  while(1) {
    updateStationPos();
    
    // Alignment with the station
    double left_speed  =  mStationAngle * angleSpeedFactor;
    double right_speed = -mStationAngle * angleSpeedFactor;

    // Attraction from the station
    if(mStationDistance < 2) {
      left_speed  += mStationDistance * distanceSpeedFactor;
      right_speed += mStationDistance * distanceSpeedFactor;
    }
    else if (mStationDistance < 100) { // in order to deal with 'inf' distance when no signal 
      left_speed  += 50;
      right_speed += 50;
    }
    
    
    double leftValue  = 0;
    double rightValue = 0;
    // Repulsion from obstacles (value for the distance sensors are used through a moving mean)
    if(obstacleAvoidance) {
      static std::deque<double> valuesR(DSSampleNumber, 0);
      static std::deque<double> valuesL(DSSampleNumber, 0);
      
      if(mDistanceSensors[2]->getValue() > DSThreshold) // Threshold on noise
        valuesR.push_front(mDistanceSensors[2]->getValue());
      else 
        valuesR.push_front(0);
      valuesR.pop_back();
      
      if(mDistanceSensors[4]->getValue() > DSThreshold)
        valuesL.push_front(mDistanceSensors[4]->getValue());
      else
        valuesL.push_front(0);
      valuesL.pop_back();
       
      leftValue  = mean(valuesL);
      rightValue = mean(valuesR);
      
      left_speed  += breitenbergFactor * leftValue  - breitenbergFactor * rightValue;
      right_speed += breitenbergFactor * rightValue - breitenbergFactor * leftValue;
      
      while((left_speed > 100.0) || (left_speed < -100.0) || (right_speed > 100.0) || (right_speed < -100.0)) {
        left_speed  = left_speed/2;
        right_speed = right_speed/2;
      }
    }

    // too close from the charging station
    if(mStationDistance < stationTooSmallDistance && mStationDistance != 0.0) {
      goAwayFromStation(stationTooSmallDistance);
      return;
    }
    // target distance reached
    else if(mStationDistance < stationMinimalDistance && mStationDistance != 0.0) {
      setSpeed(0, 0);
      return;
    }

    // if no signal from charging station turn 
    if((mStationDistance == 0.0 || mStationAngle == 0.0) && firstLook) { 
      left_speed  = 15;
      right_speed = -15;
    }
    else
      firstLook = false;
#ifdef DEBUG
    printf("mStationDistance: %.3lf  mStationAngle: %.3lf  mRobotAngle: %.3lf right: %.0lf left: %.0lf\n", mStationDistance, mStationAngle, mRobotAngle, rightValue, leftValue);
#endif
    setSpeed(left_speed, right_speed);
    myStep();
  }
}

void Ranger::goAwayFromStation(double distance) {
  const int angleSpeedFactor = 75;
  // used in order to compute the low-pass filter on stationAngle
  const int filterSize = 25;
  double angleTable[filterSize];
  double stationAngle;
  
  for(int i=0; i<filterSize; ++i)
    angleTable[i] = 0;
  
  while(1) {
    updateStationPos();
    
    // low-pass filter on stationAngle in order to get ride of abrupt motion
	for(int i=0; i<(filterSize-1); ++i)
	  angleTable[i] = angleTable[i+1];
	angleTable[filterSize-1] = mStationAngle;
	
	stationAngle = 0;
	for(int i=0; i<filterSize; ++i)
	  stationAngle += angleTable[i];
	stationAngle /= filterSize;
	
	  
    double left_speed  =  30;
    double right_speed =  30;
    
    double angleCorrection = 0;   
    if(stationAngle < 0)
      angleCorrection = stationAngle + M_PI;
    else if((stationAngle > 0) && stationAngle != M_PI)
      angleCorrection = stationAngle - M_PI;
      
    // stay aligned with the charging station
    left_speed  +=  angleCorrection * angleSpeedFactor;
    right_speed += -angleCorrection * angleSpeedFactor;
    
    // Obstacle avoidance can be added
    // left_speed  +=
    // right_speed +=
    
    // limit speed to 50
    while((left_speed > 50.0) || (left_speed < -50.0) || (right_speed > 50.0) || (right_speed < -50.0)) {
      left_speed  = left_speed/2;
      right_speed = right_speed/2;
    }
#ifdef DEBUG
    printf("\rright speed: %.3lf left speed: %.3lf angleCorrection: %.3lf", right_speed, left_speed, angleCorrection);
#endif
    setSpeed(left_speed, right_speed);
    
    // target distance from the charging station reached
    if((mStationDistance > distance) && (mStationDistance != 0.0) && (mStationAngle < -3 || mStationAngle > 3)) {
      setSpeed(0, 0);
      return;
    }
      
    myStep();
  }
}

double Ranger::mean(std::deque<double> tab) {
  double som = 0;
  for(unsigned int i = 0; i < tab.size(); ++i)
    som += tab[i];	
  return som / tab.size();
}

bool Ranger::hasLolette() {
  myStep();
  return isLolettePresent();
}

bool Ranger::waitSignalFromAttractor(int nbIteration) {
  // return when receiving a packet from the station
  // or after nbIteration TIME_STEP (if nbIteration != -1)
  int counter = 0;
  while(mReceiver->getQueueLength() > 0) { // Clean current queue
    mReceiver->nextPacket();
  }
  while(1) {
    while(mReceiver->getQueueLength() > 0) {
      if(mReceiver->getEmitterID() == ATTRACTOR_ID) {
        return true;
      }
      mReceiver->nextPacket();
    }
    if(nbIteration > 0) {
      counter++;
      if(counter > nbIteration)
        return false;
    }
    myStep();
  }
}

bool Ranger::isAttractorActif(int nbIteration) {
  while(mReceiver->getQueueLength() > 0) { // Clean current queue
    mReceiver->nextPacket();
  }
  for(int c=0; c<nbIteration; ++c) {
    while(mReceiver->getQueueLength() > 0) {
      if(mReceiver->getEmitterID() == ATTRACTOR_ID) {
        return true;
      }
      mReceiver->nextPacket();
    }
    myStep();
  }
  return false;
}

void Ranger::goToAttractor() {
   
  //playLedVideo(12, 0);
  openEyes();
  enableEncoders(TIME_STEP);
  
  const int angleSpeedFactor = 55;
  const int distanceSpeedFactor = 40;
  const double stationMinimalDistance = 0.45; // distance from the station that we want to reach
  const double maxTime = 1.5; // Maximum time with no signal from attractor
  const int DSSampleNumber = 10; // used in order to filter value of the distance sensors
  const int DSThreshold = 500; // used in order to filter value of the distance sensors
  const double breitenbergFactor = 0.1;
  const bool obstacleAvoidance = false;
  static int counter = 0;
  
  // go a litle straight (go out of the bed/table)
  if(isCharging()) {
    for(int i=0; i<200; ++i) {
      setSpeed(i*0.5, i*0.5);
      myStep();
      updateStationPos();
    }
  }
  
  while(1) {
    updateStationPos();
    if(isLolettePresent()) { // lolette has been but back
      setSpeed(0, 0);
      return;
    }
    
    // Alignment with the attractor
    double left_speed  =  mAttractorAngle * angleSpeedFactor + 15;
    double right_speed = -mAttractorAngle * angleSpeedFactor + 15;

    // Attraction from the station
    if(mAttractorDistance < 2) {
      left_speed  += mAttractorDistance * distanceSpeedFactor;
      right_speed += mAttractorDistance * distanceSpeedFactor;
    }
    else if (mAttractorDistance < 100) { // in order to deal with 'inf' distance when no signal 
      left_speed  += 50;
      right_speed += 50;
    }
    
    double leftValue  = 0;
    double rightValue = 0;
    // Repulsion from obstacles (value for the distance sensors are used through a moving mean)
    if(obstacleAvoidance) {
      static std::deque<double> valuesR(DSSampleNumber, 0);
      static std::deque<double> valuesL(DSSampleNumber, 0);
      
      if(mDistanceSensors[2]->getValue() > DSThreshold) // Threshold on noise
        valuesR.push_front(mDistanceSensors[2]->getValue());
      else 
        valuesR.push_front(0);
      valuesR.pop_back();
      
      if(mDistanceSensors[4]->getValue() > DSThreshold)
        valuesL.push_front(mDistanceSensors[4]->getValue());
      else
        valuesL.push_front(0);
      valuesL.pop_back();
       
      leftValue  = mean(valuesL);
      rightValue = mean(valuesR);
      
      left_speed  += breitenbergFactor * leftValue  - breitenbergFactor * rightValue;
      right_speed += breitenbergFactor * rightValue - breitenbergFactor * leftValue;
      
      while((left_speed > 100.0) || (left_speed < -100.0) || (right_speed > 100.0) || (right_speed < -100.0)) {
        left_speed  = left_speed/2;
        right_speed = right_speed/2;
      }
    }
    
	// No more signal from attractor -> probably switch off
    if((getTime() - mAttractorUpdateTime) > maxTime) {
      left_speed  = 10;
      right_speed = -10;
      counter++;
      getLED("LED")->set(0xFF0000);
      if(counter > 200) {
        //playLedVideo(3, 0);
        counter = 0;
        int tab[10];
        tab[0] = 1;     
        static int left = true;   
        left = !left;
        if(left)
          tab[1] = 1;
        else
          tab[1] = 2;
        emitCustomEvent(tab);
      }
    }
    // close enough from the beacon
    else if(mAttractorDistance < stationMinimalDistance && mAttractorDistance != 0.0) {
      setSpeed(0, 0);
      //playLedVideo(3, 0);
      getLED("LED")->set(0x0000FF);
      return;
    }
    else // going to the beacon
      getLED("LED")->set(0x000000);
#ifdef DEBUG
    printf("mAttractorDistance: %.3lf  mStationAngle: %.3lf left_speed: %.0lf right_speed: %.0lf\n", mAttractorDistance, mAttractorAngle, left_speed, right_speed);
#endif
    setSpeed(left_speed, right_speed);
    myStep();
  }
}

void Ranger::openEyes() {
  for(int i=4; i<NMOTORS; ++i) 
    mMotors[i]->setPosition(M_PI_4);
}
void Ranger::closeEyes() {
  for(int i=4; i<NMOTORS; ++i) 
    mMotors[i]->setPosition(0.0);
}
void Ranger::blinkEyes() {
  closeEyes();
  for(int i=0; i<6; ++i) 
    myStep();
  openEyes();
}
