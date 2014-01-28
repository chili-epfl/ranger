/*******************************************************************************************************/
/* File:         DifferentialWheels.hpp                                                                */
/* Date:         jul 2013                                                                              */
/* Description:  Wrapper of the DifferentialWheels Webots API for the Ranger real robot                */
/* Author:       david.mansolino@epfl.ch                                                               */
/*******************************************************************************************************/

#ifndef DIFFERENTIALSWHEELS_H
#define DIFFERENTIALSWHEELS_H

#include <string>
#include <map>
#include <sys/time.h>

class DashelInterface;

namespace webots {
  class Device;
  class DistanceSensor;
  class LED;
  class Receiver;
  class Accelerometer;
  class TouchSensor;
  class Motor;
  class Emitter;

    class DifferentialWheels {
        public:
            DifferentialWheels();
            ~DifferentialWheels();

            virtual int          step(int ms);
            std::string          getName() const;
            double               getTime() const;
            int                  getMode() const;
            void                 setSpeed(double left, double right);
            double               getLeftSpeed() const;
            double               getRightSpeed() const;
            void                 enableEncoders(int ms);
            void                 disableEncoders();
            double               getLeftEncoder() const;
            double               getRightEncoder() const;
            void                 batterySensorEnable(int ms);
            void                 batterySensorDisable();
            double               batterySensorGetValue();
            double               getMaxSpeed() const;
            double               getSpeedUnit() const;
            
            // Custom ranger function
            void                 playLedVideo(int videoNumber, bool repeat);
            void                 stopLedVideo();
            
            bool                 isCharging();
            bool                 isLolettePresent();
            int                  getRightMotorCurrent();
            int                  getLeftMotorCurrent();
            
            void                 emitCustomEvent(int *tab);

            DistanceSensor *getDistanceSensor(const std::string &name) const;
            LED            *getLED(const std::string &name) const;
            Receiver       *getReceiver(const std::string &name) const;
            Accelerometer  *getAccelerometer(const std::string &name) const;
            TouchSensor    *getTouchSensor(const std::string &name) const;
            Motor          *getMotor(const std::string &name) const;
            Emitter        *getEmitter(const std::string &name) const;

        protected:

            void initDevices();
            void initRanger();
            Device *getDevice(const std::string &name) const;

       private:
            DashelInterface                      *dashelInterface;
            void                                  connectToDashel();
                   
            int                                   mTimeStep;
            std::map<const std::string, Device *> mDevices;
            struct timeval                        mStartTime;
            double                                mPreviousStepTime;
            double                                mTargetSpeed[2];
            bool                                  mEncoderEnable;
            double                                mRightEncoder;
            double                                mLeftEncoder;
            double                                mRightSpeed;
            double                                mLeftSpeed;
            double                                mBatterylevel;
            int                                   mRightCurrent;
            int                                   mLeftCurrent;
            bool                                  mVideoIsPlaying;
            bool                                  mLolette;
            bool                                  mIsCharging;
    };
}

#endif // DIFFERENTIALSWHEELS_H
