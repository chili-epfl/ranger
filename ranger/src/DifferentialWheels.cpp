#include "webots/DifferentialWheels.hpp"
#include "webots/DistanceSensor.hpp"
#include "webots/Accelerometer.hpp"
#include "webots/TouchSensor.hpp"
#include "webots/Receiver.hpp"
#include "webots/Emitter.hpp"
#include "webots/Device.hpp"
#include "webots/Motor.hpp"
#include "webots/LED.hpp"

#include "dashelinterface.h"

#include <unistd.h>
#include <math.h>
#include <vector>

#include <stdio.h> // DEBUG

using namespace std;

webots::DifferentialWheels::DifferentialWheels() {
    dashelInterface = new DashelInterface();
    initRanger();
    initDevices();
    usleep(1000000);
    mPreviousStepTime = 0.0;
    gettimeofday(&mStartTime, NULL);
    mTargetSpeed[0] = NAN;
    mTargetSpeed[1] = NAN;
    mEncoderEnable = false;
    mRightEncoder = 0;
    mLeftEncoder = 0;
    mRightCurrent = 0;
    mLeftCurrent = 0;
    mVideoIsPlaying = true; // we don't know but it could be
    if(!dashelInterface->dashelConnected())
		exit(-1);
    const int args[1] = {1};
	dashelInterface->sendEvent("enableFeedback", args, 1);
	// half open eyes and eyes in the center
	((Motor *)mDevices["LeftUpperEyelid"])->setPosition(M_PI_4);
	((Motor *)mDevices["LeftLowerEyelid"])->setPosition(M_PI_4);
	((Motor *)mDevices["RightUpperEyelid"])->setPosition(M_PI_4);
	((Motor *)mDevices["RightLowerEyelid"])->setPosition(M_PI_4);
	((Motor *)mDevices["RightPupilVert"])->setPosition(0);
	((Motor *)mDevices["RightPupilHori"])->setPosition(0);
	((Motor *)mDevices["LeftPupilVert"])->setPosition(0);
	((Motor *)mDevices["LeftPupilHori"])->setPosition(0);
}

webots::DifferentialWheels::~DifferentialWheels() {
	const int args[1] = {0};
	dashelInterface->sendEvent("enableFeedback", args, 1);
	usleep(100000);
    dashelInterface->disconnectAseba();
}

int webots::DifferentialWheels::step(int ms) {
  char buffer[32];
  
  // -------- Sensors -------- //
  // DistanceSensor
  const int * sharpValues = dashelInterface->getSharpValues();
  ((DistanceSensor *)mDevices["dsBottomLeft"])->setValue(sharpValues[0]);
  ((DistanceSensor *)mDevices["dsBottomRight"])->setValue(sharpValues[1]);
  ((DistanceSensor *)mDevices["dsFrontLeft"])->setValue(sharpValues[2]);
  ((DistanceSensor *)mDevices["dsFrontRight"])->setValue(sharpValues[3]);
  ((DistanceSensor *)mDevices["dsFrontCenter"])->setValue(sharpValues[4]);
  // Accelerometer
  ((Accelerometer *)mDevices["accelerometer"])->setValues(dashelInterface->getAccelerometerValues()[0], dashelInterface->getAccelerometerValues()[1], dashelInterface->getAccelerometerValues()[2]);
  // TouchSensor
  ((TouchSensor *)mDevices["bumper"])->setValue(dashelInterface->getBumper());
  ((TouchSensor *)mDevices["balance"])->setValue(dashelInterface->getBalance());
  // Touch pannels
  for(int c=0; c<9; ++c) {
	  // right pannel
	  sprintf(buffer, "touchR%d", c);
	  ((TouchSensor *)mDevices[buffer])->setValue(dashelInterface->getTouchSensors()[c]);
	  // rear pannel
	  sprintf(buffer, "touchB%d", c);
	  ((TouchSensor *)mDevices[buffer])->setValue(dashelInterface->getTouchSensors()[c+9]);
	  // left pannel
	  sprintf(buffer, "touchL%d", c);
	  ((TouchSensor *)mDevices[buffer])->setValue(dashelInterface->getTouchSensors()[c+18]);
  }
  // Battery
  mBatterylevel = dashelInterface->getBatteryLevel();
  dashelInterface->getChargingState() == 1 ? mIsCharging = true : mIsCharging = false;
  // Wheel speeds
  mRightSpeed = dashelInterface->getSpeed()[0];
  mLeftSpeed  = -dashelInterface->getSpeed()[1];
  // Wheel motor current
  mRightCurrent = dashelInterface->getMotorsCurrent()[0];
  mLeftCurrent  = dashelInterface->getMotorsCurrent()[1];
  // Wheel encoders
  if(mEncoderEnable) {
	  const int * encodersState = dashelInterface->getEncodersValues();
	  if(encodersState[0] > 0)
		mRightEncoder = encodersState[0] + 65536*encodersState[1]; // 65536 = 2^16 = maximum value representable with 16bits 
      else 
        mRightEncoder = 65536 + encodersState[0] + 65536*encodersState[1];
     if(encodersState[2] > 0)
		mLeftEncoder = -(encodersState[2] + 65536*encodersState[3]);
      else 
        mLeftEncoder = -(65536 + encodersState[2] + 65536*encodersState[3]);
  }
  // Lolette
  if(dashelInterface->getLolette() == 1)
    mLolette = true;
  else
    mLolette = false;
  // Receiver
  vector<packet> * packets = dashelInterface->getPackets();
  ((Receiver *)mDevices["infraredReceiver"])->resetQueue();
  if(packets != NULL) {
    for(unsigned int i=0; i<packets->size(); ++i)
      ((Receiver *)mDevices["infraredReceiver"])->addPacket((*packets)[i].id, (*packets)[i].dist, (*packets)[i].angle, (*packets)[i].data);
  }
  dashelInterface->cleanPackets();
	  
  // -------- Actuators -------- //
  // Led (only one main led for now)
  if(((LED *)mDevices["LED"])->hasChanged()) {
    int color = ((LED *)mDevices["LED"])->get();
    int red = (color >> 16) & 0xFF;
    int green = (color >> 8) & 0xFF;
    int blue = color & 0xFF;
    const int args[3] = {red, green, blue};
    dashelInterface->sendEvent("setLed", args, 3);
    ((LED *)mDevices["LED"])->resetChanged();
  }
  // Motors
  if(Motor::hasChanged()) {
	  int args[8];
	  args[0] = 100 * (((Motor *)mDevices["LeftPupilHori"])->getTargetPosition()    / ((Motor *)mDevices["LeftPupilHori"])->getMaxPosition());
	  args[1] = 100 * (((Motor *)mDevices["LeftPupilVert"])->getTargetPosition()    / ((Motor *)mDevices["LeftPupilVert"])->getMaxPosition());
	  args[2] = 100 * (((Motor *)mDevices["RightPupilHori"])->getTargetPosition()   / ((Motor *)mDevices["RightPupilHori"])->getMaxPosition());
	  args[3] = 100 * (((Motor *)mDevices["RightPupilVert"])->getTargetPosition()   / ((Motor *)mDevices["RightPupilVert"])->getMaxPosition());
	  args[4] = 100 * (((Motor *)mDevices["LeftUpperEyelid"])->getTargetPosition()  / ((Motor *)mDevices["LeftUpperEyelid"])->getMaxPosition());
	  args[5] = 100 * (((Motor *)mDevices["LeftLowerEyelid"])->getTargetPosition()  / ((Motor *)mDevices["LeftLowerEyelid"])->getMaxPosition());
	  args[6] = 100 * (((Motor *)mDevices["RightLowerEyelid"])->getTargetPosition() / ((Motor *)mDevices["RightLowerEyelid"])->getMaxPosition());
	  args[7] = 100 * (((Motor *)mDevices["RightUpperEyelid"])->getTargetPosition() / ((Motor *)mDevices["RightUpperEyelid"])->getMaxPosition());
      dashelInterface->sendEvent("neuilEvent", (const int *)args, 8);
	  Motor::resetChanged();
  }

  // -------- Timing management -------- //
  double actualTime = getTime() * 1000;
  double stepDuration = actualTime - mPreviousStepTime;

  if(stepDuration < ms) { // Step too short -> wait remaining time
    usleep((ms - stepDuration) * 1000);
    mPreviousStepTime = getTime() * 1000;
    return 0;
  }
  else { // Step too long -> return step duration
    mPreviousStepTime = actualTime;
    return stepDuration;
  }
  return 0;
}

double webots::DifferentialWheels::getTime() const {
  struct timeval end;

  gettimeofday(&end, NULL);

  long seconds  = end.tv_sec  - mStartTime.tv_sec;
  long useconds = end.tv_usec - mStartTime.tv_usec;
  long mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;

  return (double) mtime/1000.0;
}

int webots::DifferentialWheels::getMode() const {
  return 1;
}

std::string webots::DifferentialWheels::getName() const {
  return "ranger";
}

double webots::DifferentialWheels::getMaxSpeed() const {
	return 19.2;
}

double webots::DifferentialWheels::getSpeedUnit() const {
	return 0.0144;
}

double webots::DifferentialWheels::getLeftSpeed() const {
	return mLeftSpeed;
}

double webots::DifferentialWheels::getRightSpeed() const {
	return mRightSpeed;
}

void webots::DifferentialWheels::enableEncoders(int ms) {
	if(mEncoderEnable == false) {
		const int args[1] = {1};
		dashelInterface->sendEvent("enableEncoders", args, 1);
		mEncoderEnable = true;
	}
}

void webots::DifferentialWheels::disableEncoders() {
	if(mEncoderEnable == true) {
		const int args[1] = {0};
		dashelInterface->sendEvent("enableEncoders", args, 1);
		mEncoderEnable = false;
	}
}

double webots::DifferentialWheels::getLeftEncoder() const {
	return mLeftEncoder;
}

double webots::DifferentialWheels::getRightEncoder() const {
	return mRightEncoder;
}

void webots::DifferentialWheels::batterySensorEnable(int ms) {
	
}

void webots::DifferentialWheels::batterySensorDisable() {
	
}

double webots::DifferentialWheels::batterySensorGetValue() {
	return mBatterylevel;
}

void webots::DifferentialWheels::connectToDashel() {
    dashelInterface->connectAseba("tcp:127.0.0.1;33333");
}

void webots::DifferentialWheels::setSpeed(double left, double right) {
	if((mTargetSpeed[1] != left) || (mTargetSpeed[0] != right)) {
		mTargetSpeed[0] = right;
		mTargetSpeed[1] = left;
		if(right < -100)
		  right = -100;
		else if(right > 100)
		  right = 100;
		if(left < -100)
		  left = -100;
		else if(left > 100)
		  left = 100;
		const int args[2] = {(int)right, (int)-left};
		dashelInterface->sendEvent("setSpeed", args, 2);
	}
}

void webots::DifferentialWheels::initRanger() {
    connectToDashel();
}

webots::Device *webots::DifferentialWheels::getDevice(const std::string &name) const {
  std::map<const std::string, webots::Device *>::const_iterator it = mDevices.find(name);
  if (it != mDevices.end())
    return (*it).second;
  return NULL;
}

webots::DistanceSensor *webots::DifferentialWheels::getDistanceSensor(const std::string &name) const {
  webots::Device *device = getDevice(name);
  if (device) {
    webots::DistanceSensor *distanceSensor = dynamic_cast<webots::DistanceSensor *> (device);
    if (distanceSensor)
      return distanceSensor;
  }
  return NULL;
}

webots::LED *webots::DifferentialWheels::getLED(const std::string &name) const {
  webots::Device *device = getDevice(name);
  if (device) {
    webots::LED *led = dynamic_cast<webots::LED *> (device);
    if (led)
      return led;
  }
  return NULL;
}

webots::Receiver *webots::DifferentialWheels::getReceiver(const std::string &name) const {
  webots::Device *device = getDevice(name);
  if (device) {
    webots::Receiver *receiver = dynamic_cast<webots::Receiver *> (device);
    if (receiver)
      return receiver;
  }
  return NULL;
}

webots::Accelerometer *webots::DifferentialWheels::getAccelerometer(const std::string &name) const {
  webots::Device *device = getDevice(name);
  if (device) {
    webots::Accelerometer *accelerometer = dynamic_cast<webots::Accelerometer *> (device);
    if (accelerometer)
      return accelerometer;
  }
  return NULL;
}

webots::TouchSensor *webots::DifferentialWheels::getTouchSensor(const std::string &name) const {
  webots::Device *device = getDevice(name);
  if (device) {
    webots::TouchSensor *touchsensor = dynamic_cast<webots::TouchSensor *> (device);
    if (touchsensor)
      return touchsensor;
  }
  return NULL;
}

webots::Motor *webots::DifferentialWheels::getMotor(const std::string &name) const {
  webots::Device *device = getDevice(name);
  if (device) {
    webots::Motor *motor = dynamic_cast<webots::Motor *> (device);
    if (motor)
      return motor;
  }
  return NULL;
}

webots::Emitter *webots::DifferentialWheels::getEmitter(const std::string &name) const {
  webots::Device *device = getDevice(name);
  if (device) {
    webots::Emitter *emitter = dynamic_cast<webots::Emitter *> (device);
    if (emitter)
      return emitter;
  }
  return NULL;
}

void webots::DifferentialWheels::initDevices() {
  char buffer[32];
  
  mDevices["dsBottomRight"]    = new webots::DistanceSensor("dsBottomRight", this);
  mDevices["dsBottomLeft"]     = new webots::DistanceSensor("dsBottomLeft", this);
  mDevices["dsFrontRight"]     = new webots::DistanceSensor("dsFrontRight", this);
  mDevices["dsFrontCenter"]    = new webots::DistanceSensor("dsFrontCenter", this);
  mDevices["dsFrontLeft"]      = new webots::DistanceSensor("dsFrontLeft", this);
  mDevices["LED"]              = new webots::LED("LED", this);
  mDevices["accelerometer"]    = new webots::Accelerometer("accelerometer", this);
  mDevices["infraredReceiver"] = new webots::Receiver("infraredReceiver", this);  
  mDevices["infraredEmitter"]  = new webots::Emitter("infraredEmitter", this, dashelInterface);  
  mDevices["bumper"]           = new webots::TouchSensor("bumper", this, webots::TouchSensor::BUMPER); 
  mDevices["balance"]          = new webots::TouchSensor("balance", this,webots::TouchSensor::FORCE); 
  // Touch pannel
  for(int c=0; c<9; ++c) {
	  // right pannel
	  sprintf(buffer, "touchR%d", c);
	  mDevices[buffer]          = new webots::TouchSensor(buffer, this, webots::TouchSensor::BUMPER);
	  // rear pannel
	  sprintf(buffer, "touchB%d", c);
	  mDevices[buffer]          = new webots::TouchSensor(buffer, this, webots::TouchSensor::BUMPER);
	  // left pannel
	  sprintf(buffer, "touchL%d", c);
	  mDevices[buffer]          = new webots::TouchSensor(buffer, this, webots::TouchSensor::BUMPER);
  }
  
  mDevices["RightPupilVert"]   = new webots::Motor("RightPupilVert", this, -0.028, 0.028);
  mDevices["RightPupilHori"]   = new webots::Motor("RightPupilHori", this, -0.028, 0.028);
  mDevices["LeftPupilVert"]    = new webots::Motor("LeftPupilVert", this, -0.028, 0.028);
  mDevices["LeftPupilHori"]    = new webots::Motor("LeftPupilHori", this, -0.028, 0.028);
  mDevices["LeftUpperEyelid"]  = new webots::Motor("LeftUpperEyelid", this, 0, M_PI_4);
  mDevices["LeftLowerEyelid"]  = new webots::Motor("LeftLowerEyelid", this, 0, M_PI_4);
  mDevices["RightUpperEyelid"] = new webots::Motor("RightUpperEyelid", this, 0, M_PI_4);
  mDevices["RightLowerEyelid"] = new webots::Motor("RightLowerEyelid", this, 0, M_PI_4);
  
}

void webots::DifferentialWheels::playLedVideo(int videoNumber, bool repeat) {
	mVideoIsPlaying = true;
	if(repeat) {
		const int args[2] = {videoNumber, 1};
		dashelInterface->sendEvent("playLedVid", args, 2);
	}
	else {
		const int args[2] = {videoNumber, 0};
		dashelInterface->sendEvent("playLedVid", args, 2);
	}
}

void webots::DifferentialWheels::stopLedVideo() {
	dashelInterface->sendEvent("stopLedVid", NULL, 0);
	mVideoIsPlaying = false;
}

void webots::DifferentialWheels::emitCustomEvent(int *tab) {
	const int * args = tab;
	dashelInterface->sendEvent("customEvent", args, 10);
}

            
bool webots::DifferentialWheels::isLolettePresent() {
	return mLolette;
}

bool webots::DifferentialWheels::isCharging() {
	return mIsCharging;
}

int webots::DifferentialWheels::getRightMotorCurrent() {
	return mRightCurrent;
}

int webots::DifferentialWheels::getLeftMotorCurrent() {
	return mLeftCurrent;
}
