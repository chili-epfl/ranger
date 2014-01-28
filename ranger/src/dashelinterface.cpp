#include "dashelinterface.h"

#include <stdio.h>
#include <unistd.h>

using namespace std;
using namespace Dashel;
using namespace Aseba;

// UTF8 to wstring
static std::wstring widen(const char *src) {
  const size_t destSize(mbstowcs(0, src, 0)+1);
  std::vector<wchar_t> buffer(destSize, 0);
  mbstowcs(&buffer[0], src, destSize);
  return std::wstring(buffer.begin(), buffer.end() - 1);
}

static std::wstring widen(const std::string& src) {
  return widen(src.c_str());
}

DashelInterface::DashelInterface() :
	isRunning(false), isConnected(false)
{
  commonDefinitions.events.push_back(NamedValue(widen("mainFeedback"), 15)); // acc (3) + prox (2) + battery (1) + bumber (1) + speed (2) + touchCompressed (3) + chargingState (1) + motorCurrent (2)
  commonDefinitions.events.push_back(NamedValue(widen("mainFeedbackWithEncoders"), 19)); // acc (3) + prox (2) + battery (1) + bumber (1) + speed (2) + touchCompressed (3) + encoders (4) + chargingState (1) + motorCurrent (2)
  commonDefinitions.events.push_back(NamedValue(widen("setLed"), 3));
  commonDefinitions.events.push_back(NamedValue(widen("receiverFeedback"), 10)); // source ID (1) + angle (1) + dist (1) + datas (7)
  commonDefinitions.events.push_back(NamedValue(widen("setSpeed"), 2));
  commonDefinitions.events.push_back(NamedValue(widen("enableEncoders"), 1));
  commonDefinitions.events.push_back(NamedValue(widen("playLedVid"), 2));
  commonDefinitions.events.push_back(NamedValue(widen("stopLedVid"), 0));
  commonDefinitions.events.push_back(NamedValue(widen("neuilFeedback"), 5));  // prox (3) + lolette (1) + balance (1)
  commonDefinitions.events.push_back(NamedValue(widen("neuilEvent"), 8)); // Motors positions (8, one per motor)
  commonDefinitions.events.push_back(NamedValue(widen("customEvent"), 10));
  commonDefinitions.events.push_back(NamedValue(widen("emitterEvent"), 7));
  commonDefinitions.events.push_back(NamedValue(widen("enableFeedback"), 1));
}

DashelInterface::~DashelInterface() {
}

bool DashelInterface::getNodeAndVarPos(const string& nodeName, const string& variableName, unsigned& nodeId, unsigned& pos) const {
  // make sure the node exists
  bool ok;
  nodeId = getNodeId(widen(nodeName), 0, &ok);
  if (!ok) {
    fprintf(stderr, "invalid node name\n");
    return false;
  }
  pos = unsigned(-1);

  // check whether variable is knwon from a compilation, if so, get position
  const NodeNameToVariablesMap::const_iterator allVarMapIt(allVariables.find(nodeName));
  if (allVarMapIt != allVariables.end()) {
    const Compiler::VariablesMap& varMap(allVarMapIt->second);
    const Compiler::VariablesMap::const_iterator varIt(varMap.find(widen(variableName)));
    if (varIt != varMap.end())
      pos = varIt->second.first;
  }

  // if variable is not user-defined, check whether it is provided by this node
  if (pos == unsigned(-1)) {
    bool ok;
    pos = getVariablePos(nodeId, widen(variableName), &ok);
    if (!ok) {
      fprintf(stderr, "variable ... does not exists in node\n");
      return false;
    }
  }
  return true;
}

// Connect to any kind of valid Dashel target (TCP, serial, CAN,...)
void DashelInterface::connectAseba(const char * target) {
  dashelParams = target;
  isRunning = true;

  int error = 0;

  // create and start the thread
  if((error = pthread_create(&this->mDashelThread, NULL, this->run, this)) !=  0) {
    fprintf(stderr, "Error while starting dashel thread\n");
    exit(-1);
  }
}

// Cleanly disconnect
void DashelInterface::disconnectAseba() {
  isRunning = false;
  Dashel::Hub::stop();
  wait();
}

void DashelInterface::sendEvent(const char * event, const int * args, int argsNumber) {
  size_t pos;
  if (!commonDefinitions.events.contains(widen(event), &pos)) {
    fprintf(stderr, "event %s is unknown\n", event);
    return;
  }

  // build event and emit
  UserMessage::DataVector data;
  for (int i=0; i<argsNumber; ++i)
    data.push_back(args[i]);
  UserMessage userMessage(pos, data);
  userMessage.serialize(stream);
  stream->flush();
}

// Dashel connection was closed
void DashelInterface::connectionClosed(Dashel::Stream* stream, bool abnormal) {
  this->stream = 0;
}

void DashelInterface::nodeDescriptionReceived(unsigned nodeId) {
  //cout << '\r';
  //cout << "Received description for node ";// << getNodeName(nodeId);
  //cout << "> ";// << widen(curShellCmd);
}

void DashelInterface::incomingData(Dashel::Stream *stream) {
  // receive message
  Message *message = 0;
  try {
    // deserialize message using Aseba::Message::receive() static function
    message = Message::receive(stream);
  }
  catch (DashelException e) {
    // if this stream has a problem, ignore it for now,
    // and let Hub call connectionClosed later.
    fprintf(stderr, "error while receiving message\n");
    return;
  }

  // pass message to description manager, which builds
  // the node descriptions in background
  DescriptionsManager::processMessage(message);

  // Event received
  const UserMessage *userMessage(dynamic_cast<UserMessage *>(message));
  if (userMessage) {
    // get Neuil feedback ( prox(3) + lolette (1) + balance)
    if(userMessage->type < commonDefinitions.events.size() && commonDefinitions.events[userMessage->type].name.c_str() == widen("neuilFeedback")) {
      if(userMessage->data.size() >= 5) {
        mSharpValues[2]     = userMessage->data[0];
        mSharpValues[3]     = userMessage->data[1];
        mSharpValues[4]     = userMessage->data[2];
        mLolette            = userMessage->data[3];
        mBalance            = userMessage->data[4];
      }
      else
        fprintf(stderr, "Incomplete event 'neuilFeedback' received (only %d arguments instead of 5)\n", userMessage->data.size());
    }
    // get main feedback ( acc (3) + prox (2) + battery (1) + balance (2) + bumber (1) + speed (2) + touchCompressed (3) + chargingState (1) + motorCurrent (2))
    else if(userMessage->type < commonDefinitions.events.size() && commonDefinitions.events[userMessage->type].name.c_str() == widen("mainFeedback")) {
      if(userMessage->data.size() >= 15) {
        mAccelValues[0]     = userMessage->data[0];
        mAccelValues[1]     = userMessage->data[1];
        mAccelValues[2]     = userMessage->data[2];
        mSharpValues[0]     = userMessage->data[3];
        mSharpValues[1]     = userMessage->data[4];
        mBattery            = userMessage->data[5];
        mBumper             = userMessage->data[6];
        mSpeed[0]           = userMessage->data[7];
        mSpeed[1]           = userMessage->data[8];
        decompressTouch(userMessage->data[9], userMessage->data[10], userMessage->data[11]);
        mChargingState      = userMessage->data[12];
        mMotorsCurrent[0] = userMessage->data[13];
        mMotorsCurrent[1] = userMessage->data[14];
      }
      else
        fprintf(stderr, "Incomplete event 'mainFeedback' received (only %d arguments instead of 15)\n", userMessage->data.size());
    }
    // get main feedback with wheel encoder ( acc (3) + prox (2) + battery (1) + balance (2) + bumber (1) + speed (2) + touchCompressed (3) + encoderStates (4) + chargingState (1) + motorCurrent (2))
    else if(userMessage->type < commonDefinitions.events.size() && commonDefinitions.events[userMessage->type].name.c_str()== widen("mainFeedbackWithEncoders")) {
      if(userMessage->data.size() >= 19) {
        mAccelValues[0]     = userMessage->data[0];
        mAccelValues[1]     = userMessage->data[1];
        mAccelValues[2]     = userMessage->data[2];
        mSharpValues[0]     = userMessage->data[3];
        mSharpValues[1]     = userMessage->data[4];
        mBattery            = userMessage->data[5];
        mBumper             = userMessage->data[6];
        mSpeed[0]           = userMessage->data[7];
        mSpeed[1]           = userMessage->data[8];
        decompressTouch(userMessage->data[9], userMessage->data[10], userMessage->data[11]);
        for(int c=0; c<4; c++) 
          mEncoders[c]      = userMessage->data[c+12];
        mChargingState      = userMessage->data[16];
        mMotorsCurrent[0]   = userMessage->data[17];
        mMotorsCurrent[1]   = userMessage->data[18];
      }
      else
        fprintf(stderr, "Incomplete event 'mainFeedbackWithEncoders' received (only %d arguments instead of 19)\n", userMessage->data.size());
    }
    // get receiver id (1) + dist (1) + angle (1) + data (7)
    else if (userMessage->type < commonDefinitions.events.size() && commonDefinitions.events[userMessage->type].name.c_str() == widen("receiverFeedback")) {
      if(userMessage->data.size() >= 10) {
        struct packet receive;
        receive.id = userMessage->data[0];
        receive.dist = userMessage->data[2];
        receive.angle = userMessage->data[1];
        for(int c=0; c<7; ++c)
          receive.data[c] = userMessage->data[c+3];
        mPackets.push_back(receive);
      }
      else
        fprintf(stderr, "Incomplete event 'receiverFeedback' received (only %d arguments instead of 10)\n", userMessage->data.size());
    }
    // other event (unused)
    /*else {
      for (size_t i = 0; i < userMessage->data.size(); ++i)
        event.append(" " + QString::number(userMessage->data[i]));
    }*/
  }
  delete message;
}

bool DashelInterface::dashelConnected(void) {
  return isConnected;
}

void DashelInterface::dashelConnect(void) {
  stream = Dashel::Hub::connect(dashelParams);
}

void DashelInterface::dashelRun(void) {
  Dashel::Hub::run();
}

// internals
void * DashelInterface::run(void *param) {
  DashelInterface * instance = ((DashelInterface*)param);
  while (1) {
    try {
      instance->dashelConnect();
      printf("Connected to target: %s\n", instance->dashelParams);
      instance->isConnected = true;
      GetDescription getDescription;
      getDescription.serialize(instance->stream);
      instance->stream->flush();
      break;
    }
    catch (Dashel::DashelException e) {
      fprintf(stderr, "Cannot connect to target: %s\n", instance->dashelParams);
      instance->isConnected = false;
      sleep(1000000L);	// 1s
    }

  }

  while (instance->isRunning)
    instance->dashelRun();
		
  return NULL;
}

int * DashelInterface::getSharpValues() {
  return mSharpValues;
}

int * DashelInterface::getAccelerometerValues() {
  return mAccelValues;
}

int * DashelInterface::getEncodersValues() {
  return mEncoders;
}

int DashelInterface::getBatteryLevel() {
  return mBattery;
}

int DashelInterface::getBalance() {
  return mBalance;
}

int DashelInterface::getBumper() {
  return mBumper;
}

int * DashelInterface::getSpeed() {
  return mSpeed;
}

int * DashelInterface::getTouchSensors() {
  return mTouch;
}

int DashelInterface::getLolette() {
  return mLolette;
}

int DashelInterface::getChargingState() {
  return mChargingState;
}

int * DashelInterface::getMotorsCurrent() {
  return mMotorsCurrent;
}

void DashelInterface::decompressTouch(int right, int left, int rear) {
  int i = 1;
  
  for(int c=0; c<9; ++c) {
    mTouch[c]    = (right/i) & 0x1;
    mTouch[c+9]  = (rear/i) & 0x1;
    mTouch[c+18] = (left/i) & 0x1;
    i *= 2;
  }
}

vector<packet> * DashelInterface::getPackets() {
  if(mPackets.size() > 0)
    return &mPackets;
  else
    return NULL;
}

void DashelInterface::cleanPackets() {
  while(mPackets.size() > 0)
    mPackets.pop_back();
}
