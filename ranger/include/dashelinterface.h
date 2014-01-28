/*******************************************************************************************************/
/* File:         dashelinterface.h                                                                     */
/* Date:         Jul 2013                                                                              */
/* Description:  Dashel interface for communicating with Ranger robot Aseba nodes                      */
/*               and interfacing it with Webots c++ API                                                */
/* Author:       david.mansolino@epfl.ch                                                               */
/*******************************************************************************************************/

#ifndef DASHELINTERFACE_H
#define DASHELINTERFACE_H

#include <pthread.h>

#include <vector>

#include <dashel/dashel.h>
#include "../common/msg/msg.h"
#include "../common/msg/descriptions-manager.h"

struct packet {
  int id;
  int dist;
  int angle;
  int data[7];
};

class DashelInterface : public Dashel::Hub, public Aseba::DescriptionsManager
{
  public:
    typedef std::map<std::string, Aseba::Compiler::VariablesMap> NodeNameToVariablesMap;

    DashelInterface();
    ~DashelInterface();
    void connectAseba(const char * target);
    void disconnectAseba();
    void dashelConnect();
    void dashelRun();
    bool dashelConnected();
    bool getNodeAndVarPos(const std::string& nodeName, const std::string& variableName, unsigned& nodeId, unsigned& pos) const;

    void sendEvent(const char * event, const int * args, int argsNumber);

    int * getSharpValues();
    int * getAccelerometerValues();
    int * getEncodersValues();
    int   getBatteryLevel();
    int   getBalance();
    int   getBumper();
    int * getSpeed();
    int * getTouchSensors();
    int   getLolette();
    int   getChargingState();
    int * getMotorsCurrent();

    std::vector<packet> * getPackets();
    void cleanPackets();

  protected:
    Aseba::CommonDefinitions commonDefinitions;
    NodeNameToVariablesMap allVariables;

    virtual void nodeDescriptionReceived(unsigned nodeId);

    // to be called by pthread
    static void * run(void *param);

    // from Dashel::Hub
    virtual void incomingData(Dashel::Stream *stream);
    virtual void connectionClosed(Dashel::Stream *stream, bool abnormal);

    // members
    Dashel::Stream* stream;
    const char * dashelParams;
    bool isRunning;
    bool isConnected;

    int                 mSharpValues[5];
    int                 mAccelValues[3];
    int                 mEncoders[4];
    int                 mBattery;
    int                 mBalance;
    int                 mBumper;
    int                 mSpeed[2];
    int                 mTouch[27];
    int                 mMotorsCurrent[2];
    int                 mLolette;
    int                 mChargingState;
    std::vector<packet> mPackets;
    pthread_t           mDashelThread;
        
    void decompressTouch(int right, int left, int rear);
};



#endif // DASHELINTERFACE_H
