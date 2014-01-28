/*******************************************************************************************************/
/* File:         Packet.hpp                                                                            */
/* Date:         jul 2013                                                                              */
/* Description:  Implementation of a receiver packet                                                   */
/* Author:       david.mansolino@epfl.ch                                                               */
/*******************************************************************************************************/

#ifndef PACKET_HPP
#define PACKET_HPP


namespace webots {
  class Packet  {
    public:
                    Packet(int id, int distance, int angle, int * data);
      virtual      ~Packet();
      
      // getter
      const double* getEmitterDirection() const;
      double        getSignalStrength() const;
      int           getEmitterID() const;
      const void *  getPacketData() const;
      
      void          resetPacket();

    private:
      double        mEmitterDirection[3];
      double        mSignalStrength;
      int           mEmitterID;
      int           mData[8];
  };
}

#endif // PACKET_HPP
