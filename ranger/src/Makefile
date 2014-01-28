###############################################################
#
# Purpose: Makefile for building a wrapper between the Webots
#          API and the Ranger real robot framework
# Author : david.mansolino@epfl.ch
# Date   : December 2013
#
###############################################################

# -------------------------------------------------------------

# pathes
DASHEL_INCLUDE = /home/david/aseba/dashel

# -------------------------------------------------------------

TARGET = rangerWrapper.a

CXX_SOURCES = \
  ./dashelinterface.cpp \
  ./Accelerometer.cpp \
  ./DifferentialWheels.cpp \
  ./DistanceSensor.cpp \
  ./LED.cpp \
  ./Packet.cpp \
  ./Receiver.cpp \
  ./Emitter.cpp \
  ./Motor.cpp \
  ./TouchSensor.cpp

OBJECTS = $(CXX_SOURCES:.cpp=.o)
INCLUDE_DIRS = -I$(DASHEL_INCLUDE) -I../include

AR = ar
ARFLAGS = rv
CXX = g++
CXXFLAGS += -c -O2 -DLINUX -Wall $(INCLUDE_DIRS)

all: $(TARGET)

clean:
	rm -f $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	$(AR) $(ARFLAGS) $(TARGET) $(OBJECTS) $(LIBS) $(LINK_DEPENDENCIES)
	chmod 755 $(TARGET)
