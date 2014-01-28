#include "Ranger.hpp"

#include <stdio.h>

using namespace webots;

int main(int argc, char *argv[])
{
  Ranger* controller = new Ranger();
  
  while(1) {
    if(controller->hasLolette()) {
      controller->approachChargingStation();
      controller->gotToChargeStation();
    }
    else {
	  controller->goToAttractor();
	}
  }
  delete controller;
  return 0;
}
