#ifndef OpenRC_H
#define OpenRC_H

#include <Arduino.h>


class OpenRC
{
  public:
  OpenRC();

  void InitESPNow();
  void Calibration();
  uint AdctoAngle (uint adcport);
  uint IOtoAngle (uint adcport);
  uint calibration[6];
  uint direction[6];
  uint dualrate[6];
  int trim[6];
  uint8_t adcpins[6] = {36,39,34,35,32,33}; 
  uint8_t digipins[8] = {4,13,12,27,15,32,14,18};
  private:



};


#endif