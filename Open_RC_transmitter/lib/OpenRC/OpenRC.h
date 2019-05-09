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
  uint calibration[16];
  uint direction[16];
  uint dualrate[16];
  int trim[16];
  uint8_t adcpins[16] = {39,36,4}; 
  private:



};


#endif