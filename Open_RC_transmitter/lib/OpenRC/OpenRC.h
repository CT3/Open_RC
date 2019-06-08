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
  uint calibration[6]= {1800,1800,1850,1850,1800,1800};
  uint direction[6] = {0,0,1,1,1,0};
  uint dualrate[6]= {0,0,0,0,0,0};
  int trim[6];
  //uint8_t adcpins[6] = {34,35,32,33,36,39}; 
  uint8_t adcpins[6] = {32,33,34,35,36,39}; 
  uint8_t digipins[8] = {14,12,19,18,15,17,16,13};
  private:



};


#endif