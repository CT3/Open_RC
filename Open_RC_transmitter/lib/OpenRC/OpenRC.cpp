#include <arduino.h>
#include "OpenRC.h"


OpenRC::OpenRC(){
}

void OpenRC::Calibration(){
  
int subtotal = 0;
  for (int x = 0; x<6 ; x++){
        for (int i = 0; i<20; i++){
        subtotal = subtotal  + analogRead(adcpins[x]);
        delay(50);
         }

calibration[x] = subtotal /20;
//Serial.println(calibration[x]);
subtotal = 0;
delay(50);
  }

}

uint OpenRC::AdctoAngle (uint adcport){ ///0-5ass showed in adcpins[]

  uint adc = analogRead(adcpins[adcport]);
  //Serial.println(adc);
  int val= 0;
  uint cal = calibration[adcport] ;
  if (direction[adcport] == 0) {// 0= direction not reverced
    if (adc < cal){
  val = map (adc,0,cal,0+dualrate[adcport],89+trim[adcport]);
    }
   else {
 val = map (adc,cal,4095,89+trim[adcport],180-dualrate[adcport]);
    }
  }

  if (direction[adcport] > 0){ // direction reversed
    if (adc < cal){
  val = map (adc,0,cal,180-dualrate[adcport],90+trim[adcport]);
    }
    else {
 val = map (adc,cal,4095,90+trim[adcport],0+dualrate[adcport]);
    }
  }
//return direction
return val;
}


uint OpenRC::IOtoAngle (uint ioport){ 
uint IOin = digitalRead(digipins[ioport]);
  int val= 0;
  if (IOin == HIGH){
    val = 90;
  }

  if (IOin == LOW){
    val = 180;
  }

//return direction
return val;

}
