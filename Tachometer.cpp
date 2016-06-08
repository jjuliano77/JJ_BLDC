/*
  Tachometer.h

  Jaimy Juliano
*/

#include "Tachometer.h"

Tachometer::Tachometer(volatile unsigned int* commStep){
  _commStep = commStep;
}

void Tachometer::update(){
  //I want to try basing it on a single commutation step
  if(_commStep != _lastcommStep){
    _commPeriod = _tachTimer;
    _tachTimer = 0;
    _lastcommStep = _commStep;
  }else if(tachTimer > STOP_TIMEOUT){
    _commPeriod = 999999;
    _tachTimer = 0;
  }
}

void Tachometer::setWheelDiameter(float wheelDiameter){
  _wheelDiameter = wheelDiameter;
  _wheelCircumference = wheelDiameter * 3.14159; //Just do this too while we are at it
}

void Tachometer::setGearRatio(float gearRatio){
  _gearRatio = gearRatio;
}

void Tachometer::setPolePairs(unsigned int polePairs){
  _polePairs = polePairs;
}

// Get Electrical RPM Of The Motor
float Tachometer::getERPM(){

  noInterrupts(); //Just in case
  _eRotPeriod = _commPeriod * _polePairs;
  interupts();

  if(eRotPeriod > 10) //some small threshold
    return (((1 / (double) _eRotPeriod) * 1000000) * 60);

  return 0;
}

// Calculate RPM
float Tachometer::getRPM(){
  return getERPM() / _polePairs;
}

// Calculate MPH Based On RPM, Gear Ratio, and Tire Dia
float Tachometer::getMPH(){

  return ((getRPM()/_gearRatio) * _wheelCircumference * 60) / INCHES_IN_MILE;

}
