/*
  Tachometer.h

  Jaimy Juliano
*/

#include "Tachometer.h"

// Tachometer::Tachometer(volatile unsigned int* commStep){
//   _commStep = commStep;
// }

void Tachometer::update(unsigned int commStep){
  _commStep = commStep;
  //I want to try basing it on a single commutation step
  if(_commStep != _lastCommStep){
    _commPeriod = _tachTimer;
    _tachTimer = 0;
    _lastCommStep = _commStep;
  }else if(_tachTimer > STOP_TIMEOUT){
    _commPeriod = 999999;
    _tachTimer = 0;
  }
}

void Tachometer::setWheelCircumference(float wheelCircumference){
  _wheelCircumference = wheelCircumference; //Just do this too while we are at it
}

void Tachometer::setGearRatio(float gearRatio){
  _gearRatio = gearRatio;
}

void Tachometer::setPolePairs(unsigned int polePairs){
  _polePairs = polePairs;
}

// Get Electrical RPM Of The Motor
float Tachometer::getERPM(){

  _eRotPeriod = _commPeriod * _polePairs;

  if(_eRotPeriod > 10){ //some small threshold
    return (((1 / (double) _eRotPeriod) * 1000000) * 60);
  }else{
    return 0;
  }
}

// Calculate RPM
float Tachometer::getRPM(){
  return getERPM() / _polePairs;
}

// Calculate MPH Based On RPM, Gear Ratio, and Tire Dia
float Tachometer::getMPH(){

  return ((getRPM()/_gearRatio) * _wheelCircumference * 60) / INCHES_IN_MILE;

}
