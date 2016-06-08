/*
  Tachometer.h

  Jaimy Juliano
*/
#ifndef Tachometer_h
#define Tachometer_h

#include <arduino.h>

class Tachometer{
public:

  Tachometer(volatile unsigned int*); //Take a pointer to the commutation period variable
  ~Tachometer();

  float getRPM();
  float getMPH();

  void setWheelDiameter(float);
  void setGearRatio(float);
  void setPolePairs(unsigned int);
  void update();

private:
  float getERPM();

  float _wheelDiameter;
  float _wheelCircumference; //I'll just set this too in the setWheelDiameter() member
  float _gearRatio;
  unsigned int _polePairs;
  unsigned int* _commStep;
  unsigned int _lastCommStep;
  unsigned long _commPeriod;
  unsigned long _eRotPeriod;
  elapsedMicros _tachTimer;


  const unsigned int INCHES_IN_MILE = 63360;
  const unsigned long STOP_TIMEOUT = 50000; //stop detection timeout (in uS)

};
#endif
