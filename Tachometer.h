/*
  Tachometer.h

  Jaimy Juliano
*/
#ifndef Tachometer_h
#define Tachometer_h

#include <arduino.h>

class Tachometer{
  public:

    // Tachometer(volatile unsigned int*); //Take a pointer to the commutation period variable
                                        //This isn't working. GCC wont let me pass a volatile pointer
    float getRPM();
    float getMPH();
    float getERPM(); //Public for dubugging

    void setWheelCircumference(float);
    void setGearRatio(float);
    void setPolePairs(unsigned int);
    void update(unsigned int);

    //elapsedMicros _tachTimer; //Public for dubugging

  private:
    //float getERPM();

    //float _wheelDiameter;
    float _wheelCircumference = 31.4159; //Default fo 10" diameter wheel
    float _gearRatio = 4;
    unsigned int _polePairs = 6;
    unsigned int _commStep;
    unsigned int _lastCommStep;
    unsigned long _commPeriod;
    unsigned long _eRotPeriod;
    elapsedMicros _tachTimer;


    const unsigned int INCHES_IN_MILE = 63360;
    const unsigned long STOP_TIMEOUT = 50000; //stop detection timeout (in uS)

};
#endif
