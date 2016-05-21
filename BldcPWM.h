/*
  BldcPWM.h - Library for managing PWM generation for BLDC motor
  control with the Teensy 3.1
  
  Jaimy Juliano
*/
#ifndef BldcPWM_h
#define BldcPWM_h

//#include <kenetis.h>
//#include <mk20dx128.h>
#include <Arduino.h>

//#define FTM0_CLK_PRESCALE 1          // FTM0 Prescaler (The actual prescale factor, not the bit setting)
#define DTPS_SETTING        0          // Deadtime prescaler, I think hard coded prescale of 1 should be fine (max 1.3uS)
#define MIN_PWM_FREQ        6000       // Min allowed frequency (picked arbitrarily for now)
#define MAX_PWM_FREQ        16000      // Max allowed frequency (picked arbitrarily for now)
#define PWM_OUT_RESOLUTION  12         // Output resolution, equivalent to analogWriteResolution
#define PWM_COUNTS          4096
//#define  MIN_DUTY  0.05
//#define  MAX_DUTY  0.95              //95%        
//#define  MAX_DUTY_COUNTS  (2 ^ PWM_OUT_RESOLUTION) * MAX_DUTY //This might come in handy

//PWM Mode
const uint8_t   PWM_INDEPENDANT = 0;
const uint8_t   PWM_UNIPOLAR    = 1;
const uint8_t   PWM_BIPOLAR     = 2;
const uint8_t   PWM_SINE        = 3;

//PWM Alignment
const boolean   PWM_CENTER  = 0;
const boolean   PWM_EDGE    = 1;

class BldcPWM
{
  public:
    BldcPWM(uint8_t mode, boolean alignment, uint16_t PWMfreq, uint8_t deadTime);    //Constructor
    
    void setPwmFreq(uint16_t pwmFreq);              //Frequency in Hz
    void setDeadTime(uint8_t deadTime);             //Deadtime in F_BUS ticks (1 tick = 20.83nS)
    void setAlignment(boolean alignment);           //Set PWM alignment (CENTER or EDGE)
    
    void setDutyCycle(uint16_t dutyCycle);                           //Duty cycle for all phases
    void setDutyCycle(uint16_t aDuty, uint16_t bDuty, uint16_t cDuty); //Duty cycle for each phase
    void setDutyCycleFloat(float aDuty, float bDuty, float cDuty);       //Duty cycle for each phase
    
    void setPwmMode(uint8_t mode);                  //Set the PWM operating mode
//    void pwmDisable();                              //Force all outputs LOW
//    void pwmEnable();                               //Allow PWM output again
    void pwmOUTMASK(uint8_t mask);                  //Directly apply an output mask to FTM0
    void pwmSWOCTRL(uint16_t mask);                 //Directly apply SWOCTRL to FTM0 outputs
    
  private:  
    uint8_t  _mode;
    boolean  _alignment;
};
#endif
