/*
  BldcStatus.h - Class for storing and managing the current status of the motor
  I may want to just re-implement this as just a namespace, but for right now my
  brain wants o think of it as a Class

  Jaimy Juliano
*/
#ifndef BldcStatus_h
#define BldcStatus_h

#include <arduino.h>
#include "RunningAverage.h"
#include "defaults.h"

class BldcStatus{
   public:
      BldcStatus():fetTemp_RA(RUNNING_AVG_BLOCK_SIZE), motorCurrent_RA(RUNNING_AVG_BLOCK_SIZE){}
      int getMotorCurrent_Raw() {return motorCurrent_raw;}
      int getFilteredMotorCurrent_Raw();
      float getMotorCurrent();
      float getFilteredMotorCurrent();

      int getBusCurrent_Raw();
      int getFilteredBusCurrent_Raw();
      float getBusCurrent();
      float getFilteredBusCurrent();

      int getBusVoltage_Raw();
      int getFilteredBusVoltage_Raw();
      float getBusVoltage();
      void getBemfVoltage_Raw();

      int getFetTemp_Raw();
      float getFetTemp_DegC();
      int getFilteredFetTemp_Raw();
      float getFilteredFetTemp_DegC();

      int getThrottle() {return throttle;}
      int getFilteredThrottle();

      void setThrottleOffset(int); //Implement these later if needed, just set public members for now
      void setCurrentOffsets(int, int);   //Implement these later if needed, just set public members for now

      //Inline functions to update the readings. They simply store the readings
      //and apply offsets where needed. It's not 100% necessary and I'm not
      //even sure if it's the best way to handle this. It might be better to have
      //the math exposed for troubleshooting.
      void iSense1Update(int adcVal) {iSense1_raw = adcVal - iSense1_offset;}
      void iSense2Update(int adcVal) {iSense2_raw = adcVal - iSense2_offset;}

      void motorCurrentUpdate(int adcVal) {motorCurrent_raw = adcVal; motorCurrent_RA.addValue(adcVal);}

      void vBusUpdate(int adcVal)  {vBus_raw = adcVal;}
      void vBemfUpdate(int adcVal) {vBemf_raw = adcVal - (vBus_raw / 2);}
      void fetTempUpdate(int adcVal) {fetTemp_raw = adcVal; fetTemp_RA.addValue(adcVal);}
      void throttleUpdate(int adcVal) {
        lastThrottle = throttle;
        throttle = adcVal - throttle_offset;
        //I'm pretty sure this could be handled better!!!
        if(throttle > ADC_MAX_VALUE){
          throttle = 0; //we must have gone less than 0 and wrapped around
        }
      }

      //Public members
      int throttle_offset  = 0;
      int iSense1_offset = 0;
      int iSense2_offset = 0;

      volatile int iSense1_raw;
      volatile int iSense2_raw;
      volatile int motorCurrent_raw;
      volatile int vBus_raw;
      volatile int vBemf_raw;
      volatile int fetTemp_raw;
      volatile unsigned int throttle;

      unsigned int lastThrottle;

      RunningAverage fetTemp_RA;
      RunningAverage motorCurrent_RA;

   private:
     int calcNtcRes(int);
     float countsToVolts(float counts) {return (counts*3.3)/ADC_MAX_VALUE;}
     int voltsToCounts(float volts) {return (volts * ADC_MAX_VALUE) / 3.3;}

    //  volatile int iSense1_raw;
    //  volatile int iSense2_raw;
     //It would be cool to track consumed power eventually
     //I'll just put these here for now so I don't forget
     int ampHrsUsed;
     elapsedMillis ampHrTimer;
    //  RunningAverage fetTemp_RA;
    //  RunningAverage motorCurrent_RA;
     const int   ADC_MAX_VALUE = 4096;
     const float BUS_VOLTAGE_FACTOR = 0.015;  //Counts to bus Volts (voltage divider)
     const float SHUNT_CURRENT_FACTOR = 100.0; //Its * 100 because the diff amp already has a gain of 10
                                              //So - I = (Vshunt * 10) * 100

};

//Brainstorming a meter class that could be re-used for all measurments
class BldcMeter{
  volatile int _rawValue;
  int   _offset = 0;
  float _scaleFactor = 1;
  char  *_units;
  bool _isThermistor;

  RunningAverage filteredValue;    //I'm just going to do a running average using the class for now

  float calcNtcTemperature(int);        //Just for thermisor measurements


  public:
    BldcMeter(int samplesToAverage, char *units, bool isThermistor);

    int   getRawValue(void) {return _rawValue;}
    int   getRawValueFiltered(void);
    float getScaledValue(void);
    float getScaledValueFiltered(void);

    float getScaleFactor(void) {return _scaleFactor;}
    int   getOffset(void) {return _offset;}

    void  setOffset(int);
    void  setScaleFactor(float);

    void  update(int adcVal) {
      _rawValue = adcVal - _offset;       //Do the update inline as it is likely to be used inside an ISR
      filteredValue.addValue(_rawValue);  //Add value to the running average buffer too
    }
};

////////////////////////////////////////////////////////
// Buffer Class to hold triggered real time aquisition
// ultimately, I might want to pull the running average
// out of this buffer as well. not sure yet
class acquisitionBuffer{
  unsigned int _size;
  unsigned int _idx;
  unsigned int _postTriggerSamples;
  bool _isTriggered;
  enum state {
    preTrigger = 0,
    postTrigger,
    complete
  };
  state acquisitionState = preTrigger;
  int * _buffer;

public:
  acquisitionBuffer(int);
  ~acquisitionBuffer(void);

  void addSample(int);
  int  getAverage(int); //wont use this, just keeping in for referance now
  int  getSample(unsigned int);
  void trigger(void);
  void arm(void);
  bool samplesReady(void);
  unsigned int bufferSize(void){return _size;}
  // void setTriggerMode(int mode); //software, Internal value, External value
  // void setTriggerInput(int *trigInput);
  // void setTriggerThreshold(int trigThreshold);
  void clear(void);
};

//PID Settings !!!NOT CURRENTLY USING THESE!!!
//IF YOU CHANGE THESE, YOU NEED TO CHANGE THE CONFIG VERSION!!!!
typedef struct {
  float p;
  float i;
  float d;
} mc_pidSettings;

//Config data to be stored in EEPROM
typedef struct {
  //IF YOU CHANGE THIS, YOU NEED TO CHANGE THE CONFIG VERSION!!!!
  uint16_t throttleOut_max;    //0-4096
  uint16_t throttleOut_min;    //0-4096
  uint16_t dutyCycle_max;      //0-4096
  uint16_t dutyCycle_min;      //0-4096
  uint16_t polePairs;
  uint16_t pwmOutFreq;         //Hz
  uint8_t controlMode;
  //PID params
  // float currentControl_kP;
  // float currentControl_kI;
  // float currentControl_kD;
  mc_pidSettings currentPid;

  //Version info
  float configVersion;  //We can check that this matches to see if data has been written before

} mc_configData_NEW;
//IF YOU CHANGE THIS, YOU NEED TO CHANGE THE CONFIG VERSION!!!!
//stand alone limit Struct
//Limit config data to be stored in EEPROM
 typedef struct {
  float maxCurrent_HW;    //0-4096
  float maxCurrent_motor; //0-4096
  float maxCurrent_batt;  //0-4096
  float maxCurrent_regen; //0-4096
  float maxBusVoltage;    //0-4096
  float minBusVoltage;    //0-4096
  float maxTemp_FETs;     //degC
} mc_limits_NEW;

#endif
