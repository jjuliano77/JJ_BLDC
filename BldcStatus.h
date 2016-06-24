/*
  BldcStatus.h - Class for storing and managing the current status of the motor
  I may want to just re-implement this as just a namespace, but for right now my
  brain wants o think of it as a Class

  Jaimy Juliano
*/
#ifndef BldcStatus_h
#define BldcStatus_h

#include <arduino.h>

class BldcStatus{
   public:

      int getPhaseCurrent_Raw();
      int getFilteredPhaseCurrent_Raw();
      float getPhaseCurrent_Amps();
      float getFilteredPhaseCurrent_Amps();

      //Inline functions to update the phase current readings. It simply stores
      //the readings and applies the offsets. It's not 100% necessary and I'm not
      //even sure if it's the best way to handle this. It might be better to have
      //the math exposed for troubleshooting.
      void iSense1Update(int adcVal) {iSense1_raw = adcVal - iSense1_offset;}
      void iSense2Update(int adcVal) {iSense2_raw = adcVal - iSense2_offset;}

      int getFilteredBattCurrent_Raw();
      float getBattCurrent_Amps();
      //void  updateBattCurrent(int);

      int getBusVoltage_Raw();
      int getVirtGround_Raw();
      float getBusVoltage_Volts();
      void getBemfVoltage_Raw();

      //Inline functions to update the bus voltage readings. It simply stores
      //the readings
      void vBusUpdate(int adcVal)  {vBus_raw = adcVal;}
      //Inline functions to update the BEMF voltage readings. It simply stores
      //the readings and applies the offsets.
      void vBemfUpdate(int adcVal) {vBemf_raw = adcVal - (vBus_raw / 2);}

      int getFetTemp_Raw();
      //void updateFetTemp(int);
      int calcNtcRes(int);
      float getFetTemp_DegC(int);

      //Public members
      volatile int throttle;
      int lastThrottle;
      int throttle_offset  = 0;

      volatile int iSense1_raw;
      volatile int iSense2_raw;
      int iSense1_offset = 0;
      int iSense2_offset = 0;

      volatile int vBus_raw;

      volatile int vBemf_raw;

      volatile int fetTemp_raw;

   private:
     //It would be cool to track consumed power eventually
     //I'll just put these here for now so I don't forget
     int ampHrsUsed;
     elapsedMillis ampHrTimer;

     const float BUS_VOLTAGE_FACTOR = 0.015;  //Counts to bus Volts (voltage divider)
     const int   SHUNT_CURRENT_FACTOR = -100; //Its * 100 because the diff amp already has a gain of 10
                                              //So - I = (Vshunt * 10) * 100

};

//PID Settings
//IF YOU CHANGE THIS, YOU NEED TO CHANGE THE CONFIG VERSION!!!!
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
