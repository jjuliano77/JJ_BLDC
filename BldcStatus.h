/*
  BldcStatus.h - Class for storing and managing the current status of the motor
  I may want to just re-implement this as just a namespace, but for right now my
  brain wants o think of it as a Class

  Jaimy Juliano
*/
#ifndef BldcStatus_h
#define BldcStatus_h

#include <arduino.h>

class BldcStatus
{
   public:
      BldcStatus(void);
      //~BldcStatus();

      uint16_t getPhaseCurrent_Raw();
      uint16_t getFilteredPhaseCurrent_Raw();
      float getPhaseCurrent_Amps();
      float getFilteredPhaseCurrent_Amps();
      void  updatePhaseCurrents(uint16_t, uint16_t); //just always do both or do update based on current comm step?

      uint16_t getFilteredBattCurrent_Raw();
      float getBattCurrent_Amps();
      void  updateBattCurrent(uint16_t);

      uint16_t getBusVoltage_Raw();
      uint16_t getVirtGround_Raw();
      float getBusVoltage_Volts();
      void updateBusVoltage(uint16_t);
      void updateBemfVoltage(uint16_t);
      void getBemfVoltage_Raw();

      uint16_t getFetTemp_Raw();
      void updateFetTemp(uint16_t);
      uint16_t calcNtcRes(uint16_t);
      float getFetTemp_DegC(uint16_t);

      //Public members
      static volatile uint16_t  throttle;
      uint16_t lastThrottle;
      uint16_t throttle_offset  = 0;

      static volatile uint16_t iSense1_raw;
      static volatile uint16_t iSense2_raw;
      uint16_t iSense1_offset = 0;
      uint16_t iSense2_offset = 0;

      static volatile uint16_t vBus_raw;

      static volatile uint16_t vBemf_raw;

      static volatile uint16_t fetTemp_raw;

   private:
     //It would be cool to track consumed power eventually
     //I'll just put these here for now so I don't forget
     uint16_t ampHrsUsed;
     elapsedMillis ampHrTimer;

     const float BUS_VOLTAGE_FACTOR = 0.015;  //Counts to bus Volts (voltage divider)
     const int   SHUNT_CURRENT_FACTOR = -100; //Its * 100 because the diff amp already has a gain of 10
                                              //So - I = (Vshunt * 10) * 100

};
#endif
