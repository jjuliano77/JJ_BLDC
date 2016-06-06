/*
  BldcStatus.h - Class for storing and managing the current status of the motor
  I may want to just re-implement this as just a namespace, but for right now my
  brain wants o think of it as a Class

  Jaimy Juliano
*/
#ifndef BldcStatus_h
#define BldcStatus_h

#include "arduino"


class BLDCStatus
{
   public:
      BldcStatus(void);
      ~BldcStatus();

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
      void updatePhaseVoltage(uint16_t); //This will need to account for what phase we are updating (or we need 3 functions)

      uint16_t getFetTemp_Raw();
      void updateFetTemp(uint16_t);
      uint16_t calcNtcRes(uint16_t);
      float getFetTemp_DegC(uint16_t);	(1.0 / ((logf(getNtcRes(adc_val) / 10000.0) / 3434.0) + (1.0 / 298.15)) - 273.15)


      const float BUS_VOLTAGE_FACTOR = 0.015;  //Counts to bus Volts (voltage divider)
      const int   SHUNT_CURRENT_FACTOR = -100; //Its * 100 because the diff amp already has a gain of 10
                                               //So - I = (Vshunt * 10) * 100
      //Public members
      volatile uint16_t  throttle;
      volatile uint16_t lastThrottle;
      uint16_t throttle_offset  = 0;

      volatile uint16_t iSense1_raw;
      uint16_t iSense2_raw;
      uint16_t iSense1_offset;
      uint16_t iSense2_offset;

      uint16_t vBus_raw;
      uint16_t vSenseA_raw;
      uint16_t vSenseB_raw;
      uint16_t vSenseC_raw;
      uint16_t vBemf_raw;

      volatile uint16_t fetTemp_raw;

   private:
     //It would be cool to track consumed power eventually
     //I'll just put these here for now so I don't forget
     uint16_t ampHrsUsed;
     elapsedMillis ampHrTimer;

}
