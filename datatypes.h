//Current dumping ground for potentially useful datatypes. Needs to be
//cleaned up if used at all.

// Data types
// Originally idea came from https://github.com/vedderb/bldc/blob/master/datatypes.h
// but hese have been changed quite a bit now


typedef enum {
   FB_MODE_HALL = 0,
   FB_MODE_ENC,
   FB_MODE_BEMF
} mc_fb_mode;

typedef enum {
   MC_STATE_INIT = 0,
   MC_STATE_OFF,
   MC_STATE_ALIGNMENT,
   MC_STATE_START,
   MC_STATE_STOPPED,
   MC_STATE_DRIVE,
   MC_STATE_COAST,
   MC_STATE_BRAKE,
   MC_STATE_FULL_BRAKE,
   MC_STATE_FAULT
} mc_state;

typedef enum {
	CONTROL_MODE_DUTY = 0,
	CONTROL_MODE_SPEED,
	CONTROL_MODE_CURRENT,
} mc_control_mode;

//Config data to be stored in EEPROM
typedef struct {
  //misc
  uint16_t throttleOut_max;    //0-4096
  uint16_t throttleOut_min;    //0-4096
  uint16_t dutyCycle_max;      //0-4096
  uint16_t dutyCycle_min;      //0-4096
  uint16_t polePairs;
  uint16_t pwmOutFreq;         //Hz
  mc_control_mode controlMode;
  //PID params
  float currentControl_kP;
  float currentControl_kI;
  float currentControl_kD;
  //Limits
  int drvOcLimit;       //0-4096
  int motorOcLimit;     //0-4096
  int maxCurrent_motor; //0-4096
  int maxCurrent_batt;  //0-4096
  int maxCurrent_regen; //0-4096
  int maxBusVoltage;    //0-4096
  int minBusVoltage;    //0-4096
  int maxFetTemp;     	 //degC
  //Version info
  float configVersion;  //We can check that this matches to see if data has been written before
} mc_configData;

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
} mc_limits;
