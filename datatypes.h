//Current dumping ground for potentially useful datatypes. Needs to be
//cleaned up if used at all.
        
// Data types

typedef enum {
   FB_MODE_HALL = 0,
   FB_MODE_ENC,
   FB_MODE_BEMF
} mc_fb_mode;

//From https://github.com/vedderb/bldc/blob/master/datatypes.h
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

//From https://github.com/vedderb/bldc/blob/master/datatypes.h
typedef enum {
	CONTROL_MODE_DUTY = 0,
	CONTROL_MODE_SPEED,
	CONTROL_MODE_CURRENT,
	CONTROL_MODE_CURRENT_BRAKE, //Not sure we need this?
	CONTROL_MODE_POS,
	CONTROL_MODE_NONE
} mc_control_mode;

//This may be useful eventualy, Just put it in here for referance
//From https://github.com/vedderb/bldc/blob/master/datatypes.h
typedef enum {
	FAULT_CODE_NONE = 0,        // 0
	FAULT_CODE_OVER_VOLTAGE,    // 1
	FAULT_CODE_UNDER_VOLTAGE,   // 2
	FAULT_CODE_DRV8302,         // 3
        FAULT_CODE_DRV8302_OC,      // 4
	FAULT_CODE_ABS_SOFT_OC,     // 5
	FAULT_CODE_OVER_TEMP_FET,   // 6
	FAULT_CODE_OVER_TEMP_MOTOR,  // 7
        FAULT_CODE_HALL_SENSOR      // 8
} mc_fault_code;

//Yet another idea on how to handle fault codes
//not sure if this is a good way to go or not
typedef struct {
  uint8_t drv_fault     :1 ;
  uint8_t drv_pwrGood   :1 ;
  uint8_t drv_oc        :1 ;
  uint8_t swOverCurrent :1 ;
  uint8_t swOverVoltage :1 ;
  uint8_t hallSensor    :1 ;
  uint8_t fetOverTemp   :1 ;
  uint8_t motorOverTemp :1 ;
} mc_faultStatus;

//Config data to be stored in EEPROM 
typedef struct {
  //misc
  unsigned int throttleOut_max;    //0-4096
  unsigned int throttleOut_min;    //0-4096
  unsigned int dutyCycle_max;      //0-4096
  unsigned int dutyCycle_min;      //0-4096
  unsigned int polePairs;
  unsigned int pwmOutFreq;         //Hz
  uint8_t controlMode;
  //PID params
  float currentControl_kP;
  float currentControl_kI;
  float currentControl_kD;
  //Version info
  float configVersion;  //We can check that this matches to see if data has been written before  
} mc_configData;

typedef struct {
  unsigned int maxCurrent_HW;    //Amps
  unsigned int maxCurrent_motor; //Amps
  unsigned int maxCurrent_batt;  //Amps
  unsigned int maxCurrent_regen; //Amps
  unsigned int maxBusVoltage;    //Volts  
  unsigned int minBusVoltage;    //Volts 
  unsigned int maxTemp_FETs;     //degC
} mc_limits;
