/* 
	Defaults.h - A bunch of default values for JJ_BLDC.ino
	
	Jaimy Juliano
*/

#define DEFAULT_CONTROL_MODE 0 //0 = straight duty cycle control
#define MAX_THROTTLE_OUTPUT 3000  //This needs to be configurable
#define MIN_THROTTLE          10  //This needs to be configurable
#define MAX_MOTOR_RPM       2000  //This needs to be configurable
#define STOPPED_RPM_THRESH  10    //At what RPM do transition to STOPPED

#define DEFAULT_KP	50			//Default value for PID loop kP
#define DEFAULT_KI 	 0			//Default value for PID loop kI
#define DEFAULT_KD	 0			//Default value for PID loop kD

#define  MIN_DUTY  0.05           //5%
#define  MAX_DUTY  0.95           //95%     
#define  MIN_DUTY_COUNTS  200  //This isn't working for some reason -> (2 ^ PWM_OUT_RESOLUTION) * MIN_DUTY //This might come in handy
#define  MAX_DUTY_COUNTS  3900 //This isn't working for some reason -> (2 ^ PWM_OUT_RESOLUTION) * MAX_DUTY //This might come in handy

//******************
// Limits
#define MOTOR_OC_LIMIT    300      //default motor overcurrent limit in Amps 
#define BATT_OC_LIMIT     60       //default battery current limit in Amps
#define DRV_OC_LIMIT      100      //default HW (DRV8302) overcurrent limit in Amps
#define REGEN_OC_LIMIT	  20	   //default regen current limit in Amps
#define BUS_OV_LIMIT      42       //software over voltage limit 
#define BUS_UV_LIMIT	  10	   //software over voltage limit 
#define WARNING_FET_TEMP  60       //Temperature where we should lower the SW current limit
#define MAX_FET_TEMP      80       //Max allowed FET temp
                          
#define POLE_PAIRS       6     //This also needs to be a software setting eventually
#define ALIGNMENT_DUTY 100     //??% - also should be software setting

//1 second wait now just for testing
#define ALIGNMENT_TIMOUT 50000  //Time to wait for alignment in uS

#define RUNNING_AVG_BLOCK_SIZE 100                   
//*******************
//Timing config values
#define PWMFREQ 8000            //PWM frequency in hz. This should be a configurable parameter eventually
//#define PWM_PERIOD 1000000 * (1 / PWMFREQ)
#define SERIAL_OUT_PERIOD 10    //mS
#define CONTROL_LOOP_PERIOD 1000 //uS
