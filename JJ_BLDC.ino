#include "IntervalTimer.h" //I am pretty sure we are going to need this
#include "datatypes.h"     //Useful datatypes. Based on VESC code
#include "defaults.h"
#include "ADC.h"           //Pedvide's Teensy 3.1 ADC library
#include "BldcPWM.h"       //My PWM library
#include "PID_v1.h"           //Use PID library for Current and speed control loops
#include "RunningAverage.h"
#include "Tachometer.h"
#include "BldcStatus.h"
#include <SerialCommand.h>
#include <EEPROM.h>

#define JJBLDC_FW_VERSION 0.65
#define JJBLDC_CONFIG_VERSION 1.01
#define JJBLDC_HW_VERSION 0.2

//**********************
//Pin Definitions - got to start somewhere
#define HALL1    2
#define HALL2    3
#define HALL3    4
#define ENCA     3
#define ENCB     4

#define INHA 22
#define INLA 23
#define INHB 9
#define INLB 10
#define INHC 6
#define INLC 20

#define EN_GATE  11
#define DC_CAL   12
#define FAULT_IN 7
#define PWRGD_IN 8
#define OCTW_IN  28

#define REV_SW   29

#define THROTTLE A4
#define ISENSE1  A10
#define ISENSE2  A11
#define FET_TEMP A13
#define VSENSE   A3
#define ASENSE   A2
#define BSENSE   A1
#define CSENSE   A0
//#define FLD_PWM  5    //Not really used now

#define RX  0
#define TX  1

#define LED_PIN   13
#define HW_OC_ADJ A14 //Set DRV8302 OC protection with this
                      //A14 is the 12bit DAC pin. Probably a waste of a 12 bit DAC

// Defines for other whacky fault code idea
#define FAULT_CODE_NONE                   0x00
#define FAULT_CODE_DRV8302                0x01
#define FAULT_CODE_DRV8302_OVERCURRENT    0x02
#define FAULT_CODE_DRV8302_PWRGOOD        0x03
#define FAULT_CODE_MOTOR_OVERCURRENT      0x04
#define FAULT_CODE_BATT_OVERCURRENT       0x06
#define FAULT_CODE_OVER_VOLTAGE           0x07
#define FAULT_CODE_UNDER_VOLTAGE          0x08
#define FAULT_CODE_OVER_TEMP_FET          0x09
#define FAULT_CODE_OVER_TEMP_MOTOR        0x0A
#define FAULT_CODE_HALL_SENSOR            0x0B

#define DIR_FORWARD 1
#define DIR_REVERSE 0

#define BRIDGE_STATE_ALIGN 10 //This should probably be an enum
#define BRIDGE_STATE_BRAKE 11
#define BRIDGE_STATE_COAST 12

//PDB Defines
#define PDB_CONFIG (PDB_SC_TRGSEL(15) | PDB_SC_PDBEN | PDB_SC_PDBIE \
	| PDB_SC_PRESCALER(0) | PDB_SC_MULT(0))

#define PDB_PRESCALE 1 // Actual prescale
#define usToTicks(us)    ((us) * (F_BUS / 1000) / PDB_PRESCALE / 1000)
#define ticksToUs(ticks) ((ticks) * PDB_PRESCALE * 1000 / (F_BUS / 1000))

// NTC Termistors
#define NTC_RES(adc_val)	((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_CONVERT_TEMP(adc_val)	(1.0 / ((logf(NTC_RES(adc_val) / 10000.0) / 3434.0) + (1.0 / 298.15)) - 273.15)

//bool CrudeDubugFlag = false;

const float BUS_VOLTAGE_FACTOR = 0.015;  //Counts to bus Volts (voltage divider)
//const int   mV_SF = 15;     //Counts to bus mV (voltage divider)
const float SHUNT_CURRENT_FACTOR = 100.0; //Its * 100 because the diff amp already has a gain of 10
                                         //So - I = (Vshunt * 10) * 100
const float DRV_OC_FACTOR = 10.0;  //Not used yet, no idea what it should be yet either

//******** Hall state table from DEV BLDC code for testing **************
// hall state table
// the index is the decimal value of the hall state
// the value is the position (255 is an error)
uint8_t hallOrder_0[] = {255, 2, 0, 1, 4, 3, 5, 255};
uint8_t hallOrder_1[] = {255, 5, 3, 4, 1, 0, 2, 255}; //This seems to work
//uint8_t bemfCommOrder[] = {3,4,5,0,1,2}; //Never got this working right

volatile unsigned int commStep;              //holds the current commutation state. May not be needed
volatile uint8_t lastcommStep;
volatile uint8_t bemfPin;                //holds the AIN pin for the current BEMF sensing phase.
volatile uint8_t hallState = 0;          //Global because I want this to be accessable while testing

//Sensorless variables, once I get it working I'll clean it up
volatile boolean zeroCrossFound = false; //Global because I want this to be accessable while testing
unsigned long int zeroCrossTime;
unsigned long sensorlessCommDelay;
int sensorlessCommAdvance;

unsigned int serialUpdatePeriod  = SERIAL_OUT_PERIOD;

//volatile uint32_t eRotPeriod;   //Contains the most recent electrical rotation period in uS
volatile uint32_t commPeriod;   //Contains the most recent commutation step period in uS
//elapsedMicros tachTimer;
//elapsedMicros controlLoopTimer;
//elapsedMicros zcTimer;
elapsedMillis outputTimer;

unsigned long prevTime = 0;

//Flags
boolean motorDirection = DIR_FORWARD;
//boolean motorDirection = DIR_REVERSE;
boolean PHASE_OFF_ISENSE;

boolean regenEnabled = false;
boolean serialStreamFlag = false;

//Global for troubleshooting 7-8-16
boolean drv_pwrGood;
boolean drv_fault;
boolean drv_octw;

//Fault code "fake register" (look into best way to do this)
uint16_t faultStatus = 0;   // | drv_pwrGood | drv_fault | drv_octw
                           // |      0      |     0     |     0

double motorCurrent = 0;
double motorCurrent_Avg = 0;
double motorCurrent_mA = 0; //was int, changed to double for troubleshooting -didn't do anything
double motorCurrentDuty = 0;
double motorCurrentSetpoint = 0;
double motorRPM = 0;
double speedDuty = 0;
double speedSetpoint = 0;
volatile uint16_t dutyCycle;    //Duty cycle in counts. (0 to 4096 for PWM_OUT_RESOLUTION = 12)
volatile double dutyCycleFloat; //Float representation of duty cycle 0.0 to 1.0 = 0% to 100%
double lastDutyCycle;

mc_state controllerState;
mc_state lastControllerState;

//mc_control_mode controlMode;
//mc_fault_code faultCode;
mc_fb_mode feedbackMode;
//mc_faultStatus faultStatus; //new struct to hold faults
mc_configData configData;
//mc_limits limitValues;

RunningAverage fetTemp_RA(RUNNING_AVG_BLOCK_SIZE);
RunningAverage motorCurrent_RA(RUNNING_AVG_BLOCK_SIZE);
//RunningAverage motorCurrent_mA_RA(RUNNING_AVG_BLOCK_SIZE);

ADC *adc = new ADC();    //adc object
ADC::Sync_result syncReadResult; //Structure to store synchronized read from both ADCs

//Create SerialCommand object
SerialCommand sCmd;

//Create PWM object
//!!!Only PWM_UNIPOLAR mode works right now!!!!
BldcPWM pwmout(PWM_UNIPOLAR, PWM_EDGE, PWMFREQ, 6);

// Create IntervalTimer objects
IntervalTimer comTimer;         //commutation timer
IntervalTimer controlLoopTimer; //main control loop timer

// PID controllers,is this overkill??????
PID currentPID(&motorCurrent_Avg, &motorCurrentDuty, &motorCurrentSetpoint, DEFAULT_CURRENT_KP,DEFAULT_CURRENT_KI,DEFAULT_CURRENT_KD, DIRECT);
//PID speedPID(&motorRPM, &speedDuty, &speedSetpoint, 1.0,5.0,0.0, DIRECT);

//Create Tachometer object
Tachometer tachometer;
//Try out new Status class
BldcStatus status;

//Testing acquisitionBuffer class
acquisitionBuffer scopeCurrent1(1000);
//acquisitionBuffer scopeCurrent2(1000);
acquisitionBuffer scopeBemfVolts(1000);
acquisitionBuffer scopeBusVolts(1000);

//Experimenting with Meter objects
BldcMeter busVoltage(RUNNING_AVG_BLOCK_SIZE, "V", false);
BldcMeter phaseCurrent(RUNNING_AVG_BLOCK_SIZE, "A", false);
BldcMeter throttle(RUNNING_AVG_BLOCK_SIZE,"", false);           //Not really necessary, but the filtering might be handy
BldcMeter fetTemperature(RUNNING_AVG_BLOCK_SIZE, "DegC", true);

static void FTM0_INT_ENABLE(void)  { FTM0_SC |= FTM_SC_TOIE; FTM0_C0SC |= FTM_CSC_CHIE; NVIC_ENABLE_IRQ(IRQ_FTM0);}
static void FTM0_INT_DISABLE(void) { NVIC_ENABLE_IRQ(IRQ_FTM0); }

static float countsToVolts(float counts) {return (counts*3.3)/adc->getMaxValue();}
static int voltsToCounts(float volts) {return (volts * adc->getMaxValue()) / 3.3;}
//static int voltsToCounts(float volts) {return (int)(volts * 4096) / 3.3;} // THIS DOESNT WORK!!

// static int countsTo_mV(int counts) {return ((counts*3300)/(int)adc->getMaxValue());}        //reduce need for FP
// static int mvToCounts(int mv) {return ((mv*(int)adc->getMaxValue())/3.3);}

unsigned int eeAddress = 0; // Starting EEPROM address

void setup() {
  pinMode(LED_PIN, OUTPUT);
  //pinMode(29, OUTPUT); //toggle for o-scope
  pinMode(30, OUTPUT); //toggle for o-scope

  pinMode(HALL1, INPUT); //we have external pullups so I'm not sure about this
  pinMode(HALL2, INPUT);
  pinMode(HALL3, INPUT);

	attachInterrupt(HALL1, hall_isr, CHANGE);
	attachInterrupt(HALL2, hall_isr, CHANGE);
	attachInterrupt(HALL3, hall_isr, CHANGE);

  //pinMode(THROTTLE, INPUT);

  pinMode(PWRGD_IN, INPUT_PULLUP); // Buck converter OK
  pinMode(FAULT_IN, INPUT_PULLUP); // Fault indicator
  pinMode(OCTW_IN, INPUT_PULLUP);  // Over current or over temp. DOUBLE CHECK THIS CONNECTION!

	//attachInterrupt(FAULT_IN, drvfault_isr, FALLING);
	//attachInterrupt(OCTW_IN, octw_isr, FALLING);

  pinMode(REV_SW, INPUT_PULLUP);   // Reverse switch input

  analogWriteResolution(12); //Set the PWM resolution to 12 bit

  Serial.begin(115200); //USB Serial
	Serial1.begin(9600);  //BT Serial
  //delay(1000);
  startupBlink();

  /****Setup serial command callbacks****/
	//Control commands
  sCmd.addCommand("v", cmdGetVersion);      //Report the firmware version
  sCmd.addCommand("s", cmdStream);          //Start/Stop streaming out readings
	sCmd.addCommand("r", cmdRate);						//Update rate for streaming
  sCmd.addCommand("conf", cmdPrintConfig);  //Print out the config struct data
  sCmd.addCommand("save", cmdSaveConfig);   //Save current config to EEPROM
	sCmd.addCommand("test", cmdTest);         //Use this for testing stuff
	//sCmd.addCommand("bt", cmdBluetoothEnable);//Start/Stop bluetooth (serial1) output
	//Settings
	sCmd.addCommand("tmax", cmdThrottleMax);         //get or set throttle max
	sCmd.addCommand("tmin", cmdThrottleMin);			   //get or set throttle min
  sCmd.addCommand("dutymax", cmdDutyCycMax);
	// sCmd.addCommand("dutymin", cmdDutyCycMax);
	// sCmd.addCommand("polePairs", cmdPolePairs);
	// sCmd.addCommand("pwmfreq", cmdPwmFrequency);
	sCmd.addCommand("cntrlmode",cmdControlMode);     //get or set control mode (require reboot, or only apply when throttle is 0?)
	sCmd.addCommand("kp", cmdCurrentkP);
	//sCmd.addCommand("ki", cmdCurrentkI);
	// sCmd.addCommand("kd", cmdCurrentkD);
  sCmd.addCommand("mcmax", cmdMotorCurrentMax);    //get or set maximum phase current
	//sCmd.addCommand("bcmax", cmdBusCurrentMax);    	 //get or set maximum bus current
	sCmd.addCommand("hwclimit", cmdHwCurrentLimit); //get or set the hardware over current protection limit
	// sCmd.addCommand("ovlimit", cmdOverVoltageLimit); //get or set over voltage limit level
	// sCmd.addCommand("uvlimit", cmdUnderVoltageLimit);//get or set under voltage limit
  sCmd.addCommand("ftmax", cmdFetTempMax);         //get or set the maximum FET Temperature (I might make a warning at 90% of max)

  sCmd.setDefaultHandler(cmdUnknown);       //Handler for unrecognized command

  status.fetTemp_RA.clear();
  status.motorCurrent_RA.clear();

  adc->setAveraging(0, ADC_0);   // We need fast conversion rates so lets try 0 averaging
  adc->setResolution(12, ADC_0); // set bits of resolution. 12 is fine
  adc->setConversionSpeed(ADC_HIGH_SPEED, ADC_0); // change the conversion speed
  adc->setSamplingSpeed(ADC_HIGH_SPEED, ADC_0);   // change the sampling speed
  //ADC1 setup
  adc->setAveraging(0, ADC_1);   // set number of averages
  adc->setResolution(12, ADC_1); // set bits of resolution
  adc->setConversionSpeed(ADC_HIGH_SPEED, ADC_1); // change the conversion speed
  adc->setSamplingSpeed(ADC_HIGH_SPEED, ADC_1);   // change the sampling speed

	loadConfig();  //Load configuration from EEPROM

  //analogWrite(HW_OC_ADJ, configData.maxCurrent_HW); //Lets try this again  [From datasheet -> Ioc = Vds/Rds]
  analogWrite(HW_OC_ADJ, 4096);

  //Wake up the gate driver
  pinMode(EN_GATE, OUTPUT);
  digitalWriteFast(EN_GATE, LOW);
  delay(100);
  digitalWriteFast(EN_GATE, HIGH);
	while(!digitalReadFast(FAULT_IN)){}; //Wait for it to be ready. I should really add a time out and fault here

	//noInterrupts(); //Just in case
  //doDc_Cal(); //Shunt offset calibration
	digitalWriteFast(DC_CAL, HIGH);
	delay(100); //was 1000 6-11-16
	status.iSense1_offset = -adc->analogRead(ISENSE1,ADC_1);
	status.iSense2_offset = -adc->analogRead(ISENSE2,ADC_0);
	digitalWriteFast(DC_CAL, LOW);

  status.throttle_offset = adc->analogRead(THROTTLE,ADC_0); //Throttle offset calibration
	//interrupts();
  //faultCode = FAULT_CODE_NONE;    //The whole fault code thing needs to be done better

  //**********Initial control mode**********
  //configData.controlMode = CONTROL_MODE_DUTY;
  //controlMode = CONTROL_MODE_SPEED;
  //configData.controlMode = CONTROL_MODE_CURRENT;
  feedbackMode = FB_MODE_HALL;
  //feedbackMode = FB_MODE_BEMF;

  controllerState = MC_STATE_STOPPED;    //Initial motor State

  //*************Regen Enable****************
//  regenEnabled = true;

  //Init PID loops
  currentPID.SetOutputLimits(0, configData.dutyCycle_max);
  currentPID.SetSampleTime(1);
  currentPID.SetMode(AUTOMATIC);
//  speedPID.SetOutputLimits(0, configData.dutyCycle_max);
//  speedPID.SetSampleTime(1);
//  speedPID.SetMode(AUTOMATIC);

  //Initialize timers and enable interrupts
  comTimer.begin(commutate, 20);
  controlLoopTimer.begin(controlLoop, 1000);
  initPDB(); //initialize PBD
  FTM0_INT_ENABLE(); //Enable timer overflow interupt on FTM0 (Motor PWM output)

	//testing acquisitionBuffer class here
	scopeCurrent1.arm();
	//scopeCurrent2.arm();
	scopeBemfVolts.arm();
	scopeBusVolts.arm();

	adc->enableInterrupts(ADC_0);

  Serial.print("JJ_BLDC V");
  Serial.print(JJBLDC_FW_VERSION,2);
  Serial.println(" ready");
}

void loop() {
	//Do something useful
	//I think we might end up just doing serial communications here

	sCmd.readSerial();

	//TO DO - Do UART (bluetooth) seperatly
	//outputStatsBT();

	//Do usb serial
	//if(serialStreamFlag == true){
		if(outputTimer > serialUpdatePeriod){
			if(serialStreamFlag == true) outputStats();
			outputStatsBT();
			outputTimer = 0;
		}
	//}else
	if(scopeCurrent1.samplesReady() /*&& scopeCurrent2.samplesReady()*/
	&& scopeBemfVolts.samplesReady() && scopeBusVolts.samplesReady()){
		//This is just to test the acquisitionBuffer class
		//obviosly need something better later

		for(int i=0; i < scopeCurrent1.bufferSize();i++){
			Serial.print(scopeCurrent1.getSample(i));
			Serial.print("\t");
			//  Serial.print(scopeCurrent2.getSample(i));
			//  Serial.print("\t");
			Serial.print(scopeBemfVolts.getSample(i));
			Serial.print("\t");
			Serial.println(scopeBusVolts.getSample(i));
		}
		scopeCurrent1.arm(); //re-arm
		//scopeCurrent2.arm();
		scopeBemfVolts.arm();
		scopeBusVolts.arm();
	}

}

///////////////////////////////////////////////////////
// Control Loop Timer ISR called every 1000 microseconds
// This needs some reworking. Need to take a look at the
// variable scopes. Maybe better data structure??
void controlLoop(){
  //Do Control
  uint16_t tmpDuty;
  static elapsedMillis blinkTimer = 0;

  checkDrv8302Faults(); //Not sure if this should be done here, or somewhere else

  motorCurrent = status.getMotorCurrent(); //I think we need to call this first to update the current
  //motorCurrent_Avg = fabs(motorCurrent_RA.getAverage()); // * dutyCycleFloat);
  motorCurrent_Avg = status.getFilteredMotorCurrent();

  //****************************************
  //Start of different state idea
  switch(controllerState){
    case MC_STATE_INIT:
      //do stuff
      //May not be needed because of setup()
      //Could also put all init code in seperate function and call that from
      //setup() and from here ??
      break;
    case MC_STATE_STOPPED:

      //I would like to rezereo the shunt amplifiers while we are stopped
			//but a simple single analog read wasn't working when I tried. It seems
			//like once we start doing SynchronizedSingleRead, we cant go back to a
			//plain 'ol analogRead()'

      //check reverse switch here
      if(digitalReadFast(REV_SW)){
        motorDirection = DIR_REVERSE;
      }else{
        motorDirection = DIR_FORWARD;
      }
      //check for throttle input
      if(status.getThrottle() >= configData.throttleOut_min){
        controllerState = MC_STATE_DRIVE;
        //tmpDuty = calcDutyCycle();
      }
      break;
    case MC_STATE_DRIVE:
      tmpDuty = calcDutyCycle();

      //Check DRV8302 Hardware OC Protection
      if(faultStatus & FAULT_CODE_DRV8302_OVERCURRENT){
        //reduce duty cycly. This is pretty crappy, need somthing clever here
        //Also, maybe a DRV OC should be an interupt?
        tmpDuty = tmpDuty / 2; //HW OC detected, lets try cutting the duty cycle in half (6-18-15)
      }

      //Check For Over Temp
      //TO DO: Need to implement a WARNING_TEMP that simply reduces the SW current limit (once that works)
      if(NTC_CONVERT_TEMP(status.fetTemp_RA.getAverage()) > configData.maxFetTemp){   //Try this with RA filtered readings
        //Oh crap, getting hot!
        faultStatus |= FAULT_CODE_OVER_TEMP_FET;
        //Need to add test for this fault in other areas
        //I just realized that I think I need an ohCrapShutdown() function
        //For now....
        tmpDuty = tmpDuty / 2; //Over Temp detected, lets try cutting the duty cycle in half (6-18-15)
      }

      //Check if we are over minumum allowed duty cycle
      if(tmpDuty < MIN_DUTY){ //configData.dutyCycle_min){
        tmpDuty = 0;
      }

      dutyCycle = tmpDuty;
      dutyCycleFloat = (double) dutyCycle / PWM_COUNTS; //Float representation (0.0 to 1.0)

      lastDutyCycle = tmpDuty; //keep track of what our last target was

      if((tachometer.getRPM() <= STOPPED_RPM_THRESH) && (controllerState != MC_STATE_FAULT)){
        controllerState = MC_STATE_STOPPED;
      }
      break;
    case MC_STATE_FAULT:
      //do stuff

      //If fault has cleared, transition out. This should probably be smart enough
      //to do somthing about specific faults and to transition back to MC_STATE_DRIVE when appropriate
      if(faultStatus == FAULT_CODE_NONE){
      //if((faultStatus.drv_pwrGood | faultStatus.drv_fault | faultStatus.drv_oc |) == false){
        controllerState = MC_STATE_DRIVE; //THIS IS MAKING A BIG ASSUMPTION!! Need to fix
      }

      break;
  }
}

///////////////////////////////////////////////
// Load configuration data from EEPROM if available
void loadConfig(){
  EEPROM.get(eeAddress, configData);
  if(configData.configVersion != JJBLDC_CONFIG_VERSION){
    Serial.println("No config found! Using default values");

    //Set defaults
    configData.throttleOut_max = MAX_THROTTLE_OUTPUT; //used
    configData.throttleOut_min = MIN_THROTTLE; //used
    configData.dutyCycle_max = MAX_DUTY_COUNTS; //used
    configData.dutyCycle_min = MIN_DUTY_COUNTS; //used
    configData.polePairs = POLE_PAIRS; //used
    configData.pwmOutFreq = PWMFREQ; //This is currently set when we creat the PWM object, need to set after we load the config
    configData.controlMode = CONTROL_MODE_DUTY; //used, default to direct duty cycle control

    configData.currentControl_kP = DEFAULT_CURRENT_KP; //Not used yet
    configData.currentControl_kI = DEFAULT_CURRENT_KI; //Not used yet
    configData.currentControl_kD = DEFAULT_CURRENT_KD; //Not used yet

    configData.maxCurrent_HW = voltsToCounts(DRV_OC_LIMIT * FET_RDS);
    configData.maxCurrent_motor = voltsToCounts(MOTOR_OC_LIMIT / SHUNT_CURRENT_FACTOR); //used
    configData.maxCurrent_batt = voltsToCounts(BATT_OC_LIMIT / SHUNT_CURRENT_FACTOR); //Not used yet
    configData.maxCurrent_regen = voltsToCounts(REGEN_OC_LIMIT / SHUNT_CURRENT_FACTOR);
    //configData.maxBusVoltage = voltsToCounts(BUS_OV_LIMIT / BUS_VOLTAGE_FACTOR);
		configData.maxBusVoltage = (int)(BUS_OV_LIMIT / BUS_VOLTAGE_FACTOR);
    configData.minBusVoltage = (int)(BUS_UV_LIMIT / BUS_VOLTAGE_FACTOR);
    configData.maxFetTemp = MAX_FET_TEMP; //This can stay as deg C for now
    //Write current config version
    configData.configVersion = JJBLDC_CONFIG_VERSION;
  }else{
    Serial.println("Config data found!");
  }
}

///////////////////////////////////////////////
// Save configuration data to EEPROM
void saveConfig(){
  EEPROM.put(eeAddress, configData);
  Serial.println("Configuration saved to EEPROM.");
}

///////////////////////////////////////////////
// Check the gate driver for faults. I still may want to handle this with interupts
void checkDrv8302Faults(){
  //Lets check in on the DRV8302
  drv_pwrGood = digitalReadFast(PWRGD_IN);  //power good = HI
  //faultStatus.drv_pwrGood = digitalReadFast(PWRGD_IN);  //power good = HI
  drv_fault   = !digitalReadFast(FAULT_IN); //fault = LOW
  //faultStatus.drv_fault = !digitalReadFast(FAULT_IN); //fault = LOW
  drv_octw    = !digitalReadFast(OCTW_IN);  //OC or OT = LOW
  //faultStatus.drv_oc = !digitalReadFast(OCTW_IN);  //OC or OT = LOW

  if(!drv_pwrGood || drv_fault){
    controllerState = MC_STATE_FAULT;
    faultStatus |= FAULT_CODE_DRV8302;  //for now
  }
  if(drv_octw){
    //May want to do something more clever with this fault so kept it seperate
    controllerState = MC_STATE_FAULT;
    faultStatus |= FAULT_CODE_DRV8302_OVERCURRENT ; //for now
  }
}

///////////////////////////////////////////
//Calculate the actual PWM duty cycle
int calcDutyCycle(){
  int duty;

  switch(configData.controlMode){
    case CONTROL_MODE_SPEED:
//      motorRPM = getRPM();
//      //This is not fully implemented yet! In fact, I don't know if I will bother with this
//      //Throttle needs to be mapped to actual rpm range!!
//      speedSetpoint = map(throttle,0,configData.throttleOut_max,0,MAX_MOTOR_RPM);
//      speedPID.Compute();
//      duty = speedDuty;
      break;
    case CONTROL_MODE_CURRENT:
      //Map throttle to current range.Probably need to do this mapping better!!
      motorCurrentSetpoint = map(status.getThrottle(),0,configData.throttleOut_max,0,configData.maxCurrent_motor);
      //motorCurrentSetpoint = throttle; //lets try this real quick
      currentPID.Compute();
      duty = motorCurrentDuty;
      break;
    case CONTROL_MODE_DUTY:

      //Throttle is going to need to be mapped to actual range!!
      duty = map(status.getThrottle(),0,configData.throttleOut_max,0,4000); //need Max throttle to equal 4096 (4000 now) here
      break;
  }
  return duty;
}

//******************************************************
//put this in a function just to clean up main loop
void outputStats(){
  Serial.print(status.getBusVoltage());
  Serial.print('\t');
	// Serial.print(MAX_DUTY_COUNTS);
	// Serial.print('\t');
  // Serial.print(status.iSense2_raw);
  // Serial.print('\t');
  Serial.print(motorCurrent_Avg);
  Serial.print('\t');
  // Serial.print(configData.maxCurrent_motor);
  // Serial.print('\t');
  //Serial.print(motorCurrentDuty);
  // Serial.print(CrudeDubugFlag);
  // Serial.print('\t');
	// Serial.print(status.vBemf_raw);
  // Serial.print('\t');
	// Serial.print(zeroCrossFound);
  // Serial.print('\t');
	Serial.print(status.throttle);
  Serial.print('\t');
  // Serial.print(motorCurrent_Avg * dutyCycleFloat); //This should be average battery current
  // Serial.print('\t');
  Serial.print(dutyCycle);
  Serial.print('\t');
  Serial.print(NTC_CONVERT_TEMP(status.fetTemp_RA.getAverage()));
  Serial.print('\t');
  // Serial.print(tachometer.getTimeSinceCommutation());
  // Serial.print('\t');
  // Serial.print(tachometer.getRPM(),0);
  // Serial.print('\t');
  Serial.print(tachometer.getMPH(),2);
  Serial.print('\t');
 // Serial.print(drv_pwrGood);
 // Serial.print('\t');
 // Serial.print(drv_fault);
 // Serial.print('\t');
 // Serial.print(drv_octw);
 // Serial.print('\t');
 Serial.print(faultStatus,BIN); //print out faultStatus bits
 Serial.print('\t');
 Serial.print(hallState,BIN);
 Serial.print('\t');
 Serial.print(controllerState);
 Serial.print('\t');
//  Serial.print(bemfSampleTime);
 	Serial.print('\t');
 	Serial.println(commStep);
}

//Testing output on TX1 through Bluetooth
void outputStatsBT(){

  Serial1.print("*T");
	Serial1.print(NTC_CONVERT_TEMP(status.fetTemp_RA.getAverage()),2);
  Serial1.print('*');

	Serial1.print('M');
	Serial1.print(tachometer.getMPH(),2); //replace with rpm later
  Serial1.print('*');

	Serial1.print('V');
  Serial1.print(status.getBusVoltage());
	Serial1.print('*');

	Serial1.print('C');
  Serial1.print(motorCurrent_Avg);
	Serial1.println('*');

}
////////////////////////////////////////////////////////////
//Silly hard coded startup blink. Blinks out version
void startupBlink(void){
  for(int i=0;i<6;i++){  //Have it blink out something interesting. Version number ?
    digitalWrite(LED_PIN, HIGH);   // set the LED on
    delay(100);                  // wait for a bit
    digitalWrite(LED_PIN, LOW);    // set the LED off
    delay(100);
  }
}

///////////////////////////////////////////////
// Calculate Battery Current in Amps
float getBatteryCurrent(){
  //return getMotorCurrent() * (float)(dutyCycle / 2 ^ PWM_OUT_RESOLUTION);
  return status.getMotorCurrent() * dutyCycleFloat;
}

////////////////////////////////////////////////
// Shutdown The Bridge under fault condition
void faultShutdown(void){
//  pwmout.pwmOUTMASK(0xff);      //Turn all phases off (0x3f might be better because it wont mess with
  pwmout.pwmOUTMASK(0x3f);      //unused FTM channels. Check with data sheet to make sure thats OK)
  controllerState = MC_STATE_FAULT;
}
////////////////////////////////////////////////
//Shutdown The Bridge and allow motor to freewheel - This is mostly just for readability
void bridgeOff(void){
//  pwmout.pwmOUTMASK(0xff);      //Turn all phases off (0x3f might be better because it wont mess with
  pwmout.pwmOUTMASK(0x3f);      //unused FTM channels. Check with data sheet to make sure thats OK)
  controllerState = MC_STATE_COAST;
}
///////////////////////////////////////////////
//Brake the motor by shorting all 3 phases
void fullBrake(void){
	writeBridgeState(11);
  controllerState = MC_STATE_FULL_BRAKE; //<- This may not need to be here because it should probably
                                    // be handled in the state transition code anyway
}

//////////////////////////////////////////////////////////
// FTM0 ISR - This is Used to Syncronize ADC Reads With PWM
void ftm0_isr(){

  //************************ 8-29-15 *******************
  //Experimenting with reading phase current during OFF period
	//Ok, This works great, but I'm not sure if its really what I want

  //PHASE_OFF_ISENSE = false;
  if (FTM0_C0SC & FTM_CSC_CHF){     //Do we have a channel 0 match interupt?
    if(commStep == 1 || commStep == 2){
			/***Commented out next 2 lines for sing shunt resistor hardware version - 7-18-16 ***/
			//PHASE_OFF_ISENSE = true;
      //adc->startSynchronizedSingleRead(ISENSE2,ISENSE1);
      FTM0_C0SC &= ~FTM_CSC_CHF;      //clear the flag
      return;                         //then bail
    }
    FTM0_C0SC &= ~FTM_CSC_CHF;      //clear the flag
  }

  //This read needs to happen in the middle of the PWM active period. I have
  //to introduce a delay of 1/2 the current duty cycle.I can do this
  //with the PDB

  //Trying wierd thing with throttle just for testing
  //This eventually needs to be mapped to duty cycle %
  if (dutyCycle >= 768){        //Only setup PDB if delay is more than 16uS, eventually should be MIN_DUTYCYCLE
    PDB0_IDLY = ((dutyCycle - 768) + 2) / 2; //Try with throttle as input? We need to measure at 1/2 DUTYCYCLE for the real thing
    PDB0_SC |= PDB_SC_SWTRIG; //This starts the counter
    PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;
  }else{
    //just start the chain of ADC readings now  //!!!I really should have some minumum PDB delay to avoid noise

    adc->startSynchronizedSingleRead(ISENSE2,ISENSE1);
  }

  FTM0_SC &= ~FTM_SC_TOF; //clear the overflow flag

  //digitalWriteFast(29, HIGH); //to see on scope

}

/////////////////////////////////////////////////////////////////////////
// INITIALIZE PDB - This is Used to Delay ADC Readings Within A PWM Pulse
// This could probably be a static funtion like FTM0_INT_ENABLE() is
void initPDB(void){
 // set the programmable delay block to trigger the ADC

  // Enable the PDB clock
  SIM_SCGC6 |= SIM_SCGC6_PDB;
  // Modulus Register, 1/(48 MHz / 1) so 48 ticks/uS
  PDB0_MOD = 0xffff; //not using this right right now, maybe later
  // Interrupt delay
  PDB0_IDLY = 0;
  // Setup the PDB SC Register and Load Ok
  PDB0_SC = PDB_CONFIG | PDB_SC_LDOK;

  //****I actually don't think these should be here********
  //I don't know why we start the counter and enable the
  //interrupt in the init routine.
  //Try removing the counter start and moving the IRQ enable
  //when I can test the outcome immediately (can't right now)
  PDB0_SC |= PDB_SC_SWTRIG; //This starts the counter
  NVIC_ENABLE_IRQ(IRQ_PDB); //enables the IRQ
  //*******************************************************
}
//////////////////////////////////////////////////////////////////
// PDB ISR - This is Used to Delay ADC Readings Within A PWM Pulse
void pdb_isr(void){

  //Begin conversion of current samples on ADC0 and ADC1
  //This may be really inefficient but I want it to be understable for now
  adc->startSynchronizedSingleRead(ISENSE2,ISENSE1);

  PDB0_SC = PDB_CONFIG | PDB_SC_LDOK; // (also clears interrupt flag)

}

//////////////////////////////////////////////////////////////////////
// OCTW ISR - Catch when the hardware overcurrent (or over temp) occurs
void octw_isr(void){
	digitalWriteFast(LED_PIN, HIGH); //testing
}

//////////////////////////////////////////////////////////////////////
// Hall sensor ISR - Trigger comutation on hall sensor change
void hall_isr(void){
	commutate(); //COMPLETELY UNTESTED!!
}
///////////////////////////////////////////////////////
// ADC ISR - A Whole Lot of Stuff Happens Here
void adc0_isr(){

  uint8_t adc0Pin = ADC::sc1a2channelADC0[ADC0_SC1A&ADC_SC1A_CHANNELS]; // the bits 0-4 of ADC0_SC1A have the channel
  //uint8_t adc1Pin = ADC::sc1a2channelADC1[ADC1_SC1A&ADC_SC1A_CHANNELS]; // the bits 0-4 of ADC0_SC1A have the channel

  //Determine which reading(s) are ready
  if(adc0Pin == ISENSE2){ //We must be doing the synced phase current read
      syncReadResult = adc->readSynchronizedSingle();

			status.iSense2Update(-syncReadResult.result_adc0);  //iSense2 must be read by ADC_0!!! A11 can't do SE on ADC_1
			//status.iSense1Update(-syncReadResult.result_adc1); //Need to look into why this is inverted?? 7-18-16

			if(SHUNT_CONFIG == 1){
				status.motorCurrentUpdate(-syncReadResult.result_adc1 - status.iSense1_offset);
			}else if (SHUNT_CONFIG == 2){
				status.motorCurrentUpdate(-syncReadResult.result_adc0 - status.iSense2_offset);
			}else{
				//Do duel shunt code here if I ever go back
			}

			// //test acquisitionBuffer class here
			// scopeCurrent1.addSample(status.iSense1_raw);
			// scopeCurrent2.addSample(status.iSense2_raw);

      //If we are doing PWM OFF I sense, we should not fire off the other measurments [Comented out for single shunt hardware version 7-18-16]
      // if(PHASE_OFF_ISENSE){
      //   //digitalWriteFast(LED_PIN, LOW);
      //   PHASE_OFF_ISENSE = false;
      // }else{

        adc->startSynchronizedSingleRead(bemfPin,VSENSE); //Fire off bus voltage and BEMF readings

				//test acquisitionBuffer class here
				//scopeCurrent1.addSample(status.iSense1_raw);
				scopeCurrent1.addSample(status.getMotorCurrent_Raw());
				//scopeCurrent2.addSample(status.iSense2_raw);
      //}

      //Ok, while thats happening maybe we can check the absolute current limits?
      //Is checking every single pulse really necessary? I don't know but i'll do it anyway

      ////////////Not working right now///////////////////
			//if((status.iSense1_raw > configData.maxCurrent_motor) || (status.iSense2_raw > configData.maxCurrent_motor)){

			if(status.getMotorCurrent_Raw() > configData.maxCurrent_motor){
        //Oh crap! We have exceded our limit
        // faultStatus |= FAULT_CODE_MOTOR_OVERCURRENT; //OC
        // //controllerState = MC_STATE_FAULT;
				// CrudeDubugFlag = true;
        // faultShutdown();
				pwmout.pwmOUTMASK(0x3f); //just try this until I figure out the logic and write some code to transition out of FAULT or COAST states
      }
			/////////////////////////////////////////////////////////////////

  }else if(adc0Pin == ASENSE || adc0Pin == BSENSE || adc0Pin == CSENSE){ //our bus voltage and BEMF readings are ready
      syncReadResult = adc->readSynchronizedSingle();
      //status.vBus_raw = syncReadResult.result_adc1; //Grab the bus voltage and then..
			status.vBusUpdate(syncReadResult.result_adc1);
			//status.vBemfUpdate(syncReadResult.result_adc0);
			status.vBemfUpdate(syncReadResult.result_adc0); //reading is internally offset to be centered on vBus/2

			//Want to see whats happening
			scopeBusVolts.addSample(syncReadResult.result_adc1);
			scopeBemfVolts.addSample(status.vBemf_raw);

			//////////////////////////////////////////////////////
			//!!!I NEED TO DO ZERO-CROSS DETECTION RIGHT HERE!!!
			//
			// commStep 0 => Phase C falling
			// commStep 1 => Phase B rising
			// commStep 2 => Phase A falling
			// commStep 3 => Phase C rising
			// commStep 4 => Phase B falling
			// commStep 5 => Phase A rising
			////////////////////////////////////////////////////////
			const unsigned int timeZcOff = 250;  //Blanking Period after commutation to skip ZC detection in uS, NO CLUE WHAT THIS SHOULD BE!!
																					 //For now -> 250uS = 2 pulses at 8000hz
			unsigned int adcIsrTime = tachometer.getTimeSinceCommutation();
			if((adcIsrTime >= timeZcOff) && !zeroCrossFound){
				/*TO DO LIST*/
				static int lastBemfReading;
				static unsigned long int lastAdcIsrTime;
				static unsigned long int lastZeroCrossTime;
				int bemfReading = status.vBemf_raw;

				//scopeBemfVolts.addSample(bemfReading);

				//Offset reading to be centered on virtual ground (vBus/2). I May want to just do this above when the adc is read
				//bemfReading = status.vBemf_raw - (status.vBus_raw / 2);

				//Check if BEMF is falling, if it is, invert it
				if((commStep & 0x01) == 0 ){
					bemfReading = -bemfReading;
				}

				//scopeBemfVolts.addSample(bemfReading);

				//Check for BEMF zero cross
				if(bemfReading >= 0){
					//ZC detected! Do interpolation to figure out actual time of ZC
					// or, possibly do integration method here if I can figure that out

					//Blinky light for testing
					// if(adc0Pin == ASENSE){
					// 	digitalWriteFast(LED_PIN, HIGH);
					// }else if (adc0Pin == BSENSE || adc0Pin == CSENSE){
					// 	digitalWriteFast(LED_PIN, LOW);
					// }

					/*   BEMF Integration?  */

					//interpolation
					int deltaV = bemfReading - lastBemfReading;
					int deltaT = adcIsrTime -lastAdcIsrTime;
					lastZeroCrossTime = zeroCrossTime; //do I really need this?
					zeroCrossTime = (-lastBemfReading / deltaV ) * deltaT + lastAdcIsrTime;
					//zeroCrossTime = (-lastBemfReading / (bemfReading - lastBemfReading) ) * (adcIsrTime -lastAdcIsrTime) + lastAdcIsrTime
					//sensorlessCommDelay =
					zeroCrossFound = true;
				}

				//BEMF Integration?

				//Save measured BEMF voltage and time for referance next PWM cycle
				lastAdcIsrTime = adcIsrTime;
				lastBemfReading = bemfReading;
			}
			//////////////////////////////////////////////////////

      //Now that the criticle stuff is out of the way, lets get a throttle reading
      //(and temperature!)
      adc->startSynchronizedSingleRead(THROTTLE,FET_TEMP);

      //While thats converting, lets check if bus voltage has gotten to high
      // (like from uncontrolled regen)
      if(status.vBus_raw >= configData.maxBusVoltage){
        //this needs to do something more sophisticated
        faultStatus |= FAULT_CODE_OVER_VOLTAGE; //OV
//        controllerState = MC_STATE_FAULT;
        //faultShutdown();
				//bridgeOff();
				pwmout.pwmOUTMASK(0x3f); //just try turn off the bridge until I figure out the logic and write some code to transition out of FAULT or COAST states
      }

  }else if(adc0Pin == THROTTLE){ //Get the throttle and temp reading
     //throttle = adc->readSingle() - throttle_offset;
     syncReadResult = adc->readSynchronizedSingle();

		 status.throttleUpdate(syncReadResult.result_adc0);

     status.fetTempUpdate(syncReadResult.result_adc1);

  }else{
     //ADC0_RA; //reset the interupt flag anyway
  }

  //Lets update running avereges now that we are done
  //status.motorCurrent_RA.addValue(status.getMotorCurrent());
	//status.fetTemp_RA.addValue(status.fetTemp_raw);

}

////////////////////////////////////////////////
//Timer ISR called every 20 microseconds
void commutate(){

  //noInterrupts();
	//Running sensored
  if(feedbackMode == FB_MODE_HALL){

    hallState = (digitalReadFast(HALL3) << 2) |
                (digitalReadFast(HALL2) << 1) |
                (digitalReadFast(HALL1) << 0);

    if(motorDirection == DIR_FORWARD){
        commStep = hallOrder_1[hallState];
    }else{
        commStep = hallOrder_0[hallState];
    }


    if(commStep < 6){       // we have a valid hall state

      faultStatus &= ~FAULT_CODE_HALL_SENSOR;
      //doTachometer();
      tachometer.update(commStep);

      if(controllerState == MC_STATE_DRIVE){
        writeBridgeState(commStep);
      }else if(controllerState == MC_STATE_FAULT){
        writeBridgeState(BRIDGE_STATE_COAST); // Put the motor in coast (disconnected)
      }

    }else if(commStep >= 6){
      //Something is wrong!
      faultStatus |= FAULT_CODE_HALL_SENSOR;
      faultShutdown();
      //need to do more here
    }

  //Running sensored with an encoder
  }else if(feedbackMode == FB_MODE_ENC){
    //Need to implement

  //Running sensorless
}else if(feedbackMode == FB_MODE_BEMF){   //Must be sensorless
    //Need to implement
		switch(controllerState){
			case MC_STATE_DRIVE:
				if(zeroCrossFound){
					if(tachometer.getTimeSinceCommutation() >= ((zeroCrossTime * 2) + sensorlessCommAdvance)){
						//This is probably not right AND doesnt account for direction!
						commStep++;
						if(commStep >= 6){
							commStep = 0;
						}

						tachometer.update(commStep);
						zeroCrossFound = false;
					}
				}
				writeBridgeState(commStep);
				break;
			case MC_STATE_ALIGNMENT:
				writeBridgeState(BRIDGE_STATE_ALIGN); //Align the rotor
				static unsigned int alignmentCounter;
				if(alignmentCounter >= 500){ //500 20uS intervals (100mS) MAKE THIS CONFIGURABLE LATER!!
					lastControllerState = controllerState;
					controllerState = MC_STATE_START;
				}
				break;
			case MC_STATE_START:
				//Need to ramp up open loop here

				break;
			case MC_STATE_FAULT:
				writeBridgeState(BRIDGE_STATE_COAST);
				break;
		}
  }
//  }
  //interrupts();
}

/////////////////////////////////////////////////////
// write the state according to the position
void writeBridgeState(uint8_t pos){
	noInterrupts();
	zeroCrossFound = false; //!!!!!!this is just temporary for testing!!!!!
  switch(pos){
    case 0://LOW B, HIGH A

      if(regenEnabled){
        pwmout.setDutyCycle(dutyCycle, dutyCycle, dutyCycle);
        pwmout.pwmOUTMASK(0x34);    // Was 0x31
        pwmout.pwmSWOCTRL(0x0808);  // Was 0x0202
      }else{
        pwmout.setDutyCycle(dutyCycle, dutyCycle, dutyCycle);
        pwmout.pwmOUTMASK(0x36);    // Was 0x39
        pwmout.pwmSWOCTRL(0x0808);  // Was 0x0202
      }
      bemfPin = CSENSE; //C is the un-driven phase for BEMF sensing
			break;
    case 1://LOW C, HIGH A

      if(regenEnabled){
        pwmout.setDutyCycle(dutyCycle, dutyCycle, dutyCycle);
        pwmout.pwmOUTMASK(0x1C);    // Was 0x0D
        pwmout.pwmSWOCTRL(0x2020);  // Was 0x0202
      }else{
        pwmout.setDutyCycle(dutyCycle, dutyCycle, dutyCycle);
        pwmout.pwmOUTMASK(0x1E);    // Was 0x2D
        pwmout.pwmSWOCTRL(0x2020);  // Was 0x0202
      }
      bemfPin = BSENSE; //B is the un-driven phase for BEMF sensing
      break;
    case 2://LOW C, HIGH B

      if(regenEnabled){
        pwmout.setDutyCycle(dutyCycle, dutyCycle, dutyCycle);
        pwmout.pwmOUTMASK(0x13);    // Was 0x07
        pwmout.pwmSWOCTRL(0x2020);  // Was 0x0808
      }else{
        pwmout.setDutyCycle(dutyCycle, dutyCycle, dutyCycle);
        pwmout.pwmOUTMASK(0x1B);    // Was 0x27
        pwmout.pwmSWOCTRL(0x2020);  // Was 0x0808
      }
      bemfPin = ASENSE; //A is the un-driven phase for BEMF sensing
      break;
    case 3://LOW A, HIGH B

      if(regenEnabled){
        pwmout.setDutyCycle(dutyCycle, dutyCycle, dutyCycle);
        pwmout.pwmOUTMASK(0x31);    // Was 0x34
        pwmout.pwmSWOCTRL(0x0202);  // Was 0x0808
      }else{
        pwmout.setDutyCycle(dutyCycle, dutyCycle, dutyCycle);
        pwmout.pwmOUTMASK(0x39);    // Was 0x36
        pwmout.pwmSWOCTRL(0x0202);  // Was 0x0808
      }
      bemfPin = CSENSE; //C is the un-driven phase for BEMF sensing
      break;
    case 4://LOW A, HIGH C

      if(regenEnabled){
        pwmout.setDutyCycle(dutyCycle, dutyCycle, dutyCycle);
        pwmout.pwmOUTMASK(0x0D);    // Was 0x1C
        pwmout.pwmSWOCTRL(0x0202);  // Was 0x2020
      }else{
        pwmout.setDutyCycle(dutyCycle, dutyCycle, dutyCycle);
        pwmout.pwmOUTMASK(0x2D);    // Was 0x1E
        pwmout.pwmSWOCTRL(0x0202);  // Was 0x2020
      }
      bemfPin = BSENSE; //B is the un-driven phase for BEMF sensing
      break;
    case 5://LOW B, HIGH C

      if(regenEnabled){
        pwmout.setDutyCycle(dutyCycle, dutyCycle, dutyCycle);
        pwmout.pwmOUTMASK(0x07);    // Was 0x13
        pwmout.pwmSWOCTRL(0x0808);  // Was 0x2020
      }else{
        pwmout.setDutyCycle(dutyCycle, dutyCycle, dutyCycle);
        pwmout.pwmOUTMASK(0x27);    // Was 0x1B
        pwmout.pwmSWOCTRL(0x0808);  // Was 0x2020
      }
      bemfPin = ASENSE; //A is the un-driven phase for BEMF sensing
      break;

    case 10: //Special alignment vector
      pwmout.setDutyCycle(ALIGNMENT_DUTY, ALIGNMENT_DUTY, ALIGNMENT_DUTY);
      pwmout.pwmSWOCTRL(0x0A0A);
      pwmout.pwmOUTMASK(0x05);
      break;

    case 11: //Full brake (this will short the phases, BE CAREFULL!
      //pwmout.setDutyCycle(0, 0, 0); //This redundant and I am unsure of the behavior
      pwmout.pwmSWOCTRL(0x2A2A);
      pwmout.pwmOUTMASK(0x15);
      break;

    case 12: //Coast or Fault (may not be necessary to have here)
      //pwmout.pwmSWOCTRL(0x00); //Doesn't matter
      pwmout.pwmOUTMASK(0x3F);
  }
	interrupts();
}

//*******************************************************************
//*********** Serial command functions start here *******************

//////////////////////////////////////////////
//Output the firmware version
void cmdGetVersion(void){
  Serial.print("JJ_BLDC V");
  Serial.println(JJBLDC_FW_VERSION,2);
}

//////////////////////////////////////////////
//Start or Stop outputting continuous readings
void cmdStream(void){
  if(serialStreamFlag == false){
    serialStreamFlag = true;
  }else{
    serialStreamFlag = false;
  }
}

/////////////////////////////////////////////
// Saves config data to EEPROM just a command wrapper for now
void cmdSaveConfig(void){
  saveConfig();
}
///////////////////////////////////////////////
// Print out the configuration data. Mostly just for testing
void cmdPrintConfig(void){
  Serial.print("Throttle Max = ");
  Serial.println(configData.throttleOut_max);
  Serial.print("Throttle Min = ");
  Serial.println(configData.throttleOut_min);
  Serial.print("Duty Max = ");
  Serial.println(configData.dutyCycle_max);
  Serial.print("Duty Min = ");
  Serial.println(configData.dutyCycle_min);
  Serial.print("Pole Pairs = ");
  Serial.println(configData.polePairs);
  Serial.print("PWM Frequency = ");
  Serial.println(configData.pwmOutFreq);
  Serial.print("Control Mode = ");
  Serial.println(configData.controlMode);
  Serial.print("Current PID kP = ");
  Serial.println(configData.currentControl_kP);
  Serial.print("Current PID kI = ");
  Serial.println(configData.currentControl_kI);
  Serial.print("Current PID kD = ");
  Serial.println(configData.currentControl_kD);
  Serial.print("Max HW Current = ");
  Serial.println(configData.maxCurrent_HW);
  Serial.print("Max Motor Current = ");
  Serial.println(configData.maxCurrent_motor);
//  Serial.print("Config Version = ");
//  Serial.println(configData.configVersion);
}

///////////////////////////////////////////////////////////////////////////////
//set or get the throttle control mode
void cmdControlMode(){
	char *arg1 = sCmd.next();
	char *arg2 = sCmd.next();

  if(arg1 != NULL){// && arg2 != NULL){								//see if we got the arguments
		if(arg1[0] == 'w' && arg2 != NULL){								//see if we are writing a value...
			mc_control_mode mode = (mc_control_mode) atoi(arg2);
			if(mode >= CONTROL_MODE_DUTY && mode <= CONTROL_MODE_CURRENT){
				configData.controlMode = mode; 				   			//Probably need some validation here
			}
		}else if(arg1[0] == 'r'){ //											//or reading  a value
			Serial.println(configData.controlMode);
		}
  }else{
    Serial.println("Did you forget somthing?");		//let user know they screwed somthing up
  }
}

///////////////////////////////////////////////////////////////////////////////
// Set or get kP for current control loop
void cmdCurrentkP(){
	char *arg1 = sCmd.next();
	char *arg2 = sCmd.next();

  if(arg1 != NULL){// && arg2 != NULL){								//see if we got the arguments
		if(arg1[0] == 'w' && arg2 != NULL){								//see if we are writing a value...
			configData.currentControl_kP = atof(arg2); 			//Probably need some validation here
		}else if(arg1[0] == 'r'){ //											//or reading  a value
			Serial.println(configData.currentControl_kP);
		}
  }else{
    Serial.println("Did you forget somthing?");		//let user know they screwed somthing up
  }
}

///////////////////////////////////////////////////////////////////////////////
// Get or set Max motor current
void cmdMotorCurrentMax(){
	char *arg1 = sCmd.next();
	char *arg2 = sCmd.next();

  if(arg1 != NULL){// && arg2 != NULL){								//see if we got the arguments
		if(arg1[0] == 'w' && arg2 != NULL){								//see if we are writing a value...
			configData.maxCurrent_motor = voltsToCounts(atoi(arg2) / SHUNT_CURRENT_FACTOR); //Probably need some validation here
		}else if(arg1[0] == 'r'){ //											//or reading  a value
			Serial.println(countsToVolts(configData.maxCurrent_motor) * SHUNT_CURRENT_FACTOR);
		}
  }else{
    Serial.println("Did you forget somthing?");		//let user know they screwed somthing up
  }
}

///////////////////////////////////////////////////////////////////////////////
// Get or set min DRV8302 hardware current limit
void cmdHwCurrentLimit(){
	char *arg1 = sCmd.next();
	char *arg2 = sCmd.next();

  if(arg1 != NULL){// && arg2 != NULL){								 //see if we got the arguments
		if(arg1[0] == 'w' && arg2 != NULL){								 //see if we are writing a value...
			configData.maxCurrent_HW = voltsToCounts(atof(arg2) * FET_RDS); //Probably need some validation here
			analogWrite(HW_OC_ADJ, configData.maxCurrent_HW);// for right now. should probably update this somewhere else
		}else if(arg1[0] == 'r'){ //											 //or reading  a value
			Serial.println(countsToVolts(configData.maxCurrent_HW)/FET_RDS);
		}
  }else{
    Serial.println("Did you forget somthing?");		//let user know they screwed somthing up
  }
}

void cmdFetTempMax(){
	char *arg1 = sCmd.next();
	char *arg2 = sCmd.next();

  if(arg1 != NULL){// && arg2 != NULL){								//see if we got the arguments
		if(arg1[0] == 'w' && arg2 != NULL){								//see if we are writing a value...
			configData.maxFetTemp = atof(arg2); 				//Probably need some validation here
		}else if(arg1[0] == 'r'){ //											//or reading  a value
			Serial.println(configData.maxFetTemp);
		}
  }else{
    Serial.println("Did you forget somthing?");		//let user know they screwed somthing up
  }
}
////////////////////////////////////////////////
// Set or get the min throttle input
void cmdThrottleMin(){
	char *arg1 = sCmd.next();
	char *arg2 = sCmd.next();

  if(arg1 != NULL){// && arg2 != NULL){								//see if we got the arguments
		if(arg1[0] == 'w' && arg2 != NULL){								//see if we are writing a value...
			configData.throttleOut_min = atoi(arg2); 				//Probably need some validation here
		}else if(arg1[0] == 'r'){ //											//or reading  a value
			Serial.println(configData.throttleOut_min);
		}
  }else{
    Serial.println("Did you forget somthing?");		//let user know they screwed somthing up
  }
}

////////////////////////////////////////////////
// Set or get the max throttle input
void cmdThrottleMax(){
  char *arg1 = sCmd.next();
	char *arg2 = sCmd.next();

  if(arg1 != NULL){// && arg2 != NULL){								//see if we got the arguments
		if(arg1[0] == 'w' && arg2 != NULL){								//see if we are writing a value...
			configData.throttleOut_max = atoi(arg2); 				//Probably need some validation here
		}else if(arg1[0] == 'r'){ //											//or reading  a value
			Serial.println(configData.throttleOut_max);
		}
  }else{
    Serial.println("Did you forget somthing?");		//let user know they screwed somthing up
  }
}

////////////////////////////////////////////////
void cmdDutyCycMax(){
  char *arg1 = sCmd.next();
	char *arg2 = sCmd.next();

  if(arg1 != NULL){// && arg2 != NULL){								//see if we got the arguments
		if(arg1[0] == 'w' && arg2 != NULL){								//see if we are writing a value...
			configData.dutyCycle_max = atoi(arg2); 				//Probably need some validation here
		}else if(arg1[0] == 'r'){ //											//or reading  a value
			Serial.println(configData.dutyCycle_max);
		}
  }else{
    Serial.println("Did you forget somthing?");		//let user know they screwed somthing up
  }
}

void cmdRate(){
  char *arg;
  arg = sCmd.next();
  if(arg != NULL){
    serialUpdatePeriod = atoi(arg); //Probably need some validation here
  }else{
    Serial.println("Did you forget somthing?");
  }
}

///////////////////////////////////////////////
//

///////////////////////////////////////////////
//Handle an unknown command
void cmdUnknown(const char *command){
  Serial.println("What are you talking about?");
}
////////////////////////////////////////////////
//Test command
void cmdTest(){
	scopeCurrent1.trigger();
	//scopeCurrent2.trigger();
	scopeBemfVolts.trigger();
	scopeBusVolts.trigger();

	//doDc_Cal();
	// Serial.print(status.iSense1_offset);
	// Serial.print("\t");
	// Serial.print(status.iSense2_offset);
	// Serial.print("\t");
	// Serial.println(status.throttle_offset);

	// noInterrupts();
	// int temp1 = adc->analogRead(ISENSE1,ADC_1);
	// int temp2 = adc->analogRead(ISENSE2,ADC_0);
	// int temp3 = adc->analogRead(THROTTLE,ADC_0);
	// interrupts();

	// Serial.print(temp1);
	// Serial.print("\t");
	// Serial.print(temp2);
	// Serial.print("\t");
	// Serial.println(temp3);

	// Serial.print(status.iSense1_raw);
	// Serial.print("\t");
	// Serial.print(status.iSense2_raw);
	// Serial.print("\t");
	// Serial.println(status.getThrottle());

}
