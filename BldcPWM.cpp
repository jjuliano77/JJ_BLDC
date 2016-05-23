/*
  BldcPWM.cpp - Library for managing PWM generation for BLDC motor
  control with the Teensy 3.1
  
  Jaimy Juliano
*/

//#include "kinetis.h"
#include "BldcPWM.h"

BldcPWM::BldcPWM(uint8_t mode = PWM_UNIPOLAR, boolean alignment = PWM_EDGE, uint16_t pwmFreq = 8000, uint8_t deadTime = 4)
{
  //deadTime default of 83.32nS, no idea what it should be??
  
  _mode = mode;
  //_pwmFreq = pwmFreq;
  //_deadTime = deadTime;
  _alignment = alignment;
  
  //Enable the Clock to the FTM0 Module
  SIM_SCGC6 |= SIM_SCGC6_FTM0;  
  
  // Pin Settings
  PORTC_PCR1 = PORT_PCR_MUX(0x4)  | PORT_PCR_DSE; // FTM0_CHN0 Pin 22
  PORTC_PCR2 = PORT_PCR_MUX(0x4)  | PORT_PCR_DSE; // FTM0_CHN1 Pin 23
  PORTC_PCR3 = PORT_PCR_MUX(0x4)  | PORT_PCR_DSE; // FTM0_CHN2 Pin 9
  PORTC_PCR4 = PORT_PCR_MUX(0x4)  | PORT_PCR_DSE; // FTM0_CHN3 Pin 10
  PORTD_PCR4 = PORT_PCR_MUX(0x4)  | PORT_PCR_DSE; // FTM0_CHN4 Pin 6
  PORTD_PCR5 = PORT_PCR_MUX(0x4)  | PORT_PCR_DSE; // FTM0_CHN5 Pin 20
  
  /* 36.4.9 Complementary mode
  The Complementary mode is selected when:
  FTMEN = 1
  QUADEN = 0
  DECAPEN = 0
  COMBINE = 1
  CPWMS = 0, and
  COMP = 1
  */
  
//*****************************************************************************************  
 
  FTM0_MODE |= FTM_MODE_WPDIS; //Disable Write Protection - enables changes to QUADEN, DECAPEN, etc.
  FTM0_MODE |= FTM_MODE_FTMEN; //FTMEN is bit 0 set to 1
  FTM0_QDCTRL &=~FTM_QDCTRL_QUADEN;  //Disable Encoder mode
  
  // Setup frequency
  this->setPwmFreq(pwmFreq);

  // FTMx_CnSC - contains the channel-interrupt status flag control bits
  FTM0_C0SC |= FTM_CSC_ELSB; //Edge or level select
  FTM0_C0SC &= ~FTM_CSC_ELSA; //Edge or level Select
  FTM0_C0SC |= FTM_CSC_MSB; //Channel Mode select (shouldn't matter in COMBINE mode)

  // FTMx_CnSC - contains the channel-interrupt status flag control bits
  FTM0_C1SC |= FTM_CSC_ELSB; //Edge or level select
  FTM0_C1SC &= ~FTM_CSC_ELSA; //Edge or level Select
  FTM0_C1SC |= FTM_CSC_MSB; //Channel Mode select (shouldn't matter in COMBINE mode)

  // FTMx_CnSC - contains the channel-interrupt status flag control bits
  FTM0_C2SC |= FTM_CSC_ELSB; //Edge or level select
  FTM0_C2SC &= ~FTM_CSC_ELSA; //Edge or level Select
  FTM0_C2SC |= FTM_CSC_MSB; //Channel Mode select (shouldn't matter in COMBINE mode)

  // FTMx_CnSC - contains the channel-interrupt status flag control bits
  FTM0_C3SC |= FTM_CSC_ELSB; //Edge or level select
  FTM0_C3SC &= ~FTM_CSC_ELSA; //Edge or level Select
  FTM0_C3SC |= FTM_CSC_MSB; //Channel Mode select (shouldn't matter in COMBINE mode)

  // FTMx_CnSC - contains the channel-interrupt status flag control bits
  FTM0_C4SC |= FTM_CSC_ELSB; //Edge or level select
  FTM0_C4SC &= ~FTM_CSC_ELSA; //Edge or level Select
  FTM0_C4SC |= FTM_CSC_MSB; //Channel Mode select (shouldn't matter in COMBINE mode)

  // FTMx_CnSC - contains the channel-interrupt status flag control bits
  FTM0_C5SC |= FTM_CSC_ELSB; //Edge or level select
  FTM0_C5SC &= ~FTM_CSC_ELSA; //Edge or level Select
  FTM0_C5SC |= FTM_CSC_MSB; //Channel Mode select (shouldn't matter in COMBINE mode)
  
  if((_mode == PWM_UNIPOLAR) || (_mode == PWM_BIPOLAR) || (_mode == PWM_SINE)){
    //Setup COMBINE/COMPLEMENTARY mode for the 3 channel pairs
    //with Deadtime and PWM Syncronization ENABLED 
    FTM0_COMBINE = FTM_COMBINE_DTEN0|FTM_COMBINE_SYNCEN0|FTM_COMBINE_COMBINE0|FTM_COMBINE_COMP0|
                   FTM_COMBINE_DTEN1|FTM_COMBINE_SYNCEN1|FTM_COMBINE_COMBINE1|FTM_COMBINE_COMP1|
                   FTM_COMBINE_DTEN2|FTM_COMBINE_SYNCEN2|FTM_COMBINE_COMBINE2|FTM_COMBINE_COMP2;

  }
  
  this->setDeadTime(deadTime);
  this->setDutyCycle(0,0,0);
  
}

//***** Sets the PWM frequency **********************
//Code is mostly taken straight form "pins_teensy.c"
void BldcPWM::setPwmFreq(uint16_t pwmFreq)
{
  uint32_t mod,prescale,minfreq;
 
  //Check if frequency is valid before setting
  if(pwmFreq > MAX_PWM_FREQ){                          //Check if frequency is valid before setting
    pwmFreq = MAX_PWM_FREQ;
  }else if(pwmFreq < MIN_PWM_FREQ){
    pwmFreq = MIN_PWM_FREQ;
  }
  
  //Seek out the best prescale value for the requested frequency
  for (prescale = 0; prescale < 7; prescale++) {
    minfreq = (F_BUS >> 16) >> prescale;
    if (pwmFreq > minfreq) break;
  }
  //The FTM0_MOD value can be determined with by ((F_BUS >> FTM0_PRESCALE) / frequency) - 1
  mod = (((F_BUS >> prescale) + (pwmFreq >> 1)) / pwmFreq) -1;
  if (mod > 65535) mod = 65535;
  
  FTM0_SC  = 0;                                     // Stop FTM0
  FTM0_CNT = 0;                                     // FTM Counter Value - reset counter to zero
  FTM0_MOD = mod;                                   // Set caculated MOD value
  FTM0_SC  = FTM_SC_CLKS(1) | FTM_SC_PS(prescale);  // Set clock source to "system clock"
                                                    // and set prescale
  
  //FTM0_CNTIN = 0; //Set the Counter Initial Value to 0
}

//***** Sets the deadtime in F_BUS clock cycles ***************
// 1 tick = 20.83nS. The max value is 63 (so 1.3125uS with a prescaler of 1)
void BldcPWM::setDeadTime(uint8_t deadTime)
{
  
  FTM0_DEADTIME = FTM_DEADTIME_DTPS(DTPS_SETTING) | FTM_DEADTIME_DTVAL(deadTime); 
  
}
//***** Set the duty cycle of all phases *******
// May not end up using this
void BldcPWM::setDutyCycle(uint16_t duty)
{
  //Not sure if we are really going to need this one
}

//***** Set PWM alignment (CENTER or EDGE) *****
void BldcPWM::setAlignment(boolean alignment)
{
  _alignment = alignment;
}

//***** Set the duty cycle for each phase seperatly *****
void BldcPWM::setDutyCycle(uint16_t aDuty, uint16_t bDuty, uint16_t cDuty)
{
  
  if(_alignment == PWM_CENTER){
    //This should result in a center aligned PWM waveform. If we want edge aligned
    //this will need to be done differently (see below)
    uint32_t aVal = (((uint32_t) aDuty * (uint32_t) (FTM0_MOD + 1)) >> PWM_OUT_RESOLUTION) / 2;
    uint32_t bVal = (((uint32_t) bDuty * (uint32_t) (FTM0_MOD + 1)) >> PWM_OUT_RESOLUTION) / 2;
    uint32_t cVal = (((uint32_t) cDuty * (uint32_t) (FTM0_MOD + 1)) >> PWM_OUT_RESOLUTION) / 2;
    
    unsigned mod = FTM0_MOD/2;
    FTM0_C0V = mod-aVal;
    FTM0_C1V = mod+aVal;
    FTM0_C2V = mod-bVal;
    FTM0_C3V = mod+bVal;
    FTM0_C4V = mod-cVal;
    FTM0_C5V = mod+cVal;
    FTM0_PWMLOAD |= FTM_PWMLOAD_LDOK;
    
  }else{
    uint32_t aVal = ((uint32_t) aDuty * (uint32_t) (FTM0_MOD + 1)) >> PWM_OUT_RESOLUTION;
    uint32_t bVal = ((uint32_t) bDuty * (uint32_t) (FTM0_MOD + 1)) >> PWM_OUT_RESOLUTION;
    uint32_t cVal = ((uint32_t) cDuty * (uint32_t) (FTM0_MOD + 1)) >> PWM_OUT_RESOLUTION;
    
    FTM0_C0V = 0;
    FTM0_C1V = aVal;
    FTM0_C2V = 0;
    FTM0_C3V = bVal;
    FTM0_C4V = 0;
    FTM0_C5V = cVal;
    FTM0_PWMLOAD |= FTM_PWMLOAD_LDOK;
    
  }
    
}
//****** Set the duty cycle for each phase ******************************
// This version uses a float representation of percentage (10% = 0.10)
// Not sure if I want to go this route, but it's ready if I do -JJ 5-31-15
void BldcPWM::setDutyCycleFloat(float aDuty, float bDuty, float cDuty)
{
  if(_alignment == PWM_CENTER){
    uint32_t aVal = ((uint32_t) aDuty * FTM0_MOD) / 2;
    uint32_t bVal = ((uint32_t) bDuty * FTM0_MOD) / 2;
    uint32_t cVal = ((uint32_t) cDuty * FTM0_MOD) / 2;
  
    unsigned mod = FTM0_MOD/2;
    FTM0_C0V = mod-aVal;
    FTM0_C1V = mod+aVal;
    FTM0_C2V = mod-bVal;
    FTM0_C3V = mod+bVal;
    FTM0_C4V = mod-cVal;
    FTM0_C5V = mod+cVal;
    FTM0_PWMLOAD |= FTM_PWMLOAD_LDOK;
    
  }else{
    uint32_t aVal = (uint32_t) aDuty * FTM0_MOD;
    uint32_t bVal = (uint32_t) bDuty * FTM0_MOD;
    uint32_t cVal = (uint32_t) cDuty * FTM0_MOD;
  
    FTM0_C0V = 0;
    FTM0_C1V = aVal;
    FTM0_C2V = 0;
    FTM0_C3V = bVal;
    FTM0_C4V = 0;
    FTM0_C5V = cVal;
    FTM0_PWMLOAD |= FTM_PWMLOAD_LDOK; 
    
  }
  
}

void BldcPWM::setPwmMode(uint8_t mode)
{
  //Do stuff
  _mode = mode;
  
  if((_mode == PWM_UNIPOLAR) || (_mode == PWM_BIPOLAR) || (_mode == PWM_SINE)){
    //Setup COMBINE/COMPLEMENTARY mode for the 3 channel pairs
    //with Deadtime and PWM Syncronization ENABLED 
    FTM0_COMBINE = FTM_COMBINE_DTEN0|FTM_COMBINE_SYNCEN0|FTM_COMBINE_COMBINE0|FTM_COMBINE_COMP0|
                   FTM_COMBINE_DTEN1|FTM_COMBINE_SYNCEN1|FTM_COMBINE_COMBINE1|FTM_COMBINE_COMP1|
                   FTM_COMBINE_DTEN2|FTM_COMBINE_SYNCEN2|FTM_COMBINE_COMBINE2|FTM_COMBINE_COMP2;

  }else{
    //Do stuff here -figure out how to setup independant mode
    //and frankly, if there is any reason to???
  }
  
}

//***** Turn all phases off (using mask for now) *****
//May need a exact function name since PWM is still runnning
//void BldcPWM::pwmDisable()
//{
//  //do stuff
//  this->pwmOUTMASK(0xff);
//}

//***** This is really just for convienience and readability ******
void BldcPWM::pwmOUTMASK(uint8_t mask)
{
  FTM0_OUTMASK = mask;

  //***************************** FYI **************************  
  //
  //  FTM_OUTMASK_CH7OM		0x80				//
  //  FTM_OUTMASK_CH6OM		0x40				//
  //  FTM_OUTMASK_CH5OM		0x20				//
  //  FTM_OUTMASK_CH4OM		0x10				//
  //  FTM_OUTMASK_CH3OM		0x08				//
  //  FTM_OUTMASK_CH2OM		0x04				//
  //  FTM_OUTMASK_CH1OM		0x02				//
  //  FTM_OUTMASK_CH0OM		0x01				//
}

//***** This is really just for convienience and readability ******
void BldcPWM::pwmSWOCTRL(uint16_t bits) 
{
  FTM0_SWOCTRL = bits;
  
  //***************************** FYI **************************  
  //
  //  FTM_SWOCTRL_CH7OCV	0x8000				// Channel 7 Software Output Control Value
  //  FTM_SWOCTRL_CH6OCV	0x4000				// Channel 6 Software Output Control Value
  //  FTM_SWOCTRL_CH5OCV	0x2000				// Channel 5 Software Output Control Value
  //  FTM_SWOCTRL_CH4OCV	0x1000				// Channel 4 Software Output Control Value
  //  FTM_SWOCTRL_CH3OCV	0x0800				// Channel 3 Software Output Control Value
  //  FTM_SWOCTRL_CH2OCV	0x0400				// Channel 2 Software Output Control Value
  //  FTM_SWOCTRL_CH1OCV	0x0200				// Channel 1 Software Output Control Value
  //  FTM_SWOCTRL_CH0OCV	0x0100				// Channel 0 Software Output Control Value
  //  FTM_SWOCTRL_CH7OC		0x0080				// Channel 7 Software Output Control Enable
  //  FTM_SWOCTRL_CH6OC		0x0040				// Channel 6 Software Output Control Enable
  //  FTM_SWOCTRL_CH5OC		0x0020				// Channel 5 Software Output Control Enable
  //  FTM_SWOCTRL_CH4OC		0x0010				// Channel 4 Software Output Control Enable
  //  FTM_SWOCTRL_CH3OC		0x0008				// Channel 3 Software Output Control Enable
  //  FTM_SWOCTRL_CH2OC		0x0004				// Channel 2 Software Output Control Enable
  //  FTM_SWOCTRL_CH1OC		0x0002				// Channel 1 Software Output Control Enable
  //  FTM_SWOCTRL_CH0OC		0x0001				// Channel 0 Software Output Control Enable
}
