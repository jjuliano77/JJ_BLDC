/*
  BldcStatus.cpp - Class for storing and managing the current status of the motor

  Jaimy Juliano
*/
#include "bldcStatus.h"


int BldcStatus::getPhaseCurrent_Raw(void){
  return 0;
}

int BldcStatus::getFilteredPhaseCurrent_Raw(void){
  return 0;
}

float BldcStatus::getPhaseCurrent_Amps(void){
  return 0.0;
}

float BldcStatus::getFilteredPhaseCurrent_Amps(void){
  return 0.0;
}

float BldcStatus::getBusVoltage(void){
  return vBus_raw * BUS_VOLTAGE_FACTOR;
}

//Get the resistanc of the thermistor
int BldcStatus::calcNtcRes(int adc_val){
  return ((4095.0 * 10000.0) / adc_val - 10000.0);
}

//Calculate temperature in Degrees C
float BldcStatus::getFetTemp_DegC(void){
  return (1.0 / ((logf(calcNtcRes(fetTemp_raw) / 10000.0) / 3434.0) + (1.0 / 298.15)) - 273.15);
}

///////////////////////////////////////////////////////////
// BldcMeter Class Member functions

BldcMeter::BldcMeter(int samplesToAverage,char *units, bool isThermistor) : filteredValue(samplesToAverage){
  //RunningAverage *filteredValue = new RunningAverage(samplesToAverage); //< ^ Not sure which is more appropriate
  _isThermistor = isThermistor;
  _units = units;
}

float BldcMeter::getScaledValue(void){
  if(_isThermistor){
    return calcNtcTemperature(_rawValue);
  }else{
    return _rawValue * _scaleFactor;
  }
}

void BldcMeter::setOffset(int val){
  _offset = val;
}

void BldcMeter::setScaleFactor(float val){
  _scaleFactor = val;
}

int BldcMeter::getRawValueFiltered(void){
  return (int) filteredValue.getAverage();
}

float BldcMeter::getScaledValueFiltered(void){
  if(_isThermistor){
    return calcNtcTemperature(filteredValue.getAverage());
  }else{
    return getRawValueFiltered() * _scaleFactor;
  }
}

float BldcMeter::calcNtcTemperature(int adcVal){
  int r = ((4095.0 * 10000.0) / adcVal - 10000.0);
  return (1.0 / ((logf(r / 10000.0) / 3434.0) + (1.0 / 298.15)) - 273.15);
}
