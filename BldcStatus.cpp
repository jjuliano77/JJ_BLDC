/*
  BldcStatus.cpp - Class for storing and managing the current status of the motor

  Jaimy Juliano
*/
#include "bldcStatus.h"


int BldcStatus::getPhaseCurrent_Raw(void){

}

int BldcStatus::getFilteredPhaseCurrent_Raw(void){

}

float BldcStatus::getPhaseCurrent_Amps(void){

}

float BldcStatus::getFilteredPhaseCurrent_Amps(void){

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
