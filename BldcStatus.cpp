/*
  BldcStatus.cpp - Class for storing and managing the current status of the motor

  Jaimy Juliano
*/
#include "bldcStatus.h"


BldcStatus::BldcStatus()
{

}

BldcStatus::~BldcStatus()
{
  //Really should never get here
}

//Store a new phase current reading
void BldcStatus::updatePhaseCurrents(uint16_t adc_val1, uint16_t adc_val2)
{

}

//Get the resistanc of the thermistor
uint16_t BldcStatus::calcNtcRes(uint16_t adc_val)
{
  return ((4095.0 * 10000.0) / adc_val - 10000.0);
}

//Calculate temperature in Degrees C
float BldcStatus::getFetTemp_DegC(uint16_t adc_val)
{
  return (1.0 / ((logf(calcNtcRes(adc_val) / 10000.0) / 3434.0) + (1.0 / 298.15)) - 273.15);
}
