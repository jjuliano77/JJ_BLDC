/*
  BldcStatus.cpp - Class for storing and managing the current status of the motor

  Jaimy Juliano
*/

BldcStatus::BldcStatus(void)
{

}

BldcStatus::~BldcStatus(void)
{
  //Really should never get here
}

//Store a new phase current reading
void BldcStatus::updatePhaseCurrent(uint16_t adc_val)
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
  return (1.0 / ((logf(getNtcRes(adc_val) / 10000.0) / 3434.0) + (1.0 / 298.15)) - 273.15);
}
