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

//////////////////////////////////////////////////////////////////
// acquisition Buffer member functions
//
acquisitionBuffer::acquisitionBuffer(int size){
  _size = size;
  _buffer = (int*) malloc(_size * sizeof(int));
  if (_buffer == NULL) _size = 0;
  clear();
}

acquisitionBuffer::~acquisitionBuffer(void){
  if (_buffer != NULL) free(_buffer);
}

void acquisitionBuffer::addSample(int value){
  if (acquisitionState == preTrigger || acquisitionState == postTrigger){ //only add if we are aquiring
    if (_buffer == NULL) return;

    _buffer[_idx] = value;

    _idx++;
    if (_idx == _size) _idx = 0;  // if at the end, loop around to 0

    if (acquisitionState == postTrigger){
      if(_postTriggerSamples < _size / 2){
        _postTriggerSamples++;
      }else{
        acquisitionState = complete;
        _postTriggerSamples = 0;
      }
    }
  }
}

int acquisitionBuffer::getSample(unsigned int sampleIndex){

  if(acquisitionState == complete){
    sampleIndex = _idx + sampleIndex;
    if (sampleIndex >= _size) sampleIndex = 0;  // if at the end, loop around to 0 (used >= as crude way to catch out-of-bounds)
    return _buffer[sampleIndex];
  }else{
    return NAN;
  }
}

bool acquisitionBuffer::samplesReady(void){
  if(acquisitionState == complete){
    return true;
  }else{
    return false;
  }
}

void acquisitionBuffer::trigger(void){
  acquisitionState = postTrigger;
}

void acquisitionBuffer::arm(void){
  _postTriggerSamples = 0;
  _idx = 0; //why not reset this too?
  acquisitionState = preTrigger;
}

////////////////////////////////////////////////////////////////////////////////////////////
//This actually wont get used. I just realized that if this is a triggered acquisition,
//we can't be doing a running average off the same buffer.If we did, the rusult would be
// frozen everytime there is a trigger! duh. I'll it keep here for now, as a referance
int acquisitionBuffer::getAverage(int windowSize){

  //I'd rather add to the buffer as fast as possible, so I'll calc the sum only when asked for
  int pos = _idx - windowSize;
  int sum = 0;

  if (pos < 0){
    pos = _size - pos;  //Loop back from end of array
  }

  for(int i = 0; i < _idx; i++){
    if(pos++ == _size) pos = 0; //If we hit the end, wrap arround
    sum += _buffer[pos];
  }
  //return the average
  return sum / windowSize;
}

void acquisitionBuffer::clear(void){
  _postTriggerSamples = 0;
  _idx = 0;
  for (int i = 0; i< _size; i++) _buffer[i] = 0;
}
