/*
this file contains most of the analog (ADC) measurement stuff


*/
#pragma once

/*#include "main.cpp"*/ // this header file requires some things to be defined in main.cpp first
#include "TLB_pinouts.h" // the pinout should be included in main.cpp BEFORE this file is included
#include "TLB_logging.h" // for a few warning-prints

#include <Arduino.h>



//////////////////////////////////////// ADC measurement ////////////////////////////////////////
float mes_3V3() {
  //// TODO: measure internal Vref (NOTE: not sure of ESP32-S3 can actually do that?, i see DEFAULT_VREF being used in esp32-hal-adc.c)
  return(3.3);
}
float mes_5V() {
  //// TODO: improve accuracy by measuring 3V3 (periodically?)
  // return(analogRead() /*math*/) // do 3V3 measurement and ADC calibration manually
  const static float ADCtoVoltScale = ((ADC_5V_MES_RES_LOW + ADC_5V_MES_RES_HIGH) / ((float)ADC_5V_MES_RES_LOW)) / 1000.0; // 1/(Rl/(Rl+Rh)) = (Rl+Rh)/Rl (multiplication beats division)
  return(analogReadMilliVolts(PIN_5V_MES) * ADCtoVoltScale); // INEFFICIENT, but fancy ADC calibration (repeats calibration every time (i think))
}
float mes_12V() {
  //// TODO: improve accuracy by measuring 3V3 (periodically?)
  // return(analogRead() /*math*/) // do 3V3 measurement and ADC calibration manually
  const static float ADCtoVoltScale = ((ADC_12V_MES_RES_LOW + ADC_12V_MES_RES_HIGH) / ((float)ADC_12V_MES_RES_LOW)) / 1000.0; // 1/(Rl/(Rl+Rh)) = (Rl+Rh)/Rl (multiplication beats division)
  return(analogReadMilliVolts(PIN_12V_MES) * ADCtoVoltScale); // INEFFICIENT, but fancy ADC calibration (repeats calibration every time (i think))
}
float mes_VBAT() {
  //// TODO: improve accuracy by measuring 3V3 (periodically?)
  // return(analogRead() /*math*/) // do 3V3 measurement and ADC calibration manually
  const static float ADCtoVoltScale = ((ADC_VBAT_MES_RES_LOW + ADC_VBAT_MES_RES_HIGH) / ((float)ADC_VBAT_MES_RES_LOW)) / 1000.0; // 1/(Rl/(Rl+Rh)) = (Rl+Rh)/Rl (multiplication beats division)
  return(analogReadMilliVolts(PIN_VBAT_MES) * ADCtoVoltScale); // INEFFICIENT, but fancy ADC calibration (repeats calibration every time (i think))
}

float mes_Lcurrent() {
  //// TODO: calibrate!, and also measure 3V3 (periodically?). I'll add some averaging in another function
  const static float ADCtoAmpOffset = (-3.3 * 0.5) * 1000; // 0 Amps is at middle of voltage range
  const static float ADCtoAmpScale = 1.0/((float)ADC_CUR_SENSE_L_SCALE);
  return((analogReadMilliVolts(PIN_CUR_SENSE_L) + ADCtoAmpOffset) * ADCtoAmpScale); // INEFFICIENT, but fancy ADC calibration (repeats calibration every time (i think))
}
float mes_M1current() {
  //// TODO: calibrate!, and also measure 3V3 (periodically?). I'll add some averaging in another function
  const static float ADCtoAmpOffset = (-3.3 * 0.5) * 1000; // 0 Amps is at middle of voltage range
  const static float ADCtoAmpScale = 1.0/((float)ADC_CUR_SENSE_M1_SCALE);
  return((analogReadMilliVolts(PIN_CUR_SENSE_M1) + ADCtoAmpOffset) * ADCtoAmpScale); // INEFFICIENT, but fancy ADC calibration (repeats calibration every time (i think))
}
float mes_M2current() {
  //// TODO: calibrate!, and also measure 3V3 (periodically?). I'll add some averaging in another function
  const static float ADCtoAmpOffset = (-3.3 * 0.5) * 1000; // 0 Amps is at middle of voltage range
  const static float ADCtoAmpScale = 1.0/((float)ADC_CUR_SENSE_M2_SCALE);
  return((analogReadMilliVolts(PIN_CUR_SENSE_M2) + ADCtoAmpOffset) * ADCtoAmpScale); // INEFFICIENT, but fancy ADC calibration (repeats calibration every time (i think))
}



//// TODO: quick bounds check functions


//// some constants for interpreting battery voltages
const float _lithiumCellThresholds[4] = {3.25,3.65,4.25,4.45}; // (Volts) regular voltage thresholds for lithium cells
// const float _lithiumCellCalcLimit = 60.0f; // (Volts) don't bother counting cells beyond this voltage
const uint8_t _lithiumCellCalcLimit = 60.0f/_lithiumCellThresholds[1]; // (#ofCells) don't bother counting cells beyond this voltage

int8_t lithiumCellCalc(float VBAT_measurement) { // guesstimate how many (lithium) cells the battery has (to prevent over-discharging)
  const static float quickSkipThresh = _lithiumCellCalcLimit * _lithiumCellThresholds[2];
  if(VBAT_measurement > quickSkipThresh) { return(-1); } // don't even bother trying to count that high
  else if(VBAT_measurement < _lithiumCellThresholds[0]) { return(0); } // don't even bother trying to count that low

  int8_t extendedRangeCandidate = -2;
  for(uint8_t i=1; i<=_lithiumCellCalcLimit; i++) {
    float targets[3]; for(uint8_t j=0;j<3;j++){targets[j]=_lithiumCellThresholds[j]*i;} // inefficient/slow, but effective
    if((VBAT_measurement > targets[1]) && (VBAT_measurement < targets[2])) {
      return(i); // nice and normal situation
    } else if((VBAT_measurement > targets[0]) && (VBAT_measurement < targets[3])) { // extended battery voltage range
      if(extendedRangeCandidate < 0) { extendedRangeCandidate = i; } // store candidate for later
      else { // if there already IS a candidate (which can happen from 3S onwards)
        TLB_log_w("battery voltage fits into multiple extended ranges! %.2f (%uS -> %uS)",VBAT_measurement,extendedRangeCandidate,i); // warning (disabled in release-compile)
        extendedRangeCandidate = i; // update extendedRangeCandidate, to assume the HIGHER of the two cell counts (which should trigger low-battery warnings)
      }
    } else if(VBAT_measurement < targets[0]) { // battery voltage lies between ranges, shit's ambiguous
      if(extendedRangeCandidate > 0) { // if a candidate has been selected
        TLB_log_w("battery voltage only in extended range! %.2f (%uS)",VBAT_measurement,extendedRangeCandidate); // warning (disabled in release-compile)
      }
      return(extendedRangeCandidate);
    }
  }
  if(extendedRangeCandidate > 0) {
    TLB_log_w("battery voltage only in extended range! %.2f (%uS)",VBAT_measurement,extendedRangeCandidate); // warning (disabled in release-compile)
    return(extendedRangeCandidate);
  } /*else*/ return(-3); // it should never make it to this point, so if it does, just panic
}