/*
this file contains most of the analog (ADC) measurement stuff


*/

/*#include "main.cpp"*/ // this header file requires some things to be defined in main.cpp first
#include "TLB_pinouts.h" // the pinout should be included in main.cpp BEFORE this file is included

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
  #define ADC_CUR_SENSE_L_SCALE   132 // (mV/A) V = VCC/2 + scale*A
  #define ADC_CUR_SENSE_M1_SCALE  44  // (mV/A) V = VCC/2 + scale*A
  #define ADC_CUR_SENSE_M2_SCALE  44  // (mV/A) V = VCC/2 + scale*A
float mes_Lcurrent() {
  //// TODO: calibrate!, and also measure 3V3 (periodically?). I'll add some averaging in another function
  const static float ADCtoAmpOffset = (-3.3 / 2) * 1000; // 0 Amps is at middle of voltage range
  const static float ADCtoAmpScale = 1.0/((float)ADC_CUR_SENSE_L_SCALE) / 1000.0;
  return((analogReadMilliVolts(PIN_CUR_SENSE_L) + ADCtoAmpOffset) * ADCtoAmpScale); // INEFFICIENT, but fancy ADC calibration (repeats calibration every time (i think))
}
float mes_M1current() {
  //// TODO: calibrate!, and also measure 3V3 (periodically?). I'll add some averaging in another function
  const static float ADCtoAmpOffset = (-3.3 / 2) * 1000; // 0 Amps is at middle of voltage range
  const static float ADCtoAmpScale = 1.0/((float)ADC_CUR_SENSE_M1_SCALE) / 1000.0;
  return((analogReadMilliVolts(PIN_CUR_SENSE_M1) + ADCtoAmpOffset) * ADCtoAmpScale); // INEFFICIENT, but fancy ADC calibration (repeats calibration every time (i think))
}
float mes_M2current() {
  //// TODO: calibrate!, and also measure 3V3 (periodically?). I'll add some averaging in another function
  const static float ADCtoAmpOffset = (-3.3 / 2) * 1000; // 0 Amps is at middle of voltage range
  const static float ADCtoAmpScale = 1.0/((float)ADC_CUR_SENSE_M2_SCALE) / 1000.0;
  return((analogReadMilliVolts(PIN_CUR_SENSE_M2) + ADCtoAmpOffset) * ADCtoAmpScale); // INEFFICIENT, but fancy ADC calibration (repeats calibration every time (i think))
}



//// TODO: quick bounds check functions