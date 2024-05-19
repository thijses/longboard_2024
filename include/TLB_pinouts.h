/*
this file defines all the pins for each PCB revision


NOTE: the RBG LED pin is defined in the custom pins_arduino.h header (see boards/variants)
*/
#pragma once

/*#include "main.cpp"*/ // this header file requires some things to be defined in main.cpp first



/////////////////////////////////////// pinout definition ///////////////////////////////////////
#ifdef PCB_R01
  //// ADC inputs (note: ADC1 only, as ADC2 conflicts with WiFi)
  #define PIN_5V_MES        6 // simple voltage divider to measure 5V supply
  #define PIN_12V_MES       7 // simple voltage divider to measure 12V supply
  #define PIN_VBAT_MES      3 // simple voltage divider to measure VBAT
  #define PIN_CUR_SENSE_L   9 // 10A current sensor for everything but the motors (e.g. power supplies). Measured at VBAT high-side
  #define PIN_CUR_SENSE_M1  8 // 30A current sensor for motor 1. Measured at VBAT high-side
  #define PIN_CUR_SENSE_M2 10 // 30A current sensor for motor 2. Measured at VBAT high-side

  //// digital inputs
  #define PIN_TPS_PG       37 // TPS82130 (12V->5V) Power-Good
  #define PIN_MAX_PG       35 // MAX17504 (VBAT->12V) Power-Good

  #define PIN_HALL_BEMF_1A 18 // (digital) 
  #define PIN_HALL_BEMF_1B  4 // (analog)
  #define PIN_HALL_BEMF_1C  5 // (analog)
  #define PIN_HALL_BEMF_2A  1 // (analog) (shared)
  #define PIN_HALL_BEMF_2B  2 // (analog) (shared)
  #define PIN_HALL_BEMF_2C 38 // (digital)
  #define PCB_HALL_PULLUPS Pullup::USE_INTERN // (see simpleFOC library) PCB R01 has no external pullups

  //// digital outputs
  #define PIN_TPS_EN       39 // TPS82130 (12V->5V) ENable
  #define PIN_MAX_EN       36 // MAX17504 (VBAT->12V) ENable (via EVLO pin)
  //// NOTE: the RBG LED pin is defined in the custom pins_arduino.h header (see boards/variants)
  // #define PIN_RGB_LED_1    41 // a little RGB LED on the side of the PCB

  //// digital GPIO (TBD)
  #define PIN_ENC1_SDA      1 // (shared) 
  #define PIN_ENC1_SCL      2 // (shared) 
  #define PIN_ENC2_SDA     18 // (shared) 
  #define PIN_ENC2_SCL      4 // (shared) 
  #define PIN_EXT_PERIPH_1 42 // one of the pins on the extra peripheral connector (X104.2)
  #define PIN_EXT_PERIPH_2 40 // one of the pins on the extra peripheral connector (X104.3)

  //// ESC driver pins
  #define ESC1_HIN1        12 // motor 1 driver High-side input 1
  #define ESC1_LIN1        15 // motor 1 driver Low-side input 1
  #define ESC1_HIN2        11 // motor 1 driver High-side input 2
  #define ESC1_LIN2        16 // motor 1 driver Low-side input 2
  #define ESC1_HIN3        46 // motor 1 driver High-side input 3
  #define ESC1_LIN3        17 // motor 1 driver Low-side input 3

  #define ESC2_HIN1        45 // motor 2 driver High-side input 1
  #define ESC2_LIN1        21 // motor 2 driver Low-side input 1
  #define ESC2_HIN2        48 // motor 2 driver High-side input 2
  #define ESC2_LIN2        14 // motor 2 driver Low-side input 2
  #define ESC2_HIN3        47 // motor 2 driver High-side input 3
  #define ESC2_LIN3        13 // motor 2 driver Low-side input 3

  //// voltage measurement resistor-dividers
  #define ADC_5V_MES_RES_LOW  3300 // (Ohms) resistor to GND for measuring 5V supply
  #define ADC_5V_MES_RES_HIGH 4700 // (Ohms) resistor to source for measuring 5V supply
  #define ADC_12V_MES_RES_LOW  4700  // (Ohms) resistor to GND for measuring 12V supply
  #define ADC_12V_MES_RES_HIGH 20000 // (Ohms) resistor to source for measuring 12V supply
  #define ADC_VBAT_MES_RES_LOW  6800   // (Ohms) resistor to GND for measuring VBAT
  #define ADC_VBAT_MES_RES_HIGH 120000 // (Ohms) resistor to source for measuring VBAT

  //// current measurement scaling: https://atta.szlcsc.com/upload/public/pdf/source/20210401/C469386_736CACCB33E89B0F82D9E8414A64AB06.pdf
  #define ADC_CUR_SENSE_L_SCALE   132 // (mV/A) V = VCC/2 + scale*A
  #define ADC_CUR_SENSE_M1_SCALE  44  // (mV/A) V = VCC/2 + scale*A
  #define ADC_CUR_SENSE_M2_SCALE  44  // (mV/A) V = VCC/2 + scale*A
  #define ESC_CURRENT_SENSOR_MAX  (30.0f * 0.9f) // (Amps) i'm using the CC6903SO-30A hall-effect current sensor. Only using 90% of range, to have room to detect issues

  //// digital pin active-states
  #define PIN_TPS_PG_ACTIVE   HIGH // TPS82130 (12V->5V) Power-Good
  #define PIN_MAX_PG_ACTIVE   HIGH // MAX17504 (VBAT->12V) Power-Good
  #define PIN_TPS_EN_ACTIVE   HIGH // TPS82130 (12V->5V) ENable
  //#define PIN_TPS_EN_HIGH_Z   (!PIN_TPS_PG_ACTIVE) // define this to initialze this pin as open-drain or open-source
  #define PIN_MAX_EN_ACTIVE   HIGH // MAX17504 (VBAT->12V) ENable (via EVLO pin)
  #define PIN_MAX_EN_HIGH_Z   (!PIN_MAX_EN_ACTIVE) // define this to initialze this pin as open-drain or open-source

#else // if no PCB revision is specified
  #error("no PCB revision specified") // note: this error only needs to be checked for once
#endif


//// TODO: define external peripheral pin definitions (which refer back to PIN_EXT_PERIPH_ pins)