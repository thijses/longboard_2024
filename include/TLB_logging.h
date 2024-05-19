/*
this file contains code for logging important debug data for field-testing


TODO:
- add a .ini file define which changes the level of logging. (maybe for release only errors?)

*/
#pragma once

/*#include "main.cpp"*/ // this header file requires some things to be defined in main.cpp first

#include <Arduino.h>



//////////////////////////////////////// ... ////////////////////////////////////////


#define TLB_log_e(format, ...)  log_e(format, ##__VA_ARGS__)  /*flash logging here*/
#define TLB_log_w(format, ...)  log_w(format, ##__VA_ARGS__)  /*flash logging here*/
#define TLB_log_i(format, ...)  log_i(format, ##__VA_ARGS__) // no logging, just discard
#define TLB_log_d(format, ...)  log_d(format, ##__VA_ARGS__)  /*flash logging here (sometimes)*/
#define TLB_log_v(format, ...)  log_v(format, ##__VA_ARGS__) // no logging, just discard
// #define TLB_log_e(TLB_log_code, ...)  log_e(+TLB_LOG_CODE_STRINGS[TLB_log_code]+, ##__VA_ARGS__)  /*flash logging here*/
// #define TLB_log_w(TLB_log_code, ...)  log_w(+TLB_LOG_CODE_STRINGS[TLB_log_code]+, ##__VA_ARGS__)  /*flash logging here*/
// #define TLB_log_d(TLB_log_code, ...)  log_d(+TLB_LOG_CODE_STRINGS[TLB_log_code]+, ##__VA_ARGS__)  /*flash logging here (sometimes)*/

// enum TLB_LOG_CODE_ENUM : uint8_t {
//   TLB_LOG_reserved0=0,
//   TLB_LOG_BOOT, // {%s,%s} (date, time)
//   TLB_LOG_BAT_CELLS_UNCERTAINTY,
//   TLB_LOG_BAT_CELLS_ONLY_EXTENDED,
//   TLB_LOG_reserved4, // for radio purposes (basic connection established?)
//   TLB_LOG_reserved5, // for radio purposes (RTC time sync?)
//   TLB_LOG_reserved6, // for radio purposes (bad handshake?)
//   TLB_LOG_reserved7, // for radio purposes (disconnect?)
//   TLB_LOG_reserved8, // for radio purposes (TBD)
//   TLB_LOG_MOT_CALIB_CHOPPY,
//   TLB_LOG_12V_PG_INACTIVE,
//   TLB_LOG_5V_PG_INACTIVE,
//   TLB_LOG_3V3_OUT_OF_BOUNDS,
//   TLB_LOG_5V_OUT_OF_BOUNDS,
//   TLB_LOG_12V_OUT_OF_BOUNDS,
//   TLB_LOG_12V_OUT_OF_EXTENDED_BOUNDS,
//   TLB_LOG_VBAT_OUT_OF_BOUNDS,
//   TLB_LOG_VBAT_OUT_OF_EXTENDED_BOUNDS,
//   TLB_LOG_L_CUR_OUT_OF_BOUNDS,
//   TLB_LOG_MOT_CUR_OUT_OF_BOUNDS,
//   TLB_LOG_FOC_BASIC_INIT_FAIL,
//   TLB_LOG_FOC_CALIB_FAIL,
//   TLB_LOG_VBAT_BAD,
//   TLB_LOG_VBAT_CELLS_BAD,
//   TLB_LOG_VBAT_CELLS_BAD,
// };

// const String TLB_LOG_CODE_STRINGS[] {
//   "reserved err code 0", // TLB_LOG_reserved0
//   "boot %s %s" // TLB_LOG_BOOT
//    // TLB_LOG_BAT_CELLS_UNCERTAINTY
//    // TLB_LOG_BAT_CELLS_ONLY_EXTENDED
//    // TLB_LOG_reserved4 // for radio purposes (basic connection established?)
//    // TLB_LOG_reserved5 // for radio purposes (RTC time sync?)
//    // TLB_LOG_reserved6 // for radio purposes (bad handshake?)
//    // TLB_LOG_reserved7 // for radio purposes (disconnect?)
//    // TLB_LOG_reserved8 // for radio purposes (TBD)
//    // TLB_LOG_MOT_CALIB_CHOPPY
//    // TLB_LOG_12V_PG_INACTIVE
//    // TLB_LOG_5V_PG_INACTIVE
//    // TLB_LOG_3V3_OUT_OF_BOUNDS
//    // TLB_LOG_5V_OUT_OF_BOUNDS
//    // TLB_LOG_12V_OUT_OF_BOUNDS
//    // TLB_LOG_12V_OUT_OF_EXTENDED_BOUNDS
//    // TLB_LOG_VBAT_OUT_OF_BOUNDS
//    // TLB_LOG_VBAT_OUT_OF_EXTENDED_BOUNDS
//    // TLB_LOG_L_CUR_OUT_OF_BOUNDS
//    // TLB_LOG_MOT_CUR_OUT_OF_BOUNDS
//    // TLB_LOG_FOC_BASIC_INIT_FAIL
//    // TLB_LOG_FOC_CALIB_FAIL
//    // TLB_LOG_VBAT_BAD
//    // TLB_LOG_VBAT_CELLS_BAD
//    // TLB_LOG_VBAT_CELLS_BAD
// };