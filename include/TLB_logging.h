/*
this file contains code for logging important debug data for field-testing


*/

/*#include "main.cpp"*/ // this header file requires some things to be defined in main.cpp first

#include <Arduino.h>



//////////////////////////////////////// ... ////////////////////////////////////////


#define TLB_log_e(format, ...)  log_e(format, ##__VA_ARGS__)  /*flash logging here*/
#define TLB_log_w(format, ...)  log_w(format, ##__VA_ARGS__)  /*flash logging here*/
#define TLB_log_i(format, ...)  log_i(format, ##__VA_ARGS__)
#define TLB_log_d(format, ...)  log_d(format, ##__VA_ARGS__)  /*flash logging here (sometimes)*/
#define TLB_log_v(format, ...)  log_v(format, ##__VA_ARGS__)
