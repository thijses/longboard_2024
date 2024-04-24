/*
 this tab should contain the functions/classes and whatnot for the communication protocol/scheme

*/

// #pragma once  // i'll just use the ifndef method
#ifndef CONN_COMMON
#define CONN_COMMON

#include "Arduino.h"

template<uint8_t arraySize> struct _connStruct { // a nice, consistant packet structure
  uint8_t _data[arraySize+1]; // byte array is 1 byte larger in order to store the checksum byte!
//  byteArrayStruct() {} // default initializer
//  byteArrayStruct(uint8_t* _dataInput) {for(uint8_t i=0;i<sizeof(_data);i++){_data[i]=_dataInput[i];}}
//  byteArrayStruct(uint8_t* _dataInput) { _data = _dataInput; }
  inline uint8_t* operator&() {return(_data);} //any attempts to retrieve the address of this object should be met with a byte pointer to _data (it's the same address, just an implicit typecast)
  inline uint8_t& operator[](size_t index) {return(_data[index]);} //i'm hoping the compiler will just know what i mean
  
  inline uint8_t& checksum() {return(_data[arraySize]);} // the checksum byte (last byte)
  uint8_t calcChecksum() { // calculate the checksum (checksum) NOTE: the LAST byte of each array is the checksum byte!
    uint8_t checksumVal = 0;
    for(uint8_t i=0; i<arraySize; i++) { checksumVal ^= _data[i]; } // '^' is the XOR operator
    return(checksumVal);
  }
  uint8_t* send() { // prepare for transmission by calculating and storing the checksum (note: don't forget to send the sync bytes first!)
    checksum() = calcChecksum(); // calculate and set checksum byte
    return(_data);
  }
  // inline uint8_t sendSize() {return(arraySize+1); /* return(sizeof(_data)); */ } // not needed, as sizeof(packetType) should work just fine
};

//struct telemetryPacket : public _connStruct<20> { // 20 = 4bytes * 5
//  inline uint32_t& timestamp() {return((uint32_t&)_data[0]);} // timestamp (since boot?)
//  inline float& speed() {return((float&)_data[4]);} // speed in m/s
//  inline float& dist() {return((float&)_data[8]);} // dist travelled (since boot?)
//  inline float& batteryVoltage() {return((float&)_data[12]);} // battery voltage (averaged) in Volts
//  inline float& currentDraw() {return((float&)_data[16]);} // current (averaged) in Amps
//};

#endif // CONN_COMMON
