
/*
 this tab should contain the functions/classes and whatnot for the communication protocol/scheme

*/

// #pragma once  // i'll just use the ifndef method
#ifndef NRF_CODE
#define NRF_CODE

#include "Arduino.h"
#include <SPI.h>
#include <RF24.h>

#include "comm_scheme.h"

namespace nrf {

#define THROTTLE_TYPE  int8_t
struct packetToBoard : public _connStruct<sizeof(THROTTLE_TYPE)> { // 2 = 2byte_throttle
  packetToBoard(THROTTLE_TYPE thrVal=-1) { ((THROTTLE_TYPE&)_data[0]) = thrVal; }
  inline THROTTLE_TYPE& throttle() {return((THROTTLE_TYPE&)_data[0]);} // 2 byte throttle value
};

SPIClass* SPIptr = NULL; // we'll instantiate this in the `setup()` function
RF24 radio;
const uint8_t NRFaddress[5] = {'b','o','a','r','d'};
const uint8_t frequencyChannel = 100; //frequency is 2400+channel MHz, WiFi ends at 2495MHz, so ch>95 are less prone to WiFi noise

struct NRF_PINOUT {
  uint8_t NRF_CE;
  uint8_t NRF_CSN;
  uint8_t NRF_SCK;
  uint8_t NRF_MISO;
  uint8_t NRF_MOSI;
  uint8_t NRF_SPI_PORT = HSPI; // i prefer to set this to HSPI by default, as VSPI is the default for other SPI things
  NRF_PINOUT(const uint8_t CE, const uint8_t CSN, const uint8_t SCK, const uint8_t MISO, const uint8_t MOSI, const uint8_t SPI_PORT = HSPI)
              : NRF_CE(CE), NRF_CSN(CSN), NRF_SCK(SCK), NRF_MISO(MISO), NRF_MOSI(MOSI), NRF_SPI_PORT(SPI_PORT) {}
};// defaultPinout(27,15,18,19,23,VSPI);


bool setup(bool asTransmitter, NRF_PINOUT& pinout) {
  SPIptr = new SPIClass(pinout.NRF_SPI_PORT); // by default VSPI is used
  //SPIptr->begin(); // defualt SPI pins
  SPIptr->begin(pinout.NRF_SCK, pinout.NRF_MISO, pinout.NRF_MOSI, pinout.NRF_CSN); // SCK,MISO,MOSI,CSN (can be any pins)

  if(!radio.begin(SPIptr, pinout.NRF_CE, pinout.NRF_CSN)) {
//    #ifdef debugging
      Serial.println("radio init failed");
//    #endif
    return(false);
  }
  
  radio.setAutoAck(0);
  radio.setRetries(0,0);
  radio.setPALevel(RF24_PA_MAX); //receiver
  radio.setChannel(frequencyChannel);
  if(asTransmitter) {
    radio.openWritingPipe(NRFaddress);
    radio.stopListening();
  } else {
    radio.openReadingPipe(1,NRFaddress);
    radio.startListening();
  }
  return(true);
}

bool readPacket(packetToBoard& returnVal) {
  if(!(radio.available())) { return(false); }
  packetToBoard tempPacket;
  while (radio.available()) {    // potentially read several packets (to avoid a pile-up)
    radio.read(&tempPacket, sizeof(packetToBoard));    //read the data from the receiver
  }
  bool checksumGood = (tempPacket.checksum() == tempPacket.calcChecksum());
  if(checksumGood) { // only if the data was valid
//    returnVal = tempPacket; // direct struct overwriting
    memcpy(&returnVal, &tempPacket, sizeof(tempPacket));
//    for(uint8_t i=0; i<sizeof(tempPacket); i++) {
//      returnVal._data[i] = tempPacket._data[i]; // byte-for-byte struct overwriting
//    }
  } else {
    Serial.print("NRF packet checksum faillure!: "); Serial.print(tempPacket.checksum()); Serial.print(' '); Serial.println(tempPacket.calcChecksum());
  }
  return(checksumGood);
}

bool sendPacket(packetToBoard& dataToSend) {
  dataToSend.send(); // calculate and set CRC
  return(radio.write(&dataToSend,sizeof(dataToSend))); // send data to NRF
  //return(radio.write(&(dataToSend.send()),sizeof(dataToSend))); // same thing, but in one line
}

} // namespace nrf

#endif // NRF_CODE
