/*
this file contains most of the transmitter communication stuff

the vast majority of this code is copy-pasted from one of my favorite github projects:
https://github.com/rtlopez/esp-fc
which is an ESP-based Flight Controller, which (since very recently) also includes ESP-NOW RC interfacing (intended for short-range toys)
see:
- https://github.com/rtlopez/esp-fc/pull/115/files#diff-75d96918ba77a703eb54d13800b969271b0b905c6444c0bb39a0c83d6152fcb9
- https://github.com/rtlopez/espnow-rclink-tx
- https://github.com/rtlopez/espnow-rclink

*/
#pragma once

#include <Arduino.h>
/*#include "main.cpp"*/ // this header file requires some things to be defined in main.cpp first

#include "EspNowRcLink/Receiver.h"

class TLB_radio
{
  public:
    bool begin(bool enSoftAp = true) {
      for(size_t i = 0; i < CHANNELS; i++) { _channels[i] = 1000; } // zero-init channels
      return(_rx.begin(enSoftAp));
    }

    /*IRAM_ATTR*/ bool update() {
      _rx.update();
      if(_rx.available()) {
        for(size_t i = 0; i < CHANNELS; i++) {
          _channels[i] = _rx.getChannel(i);
        }
        return(true);
      }
      return(false);
    }

    /*IRAM_ATTR*/ uint16_t get(uint8_t i) const { return(_channels[i]); }
    /*IRAM_ATTR*/ void get(uint16_t * data, size_t len) const {
      const uint16_t * src = _channels;
      while(len--) { *data++ = *src++; } // copy individual entries
    }

    /*IRAM_ATTR*/ bool setSensor(uint8_t sensorId, int16_t sensorValue) { return(!_rx.setSensor(sensorId,sensorValue)); } 

    bool unpaired() { return(_rx._state != EspNowRcLink::Receiver::State::RECEIVING); }

  protected:
    EspNowRcLink::Receiver _rx;
    static const size_t CHANNELS = EspNowRcLink::RC_CHANNEL_MAX + 1;
    uint16_t _channels[CHANNELS];
} TLB_rx; // TODO: singelton code