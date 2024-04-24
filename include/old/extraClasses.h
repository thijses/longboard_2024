/*
(this is mostly a seperate file sothat the place where this is loaded in stays clean/short
this file contains:
  - a little interupt counter class
  - button class (for cleaner UI code)
*/

// #pragma once  // i'll just use the ifndef method
#ifndef EXTRA_CLASSES_H
#define EXTRA_CLASSES_H

///////////////////////////////////////////////////////////////////////////////////// interupt counter code //////////////////////////////////////////////////////////////////////////////
#ifdef USE_PULSE_COUNTER_HW  // the ESP32 has dedicated pulse counters built in. I just haven't written the code for it yet...
  #error("pulseCounter HW TBD, you'll have to just use interrupts for now");
#else // fall back to interrupt based counting
  class interruptCounter {
    public:
    uint32_t count = 0;
    uint32_t timestamp; // (millis) time of last interrupt
    //// initialization in constructor
  //  #ifdef interruptCounterInstantInit
  //    const uint8_t pin;  const int intr_type;  const bool pullup;
  //    interruptCounter(uint8_t pin, int intr_type, bool pullup=false) : pin(pin), intr_type(intr_type), pullup(pullup) { pinMode(pin, pullup ? INPUT_PULLUP : INPUT); /* attachInterruptArg(pin, isr, this, intr_type); */ }
  //    ~interruptCounter() { detachInterrupt(pin); }
  //  #else
  //  uint8_t _pin;
  //  //interruptCounter(uint8_t pin
  //  void init(uint8_t pin, int intr_type, bool pullup=false) { _pin=pin; pinMode(pin, pullup ? INPUT_PULLUP : INPUT); attachInterruptArg(pin, isr, this, intr_type); }
  //  ~interruptCounter() { detachInterrupt(_pin); }
  //  #endif
    const uint8_t pin;  const int intr_type;  const bool pullup;
    interruptCounter(uint8_t pin, int intr_type, bool pullup=false) : pin(pin), intr_type(intr_type), pullup(pullup) { pinMode(pin, pullup ? INPUT_PULLUP : INPUT); /* attachInterruptArg(pin, isr, this, intr_type); */ }
    ~interruptCounter() { detachInterrupt(pin); }
    void init() { pinMode(pin, pullup ? INPUT_PULLUP : INPUT); attachInterruptArg(pin, isr, this, intr_type); }
    
    static void IRAM_ATTR isr(void* counterArg) {
      interruptCounter* counterObj = static_cast<interruptCounter*>(counterArg); // = (interruptCounter*)counterArg;
      counterObj->count += 1;
      counterObj->timestamp = millis();
    }
  };
#endif // USE_PULSE_COUNTER_HW


///////////////////////////////////////////////////////////////////////////////////// button code //////////////////////////////////////////////////////////////////////////////
class button {
  public:
  bool _pressedRaw = false;
  bool _longPressedRaw = false;
  bool pressHandled = false; // used to make sure pressed() only returns true once per press
  bool longPressHandled = false; // used to make sure longPressed() only returns true once per press
  uint32_t _activationTimestamp; // (millis) timestamp of last activation (used for longpresses)
  uint32_t _deactivationTimestamp; // (millis) timestamp of last deactivation (used for debouncing)
  uint32_t _debounceTime = 50; // (millis) minimum time between button pressed. Used for debouncing
  uint32_t _longpressTime = 1000; // (millis) if the button is pressed longer than this, consider it to be longPressed
  const uint8_t pin; const bool pullup; const bool activeState;  const bool useInterrupt;
  button(uint8_t pin, bool pullup=true, bool activeState=false, bool useInterrupt=false) : pin(pin), pullup(pullup), activeState(activeState), useInterrupt(useInterrupt) {}
  //{ pinMode(pin, pullup ? INPUT_PULLUP : INPUT); /* attachInterruptArg(pin, isr, this, CHANGE); */ }
  ~button() { if(useInterrupt) { detachInterrupt(pin); } }
  void init() { pinMode(pin, pullup ? INPUT_PULLUP : INPUT); if(useInterrupt) { attachInterruptArg(pin, isr, this, CHANGE); } }

  inline void _update(bool pinState) {
    uint32_t rightNow = millis(); // retrieve time only once (cleaner)
    if(pinState == activeState) {
      if(_pressedRaw) {
        if((rightNow - _activationTimestamp) > _longpressTime) { // if the button has been pressed for a while
          _longPressedRaw = true;
        }
      } else if((rightNow - _deactivationTimestamp) > _debounceTime) { // only allow pressing when it's been at least '_debounceTime' millis
        _pressedRaw = true;
        _activationTimestamp = millis();
      }
    } else { // if pinState != activeState
      if(_pressedRaw) { _deactivationTimestamp = millis(); } // if the button was released, note the time (for debouncing)
      _pressedRaw = false;
      _longPressedRaw = false;
      pressHandled = false; // setup for pressed()
      longPressHandled = false; // setup for longPressed()
    }
  }

  bool pressedRaw() { // similar to longPressedRaw(), but with no waiting period
    _update(digitalRead(pin));
    return(_pressedRaw);
  }
  bool pressed() { // will only return true ONCE per press. Use pressedRaw() to get simple (still debounced) button state
    _update(digitalRead(pin));
    if(_pressedRaw & (!pressHandled)) {
      pressHandled = true;
      return(true);
    } else {
      return(false);
    }
  }
  bool longPressedRaw() { // similar to pressedRaw, but with a delay (_longpressTime)
    _update(digitalRead(pin));
    return(_longPressedRaw);
  }
  bool longPressed() { // returns true ONCE per pess, after the button has been active for a while (_longpressTime)
    _update(digitalRead(pin));
    if(_longPressedRaw & (!longPressHandled)) {
      longPressHandled = true;
      return(true);
    } else {
      return(false);
    }
  }
  
  static void IRAM_ATTR isr(void* arg) { // using this interupt function is only useful if you need accurate _activationTimestamp and _deactivationTimestamp values. Otherwise it's rarely useful
    button* objPtr = static_cast<button*>(arg); // = (button*)arg;
    objPtr->_update(digitalRead(objPtr->pin));
  }
};


#endif // EXTRA_CLASSES_H
