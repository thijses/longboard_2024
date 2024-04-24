/*
transmitter specific code:
acceleration/smoothing code would be cool to handle here
BLE app UI????
data logging (BLE to board?)

pinout: (v0)
VREG_EN:     2
vibr_mot:   25
buzzer:     26
mag_MODE:   19
mag_CLK:    21
mag_SDA:    22
mag_PUSH:   23
switch_A:   35
switch_B:   34
trig_pot:   32
vbatADC_en: 15
vbatADC:    33
NRF_CE:      4
NRF_CSN:     5
NRF_SCK:    16
NRF_MOSI:   18
NRF_MISO:   17
LED_BLUE:   27

*/

///////////////////////////////////////////compile options//////////////////////////////////////////////

//#define ANALOG_TRIGGER // use analog trigger (instead of magenetic encoder (AS5600/MT6701))
//#define MT6701 // use MT6701 instead of AS5600 for the magnetic encoder. They're (mostly) pin-compatible, but please compile correctly to avoid damage (competing output signals)

//#define POWER_BUTTON // TODO: test on next PCB V1 (or cut trace and put diode on V0)
//#define SPEED_SWITCH_TYPE // (currently speed switch is 3-pos switch) TODO: different options for those switch pins (2-pos switch? touch input? linear potentiometer?

#define USE_BUTTONS // use button(s) for some UI stuff
#define USE_BUZZER // use piezo buzzer
#define USE_VIBRATION // use vibration motor

#define TRIGGER_CALIB // calibrate the trigger values (human debug, not automated (yet))


namespace transm {

///////////////////////////////////////////pinout//////////////////////////////////////////////
nrf::NRF_PINOUT nrfPinout(4,5,16,17,18,HSPI); // parameter order: CE, CSN, SCK, MISO, MOSI, SPI_PORT
#ifdef ANALOG_TRIGGER
  const uint8_t trigPotPin = 32; // should be a pin from ADC2, not ADC1 (as ADC1 is inaccessible during wifi/BLE operation)
#else //// AS5600/MT6701
  #ifdef MT6701
    #error("MT6701 code is still TBD!")
    //#define MT6701_USE_SPI // use SPI instead of I2C
    const uint8_t MT6701_PUSHpin = 23;
    const uint8_t MT6701_CLKpin = 22;
    const uint8_t MT6701_SCLpin = 21;
    const uint8_t MT6701_CSNpin = 19;
  #else // AS5600
    const uint8_t AS5600_PGOpin = 23;
    const uint8_t AS5600_SDApin = 22;
    const uint8_t AS5600_SCLpin = 21;
    const uint8_t AS5600_DIRpin = 19;
    #define AS5600debugPrint(x)  Serial.println(x)    //you can undefine these printing functions no problem, they are only for printing I2C errors
    //#define AS5600debugPrint(x)  log_d(x)
    #include "AS5600_thijs.h"
    AS5600_thijs trigSensor;
  #endif
#endif
#ifdef POWER_BUTTON
  const uint8_t VREG_ENpin = 2; 
  const bool VREG_ENactiveState = HIGH;
#endif
//#ifdef SPEED_SWITCH_TYPE // (TODO) currently not used, speed switch is just a 3-pos switch
const uint8_t speedSwitchPins[2] = {34, 35}; // pins for the 3-pos switch
const uint8_t vbatADCpin = 33; // goes to voltage divider to measure battery voltage
const uint8_t vbatADC_enPin = 15; // energize a mosfet which lets battery power flow through a voltage divider
UIpulseHandler LEDhandler(27, HIGH);
//const uint8_t LEDhandlers[1] = {{27, HIGH}}; // multiple LEDs
#ifdef USE_BUZZER
  UIpulseHandler buzzerHandler(26, HIGH); // buzzer (behind transistor (N type in V0), don't worry)
  // just #define the songs you'd like to include in compilation in the buzzer_tunes.h file itself
//  #include "buzzer_tunes.h"
#endif
#ifdef USE_VIBRATION
  UIpulseHandler vibrMotorHandler(25, HIGH); // vibration motor (behind mosfet (N type in V0), don't worry)
#endif
#ifdef USE_BUTTONS
  #ifdef ANALOG_TRIGGER
    #error("can't USE_BUTTON, ANALOG_TRIGGER uses the same pin!")
  #endif
  button mainButton(32); // (optional) button uses analog trigger pin!
#endif

///////////////////////////////////////////constants//////////////////////////////////////////////

//// the vbatADC circuit is a little tricky (for power consumption reasons). See here: https://tinyurl.com/26a5jgrb
const bool vbatADC_enActiveState = LOW; // see circuit
const uint32_t vbatADC_enChargeTime = 10000; // (micros) time the capacitor needs to fully charge (TODO: calculate 5 RC!, consider enSampleTime to calculate discharge rate)
const uint32_t vbatADC_enActiveTime = 2500; // (micros) how long the mosfet gate stays sufficiently charged to pass the battery voltage (TODO: calibrate (oscilloscope or this ESP itself)
const uint32_t vbatADC_enActiveDelay = 250; // (micros) how long the mosfet gate takes to open
const uint32_t vbatADC_enSampleTime = 500; // (micros) how long the ADC should collect samples for (to then calculate an average)
const uint32_t vbatADC_enSampleInterval = 25; // (micros) time between ADC samplings

const int16_t vbatADCoffset = 1647; // add this to ADC values
const float vbatADCdivider = 1293.6; // divide ADC value (post-offset) by this to get battery voltage. = ADC_per_volt / volt_div_ratio

const int16_t trigLimits[2] = {0, 4095}; // TODO: calibrate!!! (note: will likely include rollover)
//// the trigger value will be converted to a float between 0.0 and 1.0 . These thteshold values should work for an analog trigger or a magnetic-encoder one, as they're just proportions
//// finally, the (post-threshold) throttle (note: so not trigger) value, will be converted (to whatever communication format size i'm using) and sent to the board
const float trigBrakeThreshold = 0.25; // below this value, the brakes will be applied
const float trigIdleThreshold = 0.5; // between this and the brake threshold, the value will be neutral (this should be just above the trigger's springloaded-idle position)
const float trigHighThreshold = 0.9; // from this value onwards, the output will be high

const uint32_t NRFsendInterval = 20; // (millis) interval between sending data to the board (using the NRF)


///////////////////////////////////////////variables//////////////////////////////////////////////
int16_t triggerVal; // the trigger sensor data
nrf::packetToBoard dataToSend; // the packet to be sent to the board
uint32_t NRFconnTimer; // updated every time a packet is received (succesfully)

int8_t speedSetting = 1; // speed setting using 

uint32_t vbatADC_enTimer; // a timer to make sure the capacitor has time to recharge

inline float _vbatADCtoVolts(int16_t ADCval) { return((ADCval+vbatADCoffset)/vbatADCdivider); }
float getVbat() { // getting battery voltage
  uint32_t startTime = micros(); // capture time (once, so it doesn't change during the function)
  if((startTime - vbatADC_enTimer) < vbatADC_enChargeTime) {
    delayMicroseconds(vbatADC_enChargeTime - (startTime - vbatADC_enTimer)); // wait untill the capacitor is fully charged 
  }
  digitalWrite(vbatADC_enPin, vbatADC_enActiveState);
  delayMicroseconds(vbatADC_enActiveDelay);
  uint32_t returnVal = 0; // sum up ADC values to divide by averageCounter
  uint32_t averageCounter = 0; // count how many samples were collected
  startTime = micros(); // time when starting the sampling
  while((micros() - startTime) < vbatADC_enSampleTime) {
    returnVal += analogRead(vbatADCpin);
    averageCounter++;
    delayMicroseconds(vbatADC_enSampleInterval);
  }
  //if(averageCounter == 0) { freak out! }
  digitalWrite(vbatADC_enPin, !vbatADC_enActiveState);
  vbatADC_enTimer = micros();
  return(_vbatADCtoVolts(returnVal / averageCounter));
}


///////////////////////////////////////////setup//////////////////////////////////////////////
void setup() {
  #ifdef POWER_BUTTON
    bool VREG_ENread = digitalRead(VREG_ENpin); // check the state(?)
    Serial.print("VREG_EN: "); Serial.println(VREG_ENread);
    digitalWrite(VREG_ENpin, VREG_ENactiveState); // pre-write
    pinMode(VREG_ENpin, OUTPUT);
    digitalWrite(VREG_ENpin, VREG_ENactiveState); // write again just to be sure
  #endif
  pinMode(speedSwitchPins[0], INPUT_PULLUP); // pins 34 and up don't have an internal pullup
  pinMode(speedSwitchPins[1], INPUT_PULLUP);
  pinMode(vbatADCpin, INPUT);
  pinMode(vbatADC_enPin, OUTPUT);
  digitalWrite(vbatADC_enPin, !vbatADC_enActiveState);
  LEDhandler.init();
  #ifdef USE_BUZZER
    buzzerHandler.init();
  #endif
  #ifdef USE_VIBRATION
    vibrMotorHandler.init();
  #endif
  #ifdef USE_BUTTONS
    mainButton.init();
  #endif

  //// trigger sensor setup:
  #ifdef ANALOG_TRIGGER
    pinMode(trigPotPin, INPUT);
  #else //// AS5600/MT6701
    #ifdef MT6701
      pinMode(MT6701_CSNpin, INPUT_PULLUP); // 
      pinMode(MT6701_PUSHpin, INPUT); // the MT6701 has a push-button feature (not likely to be useful here)
      //TBD
    #else // AS5600
      pinMode(AS5600_PGOpin, INPUT_PULLUP); // could be OUTPUT, if you want to burn in some settings. I strongly recommend NOT to burn settings (or to burn through I2C instead)
      pinMode(AS5600_DIRpin, INPUT); // no need to use this pin, just use the angle value from the I2C bus to determine rotation direction
      // I2C pins are initialized by the library
      esp_err_t initErr = trigSensor.init(1000000, AS5600_SDApin, AS5600_SCLpin); //on the ESP32 (almost) any pins can be I2C pins
      if(initErr != ESP_OK) { 
        Serial.print("AS5600 I2C init fail. error:"); Serial.println(esp_err_to_name(initErr));
        //// let the user know:
        LEDhandler.startPulsing(3, 250, 0.5, 0.0, 1.0);
        #ifdef USE_BUZZER
          buzzerHandler.startPulsing(3, 250, 0.5, 200.0, 0.5);
        #endif
        #ifdef USE_VIBRATION
          vibrMotorHandler.startPulsing(3, 250, 0.5, 500.0, 1.0);
        #endif
        //// then wait for the UI elements to finish:
        bool waitForHandlers = true; // this boolean is required because the compiler will break the faster while(one.loop() || two.loop()) method
        while(waitForHandlers) { // a while loop that waits for all handlers to return false
          waitForHandlers = LEDhandler.loop();
          #ifdef USE_BUZZER
            waitForHandlers |= buzzerHandler.loop();
          #endif
          #ifdef USE_VIBRATION
            waitForHandlers |= vibrMotorHandler.loop();
          #endif
        }
        //// then just reset the ESP and try again
        ESP.restart();
        #ifdef POWER_BUTTON
          #error("figure out what to do, as resetting the ESP will kill power!")
        #endif
      }
      //note: on the ESP32 the actual I2C frequency is lower than the set frequency (by like 20~40% depending on pullup resistors, 1.5kOhm gets you about 800kHz)
      trigSensor.resetConfig(); //writes all 0s to the configuration registers
      //sensor.setSF(3); //set slow-filter to the mode with the least delay
      //sensor.setFTH(7); //set fast filter threshold to something... idk yet
      //// extra sensor debug:
//      trigSensor.printConfig(); //shows you the contents of the configuration registers
//      Serial.println();
      trigSensor.printStatus(); //shows you the status of the sensor (not whether it's connected, but whether the magnet is correctly positioned and stuff)
      Serial.println();
    #endif
  #endif

  //// radio setup:
  LEDhandler._setPinActive();
  if(!nrf::setup(true, nrfPinout)) { // try to initialize NRF
    //// let the user know:
    Serial.println("NRF setup failed!");
    LEDhandler.startPulsing(3, 250, 0.5, 0.0, 1.0);
    #ifdef USE_BUZZER
      buzzerHandler.startPulsing(3, 250, 0.5, 200.0, 0.5);
    #endif
    #ifdef USE_VIBRATION
      vibrMotorHandler.startPulsing(3, 250, 0.5, 500.0, 1.0);
    #endif
    //// then wait for the UI elements to finish:
    bool waitForHandlers = true; // this boolean is required because the compiler will break the faster while(one.loop() || two.loop()) method
    while(waitForHandlers) { // a while loop that waits for all handlers to return false
      waitForHandlers = LEDhandler.loop();
      #ifdef USE_BUZZER
        waitForHandlers |= buzzerHandler.loop();
      #endif
      #ifdef USE_VIBRATION
        waitForHandlers |= vibrMotorHandler.loop();
      #endif
    }
    //// then just reset the ESP and try again
    ESP.restart();
    #ifdef POWER_BUTTON
      #error("figure out what to do, as resetting the ESP will kill power!")
    #endif
  }
  LEDhandler._setPinInactive();

//  #ifdef debugRadio
//    radio.printDetails();
//  #endif
  
  #ifdef POWER_BUTTON  // DEBUG
    while(millis() < 3000) { Serial.print("wait for it... "); Serial.println(millis()); delay(500); }
    digitalWrite(VREG_ENpin, !VREG_ENactiveState); // turn off the voltage regulator
    while(true) { Serial.print("AAAH "); Serial.println(millis()); delay(1); } // wait for the capacitor to give out
  #endif

  //// setup is done, let the user know:
  #ifdef USE_BUZZER
    buzzerHandler.startPulseList(buzzer_readyPulse, sizeof(buzzer_readyPulse) / sizeof(UIpulse));
  #endif
}


///////////////////////////////////////////loop//////////////////////////////////////////////
void loop() {
//  #ifdef debugging
//    uint32_t cycleCount[2];
//    cycleCount[0] = ESP.getCycleCount(); //get raw clock cycle count (more accurate than micros()) to measure loop time
//  #endif
  
  ///////////////////////////////////////////basic stuff//////////////////////////////////////////////
  if((millis() - NRFconnTimer) >= NRFsendInterval) {
    NRFconnTimer = millis();
    
    //// read speed switch
    //temporarily store digital readings in the speedSetting variable
//    speedSetting = 0; for(uint8_t i=0; i<sizeof(speedSwitchPins); i++) { speedSetting |= )digitalRead(speedSwitchPins[i]0) << i); } // crude and direct, but effective
    speedSetting = digitalRead(speedSwitchPins[0]);
    speedSetting = speedSetting | (digitalRead(speedSwitchPins[1]) << 1);
    //by setting the correct pins (and not placing the switch upside down), the switch value can litterally output 1,2,3 for its position
    if(speedSetting == 0) { //if both pins are low, the switch is (temporarily?) contacting both
      speedSetting = 1; //revert to a safe speed
      Serial.println("HELP! speed switch illigal value");
    } else if(speedSetting > 3) { //should never happen, like ever
      speedSetting = 1;
      Serial.println("HELP! speed switch illigal value");
    }
    
    //// read trigger and calculate throttle
    #ifdef ANALOG_TRIGGER
      triggerVal = analogRead(trigPotPin);
    #else //// AS5600/MT6701
      #ifdef MT6701
        #error("MT6701 is TBD")
      #else // AS5600
        triggerVal = trigSensor.getAngle();
      #endif
    #endif
    float trigFloat = abs((triggerVal - trigLimits[0]) / ((float)(trigLimits[1]-trigLimits[0])));
    trigFloat = constrain(trigFloat, 0.0, 1.0);
    if(trigFloat < trigBrakeThreshold) { // if braking
      dataToSend.throttle() = ((trigFloat-trigBrakeThreshold)/trigBrakeThreshold) * 128; // map from -128 to -1 ( non-binary braking is not yet implemented, but it would be cool, wouldn't it?)
    } else if(trigFloat < trigIdleThreshold) { // if idling
      dataToSend.throttle() = 0; // idle
    } else if(trigFloat < trigHighThreshold) { // partial throttle
      dataToSend.throttle() = ((trigFloat-trigIdleThreshold)/(trigHighThreshold-trigIdleThreshold)) * (speedSetting*42); // map from 1 to 127 based on speedSetting
    } else { // throttle is above trigHighThreshold  // full throttle
      dataToSend.throttle() = speedSetting*42 + 1; // max throttle
    }
    
    //// send data to board
    nrf::sendPacket(dataToSend);
    
    ///////////////////////////////////////////serial debugging start//////////////////////////////////////////////
    #ifdef TRIGGER_CALIB
      Serial.print(speedSetting); Serial.print('\t');
      Serial.print(triggerVal); Serial.print('\t');
      Serial.print(trigFloat); Serial.print('\t');
      Serial.println(dataToSend.throttle());
    #endif
//    Serial.print(speedSetting); Serial.print('\t');
//    Serial.print(triggerVal); Serial.print('\t');
//    Serial.print(trigFloat); Serial.print('\t');
//    Serial.print(dataToSend.throttle()); Serial.print('\t');
//    float vbatVal = getVbat();
//    Serial.print(vbatVal, 3);
//    Serial.println();
    ///////////////////////////////////////////serial debugging end//////////////////////////////////////////////
    
  } // closes if(NRFconnTimer)

  if(mainButton.pressed()) { // will only return true ONCE per press
    Serial.println("button pressed!");
    #ifdef USE_BUZZER
      buzzerHandler.startPulsing(1, 75, 1.0, 600.0, 0.2); // (count,interval,duty,PWMfreq,PWMduty)
    #endif
  }
  if(mainButton.longPressed()) { // will only return true ONCE per press
    Serial.println("button longPressed!");
    #ifdef USE_BUZZER
      buzzerHandler.startPulsing(2, 75, 0.5, 1200.0, 0.2); // (count,interval,duty,PWMfreq,PWMduty)
    #endif
  }
  
  ///////////////////////////////////////////buzzer, vibrMotor and LED stuff//////////////////////////////////////////////
  LEDhandler.loop();
  #ifdef USE_BUZZER
    buzzerHandler.loop();
  #endif
  #ifdef USE_VIBRATION
    vibrMotorHandler.loop();
  #endif
  
} // closes loop()

} // namespace transm
