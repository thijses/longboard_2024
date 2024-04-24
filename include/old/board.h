/*
board specific code:
meaningful one-pin ESC/motor feedback
data logging (EEPROM?)
BLE app UI???

pinout: (v0)
ESC1_PWM:  23
ESC1_back: 22
ESC2_PWM:  15
ESC2_back:  4
currADC_1: 35
currADC_2: 33
vbatADC:   34
NRF_CE:    21
NRF_CSN:   19
NRF_SCK:   18
NRF_MOSI:   5
NRF_MISO:  17
SW_LED:    27
BUTTON:     0 (BOOT button)

pinout: (v1)
ESC1_PWM:  13
ESC1_back: 15
ESC2_PWM:   4
ESC2_back: 16
currADC_1: 35
currADC_2: 32
vbatADC:   34
vbatADC_en:23
NRF_CE:    14
NRF_CSN:   27
NRF_SCK:   26
NRF_MOSI:  25
NRF_MISO:  33
MPU_CS:     2
SW_LED:    21
BUTTON:    12
buzzer:    18
VREG_EN:   22
HALL_1_A:  ESC1_back
HALL_1_B:  17
HALL_2_A:  19
HALL_2_B:   5
HALL_2_C:  ESC2_back
*/

//#define SPEED_BASED_ACCELERATION // motor acceleration based on encoder sensor feedback  TBD!
#define USE_CURR_SENS // use current sensors (ACS712-30A)
#define USE_BUTTONS // use button(s) for some UI stuff
////// V1 features:
//#define DIRECTIONAL_HALL_SENSORS // use hall sensors to measure the motor speed
#define USE_BUZZER // V1 added a buzzer to the (top) PCB
//#define POWER_BUTTON // UNTESTED, but might actually work

//#define PINOUT_V0
#define PINOUT_V1

#include "nrf_code.h"
#include "extraClasses.h"

namespace board {

#if defined(PINOUT_V0)
  nrf::NRF_PINOUT nrfPinout(21,19,18,17,5,HSPI); // parameter order: CE, CSN, SCK, MISO, MOSI, SPI_PORT
  //const uint8_t motorFeedbackPins[2] = {22, 4}; // left, right
  interruptCounter motorEncoders[2] = {{22,CHANGE,true}, {4,CHANGE,true}}; // (left, right) use the motor's builtin hall sensors
  const uint8_t motorPins[2] = {23, 15}; // left, right
  const uint8_t motorPWMchannels[2] = {0, 1}; //ledC channels (mostly meaningless). You can set them to be the same, but that messes with the LEDhandler, so might as well not
  const uint8_t vbatADCpin = 34; // goes to voltage divider to measure battery voltage
  #ifdef USE_CURR_SENS
    const uint8_t currADCpins[2] = {35, 33}; // left, right
  #endif
  // NOTE: find why LEDhandler ledCchannel can't be 1 (despite only channel 0 being used by motorPWMchannels
  UIpulseHandler LEDhandler(27, HIGH, max(motorPWMchannels[0],motorPWMchannels[1])+1); // setup the LED (note: ledCchannel can't be the same as motorPWMchannels
  //const uint8_t LEDhandlers[1] = {{27, HIGH, max(motorPWMchannels[0],motorPWMchannels[1])+1}}; // multiple LEDs
  #ifdef USE_BUTTONS
    button mainButton(0);  // Currently also boot-button (for programming), so don't press at power-on/reset
  #endif
#elif defined(PINOUT_V1)
  nrf::NRF_PINOUT nrfPinout(14,27,26,33,25,HSPI); // parameter order: CE, CSN, SCK, MISO, MOSI, SPI_PORT

  #define MPU_CSpin  2
  // MPU6500 init here
  
  //const uint8_t motorFeedbackPins[2] = {15, 16}; // left, right
  interruptCounter motorEncoders[2] = {{17,CHANGE,true}, {5,CHANGE,true}}; // (left, right) use the motor's builtin hall sensors
  #ifdef DIRECTIONAL_HALL_SENSORS
    #error("direction hall sensors TBD")
    // use the other HALL pins:
    // ESC_1:
    //      - ESC1_back (15)
    //      - the third encoder pin is not connected to anything right now
    // ESC_2:
    //      - 19
    //      - ESC2_back (16)
  #endif
  const uint8_t motorPins[2] = {13, 4}; // left, right
  const uint8_t motorPWMchannels[2] = {0, 1}; //ledC channels (mostly meaningless). You can set them to be the same, but that messes with the LEDhandler, so might as well not
  #ifdef POWER_BUTTON
    const uint8_t VREG_ENpin = 22; 
    const bool VREG_ENactiveState = HIGH;
  #endif
  const uint8_t vbatADCpin = 34; // goes to voltage divider to measure battery voltage
  const uint8_t vbatADC_enPin = 15; // energize a mosfet which lets battery power flow through a voltage divider
  #ifdef USE_CURR_SENS
    const uint8_t currADCpins[2] = {35, 32}; // left, right
  #endif
  // NOTE: find why LEDhandler ledCchannel can't be 1 (despite only channel 0 being used by motorPWMchannels
  UIpulseHandler LEDhandler(21, HIGH, max(motorPWMchannels[0],motorPWMchannels[1])+1); // setup the LED (note: ledCchannel can't be the same as motorPWMchannels
  //const uint8_t LEDhandlers[1] = {{27, HIGH, max(motorPWMchannels[0],motorPWMchannels[1])+1}}; // multiple LEDs
  #ifdef USE_BUZZER
    UIpulseHandler buzzerHandler(18, HIGH, max(motorPWMchannels[0],motorPWMchannels[1])+2); // buzzer (behind transistor (N type in V0), don't worry)
    // just #define the songs you'd like to include in compilation in the buzzer_tunes.h file itself
  //  #include "buzzer_tunes.h"
  #endif
  #ifdef USE_BUTTONS
    button mainButton(12);
  #endif
#else
  #error("no pinout version selected")
#endif


///////////////////////////////////////////constants//////////////////////////////////////////////

//// the vbatADC circuit is a little tricky (for power consumption reasons). See here: https://tinyurl.com/26a5jgrb
const bool vbatADC_enActiveState = LOW; // see circuit
const uint32_t vbatADC_enChargeTime = 10000; // (micros) time the capacitor needs to fully charge (TODO: calculate 5 RC!, consider enSampleTime to calculate discharge rate)
const uint32_t vbatADC_enActiveTime = 2500; // (micros) how long the mosfet gate stays sufficiently charged to pass the battery voltage (TODO: calibrate (oscilloscope or this ESP itself)
const uint32_t vbatADC_enActiveDelay = 250; // (micros) how long the mosfet gate takes to open
const uint32_t vbatADC_enSampleTime = 500; // (micros) how long the ADC should collect samples for (to then calculate an average)
const uint32_t vbatADC_enSampleInterval = 25; // (micros) time between ADC samplings

const int16_t vbatADCoffset = 162; // add this to ADC values
const float vbatADCdivider = 103.2; // divide ADC value (post-offset) by this to get battery voltage. = ADC_per_volt / volt_div_ratio
#ifdef PINOUT_V0
  inline float vbatADCtoVolts(int16_t ADCval) { return((ADCval+vbatADCoffset)/vbatADCdivider); }
#endif

#ifdef USE_CURR_SENS
  const int16_t currADCoffset = -1885; // add this to ADC values
  const float currADCdivider = 102.2792; // divide ADC value (post-offset) by this to get current in Amps. = ADC_per_volt / ((2 * 20_amps) / 3.3_volt)
  inline float currADCtoAmps(int16_t ADCval) { return((ADCval+currADCoffset)/currADCdivider); }
#endif

const float wheelCircumference = 0.0828 * PI; // (meters) circumference of wheel
const float motorPulsesPerRotation = 20.00; // pulses per motor rotation for hall sensor
const float metersPerMotorPulse = wheelCircumference / motorPulsesPerRotation; // meters traveled with each hall sensor (multiply with encoder count)

#ifdef SPEED_BASED_ACCELERATION
//// parameters for the acceleration curve (motor PWM based on sensor speed feedback)
//TBD!
// the simplest implementation is just to do a dumb time-based curve, but if the board is going faster than the curve, accept it and jump in the curve
// note: figure out math for compensating for hill climbs(?) and acceleration timeouts (if true acceleration is below expected, add a little power)
#endif

const uint8_t brakePWM = 110;
const uint8_t freeRunSpeedPWM = 120;
const uint8_t fullSpeedPWM = 255;


///////////////////////////////////////////variables//////////////////////////////////////////////

uint8_t motorPWMval = brakePWM; // the PWM value written to the ESCs
//uint8_t motorPWMvals[2] = {brakePWM, brakePWM}; // individual motor speeds

nrf::packetToBoard receivedData; // data received from transmitter
uint32_t NRFconnTimer; // updated every time a packet is received (succesfully)

#ifdef PINOUT_V1
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
#endif


///////////////////////////////////////////setup//////////////////////////////////////////////
void setup() {
  #ifdef POWER_BUTTON
    bool VREG_ENread = digitalRead(VREG_ENpin); // check the state(?)
    Serial.print("VREG_EN: "); Serial.println(VREG_ENread);
    digitalWrite(VREG_ENpin, VREG_ENactiveState); // pre-write
    pinMode(VREG_ENpin, OUTPUT);
    digitalWrite(VREG_ENpin, VREG_ENactiveState); // write again just to be sure
  #endif
  for(uint8_t i=0; i<2; i++) {
    pinMode(motorPins[i], OUTPUT);
//    bool doLedcSetup = true;
//    if(i>1) { if(motorPWMchannels[i] == motorPWMchannels[0]) { doLedcSetup = false; } } // if the PWMchannels are the same, skip repeated setup
//    if(doLedcSetup) {
      //ledcSetup(motorPWMchannels[i], 500, 8); //2000us period, if pwmval==255 -> full high
      ledcSetup(motorPWMchannels[i], 250, 9);   //4000us period, if pwmval==255 -> 2000us high (50% duty)
//    } // doLedcSetup currently not used, as it seems that maybe ledCchannels actually CAN'T output to multiple pins at once
    ledcAttachPin(motorPins[i], motorPWMchannels[i]); // you can attach several pins to one ledc-channel.
    ledcWrite(motorPWMchannels[i], motorPWMval); // write PWM value

//    pinMode(motorFeedbackPins[i], INPUT_PULLUP);
    motorEncoders[i].init();
    #ifdef USE_CURR_SENS
      pinMode(currADCpins[i], INPUT);
    #endif
  }
  pinMode(vbatADCpin, INPUT);
  pinMode(vbatADC_enPin, OUTPUT);
  digitalWrite(vbatADC_enPin, !vbatADC_enActiveState);
  LEDhandler.init();
  #ifdef USE_BUZZER
    buzzerHandler.init();
  #endif
  #ifdef USE_BUTTONS
    mainButton.init();
  #endif

  #ifdef PINOUT_V1
    pinMode(MPU_CSpin, OUTPUT);
    digitalWrite(MPU_CSpin, HIGH); // make MPU release the SPI bus (so the NRF can initialize)
  #endif

  //// radio setup:
  LEDhandler._setPinActive();
  if(!nrf::setup(false, nrfPinout)) { // try to initialize NRF
    //// let the user know:
    Serial.println("NRF setup failed!");
    LEDhandler.startPulsing(3, 250, 0.5, 0.0, 1.0);
    #ifdef USE_BUZZER
      buzzerHandler.startPulsing(3, 250, 0.5, 200.0, 0.5);
    #endif
    //// then wait for the UI elements to finish:
    bool waitForHandlers = true; // this boolean is required because the compiler will break the faster while(one.loop() || two.loop()) method
    while(waitForHandlers) { // a while loop that waits for all handlers to return false
      waitForHandlers = LEDhandler.loop();
      #ifdef USE_BUZZER
        waitForHandlers |= buzzerHandler.loop();
      #endif
    }
    //// then just reset the ESP and try again
    ESP.restart();
    #ifdef POWER_BUTTON
      #error("figure out what to do, as resetting the ESP will kill power!")
    #endif
  }
  LEDhandler._setPinInactive();
  
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
  if(nrf::radio.available()){ // a direct check, asking the NRF chip if it's received anything recently
    LEDhandler._setPinActive(); // indicate that data was received by turning on the LED (turned off at end of this statement)
    ///////////////////// new radio comm!
//    if(nrf::readPacket(receivedData)) { // read the packet
//      Serial.print("good packet: "); Serial.print(receivedData.throttle()); Serial.print(' '); Serial.println(millis()-NRFconnTimer);
    ///////////////////// legacy radio comm!
    if(true) { // account for the extra if-statement of the new comm scheme
      uint8_t legacyData;
      while (nrf::radio.available()) {                                 //While there is data ready
        nrf::radio.read(&legacyData, sizeof(legacyData));    //read the data from the receiver
      }
  //    Serial.print("legacy data: "); Serial.print(legacyData); Serial.print(' '); Serial.println(millis()-NRFconnTimer);
      motorPWMval = map(legacyData , 55, 255, freeRunSpeedPWM+1, fullSpeedPWM);
      if(legacyData < 56) {
        motorPWMval = freeRunSpeedPWM;
        if(legacyData < 25) {
          motorPWMval = brakePWM;
        }
      }
  
      NRFconnTimer = millis();
      for(uint8_t i=0; i<2; i++) {
        ledcWrite(motorPWMchannels[i], motorPWMval); // write PWM value
      }
    
    } ///////////////////// new radio comm! extra if-statement
    Serial.print(motorPWMval);
//    Serial.print('\t'); Serial.print(vbatADCtoVolts(analogRead(vbatADCpin))); 
    #ifdef USE_CURR_SENS
      Serial.print('\t'); Serial.print(currADCtoAmps(analogRead(currADCpins[0]))); Serial.print('\t'); Serial.print(currADCtoAmps(analogRead(currADCpins[1])));
    #endif
    Serial.print('\t'); Serial.print(motorEncoders[0].count); Serial.print('\t'); Serial.print(motorEncoders[1].count);
    Serial.print('\t'); Serial.print(motorEncoders[0].count * metersPerMotorPulse); Serial.print('\t'); Serial.print(motorEncoders[1].count * metersPerMotorPulse);
    Serial.println();
    
    LEDhandler._setPinInactive();
  }

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

} // closes loop()

} // namespace board
