/*


TODO:
- logging, see note below, i actually think i've got a pretty good pitch
- move LPF_velocity setting to define
- braking (just shorting to GND, or try setting negative torque)
  * driver.setPhaseState(PhaseState
- run motor stuff (including interrupts!) on second CPU core
- rewrite defines and motor1/motor2 stuff to reduce the number of doubled code (copy-pasting is bad)
  * also define FOC_1_USE_HALL in the motor defines by default, but with an optional AS5600/BEMF override
- write little LED_STATE struct, to make sure error states can only get worse, untill an explicit clear_error function is called
- doubled voltage_limit class members? check which one is leading
- motor tuning (basics)
- radio basics
- motor tuning (in depth)
- try to ensure zero_electric_offset (sensor -> electric rotational offset) mechanically?
- BEMF??? voltage_bemf
- get USBCDC to wait for port to be opened
- do endurance test (run for 1hr+) to check for any strange behaviour.
  (the current code uses floats that get VERY big after running for a while. This is an issue for many calculations)
  * the move() function uses shaftAngle (which uses a Low-Passed sensor.getAngle(), which is absolute/growing)
    ~ fortunately, torque control mode does not use 
    ~ it DOES use shaft_velocity, but that uses HallSensor::getVelocity(), which uses integers (although somewhat lackluster with the 0-speed), so it's all good
  * see line 384 in BLDCMotor.cpp for the TODO note from the simpleFOC devs
  * if i'm reading line 104 of HallSensor.cpp correctly, float overflow/grow should NOT be an issue (because of modulo operator on integers to make angle_prev)
    ~ note: this does rely on my belief that angle_prev is used INSTEAD of getSensorAngle() in the FOC code
- use sensor.attachSectorCallback() to make the whole thing interrupt-based? (note: check function execution times first!!!)
- (not really my problem) i think the simpleFOC PP check is perhaps forgetting to account for imperfect hall sensor sampling.
  ~ during calibration, with PP set to 10, the check returned 12, and with PP set to 12, the check returned 14.4 (1.2x trend)
    i looked at the code at lines 253 of BLDCMotor.cpp, and concluded that the 'moved' position-delta was exactly 1 _HALL_STEPSIZE off each time.
    similarly, the arbetrary 0.5f threshold is failed, because PP * _HALL_STEPSIZE =~ 1.05, which is always > 0.5f.
    If the library simply calculates _HALL_STEPSIZE (using only known params, as i have), it could use that instead of 0.5f, to get a more logical threshold.
  ~ maybe i should contribute to the simpleFOC project with an issue report, or just a fixed branch
- debug: sometimes when i stop the motor by hand (in torque mode at low power), the motor gets stuck (not mechanically)
  ~ maybe some strange velocity==0, or very-rare HALL sensor glitch?

for release:
- (logging???)
  * note: maybe use __FILE__ and __LINE__ for easy integer error codes? (precompiler)
    ~ how about a nasty little class that encodes any error as a variable-size struct, with a __FILE__,__LINE__,Nargs,{args},(endmarker/CRC?)
      the variable-size allows freedom for logging, but means you should PROBABLY also store a logging 'head' location and counter.
      How about a functional FIFO, by just using a constant-size section of flash, and writing to a head with rollover (note: rollover should happen BEFORE writing!)
      If you also keep track of the 'tail' (the last non-overwritten entry) location (by seeing if the head has passed it),
      then you could even keep a count of how many valid datapoints currently exist, and when reading them, you'll know where to start (incl. chronologically).
      This 'head', 'tail' and counter stuff should be stored in a known section of flash (probably at the end, or something unlikely to get overwritten with code).
- find debug prints and disable (for speed)
- check_M_cur_bounds realistic

notes:
- some notes on recent hall sensor code fixes: https://github.com/simplefoc/Arduino-FOC/issues/270
- motor2 does 17km/h in the negative direction (at 22V no-load), but only 16 in the positive direction. There could be some tuning there
  * using SpaceVectorPWM it got up to +23, -26, but with lots of hall interrupt errors
  * Trapezoid_150 is a HUGE NONO! shit sounds awful

encoder issues debugging:
- only motor1 is regularly having issues, with both 'no state change' and 'bad state -> 000'
  * ONLY when the motor is active (so it's likely motor noise)
  * mostly on B and C pins? (or A and B pins)
- motor2 had an issue once, also with 

*/

#include <Arduino.h>

#include "TLB_logging.h"

//// extra debug stuff (for HALL interrupts)
#include "TLB_pinouts.h"
const uint8_t debugIntPins[] = {PIN_EXT_PERIPH_1,PIN_EXT_PERIPH_2};
volatile bool debugIntPinStates[sizeof(debugIntPins)] = {0}; // for a faster toggle
#define TLB_DEBUG_TOGGLE(pindex)  digitalWrite(debugIntPins[pindex], debugIntPinStates[pindex]=!debugIntPinStates[pindex])
// const uint8_t debugIntTimerCount = 4;
// volatile uint32_t debugIntTimers[debugIntTimerCount] = {0};
// volatile bool debugIntTimerFlags[debugIntTimerCount] = {0};
// #if TLB_DEBUG_VARIANT == 1
//   const uint32_t debutIntTimeThreshs[debugIntTimerCount] = {25,25,25000, 4294967295ul}; // {any ISR, no state change, dir change, TBD}
// #elif TLB_DEBUG_VARIANT == 2
//   const uint32_t debutIntTimeThreshs[debugIntTimerCount] = {25,25,25, 25}; // {A ISR, B ISR, C ISR, no state change}
// #else
//   const uint32_t debutIntTimeThreshs[debugIntTimerCount] = {4294967295ul}; // {TBD}
// #endif
// void TLB_DEBUG_TIME(uint8_t timdex) {
//   uint32_t _debugTime = micros();
//   debugIntTimerFlags[timdex] |= (_debugTime - debugIntTimers[timdex]) < debutIntTimeThreshs[timdex];
//   debugIntTimers[timdex] = _debugTime;
// }
// const uint8_t debugSpeedTimerCount = 2;
// volatile uint32_t debugSpeedTimers[debugSpeedTimerCount][2] = {0};
// volatile bool debugSpeedTimerFlags[debugSpeedTimerCount] = {0};
// #define TLB_DEBUG_SPEED(timdex,stasto) debugSpeedTimers[timdex][stasto]=ESP.getCycleCount();debugSpeedTimerFlags[timdex]|=stasto

#include <SimpleFOC.h>


#define _KPH_TO_MPS     (0.27777777778f) // 1/3.6
#define _MPS_TO_KPH     (3.60000000000f) // 1/3.6
#define _RADPS_TO_ROTPS (0.15915494309f) // 1/TWO_PI

const float speedLimit = 25.0/3.6; // (meters/sec) speed limit of longboard


#ifdef ARDUINO_USB_CDC_ON_BOOT
  #define debugSerial  Serial // use ESP32-S3's native USB JTAG+serial for debug output
  //#define HWserial0 Serial0
#else
  #define debugSerial  USBSerial // use ESP32-S3's native USB JTAG+serial for debug output
  //#define HWserial Serial
#endif
// #if SOC_UART_NUM > 1
//   #define HWserial1 Serial1
// #endif
// #if SOC_UART_NUM > 2
//   #define HWserial2 Serial2
// #endif

#define RELEASE_BUILD_CHECK  (CORE_DEBUG_LEVEL > 0) // if building in release mode, speed and reliability are more important than debug data no-one will/can read

#include "TLB_pinouts.h" // defines pins (based on PCB_R0x)

#include "TLB_ESCs.h" // ESC stuff got too long, so i put it in a header file

///////////////////////////////////////// ADC constants /////////////////////////////////////////
const float check_3V3_bounds[2] = {3.2, 3.4}; // (Volt) 3V3 measurements should fall within these bounds
const float check_5V_bounds[2] = {4.9, 5.3}; // (Volt) 5V measurements should fall within these bounds
const float check_12V_bounds[2] = {11.8, 12.3}; // (Volt) 12V measurements should fall within these bounds (for happy operation. If it exits this range, maybe slow down (gently))
const float check_12V_bounds_ext[2] = {9.6, 12.3}; // (Volt) 12V measurements MUST fall within these bounds (for minimum operation)
const float check_VBAT_bounds[2] = {13.5, 54.6}; // (Volt) VBAT measurements should fall within these bounds (for happy operation. If it exits this range, maybe slow down (gently))
const float check_VBAT_bounds_ext[2] = {9.7, 60.0}; // (Volt) VBAT measurements MUST fall within these bounds (for minimum operation. Note: UVLO will kick in)
const float check_L_cur_bounds[2] = {-0.25, 1.0}; // (Amps) VBAT_L current measurements should fall within these bounds (for happy operation)
const float check_L_cur_bounds_ext[2] = {-0.5, 5.0}; // (Amps) VBAT_L current measurements MUST fall within these bounds (otherwise something is on fire)
const float check_M_cur_bounds[2] = {-0.5, ESC1_CURRENT_LIMIT}; // (Amps) VBAT_M# current measurements should fall within these bounds (for happy operation)
const float check_M_cur_bounds_ext[2] = {-2.0, 25.0}; // (Amps) VBAT_M# current measurements MUST fall within these bounds (otherwise something is on fire)
// #if defined(ESC1_CURRENT_LIMIT) && defined(ESC2_CURRENT_LIMIT) && (ESC1_CURRENT_LIMIT != ESC1_CURRENT_LIMIT) // seperate current limits for each motor?
//   const float check_M2_cur_bounds[2] = {-5.0, 5.0}; // (Amps) VBAT_M# current measurements should fall within these bounds (for happy operation)
//   const float check_M2_cur_bounds_ext[2] = {-25.0, 25.0}; // (Amps) VBAT_M# current measurements MUST fall within these bounds (otherwise something is on fire)
// #endif

#include "TLB_analog.h" // ADC stuff got too long, so i put it in a header file

int8_t cellCount; // gets initialized in setup()
// const uint8_t minimumCellCount = ceil(check_VBAT_bounds_ext[0] / _lithiumCellThresholds[0]); // battery can NEVER drop below absolute minimum cell voltage
const uint8_t minimumCellCount = floor(check_VBAT_bounds_ext[0] / _lithiumCellThresholds[1]); // fuck it, let the user decide when to fall of their board

///////////////////////////////////////// RGB LED colors ////////////////////////////////////////
//// these defines are used in neopixelWrite(pin,r,g,b) like (pin,defined), where 'defined' contains the last 2 commas
#define RGB_LED_OFF         0,  0,  0 // off
#define RGB_LED_OK          0, 64,  0 // green
#define RGB_LED_WARNING    55, 20,  0 // orange
#define RGB_LED_ERROR      64,  0,  0 // red
#define RGB_LED_INFO_1      0,  0, 64 // blue
#define RGB_LED_INFO_2     64,  0, 64 // purple
#define RGB_LED_INFO_3     64, 64, 64 // white



bool initPSUs() {
  //// first, enable pins
  #ifdef PIN_MAX_EN_HIGH_Z
    pinMode(PIN_MAX_EN, OUTPUT_OPEN_DRAIN);
  #else
    pinMode(PIN_MAX_EN, OUTPUT);
  #endif
  digitalWrite(PIN_MAX_EN, PIN_MAX_EN_ACTIVE); // make sure the device is enabled
  #ifdef PIN_TPS_EN_HIGH_Z
    pinMode(PIN_TPS_EN, OUTPUT_OPEN_DRAIN);
  #else
    pinMode(PIN_TPS_EN, OUTPUT);
  #endif
  digitalWrite(PIN_TPS_EN, PIN_TPS_EN_ACTIVE); // make sure the device is enabled
  //// next, power-good pins
  pinMode(PIN_MAX_PG, INPUT);
  pinMode(PIN_TPS_PG, INPUT);
  delay(10); // idk, why not
  if(digitalRead(PIN_MAX_PG) != PIN_MAX_PG_ACTIVE) { TLB_log_w("MAX (12V) Power-Good is not ACTIVE!?"); }
  if(digitalRead(PIN_TPS_PG) != PIN_TPS_PG_ACTIVE) { TLB_log_w("TPS (5V) Power-Good is not ACTIVE!?"); }
  return(true); // TODO: return something helpful
}

bool initADCpins() {
  pinMode(PIN_5V_MES, ANALOG); // simple voltage divider to measure 5V supply
  pinMode(PIN_12V_MES, ANALOG); // simple voltage divider to measure 12V supply
  pinMode(PIN_VBAT_MES, ANALOG); // simple voltage divider to measure VBAT
  pinMode(PIN_CUR_SENSE_L, ANALOG); // 10A current sensor for everything but the motors (e.g. power supplies). Measured at VBAT high-side
  pinMode(PIN_CUR_SENSE_M1, ANALOG); // 30A current sensor for motor 1. Measured at VBAT high-side
  pinMode(PIN_CUR_SENSE_M2, ANALOG); // 30A current sensor for motor 2. Measured at VBAT high-side
  float testVal; // give everything a quick test, just make sure nothing is immediately on fire
  testVal=mes_3V3(); if((testVal<check_3V3_bounds[0])||(testVal>check_3V3_bounds[1])) {TLB_log_w("3V3 out of bounds! %.2fV", testVal); neopixelWrite(RGB_BUILTIN, RGB_LED_WARNING);}
  testVal=mes_5V(); if((testVal<check_5V_bounds[0])||(testVal>check_5V_bounds[1])) {TLB_log_w("5V out of bounds! %.2fV", testVal); neopixelWrite(RGB_BUILTIN, RGB_LED_WARNING);}
  testVal=mes_12V(); if((testVal<check_12V_bounds[0])||(testVal>check_12V_bounds[1])) {TLB_log_w("12V out of bounds! %.2fV", testVal); neopixelWrite(RGB_BUILTIN, RGB_LED_WARNING);}
  else if((testVal<check_12V_bounds_ext[0])||(testVal>check_12V_bounds_ext[1])) {TLB_log_w("12V out of EXTENDED bounds! %.2fV", testVal); neopixelWrite(RGB_BUILTIN, RGB_LED_ERROR);}
  testVal=mes_VBAT(); if((testVal<check_VBAT_bounds[0])||(testVal>check_VBAT_bounds[1])) {TLB_log_w("VBAT out of bounds! %.2fV", testVal); neopixelWrite(RGB_BUILTIN, RGB_LED_WARNING);}
  else if((testVal<check_VBAT_bounds_ext[0])||(testVal>check_VBAT_bounds_ext[1])) {TLB_log_w("VBAT out of EXTENDED bounds! %.2fV", testVal); neopixelWrite(RGB_BUILTIN, RGB_LED_ERROR);}
  testVal=mes_Lcurrent(); if((testVal<check_L_cur_bounds[0])||(testVal>check_L_cur_bounds[1])) {TLB_log_w("VBAT_L current out of bounds! %.2fA", testVal); neopixelWrite(RGB_BUILTIN, RGB_LED_WARNING);}
  else if((testVal<check_L_cur_bounds_ext[0])||(testVal>check_L_cur_bounds_ext[1])) {TLB_log_w("VBAT_L current out of EXTENDED bounds! %.2fA", testVal); neopixelWrite(RGB_BUILTIN, RGB_LED_ERROR);}
  testVal=mes_M1current(); if((testVal<check_M_cur_bounds[0])||(testVal>check_M_cur_bounds[1])) {TLB_log_w("Motor 1 current out of bounds! %.1fA", testVal); neopixelWrite(RGB_BUILTIN, RGB_LED_WARNING);}
  else if((testVal<check_M_cur_bounds_ext[0])||(testVal>check_M_cur_bounds_ext[1])) {TLB_log_w("Motor 1 current out of EXTENDED bounds! %.1fA", testVal); neopixelWrite(RGB_BUILTIN, RGB_LED_ERROR);}
  testVal=mes_M2current(); if((testVal<check_M_cur_bounds[0])||(testVal>check_M_cur_bounds[1])) {TLB_log_w("Motor 2 current out of bounds! %.1fA", testVal); neopixelWrite(RGB_BUILTIN, RGB_LED_WARNING);}
  else if((testVal<check_M_cur_bounds_ext[0])||(testVal>check_M_cur_bounds_ext[1])) {TLB_log_w("Motor 2 current out of EXTENDED bounds! %.1fA", testVal); neopixelWrite(RGB_BUILTIN, RGB_LED_ERROR);}
  return(true); // TOOD: return something helpful
}

bool initRGB_LED() {
  pinMode(LED_BUILTIN, OUTPUT); /*digitalWrite(LED_BUILTIN, LOW);*/
  neopixelWrite(RGB_BUILTIN, RGB_LED_OFF); // set a state (because the LED will maintain its status between resets)
  return(true); // TOOD: return something helpful
}

void errorHaltHandler() {
  while(debugSerial.available()) { debugSerial.read(); } // flush serial buffer
  while(1) {
    delay(1000);neopixelWrite(RGB_BUILTIN, RGB_LED_OFF);delay(1000);neopixelWrite(RGB_BUILTIN, RGB_LED_ERROR);
    TLB_log_i("stopped");
    while(debugSerial.available()) { char readChar=debugSerial.read(); if((readChar=='r')||(readChar=='r')) { ESP.restart(); } }
  }
}

/////////////////////////////////////////// main setup //////////////////////////////////////////

uint32_t sinOffset;

void setup() {
  debugSerial.begin(115200);
  #if(RELEASE_BUILD_CHECK) // if this is NOT a release build
    debugSerial.setDebugOutput(true); // posts ESP (RTOS) debug here as well?
    // while(!debugSerial) { /*wait*/ } // doesn't wait for serial port to be OPENEND, BUT i believe it enabled re-uploading?!? (not sure why, but uploading can fail without this check)
    delay(3000); // patch, because USBCDC can't be bothered to wait
  #endif
  TLB_log_i("boot %s %s", __DATE__, __TIME__); // even in release build, print something to me know the ESP is alive (and which code it's running)

  #if TLB_DEBUG_VARIANT >= 0
    TLB_log_d("TLB_DEBUG_VARIANT %u",TLB_DEBUG_VARIANT);
    for(uint8_t i=0;i<sizeof(debugIntPins);i++) { pinMode(debugIntPins[i],OUTPUT); digitalWrite(debugIntPins[i], debugIntPinStates[i]); }
  #endif

  initRGB_LED(); // setup RGB LED (for debug)
  TLB_log_v("LED init done");
  initPSUs(); // setup step-down converter pins
  TLB_log_v("PSU init done");
  delay(10); // idk, why not
  initADCpins(); // setup ADC pins
  TLB_log_v("ADC init done");
  
  float VBAT_used_for_FOC = mes_VBAT();
  TLB_log_i("VBAT used for FOC: %.2f",VBAT_used_for_FOC);
  cellCount = lithiumCellCalc(VBAT_used_for_FOC);
  TLB_log_i("cell count: %d",cellCount);
  // if(cellCount >= minimumCellCount) { VBAT_used_for_FOC = cellCount * _lithiumCellThresholds[2]; } // set limits to typical fully charged cell voltage
  simpleFOCinit(VBAT_used_for_FOC); // init ESC pins and whatnot (multicore note: motor_status will be motor_initializing)
  if(ESC1_motor.motor_status != FOCMotorStatus::motor_uncalibrated) { // should be motor_uncalibrated, otherwise will be motor_init_failed
    TLB_log_e("FOC init (pins) failed! %x",ESC1_motor.motor_status); neopixelWrite(RGB_BUILTIN, RGB_LED_ERROR);
    errorHaltHandler();
  }
  TLB_log_v("FOC basic init done");

  //// init other stuff...

  #if TLB_DEBUG_VARIANT > 0
    ESC1_initSensor();
  #else
    //// first, check the battery voltage
    if((VBAT_used_for_FOC<check_VBAT_bounds_ext[0])||(VBAT_used_for_FOC>check_VBAT_bounds_ext[1])) {
      TLB_log_e("not starting FOC, VBAT sucks! %.2fV", VBAT_used_for_FOC); neopixelWrite(RGB_BUILTIN, RGB_LED_ERROR);
      errorHaltHandler();
    }
    if(cellCount < minimumCellCount) {
      TLB_log_e("not starting FOC, VBAT cell count estimate sucks! %uV", cellCount); neopixelWrite(RGB_BUILTIN, RGB_LED_ERROR);
      errorHaltHandler();
    }

    //// start FOC
    if((ESC_CONTROL_TYPE==MotionControlType::torque) || (ESC_CONTROL_TYPE==MotionControlType::velocity) || (ESC_CONTROL_TYPE==MotionControlType::angle)) {
      TLB_log_d("starting FOC!");
      simpleFOCstart(); // align encoder and start full FOC (not needed for open-loop)
    } else { TLB_log_v("skipping FOC init, we're running open-loop!"); }
    if(ESC1_motor.motor_status != FOCMotorStatus::motor_ready) { // should be motor_ready, otherwise will be motor_calib_failed
      TLB_log_e("FOC init (calib) failed! %x",ESC1_motor.motor_status); neopixelWrite(RGB_BUILTIN, RGB_LED_ERROR);
      errorHaltHandler();
    }

    // //// DEBUG: move to relevant section
    // #if (defined(MOTOR1_HUB83MM) || defined(MOTOR1_BRH5065) || defined(MOTOR1_debugMotor)) && defined(FOC_1_USE_HALL)
    //   ESC1_motor.LPF_velocity.Tf = 0.05; // hall sensors have a low resolution, so the velocity-measurement Low-Pass filter should be adjusted accordingly
    //   // ESC1_motor.PID_velocity.P = 0.005; // just trying something
    //   // ESC1_motor.PID_velocity.I = 0.1; // just trying something
    // #endif
    // #if (defined(MOTOR2_HUB83MM) || defined(MOTOR2_BRH5065)) && defined(FOC_2_USE_HALL)
    //   ESC2_motor.LPF_velocity.Tf = 0.05; // hall sensors have a low resolution, so the velocity-measurement Low-Pass filter should be adjusted accordingly
    //   // ESC2_motor.PID_velocity.P = 0.005; // just trying something
    //   // ESC2_motor.PID_velocity.I = 0.1; // just trying something
    // #endif
  #endif

  //// if it made it here, all is OK, let's fucking go
  TLB_log_v("let's fucking go");
  neopixelWrite(RGB_BUILTIN, RGB_LED_OK);

  // ESC1_motor.disable(); // untill i resolve the encoder issues
  ESC2_motor.disable(); // while i resolve the encoder issues

  sinOffset = millis(); // make simple sinusoid input start at 0, instead of starting at high speed
}


/////////////////////////////////////////// main loop ///////////////////////////////////////////

uint32_t debugPrintTimer;
const uint32_t debugPrintInterval = 100; // (millis)

void loop() {
  ESC2_motor.target = sin((millis()-sinOffset) / 3000.0f) * ESC2_driver.voltage_limit; // set motor torque in 'volt' (out of max_volt)
  // ESC1_motor.target = max(min(ESC1_motor.target, ESC1_VOLTAGE_LIMIT), -ESC1_VOLTAGE_LIMIT); // not strictly needed, as power is constrained later as well
  // ESC1_motor.target = max(min(ESC1_motor.target, 3.0f), -3.0f); // not strictly needed, as power is constrained later as well

  simpleFOCupdate(); // run as often as possible
  // ESC1_sensor.update();
  
  // // virtual link code (from video: https://youtu.be/xTlv1rPEqv4?si=aYMAJuWkvaOria6V&t=126)
  ESC1_motor.shaft_angle = ESC1_motor.shaftAngle();  ESC2_motor.shaft_angle = ESC2_motor.shaftAngle(); // after my optimization, this has become necessary for the code below
  const float linkPower = 5; // how strong the 'link' should be ('voltage' torque response proportional to angle difference (in radians))
  ESC1_motor.move( linkPower*((-ESC2_motor.shaft_angle) - ESC1_motor.shaft_angle));
  // ESC2_motor.move( linkPower*((-ESC1_motor.shaft_angle) - ESC2_motor.shaft_angle));


  if((millis()-debugPrintTimer)>=debugPrintInterval) {
    debugPrintTimer = millis();

    // ESC1_motor.monitor();
    // debugSerial.println(ESC1_sensor.getAngle());
    // debugSerial.println(ESC1_motor.shaft_velocity,6);
    // debugSerial.println(ESC1_motor.target);
    // debugSerial.printf("%.2f, %.2f\n", ESC1_motor.shaft_angle, ESC2_motor.shaft_angle);
    // debugSerial.printf("%.2f, %.2f\n", ESC1_motor.shaft_angle, ESC1_motor.shaft_velocity);
    // debugSerial.printf("%.2f, %.2f\n", ESC2_motor.shaft_angle, ESC2_motor.shaft_velocity);
    // debugSerial.printf("%.2f, %.2f\n", ESC2_motor.shaft_angle, ESC2_motor.shaft_velocity * _RADPS_TO_ROTPS * MOTOR1_WHEELCIRCUM * _MPS_TO_KPH);
    // debugSerial.printf("%.2f,  %.2f, %.2f\n", mes_Lcurrent(), mes_M1current(), mes_M2current());

    // #if TLB_DEBUG_VARIANT == 1
    //   // for(uint8_t i=0;i<debugSpeedTimerCount;i++) {if(debugSpeedTimerFlags[i]){debugSerial.printf("TBD speed %u %lu\n",i,debugSpeedTimers[i][1]-debugSpeedTimers[i][0]);}}
    // #elif TLB_DEBUG_VARIANT == 2
    //   // for(uint8_t i=0;i<debugSpeedTimerCount;i++) {if(debugSpeedTimerFlags[i]){debugSerial.printf("TBD speed %u %lu\n",i,debugSpeedTimers[i][1]-debugSpeedTimers[i][0]);}}
    // #elif TLB_DEBUG_VARIANT == 3
    //   // for(uint8_t i=0;i<debugSpeedTimerCount;i++) {if(debugSpeedTimerFlags[i]){debugSerial.printf("TBD speed %u %lu\n",i,debugSpeedTimers[i][1]-debugSpeedTimers[i][0]);}}
    //   if(debugIntTimerFlags[0]) {debugSpeedTimerFlags[0]=0;debugSerial.printf("updateState speed %lu\n",debugSpeedTimers[0][1]-debugSpeedTimers[0][0]);}
    //   if(debugIntTimerFlags[1]) {debugSpeedTimerFlags[1]=0;debugSerial.printf("update/noInterrupts speed %lu\n",debugSpeedTimers[1][1]-debugSpeedTimers[1][0]);}
    // #endif
  }

  // #if TLB_DEBUG_VARIANT == 1
  //   // for(uint8_t i=0;i<debugIntTimerCount;i++) {if(debugIntTimerFlags[i]){debugSerial.printf("TBD timer %u %lu\n",i,debugIntTimers[i]);}}
  //   if(debugIntTimerFlags[0]) {debugIntTimerFlags[0]=0;debugSerial.printf("ISR timer %lu\n",debugIntTimers[0]);}
  //   if(debugIntTimerFlags[1]) {debugIntTimerFlags[1]=0;debugSerial.printf("no state change %lu\n",debugIntTimers[1]);}
  //   if(debugIntTimerFlags[2]) {debugIntTimerFlags[2]=0;debugSerial.printf("fast dir change %lu\n",debugIntTimers[2]);}
  //   // if(debugIntTimerFlags[3]) {debugIntTimerFlags[3]=0;debugSerial.printf("TBD timer %lu\n",debugIntTimers[3]);}
  // #elif TLB_DEBUG_VARIANT == 2
  //   // for(uint8_t i=0;i<debugIntTimerCount;i++) {if(debugIntTimerFlags[i]){debugSerial.printf("TBD timer %u %lu\n",i,debugIntTimers[i]);}}
  //   if(debugIntTimerFlags[0]) {debugIntTimerFlags[0]=0;debugSerial.printf("A ISR timer %lu\n",debugIntTimers[0]);}
  //   if(debugIntTimerFlags[1]) {debugIntTimerFlags[1]=0;debugSerial.printf("B ISR timer %lu\n",debugIntTimers[1]);}
  //   if(debugIntTimerFlags[2]) {debugIntTimerFlags[2]=0;debugSerial.printf("C ISR timer %lu\n",debugIntTimers[2]);}
  //   if(debugIntTimerFlags[3]) {debugIntTimerFlags[3]=0;debugSerial.printf("no state change %lu\n",debugIntTimers[3]);}
  // #elif TLB_DEBUG_VARIANT == 3
  //   // for(uint8_t i=0;i<debugIntTimerCount;i++) {if(debugIntTimerFlags[i]){debugSerial.printf("TBD timer %u %lu\n",i,debugIntTimers[i]);}}
  // #endif
}
