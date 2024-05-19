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
#include <SimpleFOC.h>
#include "driver/temp_sensor.h"


#define _KPH_TO_MPS     (0.27777777778f) // 1/3.6
#define _MPS_TO_KPH     (3.60000000000f) // 1/3.6
#define _RADPS_TO_ROTPS (0.15915494309f) // 1/TWO_PI

const float speedLimit = 25.0/3.6; // (meters/sec) speed limit of longboard
const float CPU_temperature_limit = 85.0f; // (deg Celsius) software temperature limit (starts ESCs free-wheeling)
const uint32_t radioSilenceTimeout = 2000; // (millis) if radio has been silent for this long, start free-wheeling

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
#include "TLB_comm.h"

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

float CPU_temperature = 20.0;
const float temperature_good_hyst = 5.0; // after an over-temperature event, the temperature needs to drop at least this much to be considered 'good' again
volatile bool CPU_temperature_good=true;

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

bool initEXT_PERIPHpins() { // these pins can also be used for debugging
  pinMode(PIN_EXT_PERIPH_1, OUTPUT); /*digitalWrite(PIN_EXT_PERIPH_1, LOW);*/
  pinMode(PIN_EXT_PERIPH_2, OUTPUT); /*digitalWrite(PIN_EXT_PERIPH_1, LOW);*/
  return(true);
}

bool initRGB_LED() {
  pinMode(LED_BUILTIN, OUTPUT); /*digitalWrite(LED_BUILTIN, LOW);*/
  neopixelWrite(RGB_BUILTIN, RGB_LED_OFF); // set a state (because the LED will maintain its status between resets)
  return(true); // TOOD: return something helpful
}

void initTempSensor(){
    temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
    temp_sensor.dac_offset = TSENS_DAC_L2;  // TSENS_DAC_L2 is default; L4(-40°C ~ 20°C), L2(-10°C ~ 80°C), L1(20°C ~ 100°C), L0(50°C ~ 125°C)
    temp_sensor_set_config(temp_sensor);
    temp_sensor_start();
}

void errorHaltHandler() {
  if(debugSerial) { while(debugSerial.available()) { debugSerial.read(); } } // flush serial buffer
  while(1) {
    delay(1000);neopixelWrite(RGB_BUILTIN, RGB_LED_OFF);delay(1000);neopixelWrite(RGB_BUILTIN, RGB_LED_ERROR);
    TLB_log_i("stopped");
    if(debugSerial) { while(debugSerial.available()) { char readChar=debugSerial.read(); if((readChar=='r')||(readChar=='r')){ ESP.restart(); } } }
  }
}

/////////////////////////////////////////// main setup //////////////////////////////////////////

uint32_t sinOffset;

void setup() {
  debugSerial.begin(115200);
  initRGB_LED(); // setup RGB LED (for debug) (note: LED init is done early, for easier debug)
  TLB_log_v("LED init done");
  neopixelWrite(RGB_BUILTIN, RGB_LED_INFO_1); // set a state (because the LED will maintain its status between resets)
  #if(RELEASE_BUILD_CHECK) // if this is NOT a release build
    debugSerial.setDebugOutput(true); // posts ESP (RTOS) debug here as well?
    // while(!debugSerial) { /*wait*/ } // doesn't wait for serial port to be OPENEND, BUT i believe it enabled re-uploading?!? (not sure why, but uploading can fail without this check)
    delay(3000); // patch, because USBCDC can't be bothered to wait
  #endif
  TLB_log_d("boot %s %s", __DATE__, __TIME__); // even in release build, print something to me know the ESP is alive (and which code it's running)

  initEXT_PERIPHpins();
  TLB_log_v("EXT_PERIPH init done");

  initPSUs(); // setup step-down converter pins
  TLB_log_v("PSU init done");
  delay(10); // idk, why not
  initADCpins(); // setup ADC pins
  TLB_log_v("ADC init done");

  initTempSensor();  temp_sensor_read_celsius(&CPU_temperature);
  // TLB_log_v("temp sensor init done");
  
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

  //// first, check the battery voltage
  if((VBAT_used_for_FOC<check_VBAT_bounds_ext[0])||(VBAT_used_for_FOC>check_VBAT_bounds_ext[1])) {
    TLB_log_e("not starting FOC, VBAT sucks! %.2fV", VBAT_used_for_FOC); neopixelWrite(RGB_BUILTIN, RGB_LED_ERROR);
    errorHaltHandler();
  }
  if(cellCount < minimumCellCount) {
    TLB_log_e("not starting FOC, VBAT cell count estimate sucks! %d", cellCount); neopixelWrite(RGB_BUILTIN, RGB_LED_ERROR);
    errorHaltHandler();
  }

  //// start FOC
  if((ESC_CONTROL_TYPE==MotionControlType::torque) || (ESC_CONTROL_TYPE==MotionControlType::velocity) || (ESC_CONTROL_TYPE==MotionControlType::angle)) {
    TLB_log_v("starting FOC!");
    simpleFOCstart(); // align encoder and start full FOC (not needed for open-loop)
  } else { TLB_log_v("skipping FOC init, we're running open-loop!"); }
  if(
  #if (ESC1_DEFINED) && defined(FOC_1_USE_HALL)
    (ESC1_motor.motor_status != FOCMotorStatus::motor_ready)
  #else
    0
  #endif
  #if (ESC2_DEFINED) && defined(FOC_2_USE_HALL)
    || (ESC1_motor.motor_status != FOCMotorStatus::motor_ready)
  #endif
  ) { // should be motor_ready, otherwise will be motor_calib_failed
    TLB_log_e("FOC init (calib) failed! %x",ESC1_motor.motor_status); neopixelWrite(RGB_BUILTIN, RGB_LED_ERROR);
    errorHaltHandler();
  }

  // //// DEBUG: move to relevant section (in TLB_ESC?)
  // #if (ESC1_DEFINED) && defined(FOC_1_USE_HALL)
  //   ESC1_motor.LPF_velocity.Tf = 0.05; // hall sensors have a low resolution, so the velocity-measurement Low-Pass filter should be adjusted accordingly
  //   // ESC1_motor.PID_velocity.P = 0.005; // just trying something
  //   // ESC1_motor.PID_velocity.I = 0.1; // just trying something
  // #endif
  // #if (ESC2_DEFINED) && defined(FOC_2_USE_HALL)
  //   ESC2_motor.LPF_velocity.Tf = 0.05; // hall sensors have a low resolution, so the velocity-measurement Low-Pass filter should be adjusted accordingly
  //   // ESC2_motor.PID_velocity.P = 0.005; // just trying something
  //   // ESC2_motor.PID_velocity.I = 0.1; // just trying something
  // #endif

  //// if it made it here, all is OK, let's fucking go
  TLB_log_v("let's fucking go");
  neopixelWrite(RGB_BUILTIN, RGB_LED_OK);

  //// 'disabling' the motor seems to instruct the MOSFETs to connect all phases to GND, effectively creating resistive braking on the motor (generating a ton of heat)
  // ESC1_motor.freeWheel(); // freeWheel is a function i added, which does what i want 'disable' to do
  // ESC2_motor.freeWheel(); // freeWheel is a function i added, which does what i want 'disable' to do

  //// receiver stuff (move to better location?)
  /*bool initSuccess = */
  TLB_rx.begin(true); // init ESP-NOW receiver (incl. softAP)
  TLB_rx.setSensor(0, VBAT_used_for_FOC*100);

  sinOffset = millis(); // make simple sinusoid input start at 0, instead of starting at high speed
}


/////////////////////////////////////////// main loop ///////////////////////////////////////////

uint32_t debugPrintTimer;
const uint32_t debugPrintInterval = 500; // (millis)

const float LPF_cur_Tf = 0.1; // Low-Pass Filter time constant for current sensors
LowPassFilter LPF_cur_L_debug{LPF_cur_Tf}; // TODO: relpace with efficient FIFO or some shit
LowPassFilter LPF_cur_M1_debug{LPF_cur_Tf}; // TODO: relpace with efficient FIFO or some shit
LowPassFilter LPF_cur_M2_debug{LPF_cur_Tf}; // TODO: relpace with efficient FIFO or some shit
LowPassFilter LPF_VBAT_debug{LPF_cur_Tf}; // TODO: relpace with efficient FIFO or some shit
LowPassFilter LPF_speed_debug{LPF_cur_Tf}; // deleteme

void loop() {
  // ESC1_motor.target = sin((millis()-sinOffset) / 3000.0f) * ESC1_motor.voltage_limit; // set motor torque in 'volt' (out of max_volt)
  // ESC2_motor.target = sin((millis()-sinOffset) / 3000.0f) * ESC2_motor.voltage_limit; // set motor torque in 'volt' (out of max_volt)

  // uint32_t one = ESP.getCycleCount(); // the simpleFOCupdate (for both motors) appears to take about <100us, so that's not too terrible
  simpleFOCupdate(); // run as often as possible 
  // ESC1_sensor.update();
  // uint32_t two = ESP.getCycleCount();

  // uint32_t one = ESP.getCycleCount(); // the current sensor LPF stuff below appears to take about ~330us, so that's not great
  // float cur_L_now = LPF_cur_L_debug(mes_Lcurrent());
  // float cur_M1_now = LPF_cur_M1_debug(mes_M1current());
  // float cur_M2_now = LPF_cur_M2_debug(mes_M2current());
  // uint32_t two = ESP.getCycleCount();

  // float VBAT_now = LPF_VBAT_debug(mes_VBAT());

  // uint32_t spd_now = LPF_speed_debug(two-one);

  temp_sensor_read_celsius(&CPU_temperature);
  if((CPU_temperature > CPU_temperature_limit) && CPU_temperature_good) { // if overheated
    CPU_temperature_good = false; // avoid spam
    TLB_log_w("CPU overtemp reached: %.2f", CPU_temperature);
    neopixelWrite(RGB_BUILTIN, RGB_LED_WARNING);
  } else if((!CPU_temperature_good) && (CPU_temperature < (CPU_temperature_limit-temperature_good_hyst))) { // once cooled down
    CPU_temperature_good = true;
    TLB_log_i("CPU overtemp restored");
    neopixelWrite(RGB_BUILTIN, RGB_LED_OK); // TODO: check other factors
  }
  
  // virtual link code (from video: https://youtu.be/xTlv1rPEqv4?si=aYMAJuWkvaOria6V&t=126)
  // ESC1_motor.shaft_angle = ESC1_motor.shaftAngle();  ESC2_motor.shaft_angle = ESC2_motor.shaftAngle(); // after my optimization, this has become necessary for the code below
  // const float linkPower = 5; // how strong the 'link' should be ('voltage' torque response proportional to angle difference (in radians))
  // ESC1_motor.move( linkPower*((-ESC2_motor.shaft_angle) - ESC1_motor.shaft_angle));
  // ESC2_motor.move( linkPower*((-ESC1_motor.shaft_angle) - ESC2_motor.shaft_angle));

  static uint32_t lastTime = millis();
  if(TLB_rx.unpaired() || ((millis()-lastTime)>radioSilenceTimeout)) {
      if(ESC1_motor.enabled) { TLB_log_v("freewheeling"); simpleFOCstartFreeWheel(); neopixelWrite(RGB_BUILTIN, RGB_LED_INFO_2); }
      if(TLB_rx.update()) { lastTime = millis(); } // still need to run update function (to pair and such)
  } else {
    if(!ESC1_motor.enabled) { TLB_log_v("re-enabling"); simpleFOCstopFreeWheel(); neopixelWrite(RGB_BUILTIN, RGB_LED_OK); }
    if(TLB_rx.update()) {
      const float maxBrakingTorque = DEF_VOLTAGE_SENSOR_ALIGN;
      const float maxForwardTorque = ESC_VOLTAGE_LIMIT;
      uint32_t thisTime = millis();
      int16_t newTargetRaw = TLB_rx.get(0);
      float newTargetConverted = 0.0;
      #ifdef LEGACY_TRANSMITTER
        newTargetRaw -= 1000; // rtlopez/espnow-rclink library only deals in RC 880~2120 values
        lastTime = thisTime;
        if(newTargetRaw < 25) { // braking
          newTargetConverted = newTargetRaw; // copy int to float
          newTargetConverted -= 25.0f; // get negative number
          newTargetConverted *= (maxBrakingTorque / (25.0f)); // scale
        } else if(newTargetRaw < 56) { // idling
          // if(ESC1_motor.enabled) { simpleFOCstartFreeWheel(); }
          newTargetConverted = 0.0; // 0-torque is functional free-wheel
        } else if(CPU_temperature_good) { // positive throttle
          newTargetConverted = newTargetRaw - 55; // offset for zero-throttle
          newTargetConverted *= (maxForwardTorque / ((255.0f)-(55.0f))); // scale
        }// else { newTargetConverted = 0.0;}
      #else
        newTargetRaw -= 1500; // rtlopez/espnow-rclink library only deals in RC 880~2120 values
        if(newTargetRaw < 0) { // braking
          newTargetConverted = newTargetRaw; // copy int to float
          newTargetConverted *= (maxBrakingTorque / (128.0f)); // scale
        } else if(newTargetRaw == 0) { // idling
          // if(ESC1_motor.enabled) { simpleFOCstartFreeWheel(); }
          newTargetConverted = 0.0; // 0-torque is functional free-wheel
        } else if(CPU_temperature_good) { // positive throttle
          newTargetConverted = newTargetRaw; // copy int to float
          newTargetConverted *= (maxForwardTorque / (127.0f)); // scale
        }// else { newTargetConverted = 0.0;}
      #endif
      // TLB_log_v("received: %d->%.2f  %lu  %u", newTargetRaw, newTargetConverted, thisTime-lastTime, CPU_temperature_good);
      lastTime = thisTime;
      ESC1_motor.target = -newTargetConverted; // reverse input!
      ESC2_motor.target = newTargetConverted;
    }
  }

  if((millis()-debugPrintTimer)>=debugPrintInterval) {
    debugPrintTimer = millis();

    // ESC1_motor.monitor();
    // debugSerial.println(ESC1_sensor.getAngle());
    // debugSerial.println(ESC1_motor.shaft_velocity,6);
    // debugSerial.println(ESC1_motor.target);
    // debugSerial.printf("%.2f, %.2f\n", ESC1_motor.shaft_angle, ESC2_motor.shaft_angle);
    // debugSerial.printf("%.2f, %.2f\n", ESC1_motor.shaft_angle, ESC1_motor.shaft_velocity);
    // debugSerial.printf("%.2f, %.2f\n", ESC2_motor.shaft_angle, ESC2_motor.shaft_velocity);
    // debugSerial.printf("%.2f, %.2f\n", ESC1_motor.shaft_velocity * _RADPS_TO_ROTPS * MOTOR1_WHEELCIRCUM * _MPS_TO_KPH, ESC2_motor.shaft_velocity * _RADPS_TO_ROTPS * MOTOR1_WHEELCIRCUM * _MPS_TO_KPH);
    // debugSerial.printf("cur %.2f,  %.2f, %.2f \t VBAT: %.2f\n", cur_L_now, cur_M1_now, cur_M2_now, VBAT_now);
    // debugSerial.printf("spd: %lu\n", spd_now);
    debugSerial.printf("CPU temp: %.2f\n",CPU_temperature);
  }
}
