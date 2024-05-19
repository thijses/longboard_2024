/*
this file contains most of the ESC stuff

dead_zone

_configure6PWM
*/
#pragma once

/*#include "main.cpp"*/ // this header file requires some things to be defined in main.cpp first
#include "TLB_pinouts.h" // the pinout should be included in main.cpp BEFORE this file is included

#include <SimpleFOC.h>


//// define which motors are being used:
#define MOTOR1_HUB83MM // motor 1 is an 83mm hub-motor i got from aliexpress (on two seperate occaisions)
#define MOTOR2_HUB83MM // motor 2 is an 83mm hub-motor i got from aliexpress (on two seperate occaisions)
// #define MOTOR1_BRH5065 // motor 1 is a 'BRH5065-200KV' motor
// #define MOTOR2_BRH5065 // motor 2 is an 83mm hub-motor i got from aliexpress (on two seperate occaisions)
// #define MOTOR1_debugMotor // a small gimbal motor i use for testing

//// define which sensors are begin used:
#define FOC_1_USE_HALL // use HALL sensor as input for motor 1
#define FOC_2_USE_HALL // use HALL sensor as input for motor 2
// #define FOC_1_USE_AS5600 // use AS5600 I2C magnetic sensor as input for motor 1
// #define FOC_2_USE_AS5600 // use AS5600 I2C magnetic sensor as input for motor 2
// #define FOC_1_USE_BEMF // use Back-EFM as input for motor 1
// #define FOC_2_USE_BEMF // use Back-EFM as input for motor 2

//// other/debug defines:
//#define SIMPLEFOC_DISABLE_DEBUG // disables simpleFOC debug prints

#define SIMPLEFOC_ESP32_HW_DEADTIME true // explicitly specify that the ESP32's MCPWM peripheral dead-time should be used (instead of SW deadtime)
#define SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH true // explicitly specifiy the gate driver inputs logic level
#define SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH true // explicitly specifiy the gate driver inputs logic level

#define ESC1_DEFINED  (defined(MOTOR1_HUB83MM) || defined(MOTOR1_BRH5065) || defined(MOTOR1_debugMotor))
#define ESC2_DEFINED  (defined(MOTOR2_HUB83MM) || defined(MOTOR2_BRH5065))

//// a handy preprocessor macro for debugging
#define __PREPROSTRING(s) #s // converts 's' into a string (e.g. __PREPROSTRING(DEFINED_VALUE) -> "DEFINED_VALUE"). Use _PREPROSTRING() for a string of the CONTENTS of the defined value
#define _PREPROSTRING(s) __PREPROSTRING(s) // converts 's' into a string (e.g. _PREPROSTRING(DEFINED_VALUE) -> {string-of-defined-value}). Use __PREPROSTRING() for a string of "DEFINED_VALUE"

/////////////////////////////////////// control constants ///////////////////////////////////////
// https://docs.simplefoc.com/cheetsheet/options_reference
#define ESC_CONTROL_TYPE      MotionControlType::torque // i'm choosing torque for now {torque,velocity,angle,velocity_openloop,angle_openloop}
#define ESC_TORQUE_TYPE       TorqueControlType::voltage // torque control type, without current sensors, only TorqueControlType::voltage is available
#define ESC_MODULATION_TYPE   FOCModulationType::Trapezoid_120 // trapezoidal 120 should work best with HALL sensors {SinePWM,SpaceVectorPWM,Trapezoid_120,Trapezoid_150}
#define ESC_PWM_FREQ          20000 // (Hz) PWM freq used for motor control. Lower reduces switching power, higher improves consistancy and noise
// #define ESC_VOLTAGE_LIMIT     24.0f // (Volts) limit 'voltage' (assumed/calculated), effectively limiting power
#define ESC_VOLTAGE_LIMIT     12.0f // (Volts) limit 'voltage' (assumed/calculated), effectively limiting power // early testing
// #define ESC_VOLTAGE_LIMIT      6.0f // (Volts) limit 'voltage' (assumed/calculated), effectively limiting power // DEBUG
#define ESC_SENSOR_ALIGN      DEF_VOLTAGE_SENSOR_ALIGN // (Volts) note: will get constrained to ESC_VOLTAGE_LIMIT
#define ESC_VELOC_INDEX       DEF_INDEX_SEARCH_TARGET_VELOCITY // (radians/sec)
//// the gate driver i'm using: FD6288Q  https://static.qingshow.net/fortiortech/file/1597746029372.pdf
////  has an internal dead time of {100,200,300} {min,typ,max} nanoseconds, so the libary does not strictly need to do this as well
// #define ESC_DEADTIME_PERCENT   0.0f // (%) dead-time added by simpleFOC. On ESP32, this is a built-in feature of the MCPWM peripheral.
//////////////////////////////////////// motor constants ////////////////////////////////////////
//// see: https://docs.simplefoc.com/bldcmotor
#if ESC1_DEFINED
  #ifdef MOTOR1_HUB83MM
    //// a hub-motor i bought on Aliexpress, advertised as 400W, 2160rpm, 22A max, 24-36V, 0.557 Ohm, hall-sensors built-in
    //// https://www.aliexpress.com/item/32949667786.html?spm=a2g0o.order_list.order_list_main.94.471b1802dKUA8K
    #define ESC1_POLE_PAIRS         10 // originally i guesstimated 10 (by hand-turning after powering 1 coil), but initFOC() suggested it's actually 1.2 timer higher (always)
    #define ESC1_PHASE_RESIST   0.557f // (Ohm) advertised
    #define ESC1_KV_RATING         100 // (kv) i think the motor is advertised as 80kv, but simpleFOC recommends using 150~200% of the advertised kv for this library
    #define ESC1_PHASE_INDUCT 0.00037f // (Henry) measured using PROSTER BM4070, 2mH range
    #define _ESC1_VELOC_MAX_1     (2160.0f*_RPM_TO_RADS) // (radians/sec) a simple speed limit, based on spec limit
    #define _ESC1_VELOC_MAX_2     ((float)(TWO_PI*speedLimit/MOTOR1_WHEELCIRCUM)) // (radians/sec) calculate FOC speed limit using longboard speed limit (in meters/sec)
    #define ESC1_VELOC_MAX        min(_ESC1_VELOC_MAX_1,_ESC1_VELOC_MAX_2) // whichever speed limit is MORE CONSERVATIVE, use that one.
    //#define FOC_1_USE_HALL // defining default sensor type here is TODO!
    #define MOTOR1_WHEELCIRCUM   (0.0828f*PI) // (meters) circumference of wheel
    #define _HALL1_STEPSIZE  (TWO_PI / ((float)((2*ESC1_POLE_PAIRS) * 3))) // each hall sensor does 2*poles=20 steps per rotation, and there's 3 sensors (phase-offset)
    //// the following parameters are based on calibration
    #define MOTOR1_CALIBRATED_DIRECTION   Direction::CW // with {yellow, green, blue} as {A,B,C} phases (both sensor AND motor!)
    #define MOTOR1_CALIBRATED_ZERO_ANGLE  (10*_HALL1_STEPSIZE) // calibration routine printed 1.05, and we can only measure discrete steps (without a second sensor)
    //// per-motor power limits:
    #define ESC1_CURRENT_LIMIT    min(22.0f,ESC_CURRENT_SENSOR_MAX) // motor is advertised as 22A max (even though this does not match the 24-36V rating)
    #define ESC1_VOLTAGE_LIMIT    min(ESC_VOLTAGE_LIMIT,400.0f/ESC1_CURRENT_LIMIT) // motor is rated for 400W, so this attempts to limit it to that
  #elif defined(MOTOR1_BRH5065)
    #warning("BRH5065 motor parameters are unfinished")
    //// an outrunner motor, advertised as 200kv, ~1200W, hall-sensors built-in
    //// i couldn't find the original sales page, but i did find: https://www.aliexpress.com/item/1005001634506668.html
    #define ESC1_POLE_PAIRS          7 // determined using a small magnet held agains the outside
    //#define ESC1_PHASE_RESIST  NOT_SET // (Ohm) TBD
    #define ESC1_KV_RATING         200 // (kv) it says 200kv on the side of the motor, but simpleFOC recommends using 150~200% of the advertised kv for this library
    #define ESC1_PHASE_INDUCT 0.0000616f // (Henry) measured using PROSTER BM4070, 200uH range
    #define ESC1_VELOC_MAX_1  (7220.0f*_RPM_TO_RADS) // (radians/sec) a simple speed limit, based on spec limit
    //#define FOC_1_USE_HALL // defining default sensor type here is TODO!
    #define ESC1_CURRENT_LIMIT    min(46.0f,ESC_CURRENT_SENSOR_MAX) // motor is advertised as 45A max
    #define ESC1_VOLTAGE_LIMIT    min(min(ESC_VOLTAGE_LIMIT,1200.0f/ESC1_CURRENT_LIMIT),12.0f*4.2f) // motor is rated for 1200W or 12S, so this attempts to limit it to that
    //// REMEMBER: hall sensor pinout: black=GND, yellow=U, white=V, blue=W, red=+5V
  #elif defined(MOTOR1_debugMotor)
    //// a small motor (from Aliexpress) i use to debug FOC stuff, as i made a little AS5600 mount already.
    //// advertised as 2204 size, 3-4S, 260kv, 12N14P (i also counted 14 magnets)
    //// https://www.aliexpress.com/item/33025980240.html?spm=a2g0o.order_list.order_list_main.89.471b1802dKUA8K
    #define ESC1_POLE_PAIRS          7 // from advertised data, confirmed by just counting magnets
    #define ESC1_PHASE_RESIST      8.9 // (Ohms) measured using PROSTER BM4070, 200 Ohm range
    #define ESC1_KV_RATING         260 // (kv) advertised
    #define ESC1_PHASE_INDUCT  0.00112 // (Henry) measured using PROSTER BM4070, 2mH range
    //#define FOC_1_USE_AS5600 // defining default sensor type here is TODO!
  #else
    #warning("motor 1 parameters are not well-defined!")
    #define ESC1_POLE_PAIRS         11 // TODO!
  #endif
  #ifndef ESC1_PHASE_RESIST // if optional parameter is NOT defined, use default
    #define ESC1_PHASE_RESIST  NOT_SET // (optional)
  #endif
  #ifndef ESC1_KV_RATING // if optional parameter is NOT defined, use default
    #warning("ESC1_KV_RATING not set") // kv rating is not unknowable
    #define ESC1_KV_RATING     NOT_SET // (optional)
  #endif
  #ifndef ESC1_PHASE_INDUCT // if optional parameter is NOT defined, use default
    #define ESC1_PHASE_INDUCT  NOT_SET // (optional)
  #endif
  #ifndef ESC1_VELOC_MAX // if parameter is NOT defined, use default
    #define ESC1_VELOC_MAX     1000.0f // (radians/sec) a simple speed limit, based on expected usage
  #endif
  #ifndef ESC1_SENSOR_OFFSET // if parameter is NOT defined, use default
    #define ESC1_SENSOR_OFFSET    0.0f // (radians) for some applications it is convenient to specify the sensor absolute zero offset
  #endif
  #ifndef ESC1_DOWNSAMPLING // if parameter is NOT defined, use default
    #define ESC1_DOWNSAMPLING        0 // (count) how many torque control loops to run for each motion control loop
  #endif
#endif // any MOTOR1_ defined

#if ESC2_DEFINED
  #ifdef MOTOR2_HUB83MM
    //// a hub-motor i bought on Aliexpress, advertised as 400W, 2160rpm, 22A max, 24-36V, 0.557 Ohm, hall-sensors built-in
    //// https://www.aliexpress.com/item/32949667786.html?spm=a2g0o.order_list.order_list_main.94.471b1802dKUA8K
    #define ESC2_POLE_PAIRS         10 // originally i guesstimated 10 (by hand-turning after powering 1 coil), but initFOC() suggested it's actually 1.2 timer higher (always)
    #define ESC2_PHASE_RESIST   0.557f // (Ohm) advertised
    #define ESC2_KV_RATING         100 // (kv) i think the motor is advertised as 80kv, but simpleFOC recommends using 150~200% of the advertised kv for this library
    #define ESC2_PHASE_INDUCT 0.00037f // (Henry) measured using PROSTER BM4070, 2mH range
    #define _ESC2_VELOC_MAX_1     (2160.0f*_RPM_TO_RADS) // (radians/sec) a simple speed limit, based on spec limit
    #define _ESC2_VELOC_MAX_2     ((float)(TWO_PI*speedLimit/MOTOR2_WHEELCIRCUM)) // (radians/sec) calculate FOC speed limit using longboard speed limit (in meters/sec)
    #define ESC2_VELOC_MAX        min(_ESC2_VELOC_MAX_1,_ESC2_VELOC_MAX_2) // whichever speed limit is MORE CONSERVATIVE, use that one.
    //#define FOC_2_USE_HALL // defining default sensor type here is TODO!
    #define MOTOR2_WHEELCIRCUM   (0.0828f*PI) // (meters) circumference of wheel
    #define _HALL2_STEPSIZE  (TWO_PI / ((float)((2*ESC2_POLE_PAIRS) * 3))) // each hall sensor does 2*poles=20 steps per rotation, and there's 3 sensors (phase-offset)
    //// the following parameters are based on calibration
    #define MOTOR2_CALIBRATED_DIRECTION   Direction::CW // with {yellow, green, blue} as {A,B,C} phases (both sensor AND motor!)
    #define MOTOR2_CALIBRATED_ZERO_ANGLE  (10*_HALL2_STEPSIZE) // calibration routine printed 1.05, and we can only measure discrete _HALL2_STEPSIZE steps (without a second sensor)
    //// per-motor power limits:
    #define ESC2_CURRENT_LIMIT    min(22.0f,ESC_CURRENT_SENSOR_MAX) // motor is advertised as 22A max (even though this does not match the 24-36V rating)
    #define ESC2_VOLTAGE_LIMIT    min(ESC_VOLTAGE_LIMIT,400.0f/ESC2_CURRENT_LIMIT) // motor is rated for 400W, so this attempts to limit it to that
  #elif defined(MOTOR2_BRH5065)
    #error("copy BRH5065 parameters from motor 1 once they're finished")
  #else
    #warning("motor 1 parameters are not well-defined!")
    #define ESC2_POLE_PAIRS         11 // TODO!
  #endif
  #ifndef ESC2_PHASE_RESIST // if optional parameter is NOT defined, use default
    #define ESC2_PHASE_RESIST  NOT_SET // (optional)
  #endif
  #ifndef ESC2_KV_RATING // if optional parameter is NOT defined, use default
    #warning("ESC2_KV_RATING not set") // kv rating is not unknowable
    #define ESC2_KV_RATING     NOT_SET // (optional)
  #endif
  #ifndef ESC2_PHASE_INDUCT // if optional parameter is NOT defined, use default
    #define ESC2_PHASE_INDUCT  NOT_SET // (optional)
  #endif
  #ifndef ESC2_VELOC_MAX // if parameter is NOT defined, use default
    #define ESC2_VELOC_MAX     1000.0f // (radians/sec) a simple speed limit, based on expected usage
  #endif
  #ifndef ESC2_SENSOR_OFFSET // if parameter is NOT defined, use default
    #define ESC2_SENSOR_OFFSET    0.0f // (radians) for some applications it is convenient to specify the sensor absolute zero offset
  #endif
  #ifndef ESC2_DOWNSAMPLING // if parameter is NOT defined, use default
    #define ESC2_DOWNSAMPLING        0 // (count) how many torque control loops to run for each motion control loop
  #endif
#endif // any MOTOR2_ defined
//////////////////////////////////////// extra constants ////////////////////////////////////////
//// hall sensors will discard 'outlier' datapoints, but what constitutes an outlier is somewhat objective
#if ESC1_DEFINED
  #ifdef FOC_1_USE_HALL // check not needed, just nice for humans
    #define ESC1_HALL_VELOC_ABS_MIN   200.0f // (radians/sec) even for the lowest-speed use-cases, a certain minimum is useful (also consider momentary shocks to the system)
    #define ESC1_HALL_VELOC_ABS_MAX  ((float)(50.0f*TWO_PI)) // (radians/sec) 50RPS=3000RPM should be about fast enough
    #define ESC1_HALL_VELOC_MAX_MULT   10.0f // (multiplier for radians/sec) similar to sampling theory, the measurable speed should exceed the inteded max target speed
  #endif
#endif // any MOTOR1_ defined
#if ESC2_DEFINED
  #ifdef FOC_2_USE_HALL // check not needed, just nice for humans
    #define ESC2_HALL_VELOC_ABS_MIN   200.0f // (radians/sec) even for the lowest-speed use-cases, a certain minimum is useful (also consider momentary shocks to the system)
    #define ESC2_HALL_VELOC_ABS_MAX  ((float)(50.0f*TWO_PI)) // (radians/sec) 50RPS=3000RPM should be about fast enough
    #define ESC2_HALL_VELOC_MAX_MULT   10.0f // (multiplier for radians/sec) similar to sampling theory, the measurable speed should exceed the inteded max target speed
  #endif
#endif // any MOTOR2_ defined

///////////////////////////////////////// setup sensors /////////////////////////////////////////
#if ESC1_DEFINED
  #ifdef FOC_1_USE_HALL
    //// https://docs.simplefoc.com/hall_sensors
    HallSensor ESC1_sensor = HallSensor(PIN_HALL_BEMF_1A,PIN_HALL_BEMF_1B,PIN_HALL_BEMF_1C, ESC1_POLE_PAIRS); // {pin_A,pin_B,pin_C, pole_pairs}
    static void IRAM_ATTR ISR_HALL_1A(){ESC1_sensor.handleA();}
    static void IRAM_ATTR ISR_HALL_1B(){ESC1_sensor.handleB();}
    static void IRAM_ATTR ISR_HALL_1C(){ESC1_sensor.handleC();}
    void ESC1_initSensor() {
      ESC1_sensor.pullup = PCB_HALL_PULLUPS;
      ESC1_sensor.velocity_max = max(ESC1_HALL_VELOC_ABS_MIN, min(ESC1_VELOC_MAX * ESC1_HALL_VELOC_MAX_MULT, ESC1_HALL_VELOC_ABS_MAX)); // see definitions above
      ESC1_sensor.init();
      ESC1_sensor.enableInterrupts(ISR_HALL_1A, ISR_HALL_1B, ISR_HALL_1C);
    }
    #if (defined(FOC_1_USE_AS5600) || defined(FOC_1_USE_BEMF))
      #error("multiple input methods defined simultaniously")
    #endif
  #elif defined(FOC_1_USE_AS5600)
    #error("AS5600 setup is TODO!") // https://docs.simplefoc.com/magnetic_sensor_i2c
    // MagneticSensorI2CConfig_s
    MagneticSensorI2C ESC1_sensor = MagneticSensorI2C(AS5600_I2C); // ...
    void ESC1_initSensor() { /*setup dedicated I2C bus?*/ ESC1_sensor.init(&Wire); }
    #if (defined(FOC_1_USE_HALL) || defined(FOC_1_USE_BEMF))
      #error("multiple input methods defined simultaniously")
    #endif
  #elif defined(FOC_1_USE_BEMF)
    #error("BEMF setup is TODO!")
    #if (defined(FOC_1_USE_HALL) || defined(FOC_1_USE_AS5600))
      #error("multiple input methods defined simultaniously")
    #endif
  #else
    #error("no sensor input defined for motor 1")
  #endif
#endif // any MOTOR1_ defined
#if ESC2_DEFINED
  #ifdef FOC_2_USE_HALL
    //// https://docs.simplefoc.com/hall_sensors
    HallSensor ESC2_sensor = HallSensor(PIN_HALL_BEMF_2A,PIN_HALL_BEMF_2B,PIN_HALL_BEMF_2C, ESC2_POLE_PAIRS); // {pin_A,pin_B,pin_C, pole_pairs}
    static void IRAM_ATTR ISR_HALL_2A(){ESC2_sensor.handleA();}  static void IRAM_ATTR ISR_HALL_2B(){ESC2_sensor.handleB();}  static void IRAM_ATTR ISR_HALL_2C(){ESC2_sensor.handleC();}
    void ESC2_initSensor() {
      ESC2_sensor.pullup = PCB_HALL_PULLUPS;
      ESC1_sensor.velocity_max = max(ESC2_HALL_VELOC_ABS_MIN, min(ESC2_VELOC_MAX * ESC2_HALL_VELOC_MAX_MULT, ESC2_HALL_VELOC_ABS_MAX)); // see definitions above
      ESC2_sensor.init();
      ESC2_sensor.enableInterrupts(ISR_HALL_2A, ISR_HALL_2B, ISR_HALL_2C);
    }
    #if (defined(FOC_2_USE_AS5600) || defined(FOC_2_USE_BEMF))
      #error("multiple input methods defined simultaniously")
    #endif
  #elif defined(FOC_2_USE_AS5600)
    #error("AS5600 setup is TODO!") // https://docs.simplefoc.com/magnetic_sensor_i2c
    // MagneticSensorI2CConfig_s
    MagneticSensorI2C ESC2_sensor = MagneticSensorI2C(AS5600_I2C); // ...
    void ESC2_initSensor() { /*setup dedicated I2C bus?*/ ESC2_sensor.init(&Wire); }
    #if (defined(FOC_2_USE_HALL) || defined(FOC_2_USE_BEMF))
      #error("multiple input methods defined simultaniously")
    #endif
  #elif defined(FOC_2_USE_BEMF)
    #error("BEMF setup is TODO!")
    #if (defined(FOC_2_USE_HALL) || defined(FOC_2_USE_AS5600))
      #error("multiple input methods defined simultaniously")
    #endif
  #else
    #error("no sensor input defined for motor 2")
  #endif
#endif // any MOTOR2_ defined
///////////////////////////////////////// setup drivers /////////////////////////////////////////
#if ESC1_DEFINED
  #ifndef ESC1_ENABLE
    #define ESC1_ENABLE NOT_SET // motor 1 driver enable pin not present in current design
  #endif
  BLDCDriver6PWM ESC1_driver = BLDCDriver6PWM(ESC1_HIN1,ESC1_LIN1,ESC1_HIN2,ESC1_LIN2,ESC1_HIN3,ESC1_LIN3, ESC1_ENABLE); // ...
  bool ESC1_initDriver(float supplyVoltage) {
    ESC1_driver.pwm_frequency = ESC_PWM_FREQ; // pwm frequency to be used [Hz]
    ESC1_driver.voltage_power_supply = supplyVoltage; // power supply voltage [V] (note: i'm just using 0.0~1.0 scale)
    // ESC1_driver.voltage_power_supply = ESC_VOLTAGE_SUPPLY; // power supply voltage [V]
    ESC1_driver.voltage_limit = ESC1_VOLTAGE_LIMIT; // (see ESC_VOLTAGE_SUPPLY for scale) Max DC voltage allowed - default voltage_power_supply
    #ifdef ESC_DEADTIME_PERCENT
      ESC1_driver.dead_zone = ESC_DEADTIME_PERCENT;
    #endif
    return(ESC1_driver.init()); // driver init
  }
#endif // any MOTOR1_ defined
#if ESC2_DEFINED
  #ifndef ESC2_ENABLE
    #define ESC2_ENABLE NOT_SET // motor 2 driver enable pin not present in current design
  #endif
  BLDCDriver6PWM ESC2_driver = BLDCDriver6PWM(ESC2_HIN1,ESC2_LIN1,ESC2_HIN2,ESC2_LIN2,ESC2_HIN3,ESC2_LIN3, ESC2_ENABLE); // ...
  bool ESC2_initDriver(float supplyVoltage) {
    ESC2_driver.pwm_frequency = ESC_PWM_FREQ; // pwm frequency to be used [Hz]
    ESC2_driver.voltage_power_supply = supplyVoltage; // power supply voltage [V]
    // ESC2_driver.voltage_power_supply = ESC_VOLTAGE_SUPPLY; // power supply voltage [V]
    ESC2_driver.voltage_limit = ESC2_VOLTAGE_LIMIT; // (see ESC_VOLTAGE_SUPPLY for scale)1 Max DC voltage allowed - default voltage_power_supply
    #ifdef ESC_DEADTIME_PERCENT
      ESC2_driver.dead_zone = ESC_DEADTIME_PERCENT;
    #endif
    return(ESC2_driver.init()); // driver init
  }
#endif // any MOTOR2_ defined
//////////////////////////////////////// current sensors ////////////////////////////////////////
/* simpleFOC has current sensing stuff built-in, but this is for single-phases, not the whole motor*/
//// https://docs.simplefoc.com/current_sense
//void ESC1_initCurrent() {}
////current_sense.skip_align  = true; // default false
//////////////////////////////////////// last FOC objects ///////////////////////////////////////
#if ESC1_DEFINED
  BLDCMotor ESC1_motor = BLDCMotor(ESC1_POLE_PAIRS,ESC1_PHASE_RESIST,ESC1_KV_RATING,ESC1_PHASE_INDUCT); // ...
  void ESC1_initMotor(float supplyVoltage) {
    ESC1_initSensor(); // init sensor
    //// TODO: check ESC1_sensor.initialized ?
    ESC1_motor.linkSensor(&ESC1_sensor); // link the motor to the sensor
    bool initSuccess = ESC1_initDriver(supplyVoltage); // init driver
    //// TODO: check ESC1_driver.initialized ? (same as initSuccess)
    ESC1_motor.linkDriver(&ESC1_driver); // link the motor to the driver
    //ESC1_motor.linkCurrentSense(&ESC1_current); // link the motor to current sense
    ESC1_motor.useMonitoring(debugSerial); // debug!
    ESC1_motor.controller = ESC_CONTROL_TYPE; // set control loop type to be used
    ESC1_motor.torque_controller = ESC_TORQUE_TYPE; // set torque control type. Without current sensors, only TorqueControlType::voltage is available
    ESC1_motor.foc_modulation = ESC_MODULATION_TYPE; // set FOC modulation type
    ESC1_motor.voltage_sensor_align = ESC_SENSOR_ALIGN; // sensor and motor align voltage
    //ESC1_motor.velocity_index_search = ESC_VELOC_INDEX; // index search velocity (radians/sec)
    ESC1_motor.velocity_limit = ESC1_VELOC_MAX; // velocity limit (radians/sec)
    ESC1_motor.sensor_offset = ESC1_SENSOR_OFFSET; // sensor absolute-zero offset (radians)
    ESC1_motor.voltage_limit = ESC1_driver.voltage_limit; // not sure why there are 2 seperate limits, but just sync them
    ESC1_motor.current_limit = ESC1_CURRENT_LIMIT;// "not set on the begining"?
    #ifdef MOTOR1_CALIBRATED_DIRECTION // if sensor direction calibration has already been done
      ESC1_motor.sensor_direction = MOTOR1_CALIBRATED_DIRECTION;
    #else // if calibration still needs to be done
      #warning("compiling for motor 1 calibration at boot!")
      if((ESC_MODULATION_TYPE == FOCModulationType::Trapezoid_120) || (ESC_MODULATION_TYPE == FOCModulationType::Trapezoid_150)) {
        TLB_log_w("warning: motor 1 calibration will be rather choppy, because ESC_MODULATION_TYPE == %s",_PREPROSTRING(ESC_MODULATION_TYPE));
      }
    #endif
    #ifdef MOTOR1_CALIBRATED_ZERO_ANGLE // if sensor zero-offset calibration has already been done
      ESC1_motor.zero_electric_angle = MOTOR1_CALIBRATED_ZERO_ANGLE;
    #endif
    ESC1_motor.init(); // initialize motor
    //ESC1_initCurrent(); // init current sensor(s)
  }
#endif // any MOTOR1_ defined
#if ESC2_DEFINED
  BLDCMotor ESC2_motor = BLDCMotor(ESC2_POLE_PAIRS,ESC2_PHASE_RESIST,ESC2_KV_RATING,ESC2_PHASE_INDUCT); // ...
  void ESC2_initMotor(float supplyVoltage) {
    ESC2_initSensor(); // init sensor
    //// TODO: check ESC2_sensor.initialized ?
    ESC2_motor.linkSensor(&ESC2_sensor); // link the motor to the sensor
    bool initSuccess = ESC2_initDriver(supplyVoltage); // init driver
    //// TODO: check ESC2_driver.initialized ? (same as initSuccess)
    ESC2_motor.linkDriver(&ESC2_driver); // link the motor to the driver
    //ESC2_motor.linkCurrentSense(&ESC2_current); // link the motor to current sense
    //ESC2_motor.useMonitoring(debugSerial); // debug!
    ESC2_motor.controller = ESC_CONTROL_TYPE; // set control loop type to be used
    ESC2_motor.torque_controller = ESC_TORQUE_TYPE; // set torque control type. Without current sensors, only TorqueControlType::voltage is available
    ESC2_motor.foc_modulation = ESC_MODULATION_TYPE; // set FOC modulation type
    ESC2_motor.voltage_sensor_align = ESC_SENSOR_ALIGN; // sensor and motor align voltage
    //ESC2_motor.velocity_index_search = ESC_VELOC_INDEX; // index search velocity (radians/sec)
    ESC2_motor.velocity_limit = ESC2_VELOC_MAX; // velocity limit (radians/sec)
    ESC2_motor.sensor_offset = ESC2_SENSOR_OFFSET; // sensor absolute-zero offset (radians)
    ESC2_motor.voltage_limit = ESC2_driver.voltage_limit; // not sure why there are 2 seperate limits, but just sync them
    ESC2_motor.current_limit = ESC2_CURRENT_LIMIT;// "not set on the begining"?
    #ifdef MOTOR2_CALIBRATED_DIRECTION // if sensor direction calibration has already been done
      ESC2_motor.sensor_direction = MOTOR2_CALIBRATED_DIRECTION;
    #else // if calibration still needs to be done
      #warning("compiling for motor 2 calibration at boot!")
      if((ESC_MODULATION_TYPE == FOCModulationType::Trapezoid_120) || (ESC_MODULATION_TYPE == FOCModulationType::Trapezoid_150)) {
        TLB_log_w("warning: motor 2 calibration will be rather choppy, because ESC_MODULATION_TYPE == %s",_PREPROSTRING(ESC_MODULATION_TYPE));
      }
    #endif
    #ifdef MOTOR2_CALIBRATED_ZERO_ANGLE // if sensor zero-offset calibration has already been done
      ESC2_motor.zero_electric_angle = MOTOR2_CALIBRATED_ZERO_ANGLE;
    #endif
    ESC2_motor.init(); // initialize motor
    //ESC2_initCurrent(); // init current sensor(s)
  }
#endif // any MOTOR2_ defined
////////////////////////////////////// GPIO initialization //////////////////////////////////////
void simpleFOCinit(float supplyVoltage) { // init ESC pins
  #if ESC1_DEFINED
    ESC1_initMotor(supplyVoltage);
  #else
    pinMode(ESC1_HIN1, OUTPUT); pinMode(ESC1_HIN2, OUTPUT); pinMode(ESC1_HIN3, OUTPUT);
    pinMode(ESC1_LIN1, OUTPUT); pinMode(ESC1_LIN2, OUTPUT); pinMode(ESC1_LIN3, OUTPUT);
    digitalWrite(ESC1_HIN1, LOW); digitalWrite(ESC1_HIN2, LOW); digitalWrite(ESC1_HIN3, LOW);
    digitalWrite(ESC1_LIN1, LOW); digitalWrite(ESC1_LIN2, LOW); digitalWrite(ESC1_LIN3, LOW);
  #endif
  #if ESC2_DEFINED
    ESC2_initMotor(supplyVoltage);
  #else
    pinMode(ESC2_HIN1, OUTPUT); pinMode(ESC2_HIN2, OUTPUT); pinMode(ESC2_HIN3, OUTPUT);
    pinMode(ESC2_LIN1, OUTPUT); pinMode(ESC2_LIN2, OUTPUT); pinMode(ESC2_LIN3, OUTPUT);
    digitalWrite(ESC2_HIN1, LOW); digitalWrite(ESC2_HIN2, LOW); digitalWrite(ESC2_HIN3, LOW);
    digitalWrite(ESC2_LIN1, LOW); digitalWrite(ESC2_LIN2, LOW); digitalWrite(ESC2_LIN3, LOW);
  #endif
}
void simpleFOCstart() { // align encoder and start FOC
  #if ESC1_DEFINED
    ESC1_motor.initFOC();
  #endif
  #if ESC2_DEFINED
    ESC2_motor.initFOC();
  #endif
}
/*IRAM_ATTR*/ void simpleFOCupdate(float target1 = NOT_SET, float target2 = NOT_SET) { // sensor update & FOC algorithm function. Run as often as possible
  //// NOTE: in torque mode, loopFOC() just does: {sensor->update(); electrical_angle=electricalAngle(); setPhaseVoltage(voltage.q, voltage.d, electrical_angle);}
  //// NOTE: in torque mode, .move() just updates voltage.q and voltage.d based on shaft_velocity (and voltage_bemf)
  ////        (also note: voltage.d (lag compensation if known inductance) is not used in FOCModulationType::Trapezoid_120)
  ////        (also note: current.q is estimated (despite no current sensor being present), but not used in TorqueControlType::voltage)
  #if ESC1_DEFINED
    ESC1_motor.loopFOC();
    ESC1_motor.move(target1);
  #endif
  #if ESC2_DEFINED
    ESC2_motor.loopFOC();
    ESC2_motor.move(target2);
  #endif
}

void simpleFOCstartFreeWheel() { // start free-wheeling (return to normal using)
  #if ESC1_DEFINED
    ESC1_motor.freeWheel();
  #endif
  #if ESC2_DEFINED
    ESC2_motor.freeWheel();
  #endif
}
void simpleFOCstopFreeWheel() {
  #if ESC1_DEFINED
    ESC1_motor.enable();
  #endif
  #if ESC2_DEFINED
    ESC2_motor.enable();
  #endif
}

//// calculate how many many meters this system can travel before rolling over
// #define MOTOR1_ROLLOVER_DISTANCE  (((float)(((1UL<<((sizeof(ESC1_sensor.electric_rotations)*8)-1))-1) / ESC1_POLE_PAIRS)) * MOTOR1_WHEELCIRCUM)
// #define MACRO_ASSERT(condition,message) extern int macro_assert[!!(condition)-1] // a handy trick to check constants (like sizeof()) at compile-time (because #if is PRE-compile)
// MACRO_ASSERT(MOTOR1_ROLLOVER_DISTANCE > 50000.0f, "due to integer rollover, unexpected behaviour WILL occur within absolute maximum expected range!"); // 50KM range check
