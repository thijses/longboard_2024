//transmitter sketch
//now with 3-speed switch
//same trigger as the old one

#include <SPI.h>
#include <RF24.h>

#define indexFinger (A0)
#define switchOne (5)
#define switchTwo (4)
//the 3-pos switch has a common to GND, and 2 pulled-up digital pins

//#define LED_BUILTIN 2  //ESP-12 modules (normally) have an LED on GPIO2. If for some reason undefined, define here
#define statusLEDpin LED_BUILTIN //there's not that many pins on an 8266, but if you dont want the builtin LED, you can i guess (comment out to disable)

#define invertTriggerReading 1024 //if defined, subtract this value by the analogRead value
#define indexFingerThreshold (510)
#define indexFingerStickHigh (990)
#define indexFingerStickLowest (0)

const uint32_t sendingInterval = 20; //interval between sending data (in micros)
//#define simpleDelayInterval //if for some reason you'd prefer to simply delay() instead of using timers, uncomment this


#ifndef simpleDelayInterval
  uint32_t sendingTimer = 0; //holds millis() value of last transmission
#endif

#ifdef statusLEDpin
  #define statusLEDon  LOW
  #define statusLEDoff HIGH
  #define statusLEDdefaultState  statusLEDon   //turn on LED as proof of life (just a power indication LED)
  #define LED_BLINK_FAST 125  //ms
  #define LED_BLINK_SLOW 333  //ms
  #define LED_BLINK_MIN  25   //ms
  #define LED_BLINK_MAX  2000 //ms
  
  bool statusLEDstate = statusLEDdefaultState;
  bool statusLEDblinking = false;
  uint32_t statusLEDblinkTimer = 0;
  uint32_t statusLEDblinkInterval = LED_BLINK_FAST; //just start at anything but 0 (shouldnt matter)
#endif


//#define debugging
//#define debugRadio
//#define calibrating


RF24 radio(15, 16);
const uint8_t NRFaddress[5] = {'b','o','a','r','d'};
const byte frequencyChannel = 100; //frequency is 2400+channel MHz, WiFi ends at 2495MHz, so ch>95 are less prone to WiFi noise

uint8_t data = 24; //you can make this an array, just remember to change radio.read, .write and serial.print .
uint8_t speedSetting = 1;
uint16_t indexFingerAnalogVal = indexFingerThreshold;

const byte dataBrakeThreshold = 25; //anything below this value will turn on the brakes  //TAKEN FROM longboard_board_2020 SKETCH! (please sync values)

bool radioBeginError = false;
bool dataTransmitError = false;
bool speedSettingError = false;
bool brakingBlink = false;


///////////////////////////////////////////setup//////////////////////////////////////////////
void setup() {
  #ifdef debugging
    Serial.begin(74880);
  #endif

  #ifdef statusLEDpin
    pinMode(statusLEDpin, OUTPUT);
    digitalWrite(statusLEDpin, statusLEDstate=statusLEDdefaultState);
  #endif

  radioBeginError = !radio.begin(); //radio.begin() returns a 1 if successfull
  if(radioBeginError) {
    Serial.begin(74880);
    Serial.println("!!! radio.begin error !!!");
    while(radioBeginError) {
      delay(500); //plenty of time between retries
      radioBeginError = !radio.begin(); //keep trying
      #ifdef statusLEDpin
        //blink
        digitalWrite(statusLEDpin, statusLEDstate=!statusLEDstate); //note '!=' is a boolean inequality check, '=!' is a boolean inversion
      #endif
    }
    //if the code makes it here, then retrying the begin() function actually worked, which is noteworthy
    Serial.println("radio begin works now");
  }
  radio.setAutoAck(0);
  radio.setRetries(0,0);
  radio.setPALevel(RF24_PA_MAX); //receiver
  radio.openWritingPipe(NRFaddress);
  radio.setChannel(frequencyChannel);
  radio.stopListening();

  #ifdef debugRadio
    radio.printDetails();
  #endif
  
  pinMode(switchOne, INPUT_PULLUP);
  pinMode(switchTwo, INPUT_PULLUP);
}

///////////////////////////////////////////loop//////////////////////////////////////////////
void loop() {
  #ifdef debugging
    uint32_t cycleCount[2];
    cycleCount[0] = ESP.getCycleCount(); //get raw clock cycle count (more accurate than micros()) to measure loop time
  #endif
  
  ///////////////////////////////////////////basic stuff//////////////////////////////////////////////
  #ifndef simpleDelayInterval
    if((millis() - sendingTimer) >= sendingInterval) { //note: not the quickest check, but it's not bothered by rollover, and the ESP is darn fast anyways
      sendingTimer = millis();
  #endif
  
  indexFingerAnalogVal = analogRead(indexFinger);
  #ifdef invertTriggerReading
    indexFingerAnalogVal = invertTriggerReading - indexFingerAnalogVal;
  #endif
  if(indexFingerAnalogVal > indexFingerThreshold) {
    if(indexFingerAnalogVal < indexFingerStickHigh) {
      data = map(indexFingerAnalogVal, indexFingerThreshold, indexFingerStickHigh, 55, (54 + (speedSetting * 67)));
    } else {
      data = (54 + (speedSetting * 67));
    }
  } else {
    data = map(indexFingerAnalogVal, indexFingerStickLowest, indexFingerThreshold, 0, 55);
  }
  
  #ifdef statusLEDpin
    //when sending brake commands
    if((data < dataBrakeThreshold) && (!speedSettingError) && (!dataTransmitError)) {
       if(!statusLEDblinking) { //if there isnt another (more important) error (or you're already blinking)
        statusLEDsetBlink(LED_BLINK_SLOW); //blink slowly
      }
      brakingBlink = true;
    } else {
      if(statusLEDblinking) { //if there isnt another error
        statusLEDsetBlink(0); //stop blinking
      }
      brakingBlink = false;
    }
  #endif

  //send data
  dataTransmitError = !radio.write(&data,sizeof(data)); //send data and record function response

  #ifdef statusLEDpin
    if(dataTransmitError) {
      statusLEDsetBlink(LED_BLINK_FAST);
    } else if(!speedSettingError) { //if sending was successfull (and there isnt another error)
      if(brakingBlink) { //if brakingblink is also on
        statusLEDsetBlink(LED_BLINK_SLOW); //blink slowly
      } else {
        statusLEDsetBlink(0); //stop blinking
      }
    }
  #endif
  
  /////////////////////////////////////////////3-speed stuff////////////////////////////////////////////////
  //temporarily store digital readings in the speedSetting variable
  speedSetting = digitalRead(switchOne);
  speedSetting = speedSetting | (digitalRead(switchTwo) << 1);
  //then translate that into a logical value
  //by setting the correct pins (and not placing the switch upside down), the switch value can litterally output 1,2,3 for its position
  if(speedSetting == 0) { //if both pins are low, the switch is (temporarily?) contacting both
    speedSetting = 1; //revert to a safe speed
  } else if(speedSetting > 3) { //should never happen, like ever
    speedSetting = 1;
    speedSettingError = true;
    //Serial.println("HELP! speed switch illigal value");
    #ifdef statusLEDpin
      statusLEDsetBlink(LED_BLINK_FAST); //turn on error blink (and dont turn off)
    #endif
  }
  
  ///////////////////////////////////////////serial debugging//////////////////////////////////////////////
  #ifdef debugging
    cycleCount[1] = ESP.getCycleCount();
    Serial.print(data);
    Serial.print(" speedSetting:");
    Serial.print(speedSetting);
    Serial.print(" millis:");
    Serial.print(millis());
    Serial.print(" loopTime(us):");
    Serial.print((cycleCount[1]-cycleCount[0])/(F_CPU/1000000.00));
    
    #ifdef calibrating
      Serial.print(" trigger last:");
      Serial.print(indexFingerAnalogVal);
      Serial.print(" trigger now:");
      Serial.print(analogRead(indexFinger));
      Serial.print(" switchOne:");
      Serial.print(digitalRead(switchOne));
      Serial.print(" switchTwo:");
      Serial.print(digitalRead(switchTwo));
    #endif
    
    if(speedSettingError) {
      Serial.print(" !speedSettingError!");
    }
    if(dataTransmitError) {
      Serial.print(" sending faillure");
    }
    Serial.println();
  #endif

  ///////////////////////////////////////////end///////////////////////////////////////////
  #ifdef simpleDelayInterval
    delay(sendingInterval);
  #else
    } //close the timer funciton
  #endif

  ///////////////////////////////////////////status LED///////////////////////////////////////////
  #ifdef statusLEDpin
    if(statusLEDblinking) { //quick check so you dont need to do unnecessary timer math
      if((millis() - statusLEDblinkTimer) >= statusLEDblinkInterval) { //note: not the quickest check, but it's not bothered by rollover, and the ESP is darn fast anyways
        statusLEDblinkTimer = millis();
        //blink
        digitalWrite(statusLEDpin, statusLEDstate=!statusLEDstate); //note '!=' is a boolean inequality check, '=!' is a boolean inversion
      }
    }
  #endif
}

///////////////////////////////////////////status LED functions///////////////////////////////////////////
#ifdef statusLEDpin
  void statusLEDsetBlink(uint16_t blinkingInterval) { //small (userfriendly?) function, just to generalize things a little
    if(blinkingInterval != 0) {
      statusLEDblinking = true;
      statusLEDblinkInterval = constrain(blinkingInterval, LED_BLINK_MIN, LED_BLINK_MAX);
    } else {
      statusLEDblinking = false;
      digitalWrite(statusLEDpin, statusLEDstate=statusLEDdefaultState);
    }
  }
#endif
