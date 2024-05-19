//longboard sketch

#define internalMotor //motor INSIDE the wheel (uncomment for external belt motor)

#include <SPI.h>
#include "RF24.h"

#define motorPin (17) // right (newer)
#define motorPWMchannel (0) //ledC channel (mostly meaningless)
#define motorPinTwo (16)  // left (used)
#define NRF_CE 27
#define NRF_CSN 15

#ifdef internalMotor
  #define wheelCircumference (0.26)  //in meters
  #define pulsesPerRotation (10.00)  //pulses per motor rotation for hall sensor
#else
  #define gearReduction (2.13)  //gear reduction from motor to wheel
  #define wheelCircumference (0.26)  //in meters
  #define pulsesPerRotation (7.00)  //pulses per motor rotation for hall sensor
#endif

#define debugging 50   //value is the millis between prints
//#define noAcceleration //for ESC programming

SPIClass* SPIptr = NULL; // we'll instantiate this in the `setup()` function
RF24 radio(NRF_CE, NRF_CSN);
const uint8_t NRFaddress[5] = {'b','o','a','r','d'};
const byte frequencyChannel = 100; //frequency is 2400+channel MHz, WiFi ends at 2495MHz, so ch>95 are less prone to WiFi noise

byte data = 24; //you can make this an array, just remember to change radio.read, write and 
unsigned long pwmTimer = 0; //this is the number that will be written to the 
byte writtenData = 24;
byte pwmVal = 110;
unsigned long connectionTimer = 0;

const byte dataBrakeThreshold = 25; //anyhting below this value will turn on the brakes

const byte brakePWM = 110;
const byte freeRunSpeedPWM = 120;
const byte fullSpeedPWM = 255;

#ifdef internalMotor
  const byte acceleration = 12; //milliSeconds per PWM value
#else
  const byte acceleration = 24; //milliSeconds per PWM value
#endif

#ifdef debugging
  unsigned long debugPrintTimer = 0;
#endif



void setup() {
  #ifdef debugging
    Serial.begin(115200);
  #endif
  
  pinMode(motorPin, OUTPUT);
  //ledcSetup(motorPWMchannel, 500, 8); //2000us period, if pwmval==255 -> full high
  ledcSetup(motorPWMchannel, 250, 9);   //4000us period, if pwmval==255 -> 2000us high (50% duty)
  ledcAttachPin(motorPin, motorPWMchannel);
  ledcAttachPin(motorPinTwo, motorPWMchannel);
  ledcWrite(motorPWMchannel, pwmVal);

  SPIptr = new SPIClass(VSPI); // by default VSPI is used
  //SPIptr = new SPIClass(HSPI); // HSPI works just as well
  SPIptr->begin(); // defualt SPI pins
  //SPIptr->begin(18, 19, 23, NRF_CSN); // SCK,MISO,MOSI,CSN (can be any pins)

  if(!radio.begin(SPIptr)) {
    #ifdef debugging
      Serial.println("radio init failed");
    #endif
  }
  radio.setAutoAck(0);
  radio.setRetries(0,0);
  radio.setPALevel(RF24_PA_MAX); //receiver
  radio.openReadingPipe(1,NRFaddress);
  radio.setChannel(frequencyChannel);
  radio.startListening();
}

void loop() {
  if( radio.available()){
    while (radio.available()) {                                 //While there is data ready
      radio.read(&data, sizeof(data));    //read the data from the receiver
    }
    connectionTimer = millis();
  }

  if((connectionTimer + 1500) < millis()) {
    if(data < dataBrakeThreshold) {
      data = (dataBrakeThreshold-1);
    } else {
      #ifdef speedData
        if(pulseLength > 21000) {
          data -= 4;
        } else {
          data = 49;
        }
      #else
        data = 49;
      #endif
    }
  }
  #ifndef noAcceleration
  if(pwmTimer < millis()) {
    if(writtenData < 255) {
      writtenData += 1;
      if(writtenData < data){
        pwmTimer = millis() + acceleration;
      }
    }
  }
  if(writtenData > data) {
  #endif
    writtenData = data;
  #ifndef noAcceleration
  }
  #endif
  
  pwmVal = map(writtenData , 55, 255, freeRunSpeedPWM+1, fullSpeedPWM);
  if(data < 56) {
    pwmVal = freeRunSpeedPWM;
    writtenData = data;
    if(data < dataBrakeThreshold) {
      pwmVal = brakePWM;
    }
  }
  ledcWrite(motorPWMchannel, pwmVal);
  
  
  #ifdef debugging
  if(debugPrintTimer < millis()) {
    debugPrintTimer = millis() + debugging;
    Serial.print(data);
    Serial.print(" writtenData:");
    Serial.print(writtenData);
    Serial.print(" pwmval:");
    Serial.print(pwmVal);
//    Serial.print(" pwmTimer:");
//    Serial.print(pwmTimer);
//    Serial.print(" connectionTimer:");
//    Serial.print(connectionTimer);
    Serial.println();
  }
  #endif
  //delay(1);
}
