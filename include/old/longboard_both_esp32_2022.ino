/* longboard code
NOW WITH CUSTOM PCBs (v0 board and v0 transmitter)
todo:
find out why board ledCchannel for LEDhandler must be 2, not 1
wifi OTA code uploading (only if button pressed or something)

*/

//#define IS_TRANSMITTER // enable this when compiling for the transmitter
#define debugging


#define abs(x) ((x)>0?(x):-(x)) //makes abs() work with floats. Alternatively, there is fabs(), but that returns a double, which old version of arduino IDE don't love

#include "user_feedback.h"
#include "nrf_code.h"
#ifdef IS_TRANSMITTER
  #include "transmitter.h"
#else
  #include "board.h"
#endif



void setup() {
  Serial.begin(115200);
  #ifdef IS_TRANSMITTER
    Serial.println("transmitter setup start");
    transm::setup();
    Serial.println("transmitter setup done");
  #else
    Serial.println("board setup start");
    board::setup();
    Serial.println("board setup done");
  #endif
}

void loop() {
  #ifdef IS_TRANSMITTER
    transm::loop();
  #else
    board::loop();
  #endif
}
