#include "libraryDefines.h"
#include <IRremote.hpp>

extern boolean IRreceived;
extern uint16_t IRresult;

void setup() {
  Serial.begin(115200); // // Establish serial communication

  #if DBG_WITH_IR_LIB
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver
  #else
    timerSetup();
    interruptSetup();
  #endif
}

void loop() {
  // please choose whether to test your own IR library, or use a already created library for verification
  #if DBG_WITH_IR_LIB
    if (IrReceiver.decode()) {
      Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); // Print "old" raw data
      IrReceiver.printIRResultShort(&Serial); // Print complete received data in one line
      IrReceiver.printIRSendUsage(&Serial);   // Print the statement required to send this data

      IrReceiver.resume(); // Enable receiving of the next value
    }
  #else  
    if(IRreceived) {
      Serial.println(IRresult, HEX);
      IRreceived = false;
    }
  #endif
} 