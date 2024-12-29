#include "libraryDefines.h"


// hardware timer configuration variables
hw_timer_t* tim0_cfg;
hw_timer_t* tim1_cfg;

// array to store individual bits of a IR packet
uint8_t irData[SONY_PACKET_SIZE] = {0};

// flag used to determine a new bit has been recieved by IR reciever
volatile bool updateFlag = false;

// flag to determine if the code is looking for a preamble or not
volatile bool preamble = true;

// flag to determine if an error is present in the IR reception waveform
volatile bool errorFlag = true;

// variable to keep track of current timer0 count value
volatile uint32_t tim0_count = 0;

// variables to determine the time stamps of falling edge interrupts
volatile uint32_t referenceFrame, prevReferenceFrame = 0;

// variable to determine the time elapsed between falling edge interrupts
volatile uint32_t deltaTime = 0;

// variable to keep track of amount of consecutive bits received by the IR reciever
volatile uint8_t bitCount = 0;

// variable to store hex value of IR packet 
volatile uint16_t IRresult = 0;

// variable to determine if a new byte has been sent
bool IRreceived = false;


/// @brief if error is present in IR reciever waveform, preform a reset of parameters
/// @param none
void errorHandler(void) {
  bitCount = 0;
  preamble = true;
  deltaTime = 0;
  memset(irData, 0, sizeof(irData)); // clear all bits in array 
}

/// @brief process the incoming timestamp and determines the time elapsed
/// @param none
void processDataLength(void) {
  bitCount++;
  prevReferenceFrame = referenceFrame;
  referenceFrame = timerRead(tim0_cfg);
  timerWrite(tim1_cfg, 0); // reset watchdog timer for IR packet
  if( prevReferenceFrame > referenceFrame) {
    deltaTime = referenceFrame + (10000 - prevReferenceFrame);
  } else {
    deltaTime = referenceFrame - prevReferenceFrame;
  }
}

/// @brief preforms various sanity checks on the data before registering it on the array
/// @param none
void sanityChecks(void) {
  if(bitCount == 1 && deltaTime < MIN_PREAMBLE_TIME)  {
    errorHandler();
  }
  if(deltaTime < MIN_LOGIC0_TIME) { // if preamble is long enough, but the data bits are too short (error in reception) (error)
    errorHandler();
  }
  if(bitCount > 0) { // if data passes the two sanity checks, update the array (if bitCount is != 0)
    (deltaTime < LOGIC1_THRESHOLD) ?  irData[bitCount-1] = 0 : irData[bitCount-1] = 1;
  }
}

/// @brief parse the irData array when the bitCount equals the Sony Packet Size
/// @param none
void parseData(void) {
  uint16_t bitMask = 0x0001;
  IRresult = 0;
  for(int i = 1 ; i < SONY_PACKET_SIZE ; i++) { // skip preamble bit in message decoding calculations
    if(irData[i]) IRresult |= bitMask;  // variable "result" will be in MSB format (right-aligned)
    else IRresult &= ~bitMask;
    bitMask <<= 1;
  }
}

/// @brief if packet contains SONY_PACKET_SIZE of bits, parse the data (set a flag to notify user of updated data)
/// @param none
void isPacketFinished(void) {
  if(bitCount == SONY_PACKET_SIZE) { // if bitCount is 12, then you have successfully recieved a legit packet of data
   parseData();
   bitCount = 0;
   preamble = true;
   IRreceived = true;
  }
}

/// @brief this ISR is present in order to call errorHandler if IR reciever waveform does not respond in time
/// @param none 
void IRAM_ATTR tim1_overflow(void) {
  errorHandler();
}

/// @brief this falling edge ISR deals with calculating the pulse distances of each bit and preamble sent
/// @param none
void fallingEdgeDetector(void) {
  if(preamble) {
    referenceFrame = timerRead(tim0_cfg); // make the preamble the new reference frame
    timerWrite(tim1_cfg, 0);              // reset watchdog timer
    preamble = false;                     // clear preamble flag
  } 
  else {
    processDataLength(); // process the length of data bit
    sanityChecks();      // check incoming data
    isPacketFinished();  // check for complete data packet 
  }
}

/// @brief initialize tim0 and tim1 for operation
/// @param none 
void timerSetup(void) {
  tim0_cfg  = timerBegin(0, HUNDRED_KHZ_CLK, true); // 1 us period clock
  timerAlarmWrite(tim0_cfg, 10000, true); // 100 ms overflow 
  timerAlarmEnable(tim0_cfg);

  tim1_cfg = timerBegin(1, HUNDRED_KHZ_CLK, true);
  timerAlarmWrite(tim1_cfg, 350, true); // 3.5 ms timer overflow
  timerAttachInterrupt(tim1_cfg, tim1_overflow, true);
  timerAlarmEnable(tim1_cfg);
}

/// @brief initialize interrupts for falling edge detection
/// @param none
void interruptSetup(void) {
  pinMode(IR_INPUT, INPUT);
  attachInterrupt(digitalPinToInterrupt(IR_INPUT), fallingEdgeDetector, FALLING);
}
