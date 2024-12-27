#ifndef JONATHANIRREMOTE_H_
#define JONATHANIRREMOTE_H_

// define which GPIO pin the IR reciever signal will be connected to
#define IR_RECEIVE_PIN   (39)

// optional: debug the IR reciever with a trusted library?
#define DBG_WITH_IR_LIB  (0)

// clock divider value needed to reach a 100kHz clock
#define HUNDRED_KHZ_CLK  (800)

// again define a GPIO pin the IR recv. signal will be sent to
#define IR_INPUT         (39)

// amount of bits an IR packet is for Sony protocol
#define SONY_PACKET_SIZE (12)

// minimum amount of time allowed for a preamble to be
#define MIN_PREAMBLE_TIME (280)

// minimum amount of time allowed for a logic 0 to be 
#define MIN_LOGIC0_TIME   (110)

// value that is the midpoint between an ideal logic1 (1.8ms) and logic0 (1.2ms)
#define LOGIC1_THRESHOLD  (150)

// optional: debug the current bitCount and check the time elapsed between bits (prints data in serial monitor!)
#define DBG_TIME_ELAPSED (0)

void errorHandler(void);
void processDataLength(void);
void sanityChecks(void);
void parseData(void);
void isPacketFinished(void);
void IRAM_ATTR tim1_overflow(void);
void fallingEdgeDetector(void);
void timerSetup(void);
void interruptSetup(void);

#endif