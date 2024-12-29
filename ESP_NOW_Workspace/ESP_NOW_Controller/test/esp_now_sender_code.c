#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define POT1_PIN  (36)
#define POT2_PIN  (39)

#define LEFT_POT  (0)
#define RIGHT_POT (1)

#define TURN_POT  (0)
#define SPEED_POT (1)

// how many milliseconds per update (default: 150ms)
#define CONTROLLER_UPDATE_RATE (150)

// number of samples in between deadtime (no ESP_NOW data transmission)
#define NUM_SAMPLES (16)
#define ADC_CENTER_OFFSET (128)

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xA0, 0xA3, 0xB3, 0x96, 0x78, 0x28};

// Structure example to send data
// Must match the receiver structure
typedef struct espNowPacket {
  int leftValue;
  int rightValue;
  bool reverseMode;
} espNowPacket;

// Create a struct_message called myData
espNowPacket userData;

esp_now_peer_info_t peerInfo;

// array to store left and right ADC avg values (final value could be negative)
int32_t avgValue[2] = {0x00};
unsigned long lastTime = 0; 

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast_Packet_Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
volatile uint8_t testValue = 0;

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of transmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void averageReadings(void) {
  uint16_t startTime = micros();
  // reset ADC array
  avgValue[TURN_POT]  = 0;
  avgValue[SPEED_POT] = 0;
  
  // capture NUM_SAMPLES of ADC readings
  for( int i = 0 ; i < NUM_SAMPLES; i++) {
    uint16_t leftSample  = analogRead(POT2_PIN);
    uint16_t rightSample = analogRead(POT1_PIN);
    avgValue[TURN_POT]  += leftSample;
    avgValue[SPEED_POT] += rightSample;
  }

  // average out all the ADC readings
  avgValue[TURN_POT]   /= NUM_SAMPLES;
  avgValue[SPEED_POT]  /= NUM_SAMPLES;

  // fit ADC readings in a byte format
  // reducing resolution also increases stability
  avgValue[TURN_POT]   >>= 4;
  avgValue[SPEED_POT]  >>= 4;

  // since speed is a scalar, there is no need for negative values
  // Make TURN_POT average value centered at 0 (subtract 128, so values from -128 to +127)
  avgValue[TURN_POT] -= ADC_CENTER_OFFSET;
  uint16_t endTime = micros();
  uint16_t timeElapsed = endTime - startTime;
  //Serial.println(timeElapsed);
} 

void loop() {
  // wait until CONTROLLER_UPDATE_RATE time has passed
  if(millis() - lastTime > CONTROLLER_UPDATE_RATE) {

    userData.leftValue  = avgValue[TURN_POT];
    userData.rightValue = avgValue[SPEED_POT];

    if(testValue % 16 == 0) userData.reverseMode = true;
    else userData.reverseMode = false;

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &userData, sizeof(userData));   
    if (result == ESP_OK) {
      Serial.println("success_send");
    }
    else {
      Serial.println("error_send");
    }
    
    Serial.println("sent data:");
    Serial.println(userData.leftValue);
    Serial.println(userData.rightValue);
    
    // increment test value
    testValue++;
    lastTime = millis();
  }
  averageReadings();
}