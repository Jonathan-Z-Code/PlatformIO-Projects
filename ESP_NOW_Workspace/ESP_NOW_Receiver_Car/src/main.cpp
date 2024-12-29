#include <Arduino.h>
#include <Wifi.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

///@author  Jonathan Zurita 
///@date    12-29-2024
///@version V1.0 
///@brief   This code determines the PWM values for the left and right wheel given the data packet recieved
///         from the ESP_NOW_Controller. Additionally, there is a 1 second watchdog timer that triggers an error condition,
///         completely stopping the car (in case the controller dies unexpectedly).

// based on ESP32S pinout
#define ONE_MHZ_TIMER (80)
#define DBG_PIN       (2)
#define REVERSE_PIN   (16)
#define PWM_LEFT_PIN  (15)
#define PWM_RIGHT_PIN (4)    

// task handle variables
TaskHandle_t testing;
TaskHandle_t speedUpdate;

hw_timer_t* tim0_cfg;
hw_timer_t* wdt_cfg;

typedef struct espNowPacket {
  int turnValue;
  int speedValue;
  bool reverseMode;
} espNowPacket;

// Create a espNowPacket structure named recvData
espNowPacket recvData;
volatile bool testFlag = false;
volatile bool errorFlag = false;

void pinSetup(void) {
  pinMode(DBG_PIN,OUTPUT);
  pinMode(PWM_LEFT_PIN, OUTPUT);
  pinMode(PWM_RIGHT_PIN, OUTPUT);
  pinMode(REVERSE_PIN, OUTPUT);
}

int pwmLeftVal  = 0;
int pwmRightVal = 0;
float speedProportion = 0;
int absTurnVal = 0;

void speedUpdateTask(void* pvParameters) {
  for(;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    digitalWrite(DBG_PIN, HIGH);

    Serial.println("esp-now-recv");

    // how much percentage is the speedValue relative to 8-bit max value? 
    speedProportion = ((float)recvData.speedValue / 256);

    // implement a ADC reading deadzone (in order to reduce jittery motion)
    if(recvData.turnValue < -10) {
      absTurnVal = recvData.turnValue * -1; // get abs()
      pwmLeftVal = (int)((float)recvData.speedValue - speedProportion*(float)(absTurnVal*2)); 
      pwmRightVal = recvData.speedValue;
    }
    if(recvData.turnValue > 10) {
      pwmRightVal = (int)((float)recvData.speedValue - speedProportion*(float)(recvData.turnValue*2));
      pwmLeftVal = recvData.speedValue;
    }
    if(recvData.turnValue >= -10 && recvData.turnValue <= 10) {
      pwmLeftVal  = recvData.speedValue;
      pwmRightVal = recvData.speedValue;
    }
    // check bounds (in terms of a 8-bit value)
    if(pwmLeftVal < 0) {
      pwmLeftVal = 0;
    }
    if(pwmRightVal < 0) {
      pwmRightVal = 0;
    }
    if(pwmLeftVal > 255) {
      pwmLeftVal = 255;
    }
    if(pwmRightVal > 255) {
      pwmRightVal = 255;
    }

    //Serial.println("pwmvals");
    //Serial.println(pwmLeftVal);
    //Serial.println(pwmRightVal);
    //Serial.println(speedProportion);

    // analog PWM must be implemented in positive values
    analogWrite(PWM_LEFT_PIN, (uint8_t)pwmLeftVal);
    analogWrite(PWM_RIGHT_PIN, (uint8_t)pwmRightVal);

    if(recvData.reverseMode) {
      digitalWrite(REVERSE_PIN, HIGH);
    }
    else {
      digitalWrite(REVERSE_PIN, LOW);
    }
    
    digitalWrite(DBG_PIN, LOW);

  }
}

// test 500ms FreeRTOS task (I wanted to see if the ESP_NOW callback function plays well with seperate RTOS tasks)
void taskFunction(void* pvParameters) {
  for(;;) {
    Serial.println("task test");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    //vPortEnterCritical(); vPortExitCritical();
  }
}

// test 100ms overflow timer
void tim0_isr(void) {
  testFlag = true;
}

void wdt_overflow(void) {
  // if you reach the overflow of this user-defined watchdog timer,
  // then you have missed X amount of esp_now packets, meaning you should reset all
  // motor values to zero (assume controller has died)
  recvData.turnValue = 0;
  recvData.speedValue = 0;
  recvData.reverseMode = false;
  errorFlag = true;
}

void timerSetup(void) {
  tim0_cfg = timerBegin(0, ONE_MHZ_TIMER, true);
  wdt_cfg  = timerBegin(1, ONE_MHZ_TIMER, true);
  timerAttachInterrupt(wdt_cfg, wdt_overflow, true);
  timerAlarmWrite(wdt_cfg, 1000000, true); // 1 second overflow
  timerAttachInterrupt(tim0_cfg, tim0_isr, true);
  timerAlarmWrite(tim0_cfg, 1000000, true); // 100ms overflow
  timerAlarmEnable(tim0_cfg);
  timerAlarmEnable(wdt_cfg);
}

void rtosSetup(void) {
  // IMPORTANT NOTE: Unike NVIC in STM32, higher numbers mean higher priority in freeRTOS!
  xTaskCreate(taskFunction, "taskFunction", 2048, NULL, 2, &testing);
  xTaskCreate(speedUpdateTask, "speedUpdateFunction", 2048, NULL, 4, &speedUpdate);
}

// worst case (without any print statements): 800 ns
// worst case (with Serial.print statements): 2.3 ms
void espNowDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  //digitalWrite(DBG_PIN, HIGH);
    memcpy(&recvData, incomingData, sizeof(recvData));
    //Serial.print("Bytes received: ");
    //Serial.println(len);
    Serial.print("turnVal: ");
    Serial.println(recvData.turnValue);
    //Serial.print("Int: ");
    //Serial.println(recvData.speedValue);
    timerWrite(wdt_cfg, 0); // reset wdt timer when new packet arrives
  //digitalWrite(DBG_PIN, LOW);

  // IMPORTANT NOTE: treat this callback function as an pseudo-interrupt, and tell RTOS to switch to speedUpdateTask
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Send notification to task from ISR ( no need for a return value )
  xTaskNotifyFromISR(speedUpdate, 0, eNoAction, &xHigherPriorityTaskWoken);
  // If the task notification triggered a higher priority task, request a context switch
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void espNowSetup(void) {
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW ( or keep trying )
  while(esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    delay(500);
  }
  // Once ESPNow is successfully Init, we will define our callback function upon data reception
  esp_now_register_recv_cb(esp_now_recv_cb_t(espNowDataRecv));
}

void setup(void) {
  Serial.begin(115200);
  pinSetup();
  rtosSetup();
  timerSetup();
  espNowSetup();
  //attachInterrupt(digitalPinToInterrupt(), testFunction, RISING);
}

void loop(void) {
  if(testFlag) {
    Serial.println("tim0_isr");
    testFlag = false;
  }
  if(errorFlag) {
    analogWrite(PWM_LEFT_PIN, 0);
    analogWrite(PWM_RIGHT_PIN, 0);
    digitalWrite(REVERSE_PIN, LOW);
    Serial.println("Controller Disconnected!");
    errorFlag = false;
  }
}