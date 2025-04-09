#include <Arduino.h>

#include "B31DGMonitor.h"

// ----- Pin Definitions -----
#define DIGITAL_OUTPUT1_PIN 26
#define DIGITAL_OUTPUT2_PIN 27
#define FREQ1_PIN 21
#define FREQ2_PIN 19
#define LED_SUM_FREQ_PIN 5
#define BUTTON_PIN 12
#define TOGGLE_LED_PIN 17

// ----- Global Monitor Object -----
B31DGCyclicExecutiveMonitor monitor(10);

// ----- Task Periods (ticks) -----
const TickType_t period1 = pdMS_TO_TICKS(4);    // 4000 µs
const TickType_t period2 = pdMS_TO_TICKS(3);    // 3000 µs
const TickType_t period3 = pdMS_TO_TICKS(10);   // 10000 µs
const TickType_t period4 = pdMS_TO_TICKS(10);   // 10000 µs
const TickType_t period5 = pdMS_TO_TICKS(5);    // 5000 µs

// ----- Frequency Measurement Globals -----
volatile float freq1 = 0, freq2 = 0;
SemaphoreHandle_t freqMutex;
unsigned long startTime;

// ----- ISR Button Handling -----
TaskHandle_t buttonTaskHandle = NULL;

void IRAM_ATTR handleButtonInterrupt() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(buttonTaskHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ----- Tasks Implementations -----
void Task1(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    monitor.jobStarted(1);
    digitalWrite(DIGITAL_OUTPUT1_PIN, HIGH);
    delayMicroseconds(250);
    digitalWrite(DIGITAL_OUTPUT1_PIN, LOW);
    delayMicroseconds(50);
    digitalWrite(DIGITAL_OUTPUT1_PIN, HIGH);
    delayMicroseconds(300);
    digitalWrite(DIGITAL_OUTPUT1_PIN, LOW);
    monitor.jobEnded(1);
    vTaskDelayUntil(&xLastWakeTime, period1);
  }
}

void Task2(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    monitor.jobStarted(2);
    digitalWrite(DIGITAL_OUTPUT2_PIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(DIGITAL_OUTPUT2_PIN, LOW);
    delayMicroseconds(50);
    digitalWrite(DIGITAL_OUTPUT2_PIN, HIGH);
    delayMicroseconds(200);
    digitalWrite(DIGITAL_OUTPUT2_PIN, LOW);
    monitor.jobEnded(2);
    vTaskDelayUntil(&xLastWakeTime, period2);
  }
}
void Task3(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    monitor.jobStarted(3);
    bool validMeasurement = true;
    bool initialState = digitalRead(FREQ1_PIN);
    unsigned long startWait = micros();

    // Wait for the pin to change from its initial state.
    while (digitalRead(FREQ1_PIN) == initialState) {
      if (micros() - startWait > 1000UL) { // 50 ms timeout
        validMeasurement = false;
        break;
      }
      taskYIELD();
    }

    float localFreq = 0;
    if (validMeasurement) {
      // Record the time at the first edge.
      unsigned long firstEdgeTime = micros();
      startWait = firstEdgeTime;
      
      // Wait for the pin to revert to the initial state.
      while (digitalRead(FREQ1_PIN) != initialState) {
        if (micros() - startWait > 1000UL) { // 1 ms timeout
          validMeasurement = false;
          break;
        }
        taskYIELD();
      }

      if (validMeasurement) {
        unsigned long halfPeriod = micros() - firstEdgeTime;
        if (halfPeriod > 0) {
          localFreq = 1000000.0f / (2.0f * halfPeriod);
        }
      }
    }

    // Update global frequency value safely.
    xSemaphoreTake(freqMutex, portMAX_DELAY);
    freq1 = localFreq;
    xSemaphoreGive(freqMutex);

    monitor.jobEnded(3);
    vTaskDelayUntil(&xLastWakeTime, period3);
  }
}

void Task4(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    monitor.jobStarted(4);
    bool validMeasurement = true;
    bool initialState = digitalRead(FREQ2_PIN);
    unsigned long startWait = micros();

    // Wait for the pin to change from its initial state.
    while (digitalRead(FREQ2_PIN) == initialState) {
      if (micros() - startWait > 1000UL) { // 50 ms timeout
        validMeasurement = false;
        break;
      }
      taskYIELD();
    }

    float localFreq = 0;
    if (validMeasurement) {
      unsigned long firstEdgeTime = micros();
      startWait = firstEdgeTime;
      
      // Wait for the pin to revert to the initial state.
      while (digitalRead(FREQ2_PIN) != initialState) {
        if (micros() - startWait > 1000UL) { // 1 ms timeout
          validMeasurement = false;
          break;
        }
        taskYIELD();
      }

      if (validMeasurement) {
        unsigned long halfPeriod = micros() - firstEdgeTime;
        if (halfPeriod > 0) {
          localFreq = 1000000.0f / (2.0f * halfPeriod);
        }
      }
    }

    // Update global frequency value safely.
    xSemaphoreTake(freqMutex, portMAX_DELAY);
    freq2 = localFreq;
    xSemaphoreGive(freqMutex);

    monitor.jobEnded(4);
    vTaskDelayUntil(&xLastWakeTime, period4);
  }
}

void Task5(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    monitor.jobStarted(5);
    monitor.doWork();
    monitor.jobEnded(5);
    vTaskDelayUntil(&xLastWakeTime, period5);
  }
}

void ButtonTask(void *pvParameters) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    digitalWrite(TOGGLE_LED_PIN, !digitalRead(TOGGLE_LED_PIN));
    monitor.doWork();
  }
}

void FrequencySumTask(void *pvParameters) {
  for (;;) {
    xSemaphoreTake(freqMutex, portMAX_DELAY);
    float sum = freq1 + freq2;
    xSemaphoreGive(freqMutex);
    digitalWrite(LED_SUM_FREQ_PIN, sum > 1500.0 ? HIGH : LOW);
    vTaskDelay(pdMS_TO_TICKS(1));  // Check every 1ms
  }
}

// ----- Setup Function -----
void setup() {
  Serial.begin(115200);

  pinMode(DIGITAL_OUTPUT1_PIN, OUTPUT);
  pinMode(DIGITAL_OUTPUT2_PIN, OUTPUT);
  pinMode(FREQ1_PIN, INPUT);
  pinMode(FREQ2_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(TOGGLE_LED_PIN, OUTPUT);
  pinMode(LED_SUM_FREQ_PIN, OUTPUT);

  freqMutex = xSemaphoreCreateMutex();

  // Create FreeRTOS tasks
  xTaskCreate(Task1, "T1", 2048, NULL, 5, NULL);
  xTaskCreate(Task2, "T2", 2048, NULL, 4, NULL);
  xTaskCreate(Task3, "T3", 2048, NULL, 3, NULL);
  xTaskCreate(Task4, "T4", 2048, NULL, 3, NULL);
  xTaskCreate(Task5, "T5", 2048, NULL, 3, NULL);
  xTaskCreate(ButtonTask, "Button", 2048, NULL, 1, &buttonTaskHandle); // High priority for quick ISR response
  xTaskCreate(FrequencySumTask, "FreqSum", 2048, NULL, 1, NULL);       // Low priority task

  // ISR Setup
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonInterrupt, FALLING);
  while (micros() % 1000 > 10) {}
  startTime = monitor.startMonitoring();
  Serial.println("Starting FreeRTOS-based implementation.");
  
  
}

// Empty loop since FreeRTOS takes control.
void loop() {}
