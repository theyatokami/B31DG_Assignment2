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
B31DGCyclicExecutiveMonitor monitor(0);

// ----- Task Periods (µs) -----
const unsigned long period1 = 4000UL;
const unsigned long period2 = 3000UL;
const unsigned long period3 = 10000UL;
const unsigned long period4 = 10000UL;
const unsigned long period5 = 5000UL;

// ----- Scheduling Variables -----
unsigned long startTime;
unsigned long next1, next2, next3, next4, next5;

// ----- Global Frequency Measurements -----
volatile float freq1 = 0, freq2 = 0;

// ----- ISR for Button Press -----
volatile bool buttonPressed = false;
void IRAM_ATTR handleButtonInterrupt() {
  buttonPressed = true;
}

// ----- Task Implementations -----
void runTask1() {
  monitor.jobStarted(1);
  digitalWrite(DIGITAL_OUTPUT1_PIN, HIGH);
  delayMicroseconds(250);
  digitalWrite(DIGITAL_OUTPUT1_PIN, LOW);
  delayMicroseconds(50);
  digitalWrite(DIGITAL_OUTPUT1_PIN, HIGH);
  delayMicroseconds(300);
  digitalWrite(DIGITAL_OUTPUT1_PIN, LOW);
  monitor.jobEnded(1);
}

void runTask2() {
  monitor.jobStarted(2);
  digitalWrite(DIGITAL_OUTPUT2_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(DIGITAL_OUTPUT2_PIN, LOW);
  delayMicroseconds(50);
  digitalWrite(DIGITAL_OUTPUT2_PIN, HIGH);
  delayMicroseconds(200);
  digitalWrite(DIGITAL_OUTPUT2_PIN, LOW);
  monitor.jobEnded(2);
}

void runTask3() {
  monitor.jobStarted(3);
  bool initialState = digitalRead(FREQ1_PIN);
  while(digitalRead(FREQ1_PIN) == initialState);
  unsigned long start = micros();
  while(digitalRead(FREQ1_PIN) != initialState);
  unsigned long duration = micros() - start;
  freq1 = duration ? (1000000.0 / (2 * duration)) : 0;
  monitor.jobEnded(3);
}

void runTask4() {
  monitor.jobStarted(4);
  bool initialState = digitalRead(FREQ2_PIN);
  while(digitalRead(FREQ2_PIN) == initialState);
  unsigned long start = micros();
  while(digitalRead(FREQ2_PIN) != initialState);
  unsigned long duration = micros() - start;
  freq2 = duration ? (1000000.0 / (2 * duration)) : 0;
  monitor.jobEnded(4);
}

void runTask5() {
  monitor.jobStarted(5);
  monitor.doWork();
  monitor.jobEnded(5);
}

// ----- Non-periodic Tasks (ISR driven) -----
void checkFrequencySum() {
  digitalWrite(LED_SUM_FREQ_PIN, (freq1 + freq2) > 1500.0 ? HIGH : LOW);
}

void handleButtonTask() {
  if (buttonPressed) {
    buttonPressed = false;
    digitalWrite(TOGGLE_LED_PIN, !digitalRead(TOGGLE_LED_PIN));
    monitor.doWork();
  }
}

// ----- Setup -----
void setup() {
  Serial.begin(115200);
  pinMode(DIGITAL_OUTPUT1_PIN, OUTPUT);
  pinMode(DIGITAL_OUTPUT2_PIN, OUTPUT);
  pinMode(FREQ1_PIN, INPUT);
  pinMode(FREQ2_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(TOGGLE_LED_PIN, OUTPUT);
  pinMode(LED_SUM_FREQ_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonInterrupt, FALLING);

  while (micros() % 1000 > 10) {}
  startTime = monitor.startMonitoring();
  next1 = next2 = next3 = next4 = next5 = startTime;
  Serial.println("Starting anchored EDF cyclic executive for Tasks 1-5");
}

// ----- Main Loop: EDF Scheduler -----
void loop() {
  unsigned long now = micros();

  // Compute deadlines
  unsigned long deadlines[] = { next1 + period1, next2 + period2, next3 + period3, next4 + period4, next5 + period5 };
  unsigned long releases[] = { next1, next2, next3, next4, next5 };

  // Find the earliest deadline ready task
  unsigned long minDeadline = ULONG_MAX;
  int taskToRun = 0;
  for (int i = 0; i < 5; i++) {
    if (now >= releases[i] && deadlines[i] < minDeadline) {
      minDeadline = deadlines[i];
      taskToRun = i + 1;
    }
  }

  // Execute the selected periodic task
  switch(taskToRun) {
    case 1: while (micros() < next1) {} runTask1(); next1 += period1; break;
    case 2: while (micros() < next2) {} runTask2(); next2 += period2; break;
    case 3: while (micros() < next3) {} runTask3(); next3 += period3; break;
    case 4: while (micros() < next4) {} runTask4(); next4 += period4; break;
    case 5: while (micros() < next5) {} runTask5(); next5 += period5; break;
  }

  // After running the periodic task, recalculate slack
  now = micros();
  minDeadline = ULONG_MAX;
  for (int i = 0; i < 5; i++) {
    unsigned long nextDeadline = releases[i] + (i == 0 ? period1 : i == 1 ? period2 : i == 2 ? period3 : i == 3 ? period4 : period5);
    if (nextDeadline < minDeadline) {
      minDeadline = nextDeadline;
    }
  }
  unsigned long slack = (minDeadline > now) ? (minDeadline - now) : 0;

  // Task 7 (higher priority slack task, ~510 µs worst-case)
  if (buttonPressed && slack >= 510) {
    unsigned long start = micros();
    handleButtonTask();  // includes monitor.doWork() (~500 µs)
    slack -= micros() - start;
  }

  // Task 6 (very low overhead, ~10 µs worst-case)
  if (slack >= 10) {
    checkFrequencySum();
  }
}
