#include "sensor_handle.h"
#include <Arduino.h>

// Định nghĩa biến toàn cục
int Flame1_Value = 0;
int Flame2_Value = 0;
int Flame3_Value = 0;

int CO1_Value = 0;
int CO2_Value = 0;
int CO3_Value = 0;

// Định nghĩa các hàm đọc cảm biến
int MQ21_getValue() {
  return digitalRead(sensorPin1) == HIGH ? 0 : 1;
}

int MQ22_getValue() {
  return digitalRead(sensorPin2) == HIGH ? 0 : 1;
}

int MQ23_getValue() {
  return digitalRead(sensorPin3) == HIGH ? 0 : 1;
}

int flame1_getValue() {
  return digitalRead(Flame1_PIN) == HIGH ? 0 : 1;
}

int flame2_getValue() {
  return digitalRead(Flame2_PIN) == HIGH ? 0 : 1;
}

int flame3_getValue() {
  return digitalRead(Flame3_PIN) == HIGH ? 0 : 1;
}
