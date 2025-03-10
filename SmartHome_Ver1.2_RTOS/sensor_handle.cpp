#include "sensor_handle.h"
#include <Arduino.h>
#include "DHT.h"
#include <Wire.h>   

// Khởi tạo ba đối tượng DHT
DHT dht1(DHTPIN1, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE);
DHT dht3(DHTPIN3, DHTTYPE);

// Định nghĩa biến toàn cục
int Flame1_Value = 0;
int Flame2_Value = 0;
int Flame3_Value = 0;

int CO1_Value = 0;
int CO2_Value = 0;
int CO3_Value = 0;

int Temp1_Value = 0;
int Temp2_Value = 0;
int Temp3_Value = 0;

int Hum1_Value = 0;
int Hum2_Value = 0;
int Hum3_Value = 0;

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
// Hàm đọc độ ẩm từ từng cảm biến
int hum1_getValue() {
  return dht1.readHumidity();
}
int hum2_getValue() {
  return dht2.readHumidity();
}
int hum3_getValue() {
  return dht3.readHumidity();
}

// Hàm đọc nhiệt độ từ từng cảm biến
int temp1_getValue() {
  return dht1.readTemperature();
}
int temp2_getValue() {
  return dht2.readTemperature();
}
int temp3_getValue() {
  return dht3.readTemperature();
}