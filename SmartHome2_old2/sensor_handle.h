#ifndef SENSOR_HANDLE_H
#define SENSOR_HANDLE_H
#include "DHT.h"

extern DHT dht1;
extern DHT dht2;
extern DHT dht3;

// Khai báo các chân cảm biến
#define sensorPin1  25
#define sensorPin2  26
#define sensorPin3  27

#define Flame1_PIN 33  
#define Flame2_PIN 35
#define Flame3_PIN 36

// Khai báo chân kết nối với cảm biến DHT11
#define DHTPIN1 13
#define DHTPIN2 12
#define DHTPIN3 14

// Loại cảm biến (DHT11)
#define DHTTYPE DHT11


// Khai báo biến toàn cục (sử dụng extern)
extern int Flame1_Value;
extern int Flame2_Value;
extern int Flame3_Value;

extern int CO1_Value;
extern int CO2_Value;
extern int CO3_Value;

extern int Temp1_Value;
extern int Temp2_Value;
extern int Temp3_Value;

extern int Hum1_Value;
extern int Hum2_Value;
extern int Hum3_Value;

// Khai báo các hàm
int MQ21_getValue();
int MQ22_getValue();
int MQ23_getValue();

int flame1_getValue();
int flame2_getValue();
int flame3_getValue();

int temp1_getValue();
int temp2_getValue();
int temp3_getValue();

int hum1_getValue();
int hum2_getValue();
int hum3_getValue();

#endif
