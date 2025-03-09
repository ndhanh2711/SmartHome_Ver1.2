#ifndef SENSOR_HANDLE_H
#define SENSOR_HANDLE_H

// Khai báo các chân cảm biến
#define sensorPin1  25
#define sensorPin2  26
#define sensorPin3  27

#define Flame1_PIN 33  
#define Flame2_PIN 35
#define Flame3_PIN 36

// Khai báo biến toàn cục (sử dụng extern)
extern int Flame1_Value;
extern int Flame2_Value;
extern int Flame3_Value;

extern int CO1_Value;
extern int CO2_Value;
extern int CO3_Value;

// Khai báo các hàm
int MQ21_getValue();
int MQ22_getValue();
int MQ23_getValue();

int flame1_getValue();
int flame2_getValue();
int flame3_getValue();

#endif
