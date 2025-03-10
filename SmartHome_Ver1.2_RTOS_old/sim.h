#ifndef SIM_H
#define SIM_H

#include <Arduino.h>
#include <HardwareSerial.h>

#define simSerial               Serial2
#define MCU_SIM_BAUDRATE        115200
#define MCU_SIM_TX_PIN          17
#define MCU_SIM_RX_PIN          16
#define MCU_SIM_EN_PIN          23
#define PHONE_NUMBER            "+84828541225"
#define TIME_THRESHOLD          10000 //  10s

extern unsigned long startTime;
extern bool overThreshold;

void sim_at_wait();
bool sim_at_cmd(String cmd);
void call();

#endif
