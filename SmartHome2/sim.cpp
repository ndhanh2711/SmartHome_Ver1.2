#include <HardwareSerial.h>
#include "sim.h"
unsigned long startTime = 0;

bool overThreshold = false;

void sim_at_wait() {
    delay(100);
    while (simSerial.available()) {
        Serial.write(simSerial.read());
    }
}

bool sim_at_cmd(String cmd) {
    simSerial.println(cmd);
    sim_at_wait();
    return true;
}

void call() {
    String temp = "ATD";
    temp += PHONE_NUMBER;
    temp += ";";
    sim_at_cmd(temp); 

    delay(10000); // Đợi 10 giây
    sim_at_cmd("ATH"); // Cúp máy
}