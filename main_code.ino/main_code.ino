#define ERA_LOCATION_VN
#define ERA_AUTH_TOKEN "b70461d1-3d3a-47fe-acd3-43b6a7f11c3e"
#include <Arduino.h>
#include <ERa.hpp>
#include <Automation/ERaSmart.hpp>
#include <Time/ERaEspTime.hpp>

const char ssid[] = "Quang Hai T3";
const char pass[] = "19741975";

#define RELAY_PIN 2  // GPIO dùng để điều khiển công tắc

ERaEspTime syncTime;
ERaSmart smart(ERa, syncTime);

void setup() {
#if defined(ERA_DEBUG)
    Serial.begin(115200);
#endif
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);  // Mặc định tắt relay

    ERa.setScanWiFi(true);
    ERa.begin(ssid, pass);
}

/* Điều khiển công tắc thông qua Virtual Pin V1 */
ERA_WRITE(V1) {
    int value = param.asInt();  // Đọc giá trị từ app (0 hoặc 1)
    digitalWrite(RELAY_PIN, value);
    ERa.virtualWrite(V1, value); // Cập nhật trạng thái về app
}

/* Gửi thông số đo lên app */
void sendDataToApp() {
    ERa.virtualWrite(V0, 17);
    ERa.virtualWrite(V2, 16);
    ERa.virtualWrite(V3, 73);
    ERa.virtualWrite(V4, 82);
    ERa.virtualWrite(V5, 68);
    ERa.virtualWrite(V6, "OK");
    ERa.virtualWrite(V7, "WARNING");
    ERa.virtualWrite(V8, "OK");
    ERa.virtualWrite(V9, "WARNING"); // Fix lỗi chính tả
    ERa.virtualWrite(V10, "OK");
    ERa.virtualWrite(V11, "WARNING");
}

void loop() {
    ERa.run();
    sendDataToApp(); // Gửi dữ liệu lên app mỗi lần chạy loop
}

