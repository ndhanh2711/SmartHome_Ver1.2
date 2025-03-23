#include "sensor_handle.h"
#include "DHT.h"
#include "sim.h"
//Wifi
#define ERA_LOCATION_VN
#define ERA_AUTH_TOKEN "b70461d1-3d3a-47fe-acd3-43b6a7f11c3e"
#include <Arduino.h>
#include <ERa.hpp>
#include <Automation/ERaSmart.hpp>
#include <Time/ERaEspTime.hpp>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Fingerprint.h>

const char ssid[] = "17ThaiBinh";
const char pass[] = "thaibinhmohoiroi";

ERaEspTime syncTime;
ERaSmart smart(ERa, syncTime);

//Task
// Khai báo Task Handle
// TaskHandle_t Task1Handle = NULL;
// TaskHandle_t Task2Handle = NULL;
// TaskHandle_t Task3Handle = NULL;
// TaskHandle_t Task4Handle = NULL;
// TaskHandle_t Task5Handle = NULL;
// TaskHandle_t Task6Handle = NULL;

#define SERVO_PIN 15   // Chân kết nối với động cơ servo

#define BUZZER_PIN 4
#define FAN_PIN 32

#define LED1 2
#define LED2 12
#define LED3 19


LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo servo1;
unsigned long previousMillis = 0; // Biến lưu thời gian cập nhật LCD


// void Data_Handle(void *pvParameters);
// void Buzzer_Fan_Handle(void *pvParameters);
int tam = 0;
int finger_state = 0;


HardwareSerial mySerial(1);
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

int delete_finger_state = 0;
int add_finger_state = 0;
uint8_t nextID = 1;  // ID tự động cho vân tay mới
// Hàm xóa vân tay theo ID (giống code mẫu ban đầu)
uint8_t deleteFingerprint(uint8_t id) {
  uint8_t p = finger.deleteModel(id);
  
  if (p == FINGERPRINT_OK) {
    Serial.println("Deleted!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
  } else if (p == FINGERPRINT_BADLOCATION) {
    Serial.println("Could not delete in that location");
  } else if (p == FINGERPRINT_FLASHERR) {
    Serial.println("Error writing to flash");
  } else {
    Serial.print("Unknown error: 0x");
    Serial.println(p, HEX);
  }
  
  return p;
}

void turnOnFingerprintLED() {
    uint8_t command[] = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x03, 0x35, 0x00, 0x39};
    Serial1.write(command, sizeof(command)); 
}

void setup() {
    Serial.begin(115200);

    ERa.setScanWiFi(true);
    ERa.begin(ssid, pass);

    Serial.println("\n\nAdafruit Fingerprint sensor enrollment");

  // Khởi tạo HardwareSerial cho cảm biến vân tay:
  // Chọn baudrate 57600, SERIAL_8N1, TX ở chân 5 và RX ở chân 18
  mySerial.begin(57600, SERIAL_8N1, 5, 18);
  finger.begin(57600);

  if (finger.verifyPassword()) {
    turnOnFingerprintLED();
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Did not find fingerprint sensor :(");
  }

  Serial.println(F("Reading sensor parameters"));
  finger.getParameters();
  Serial.print(F("Status: 0x")); Serial.println(finger.status_reg, HEX);
  Serial.print(F("Sys ID: 0x")); Serial.println(finger.system_id, HEX);
  Serial.print(F("Capacity: ")); Serial.println(finger.capacity);
  Serial.print(F("Security level: ")); Serial.println(finger.security_level);
  Serial.print(F("Device address: ")); Serial.println(finger.device_addr, HEX);
  Serial.print(F("Packet len: ")); Serial.println(finger.packet_len);
  Serial.print(F("Baud rate: ")); Serial.println(finger.baud_rate);

  finger.getTemplateCount();

  if (finger.templateCount == 0) {
    Serial.print("Sensor doesn't contain any fingerprint data. Please run the 'enroll' example.");
  }
  else {
    Serial.println("Waiting for valid finger...");
    Serial.print("Sensor contains "); Serial.print(finger.templateCount); Serial.println(" templates");
  }

    dht1.begin();
    dht2.begin();
    dht3.begin();

    // Cấu hình chân cảm biến
    pinMode(Flame1_PIN, INPUT);
    pinMode(Flame2_PIN, INPUT);
    pinMode(Flame3_PIN, INPUT);

    pinMode(sensorPin1, INPUT);
    pinMode(sensorPin2, INPUT);
    pinMode(sensorPin3, INPUT);

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);

    // Cấu hình chân đầu ra
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(FAN_PIN, OUTPUT);

    servo1.attach(SERVO_PIN);
    //pinMode(BUTTON_PIN, INPUT);

    // Khởi tạo LCD
    lcd.init();
    lcd.backlight();

    // Hiển thị thông báo ban đầu
    lcd.setCursor(0, 0);
    lcd.print("Smart Home......");

    lcd.setCursor(0, 1);
    lcd.print("Initializing");

    delay(2000); // Chờ 2s để hiển thị thông báo
    lcd.clear();
    //*************************************************************************************************SIM
    pinMode(MCU_SIM_EN_PIN, OUTPUT); 
    digitalWrite(MCU_SIM_EN_PIN, LOW);
    Serial.println("\n\n\n\n-----------------------\nSystem started!!!!");
    //delay(8000); // Đợi module SIM khởi động
    simSerial.begin(MCU_SIM_BAUDRATE, SERIAL_8N1, MCU_SIM_RX_PIN, MCU_SIM_TX_PIN);

    sim_at_cmd("AT");       // Kiểm tra module SIM
    sim_at_cmd("ATI");      // Kiểm tra thông tin thiết bị
    sim_at_cmd("AT+CPIN?"); // Kiểm tra SIM
    sim_at_cmd("AT+CSQ");   // Kiểm tra tín hiệu
    sim_at_cmd("AT+CIMI");  // Kiểm tra thông tin SIM

    // Gán ngắt khi có sự thay đổi mức tín hiệu (RISING hoặc FALLING)
    attachInterrupt(digitalPinToInterrupt(Flame1_PIN), flame1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Flame2_PIN), flame2_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Flame3_PIN), flame3_ISR, CHANGE);
    
    attachInterrupt(digitalPinToInterrupt(sensorPin1), MQ21_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(sensorPin2), MQ22_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(sensorPin3), MQ23_ISR, CHANGE);


    //****************************************************************************************************
    // Tạo Task với FreeRTOS
    // xTaskCreatePinnedToCore(Data_Handle, "Data_Handle",                   4096, NULL, 5, &Task1Handle, 1);
    // xTaskCreatePinnedToCore(Buzzer_Fan_Handle, "Buzzer_Fan_Handle",       4096, NULL, 1, &Task2Handle, 1);
    // xTaskCreatePinnedToCore(App_Handle, "App_Handle",                     8192, NULL, 4, &Task3Handle, 1);
    // xTaskCreatePinnedToCore(Send_Data_App, "Send_Data_App",               8192, NULL, 3, &Task4Handle, 0);
    // xTaskCreatePinnedToCore(LiquidCrystalDisplay, "LiquidCrystalDisplay", 4096, NULL, 2, &Task5Handle, 0);
    // xTaskCreatePinnedToCore(Finger_Handle, "Finger_Handle",               2048, NULL, 2, &Task6Handle, 0);
}

// Task đọc dữ liệu từ cảm biến
void Data_Handle() {
        // Đọc giá trị từ cảm biến
   //     Flame1_Value = flame1_getValue();
        // Flame2_Value = flame2_getValue();
        // Flame3_Value = flame3_getValue();

        // CO1_Value = MQ21_getValue();
        // CO2_Value = MQ22_getValue();
        // CO3_Value = MQ23_getValue();

        Temp1_Value = temp1_getValue();
        Temp2_Value = temp2_getValue();
        Temp3_Value = temp3_getValue();

        Hum1_Value = hum1_getValue();
        Hum2_Value = hum2_getValue();
        Hum3_Value = hum3_getValue();

        // In giá trị cảm biến để debug
        Serial.print("Flame: ");
        Serial.print(Flame1_Value); Serial.print(" ");
        Serial.print(Flame2_Value); Serial.print(" ");
        Serial.print(Flame3_Value); Serial.print(" | ");

        Serial.print("CO: ");
        Serial.print(CO1_Value); Serial.print(" ");
        Serial.print(CO2_Value); Serial.print(" ");
        Serial.println(CO3_Value);
}

// Task kiểm tra giá trị cảm biến và điều khiển buzzer + quạt
bool auto_mode = false; // Mặc định là chế độ thủ công
bool manual_mode = false; // Chế độ thủ công (do app điều khiển)
void Buzzer_Fan_Handle() {
    bool fire_detected = Flame1_Value || Flame2_Value || Flame3_Value;
    bool gas_detected = CO1_Value || CO2_Value || CO3_Value;

    if (fire_detected || gas_detected) {
        auto_mode = true; // Chuyển sang chế độ tự động
        manual_mode = false;
        digitalWrite(BUZZER_PIN, HIGH);
        digitalWrite(FAN_PIN, HIGH);
        
        if (!overThreshold) { 
            overThreshold = true; 
            startTime = millis();
        } else {
            if (millis() - startTime >= TIME_THRESHOLD) { 
                Serial.println("Cảnh báo! Gửi cuộc gọi...");
                call();
                overThreshold = false;
            }
        }
    } else {
        auto_mode = false; // Không có cảnh báo, cho phép chế độ thủ công
        digitalWrite(BUZZER_PIN, LOW);
        if (!manual_mode) { // Chỉ tắt quạt nếu không ở chế độ thủ công
            digitalWrite(FAN_PIN, LOW);
        }
        overThreshold = false;
    }   
}
// Task 1: Gửi dữ liệu lên app
void Send_Data_App() {
        if (isWiFiConnected()) {
            sendDataToApp();
        } else {
            Serial.println("WiFi Disconnected! Skipping data send...");
        }
}
// Task 2: Xử lý lệnh từ App
void App_Handle() {
        ERa.run();
}

// Kiểm tra kết nối WiFi
bool isWiFiConnected() {
    return WiFi.status() == WL_CONNECTED;
}

// Gửi dữ liệu đo lên app
void sendDataToApp() {
    ERa.virtualWrite(V0, Temp1_Value);
    ERa.virtualWrite(V1, Temp2_Value);
    ERa.virtualWrite(V2, Temp3_Value);

    ERa.virtualWrite(V3, Hum1_Value);
    ERa.virtualWrite(V4, Hum2_Value);
    ERa.virtualWrite(V5, Hum3_Value);


    ERa.virtualWrite(V6, Flame1_Value ? "WARNING" : "GOOD");
    ERa.virtualWrite(V8, Flame2_Value ? "WARNING" : "GOOD");
    ERa.virtualWrite(V7, Flame3_Value ? "WARNING" : "GOOD");

    ERa.virtualWrite(V9, CO1_Value  ? "WARNING" : "GOOD");
    ERa.virtualWrite(V10,CO2_Value  ? "WARNING" : "GOOD");
    ERa.virtualWrite(V11,CO3_Value  ? "WARNING" : "GOOD");
}
void LiquidCrystalDisplay() {
        unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 3000) {  // Cập nhật mỗi 2 giây
        previousMillis = currentMillis;

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Temp*C: ");
        lcd.print(Temp1_Value);
        lcd.print(" ");
        lcd.print(Temp2_Value);
        lcd.print(" ");
        lcd.println(Temp3_Value);

        lcd.setCursor(0, 1);
        lcd.print("Hum % : ");
        lcd.print(Hum1_Value);
        lcd.print(" ");
        lcd.print(Hum2_Value);
        lcd.print(" ");
        lcd.println(Hum3_Value);
    }
}

void loop() {
    Data_Handle();
    Buzzer_Fan_Handle();
    Send_Data_App();
    App_Handle();
    LiquidCrystalDisplay();
    //Finger_Handle();
    // Không dùng loop() khi sử dụng FreeRTOS
    if(finger_state == 0){
   getFingerprintID();
   }
}

ERA_WRITE(V13) {
    uint8_t led1 = param.getInt();
    digitalWrite(LED1, led1 ? HIGH : LOW); // Đảm bảo đúng mức logic
    ERa.virtualWrite(V13, led1); // Đồng bộ trạng thái về app
}

ERA_WRITE(V16) {
    uint8_t led2 = param.getInt();
    digitalWrite(LED2, led2 ? HIGH : LOW); // Đảm bảo đúng mức logic
    ERa.virtualWrite(V16, led2); // Đồng bộ trạng thái về app
}

ERA_WRITE(V17) {
    uint8_t led3 = param.getInt();
    digitalWrite(LED3, led3 ? HIGH : LOW); // Đảm bảo đúng mức logic
    ERa.virtualWrite(V17, led3); // Đồng bộ trạng thái về app
}
ERA_WRITE(V12) {
    uint8_t fan = param.getInt();
    if (!auto_mode) { // Chỉ cho phép điều khiển nếu không ở chế độ tự động
        manual_mode = true;
        digitalWrite(FAN_PIN, fan ? HIGH : LOW);
        ERa.virtualWrite(V12, fan); 
    }
}

// Các biến toàn cục cho servo non-blocking
int currentServoPos = 0;           // Vị trí hiện tại của servo
volatile int targetServoPos = 0;   // Vị trí mục tiêu (90 hoặc 0)
unsigned long servoLastUpdateTime = 0;
const unsigned long servoInterval = 20; // Thời gian cập nhật mỗi bước (ms)
bool servoMoving = false;          // Cờ báo hiệu servo đang chuyển động

// Callback nhận lệnh từ app qua ERA_WRITE(V14)
ERA_WRITE(V14) {
    uint8_t led1 = param.getInt();
    targetServoPos = (led1 ? 90 : 0); // Xác định góc mục tiêu

    unsigned long currentMillis = millis();
    if (currentMillis - servoLastUpdateTime >= servoInterval) {
        servoLastUpdateTime = currentMillis;

        if (currentServoPos < targetServoPos) {
            servo1.write(90);
            currentServoPos = 90;
        } 
        else if (currentServoPos > targetServoPos) {
            servo1.write(0);
            currentServoPos = 0;
        }

        if (currentServoPos == targetServoPos) {
            servoMoving = false;
        }
    }

    ERa.virtualWrite(V14, led1); // Đồng bộ trạng thái với app
}

//****************************************FINGER_PRINT_SENSOR***********************************************

//**********************************************************************************************************
uint8_t enrollFingerprint() {
  int p = -1;
  uint8_t id = nextID;
  Serial.print("Waiting for valid finger to enroll as #"); Serial.println(id);

  // Lần 1: chụp ảnh vân tay
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
      case FINGERPRINT_OK:
        Serial.println("Image taken");
        break;
      case FINGERPRINT_NOFINGER:
        Serial.print(".");
        break;
      case FINGERPRINT_PACKETRECIEVEERR:
        Serial.println("Communication error");
        break;
      case FINGERPRINT_IMAGEFAIL:
        Serial.println("Imaging error");
        break;
      default:
        Serial.println("Unknown error");
        break;
    }
  }

  p = finger.image2Tz(1);
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  Serial.println("Remove finger");
  delay(2000);
  p = 0;
  while (p != FINGERPRINT_NOFINGER) {
    p = finger.getImage();
  }

  Serial.print("Place same finger again for ID "); Serial.println(id);
  p = -1;
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
      case FINGERPRINT_OK:
        Serial.println("Image taken");
        break;
      case FINGERPRINT_NOFINGER:
        Serial.print(".");
        break;
      case FINGERPRINT_PACKETRECIEVEERR:
        Serial.println("Communication error");
        break;
      case FINGERPRINT_IMAGEFAIL:
        Serial.println("Imaging error");
        break;
      default:
        Serial.println("Unknown error");
        break;
    }
  }

  p = finger.image2Tz(2);
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  Serial.print("Creating model for #");  Serial.println(id);
  p = finger.createModel();
  if (p == FINGERPRINT_OK) {
    Serial.println("Prints matched!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_ENROLLMISMATCH) {
    Serial.println("Fingerprints did not match");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  Serial.print("Storing model for ID "); Serial.println(id);
  p = finger.storeModel(id);
  if (p == FINGERPRINT_OK) {
    Serial.println("Stored!");
    nextID++; // Tăng ID cho lần ghi sau
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_BADLOCATION) {
    Serial.println("Could not store in that location");
    return p;
  } else if (p == FINGERPRINT_FLASHERR) {
    Serial.println("Error writing to flash");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  return FINGERPRINT_OK;
}

ERA_WRITE(V15) {
  uint8_t value = param.getInt();
  if (value == 1) {
    finger_state = 1;
    delay(1000);
    Serial.println("VP15 ON: Enrolling new fingerprint...");
    enrollFingerprint();
  }
  else{
    finger_state = 0;
  }
  ERa.virtualWrite(V15, value);
}


ERA_WRITE(V19) {
  uint8_t value = param.getInt();
  delete_finger_state = value;
  if (delete_finger_state == 1) {
    finger_state = 1;
    delay(1000);
    Serial.println("GPIO2 is HIGH (trigger from VP19): Reading fingerprint for deletion...");
    
    uint8_t p = finger.getImage();
    if (p != FINGERPRINT_OK) {
      Serial.println("Failed to capture fingerprint image.");
    } else {
      p = finger.image2Tz(1);
      if (p != FINGERPRINT_OK) {
        Serial.println("Failed to convert fingerprint image.");
      } else {
        p = finger.fingerFastSearch();
        if (p == FINGERPRINT_OK) {
          uint8_t id = finger.fingerID;
          Serial.print("Fingerprint found with ID #");
          Serial.println(id);
          p = deleteFingerprint(id);
        } else {
          Serial.println("Fingerprint not found in the database.");
        }
      }
    }
  }
  else{
    finger_state = 0;
  }
  ERa.virtualWrite(V19, value);
}

unsigned long lastScanTime = 0;
const unsigned long scanInterval = 50; // 500ms mỗi lần quét

uint8_t getFingerprintID() {
    if (millis() - lastScanTime < scanInterval) {
        return FINGERPRINT_NOFINGER;  // Không quét nếu chưa đủ thời gian
    }
    lastScanTime = millis();

    uint8_t p = finger.getImage();
    switch (p) {
        case FINGERPRINT_OK:
            Serial.println("Image taken");
            break;
        case FINGERPRINT_NOFINGER:
            return p;
        case FINGERPRINT_PACKETRECIEVEERR:
            Serial.println("Communication error");
            return p;
        case FINGERPRINT_IMAGEFAIL:
            Serial.println("Imaging error");
            return p;
        default:
            Serial.println("Unknown error");
            return p;
    }

  // OK success!

  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted || Hinh anh duoc chuyen doi");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK converted!
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
//    Serial.println("Found a print match!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    tam = tam + 1;
    Serial.println("\nDid not find a match");
    Serial.println("tam = " + String(tam));

    if (tam == 3)
    {
      Serial.println("===========================");
      Serial.println("Canh bao !");
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100);
      Serial.println("Dung Canh bao !");
      digitalWrite(BUZZER_PIN, LOW);
      tam = 0;
    }
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);

  //
  Serial.println("===========================");
  Serial.println("OPEN !");
  servo1.write(90);
  tam = 0;
  delay(2000);
  
  Serial.println("===========================");
  Serial.println("CLOSE !");
  servo1.write(0);

  return finger.fingerID;
}

// returns -1 if failed, otherwise returns ID #
int getFingerprintIDez() 
{
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  return -1;

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);

  

  return finger.fingerID;
}
