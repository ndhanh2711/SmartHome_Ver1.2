#include <Adafruit_Fingerprint.h>

HardwareSerial mySerial(2);
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);
uint8_t id;

void setup() {
    Serial.begin(9600);
    Serial.println("\n\nAdafruit Fingerprint sensor enrollment");

    mySerial.begin(57600, SERIAL_8N1, 16, 17);  // Chọn baudrate và chân TX, RX
    finger.begin(57600);

    if (finger.verifyPassword()) {
        Serial.println("Đã kết nối cảm biến vân tay!");
    } else {
        Serial.println("Không tìm thấy cảm biến vân tay :(");
        while (1) { delay(1); } // Lặp vô hạn nếu không kết nối được
    }

    Serial.println(F("Thông số cảm biến:"));
    finger.getParameters();
    Serial.print(F("Trạng thái: 0x")); Serial.println(finger.status_reg, HEX);
    Serial.print(F("Dung lượng bộ nhớ: ")); Serial.println(finger.capacity);
    Serial.print(F("Security level: ")); Serial.println(finger.security_level);
    Serial.print(F("Baud rate: ")); Serial.println(finger.baud_rate);
}

uint8_t readnumber(void) {
    uint8_t num = 0;
    while (num == 0) {
        while (!Serial.available()); // Chờ dữ liệu nhập vào
        num = Serial.parseInt(); // Đọc số nguyên từ Serial
    }
    return num;
}
uint8_t dangKy() {
    int p = -1;
    Serial.println("Đặt ngón tay lên cảm biến...");

    while (p != FINGERPRINT_OK) {
        p = finger.getImage();
        if (p == FINGERPRINT_NOFINGER) Serial.print(".");
        else if (p == FINGERPRINT_PACKETRECIEVEERR) Serial.println("Lỗi giao tiếp");
        else if (p == FINGERPRINT_IMAGEFAIL) Serial.println("Lỗi xử lý ảnh");
    }
    Serial.println("\nChụp ảnh vân tay thành công!");

    do {
        p = finger.image2Tz(1);
        if (p == FINGERPRINT_IMAGEMESS) Serial.println("Ảnh mờ hoặc nhiễu, hãy thử lại...");
        else if (p != FINGERPRINT_OK) return p;
    } while (p == FINGERPRINT_IMAGEMESS);

    Serial.println("Nhấc ngón tay ra...");
    delay(2000);
    while (finger.getImage() != FINGERPRINT_NOFINGER);

    Serial.println("Đặt lại cùng ngón tay...");
    while ((p = finger.getImage()) != FINGERPRINT_OK);

    do {
        p = finger.image2Tz(2);
        if (p == FINGERPRINT_IMAGEMESS) Serial.println("Ảnh mờ hoặc nhiễu, hãy thử lại...");
        else if (p != FINGERPRINT_OK) return p;
    } while (p == FINGERPRINT_IMAGEMESS);

    Serial.println("So sánh hai ảnh vân tay...");
    p = finger.createModel();
    if (p != FINGERPRINT_OK) {
        Serial.println("Hai ảnh vân tay không khớp!");
        return FINGERPRINT_ENROLLMISMATCH;
    }
    Serial.println("Hai ảnh vân tay khớp!");

    Serial.print("Lưu vân tay với ID "); Serial.println(id);
    p = finger.storeModel(id);
    if (p == FINGERPRINT_OK) {
        Serial.println("Lưu vân tay thành công!");
        return FINGERPRINT_OK;
    } else {
        Serial.println("Lưu vân tay thất bại!");
        return p;
    }
}

void loop() {
    Serial.println("\nSẵn sàng để quét vân tay!");
    Serial.println("Nhập ID để lưu vân tay (1 - 127):");

    id = readnumber();
    if (id == 0) {
        Serial.println("ID không hợp lệ! Hãy nhập lại.");
        return;
    }

    // Kiểm tra xem ID đã tồn tại chưa
    if (finger.loadModel(id) == FINGERPRINT_OK) {
        Serial.println("ID này đã tồn tại! Vui lòng chọn ID khác.");
        return;
    }

    Serial.print("Đăng ký vân tay cho ID #");
    Serial.println(id);

    while (dangKy() != FINGERPRINT_OK);

}

