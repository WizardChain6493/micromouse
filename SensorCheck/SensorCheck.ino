#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h>

// =========================================================
// SensorCheck
// สเก็ตช์แยกต่างหาก ไม่ยุ่งกับมอเตอร์/มาเซ่เลย ใช้เช็ค/จูนเซ็นเซอร์ระยะอย่างเดียว
// เปิดมาแล้วยิง Serial ค่า mm ของเซ็นเซอร์ทั้ง 4 ตัวรัว ๆ
//
// ใช้ทำอะไรได้บ้าง:
//   - เช็คว่าต่อสายถูก / เซ็นเซอร์แต่ละตัวยังทำงานอยู่ไหม
//   - หาค่า WALL_CENTER_MM (ยืนกลางช่องที่มีกำแพง 2 ข้าง ดูค่า SL/SR)
//   - หาค่า WALL_FRONT_MM / WALL_SIDE_MM (ดูค่าตอนมี/ไม่มีกำแพง)
// =========================================================

// พินตรงกับ v6/Config.h ทุกตัว ถ้าแก้ที่นั่นก็แก้ที่นี่ให้ตรงกันด้วย
#define SDA_DIST   8
#define SCL_DIST   9

#define NUM_SENSORS  4
// index: 0=FL, 1=FR (หน้าเอียง 45), 2=SL, 3=SR (ข้างตรง)
static const char *SENSOR_NAME[NUM_SENSORS] = {"FL", "FR", "SL", "SR"};
static const int   xshutPins[NUM_SENSORS]   = {18, 6, 3, 7};

#define RANGE_PERIOD_MS   20   // เซ็นเซอร์อ่านค่าใหม่ทุกกี่ ms (ยิ่งน้อยยิ่งไว แต่ noise เพิ่ม)
#define PRINT_PERIOD_MS   100  // พิมพ์ผลทุกกี่ ms

// เกณฑ์กำแพงปัจจุบัน (เอาไว้โชว์ [WALL]/[open] เทียบให้เห็นภาพ ปรับตามที่จูนได้)
#define WALL_FRONT_MM   160
#define WALL_SIDE_MM    160

TwoWire I2C_DIST = TwoWire(0);
VL6180X sensors[NUM_SENSORS];
uint8_t distValues[NUM_SENSORS] = {255, 255, 255, 255};

void initDistSensors() {
    I2C_DIST.begin(SDA_DIST, SCL_DIST, 100000);

    // ปิดทุกตัวก่อน แล้วปลุกทีละตัวมาตั้ง address ใหม่ (ทุกตัวเกิดมาที่ 0x29 เหมือนกัน)
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(xshutPins[i], OUTPUT);
        digitalWrite(xshutPins[i], LOW);
    }
    delay(10);

    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(xshutPins[i], INPUT);
        delay(10);

        sensors[i].setBus(&I2C_DIST);
        sensors[i].init();
        sensors[i].configureDefault();
        sensors[i].setAddress(0x30 + i);
        sensors[i].setTimeout(100);
        sensors[i].startRangeContinuous(RANGE_PERIOD_MS);

        Serial.printf("[OK] %s (sensor %d) @ 0x%02X\n", SENSOR_NAME[i], i, 0x30 + i);
    }
}

// อ่านเฉพาะตัวที่มีค่าใหม่พร้อม ไม่ block รอ (ต่างจาก readRangeContinuousMillimeters ที่รอจนกว่าจะมีค่า)
void distUpdate() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        uint8_t status = sensors[i].readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO);
        if (status & 0x04) {   // bit2 = range data ready
            distValues[i] = sensors[i].readReg(VL6180X::RESULT__RANGE_VAL);
            sensors[i].writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01);
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== Sensor Check ===");
    initDistSensors();
    Serial.println("\nFormat: NAME:ค่ามม(สถานะ)\n");
}

void loop() {
    distUpdate();

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint < PRINT_PERIOD_MS) return;
    lastPrint = millis();

    for (int i = 0; i < NUM_SENSORS; i++) {
        bool isFront = (i == 0 || i == 1);
        int  thresh  = isFront ? WALL_FRONT_MM : WALL_SIDE_MM;
        bool wall    = distValues[i] < thresh;

        Serial.printf("%s:%3d(%s)  ", SENSOR_NAME[i], distValues[i], wall ? "WALL" : "open");
    }
    Serial.println();
}
