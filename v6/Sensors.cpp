#include "Sensors.h"
#include "Log.h"

#include <Wire.h>
#include <VL6180X.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

// =========================================================
// Hardware objects
// =========================================================
static TwoWire I2C_DIST = TwoWire(0);
static TwoWire I2C_GYRO = TwoWire(1);
static VL6180X sensors[NUM_SENSORS];
static Adafruit_NeoPixel pixels(1, LED_PIN, NEO_GRB + NEO_KHZ800);

static const int xshutPins[NUM_SENSORS] = {18, 6, 3, 7};   // FL, FR, SL, SR

// ค่าระยะล่าสุดของแต่ละตัว เริ่มที่ "ไกลเกิน" เพื่อไม่ให้หุ่นนึกว่ามีกำแพงตอนบูต
static uint8_t distValues[NUM_SENSORS] = {
    DIST_OUT_OF_RANGE, DIST_OUT_OF_RANGE, DIST_OUT_OF_RANGE, DIST_OUT_OF_RANGE};

// =========================================================
// Gyro state
// =========================================================
static float         currentYaw    = 0.0f;
static float         gyroZoffset   = 0.0f;
static unsigned long lastGyroTime  = 0;
static float         lastRate      = 0.0f;   // dps ล่าสุด ใช้ดูว่าชนเพดาน range หรือยัง

// =========================================================
// Encoder state
// =========================================================
static volatile long countL = 0;
static volatile long countR = 0;

static void IRAM_ATTR isrEncL() { if (digitalRead(ENC_L_B) == LOW)  countL++; else countL--; }
static void IRAM_ATTR isrEncR() { if (digitalRead(ENC_R_B) == HIGH) countR++; else countR--; }

// =========================================================
// LED
// =========================================================
void ledColor(uint8_t r, uint8_t g, uint8_t b) {
    pixels.setPixelColor(0, pixels.Color(r, g, b));
    pixels.show();
}

// =========================================================
// Init
// =========================================================
// ---------- MPU6050 registers ----------
#define MPU_PWR_MGMT_1   0x6B
#define MPU_GYRO_CONFIG  0x1B
#define MPU_WHO_AM_I     0x75
#define MPU_GYRO_ZOUT_H  0x47

static bool gyroOk = false;
bool gyroReady() { return gyroOk; }

static bool mpuWrite(uint8_t reg, uint8_t val) {
    I2C_GYRO.beginTransmission(MPU_ADDR);
    I2C_GYRO.write(reg);
    I2C_GYRO.write(val);
    return I2C_GYRO.endTransmission(true) == 0;
}

static bool mpuRead(uint8_t reg, uint8_t *val) {
    I2C_GYRO.beginTransmission(MPU_ADDR);
    I2C_GYRO.write(reg);
    if (I2C_GYRO.endTransmission(false) != 0) return false;
    if (I2C_GYRO.requestFrom((int)MPU_ADDR, 1, (int)true) != 1) return false;
    *val = I2C_GYRO.read();
    return true;
}

// เขียนแล้วอ่านกลับมาเทียบเสมอ -- ไม่งั้นคำสั่งหายไปเงียบ ๆ แล้วเราไม่รู้ตัว
static bool mpuWriteVerify(uint8_t reg, uint8_t val) {
    if (!mpuWrite(reg, val)) return false;
    delay(10);
    uint8_t back;
    return mpuRead(reg, &back) && back == val;
}

static void initGyro() {
    gyroOk = false;
    I2C_GYRO.begin(SDA_GYRO, SCL_GYRO, 400000);

    // MPU6050 ต้องใช้เวลาตั้งตัวหลังจ่ายไฟก่อนจะ ACK คำสั่งเขียน
    // เดิมโค้ดยิงคำสั่งทันทีแล้วบังเอิญรอดเพราะมี WiFi.softAP() คั่นอยู่ก่อนหน้า
    // พอย้าย WiFi ออกไป หน่วงเวลาก้อนนั้นก็หายไปด้วย -> คำสั่งปลุกหลุด ชิปหลับต่อ
    bool alive = false;
    for (int i = 0; i < 20 && !alive; i++) {
        delay(25);
        I2C_GYRO.beginTransmission(MPU_ADDR);
        alive = (I2C_GYRO.endTransmission(true) == 0);
    }
    if (!alive) {
        logPrintf("[ERR] Gyro ไม่ตอบบน I2C (addr 0x%02X) -- เช็คสาย SDA=%d SCL=%d\n",
                  MPU_ADDR, SDA_GYRO, SCL_GYRO);
        ledColor(255, 0, 0);
        return;
    }

    uint8_t who = 0;
    mpuRead(MPU_WHO_AM_I, &who);

    if (!mpuWriteVerify(MPU_PWR_MGMT_1, 0x00)) {   // ปลุกจาก sleep
        logPrintf("[ERR] ปลุก gyro ไม่สำเร็จ (WHO_AM_I=0x%02X)\n", who);
        ledColor(255, 0, 0);
        return;
    }
    delay(100);   // รอให้ gyro นิ่งหลังตื่น ก่อนจะเชื่อค่าที่อ่านได้

    // ตั้งช่วงวัด -- ถ้าไม่เขียน ชิปจะค้างที่ ±250 dps แล้วค่าจะตันตอนหุ่นหมุนอยู่กับที่
    if (!mpuWriteVerify(MPU_GYRO_CONFIG, GYRO_FS_SEL << 3)) {   // FS_SEL อยู่ที่ bit[4:3]
        logPrintf("[ERR] ตั้ง GYRO_CONFIG ไม่สำเร็จ\n");
        ledColor(255, 0, 0);
        return;
    }

    gyroOk = true;
    logPrintf("[OK] Gyro WHO_AM_I=0x%02X, FS_SEL=%d (%.1f LSB/dps)\n",
              who, GYRO_FS_SEL, GYRO_LSB_PER_DPS);
    ledColor(0, 255, 0);
}

static bool readRawZ(int16_t *out) {
    I2C_GYRO.beginTransmission(MPU_ADDR);
    I2C_GYRO.write(MPU_GYRO_ZOUT_H);
    I2C_GYRO.endTransmission(false);
    I2C_GYRO.requestFrom(MPU_ADDR, 2, true);
    if (I2C_GYRO.available() < 2) return false;
    *out = (int16_t)((I2C_GYRO.read() << 8) | I2C_GYRO.read());
    return true;
}

// เฉลี่ย raw Z ตอนหุ่นนิ่ง ๆ เพื่อหา bias -- ต้องไม่ขยับหุ่นระหว่างเรียก
static float sampleZOffset(int samples, unsigned long timeoutMs, bool verbose) {
    const unsigned long deadline = millis() + timeoutMs;
    long sum = 0;
    int  got = 0;

    for (int i = 0; i < samples; i++) {
        if (millis() > deadline) {
            if (verbose) logPrintf("[WARN] gyro sample timeout ที่ %d/%d\n", i, samples);
            break;
        }
        int16_t z;
        if (readRawZ(&z)) { sum += z; got++; }
        if (verbose && (i % 500) == 0) logPrintf("  ... %d/%d\n", i, samples);
        delay(1);   // คืน CPU ให้ FreeRTOS ด้วย ไม่ใช่แค่หน่วงเวลา
    }

    if (got < samples / 2) {
        logPrintf("[ERR] gyro อ่านได้แค่ %d/%d ตัวอย่าง -- offset อาจไม่น่าเชื่อถือ\n", got, samples);
        return got ? (float)sum / got : gyroZoffset;   // อ่านไม่ได้ ใช้ค่าเดิมดีกว่าเอา 0
    }
    return (float)sum / got;
}

static void calibrateGyro() {
    if (!gyroOk) {
        logPrintf("[ERR] ข้าม calibration เพราะ gyro ยังไม่พร้อม\n");
        return;   // เดิมมันเก็บ offset จากค่าศูนย์ล้วนแล้วเดินต่อเหมือนไม่มีอะไรเกิดขึ้น
    }
    logPrintf("Calibrating gyro... (อย่าขยับหุ่น)\n");
    gyroZoffset = sampleZOffset(GYRO_CAL_SAMPLES, GYRO_CAL_TIMEOUT_MS, true);
    logPrintf("[OK] Gyro offset: %.2f\n", gyroZoffset);
}

static void initDistSensors() {
    I2C_DIST.begin(SDA_DIST, SCL_DIST, 100000);

    // ปิดทุกตัวก่อน แล้วค่อยปลุกทีละตัวมาตั้ง address ใหม่
    // (ทุกตัวเกิดมาที่ 0x29 เหมือนกัน ถ้าปลุกพร้อมกันจะชนกัน)
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(xshutPins[i], OUTPUT);
        digitalWrite(xshutPins[i], LOW);
    }
    delay(10);

    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(xshutPins[i], INPUT);   // ปล่อยลอย -> pull-up บนบอร์ดดึงขึ้น = เปิด
        delay(10);

        sensors[i].setBus(&I2C_DIST);
        sensors[i].init();
        sensors[i].configureDefault();
        sensors[i].setAddress(0x30 + i);
        sensors[i].setTimeout(100);
        sensors[i].startRangeContinuous(RANGE_PERIOD_MS);

        logPrintf("[OK] Sensor %d @ 0x%02X\n", i, 0x30 + i);
    }
}

void sensorsBegin() {
    pixels.begin();
    ledColor(0, 0, 32);

    pinMode(ENC_L_A, INPUT_PULLUP);
    pinMode(ENC_L_B, INPUT_PULLUP);
    pinMode(ENC_R_A, INPUT_PULLUP);
    pinMode(ENC_R_B, INPUT_PULLUP);
    pinMode(START_BUTTON, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncL, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncR, RISING);

    initGyro();
    calibrateGyro();
    initDistSensors();

    ledColor(0, 0, 0);
}

// =========================================================
// Distance
// =========================================================
// v5 เรียก readRangeContinuousMillimeters() ซึ่ง "รอ" จนกว่าเซ็นเซอร์จะมีค่าใหม่
// -> block ได้ถึง 50ms x 4 ตัว ทำให้ลูปคุมมอเตอร์กระตุก
// ตรงนี้เช็ค interrupt status ก่อน มีค่าใหม่ค่อยอ่าน ไม่มีก็ใช้ค่าเดิม
void distUpdate() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        uint8_t status = sensors[i].readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO);
        if (status & 0x04) {   // bit2 = range data ready
            distValues[i] = sensors[i].readReg(VL6180X::RESULT__RANGE_VAL);
            sensors[i].writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01);
        }
    }
}

uint8_t distMM(int index) {
    if (index < 0 || index >= NUM_SENSORS) return DIST_OUT_OF_RANGE;
    return distValues[index];
}

// เซ็นเซอร์หน้าเอียง 45 องศาทั้งคู่ ต้องเห็นกำแพงพร้อมกันถึงจะเชื่อว่าเป็นกำแพงหน้าจริง
bool wallFront() { return distMM(DIST_FL) < WALL_FRONT_MM && distMM(DIST_FR) < WALL_FRONT_MM; }
bool wallLeft()  { return distMM(DIST_SL) < WALL_SIDE_MM; }
bool wallRight() { return distMM(DIST_SR) < WALL_SIDE_MM; }

// =========================================================
// Gyro
// =========================================================
void yawUpdate() {
    if (!gyroOk) return;

    unsigned long now = micros();
    if (lastGyroTime == 0) { lastGyroTime = now; return; }   // ตัวอย่างแรกใช้ตั้งนาฬิกาเฉย ๆ
    float dt = (now - lastGyroTime) / 1000000.0f;
    lastGyroTime = now;

    I2C_GYRO.beginTransmission(MPU_ADDR);
    I2C_GYRO.write(MPU_GYRO_ZOUT_H);
    I2C_GYRO.endTransmission(false);
    I2C_GYRO.requestFrom(MPU_ADDR, 2, true);

    if (I2C_GYRO.available() >= 2) {
        int16_t rawZ = (I2C_GYRO.read() << 8) | I2C_GYRO.read();
        lastRate = (rawZ - gyroZoffset) / GYRO_LSB_PER_DPS;
        if (fabsf(lastRate) > GYRO_DEADBAND) currentYaw += lastRate * dt * GYRO_SCALE;
    }
}

// ล้าง lastGyroTime ด้วย ไม่งั้นตัวอย่างแรกหลังรีเซ็ตจะกิน dt ของช่วงที่ไม่ได้เรียก yawUpdate เลย
// (เช่นระหว่าง driveCell ที่ปิด gyro assist ไว้ ห่างกันเป็นวินาที) แล้วกระโดดใส่มุมมั่ว ๆ
void yawReset() {
    currentYaw   = 0.0f;
    lastGyroTime = 0;
    lastRate     = 0.0f;
}

float yaw()     { return currentYaw; }
float yawRate() { return lastRate; }

// =========================================================
// Encoder
// =========================================================
void encReset() { countL = 0; countR = 0; }
long encLeft()  { return countL; }
long encRight() { return countR; }

// v5 วัดระยะจากล้อซ้ายอย่างเดียว ถ้าล้อซ้ายลื่นระยะจะเพี้ยน -- เฉลี่ยสองล้อแม่นกว่า
long encAvg()   { return (labs(countL) + labs(countR)) / 2; }

// =========================================================
// Button
// =========================================================
bool startPressed() { return digitalRead(START_BUTTON) == LOW; }
