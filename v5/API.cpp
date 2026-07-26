#include "esp32-hal-gpio.h"
#include "API.h"

#include <Adafruit_NeoPixel.h>
#include <math.h>
#define PIN        48 
#define NUMPIXELS   1 
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
// =========================================
// VARIABLES & TUNING
// =========================================
// ... existing variables ...
// =========================================
// WALL FOLLOWING TUNING
// =========================================
const float KP_WALL = 1.3850;       // แรงดึงกลับเข้ากลาง (ถ้าวิ่งชนกำแพงให้เพิ่ม, ถ้าส่ายให้ลด)
const float KD_WALL = 0.8250;      // แรงหน่วงการส่าย (ช่วยหยุดการแกว่ง) <-- เพิ่มตัวนี้
const int WALL_CENTER_DIST = 50.625; // ระยะห่างจากกำแพงที่ต้องการ (mm)
const int WALL_VALID_MAX = 55;  
const int MAX_WALL_ADJUST = 65;

// ตัวแปรสำหรับจำค่า Error รอบที่แล้ว (เพื่อหา Derivative)
float prevWallError = 0;


// ... existing variables
const int CELL_DISTANCE_TICKS = 1000 ; 

// Global Objects
TwoWire I2C_DIST = TwoWire(0);
TwoWire I2C_GYRO = TwoWire(1);
VL6180X sensors[NUM_SENSORS];
uint16_t distValues[NUM_SENSORS] = {0,0,0,0}; 

const int xshut_pins[] = {18, 6, 3, 7};//0,1,2,3 //fl,fr,sl,sr 

// Gyro Vars
float currentYaw = 0.0;
float gyroZoffset = 0.0;
unsigned long lastGyroTime = 0;
float GYRO_SCALE = (90.0/47); 

// Encoder Vars1
volatile long countL = 0;
volatile long countR = 0;
unsigned long rampStartTime = 0;

// PID Constants
float Kp_turn = 5.00; float Ki_turn = 0.00; float Kd_turn = 0.0;

// ตัวแปรสำหรับเลี้ยวขวา (ค่าบวก)
float Kp_turn_R = 1.7300;
float Ki_turn_R = 0.0000;
float Kd_turn_R = 0.008125;

// ตัวแปรสำหรับเลี้ยวซ้าย (ค่าลบ)
float Kp_turn_L = 1.7300; // อาจจะต้องจูนให้ต่างจากขวา
float Ki_turn_L = 0.0000;
float Kd_turn_L = 0.008125;

// Driving Constants
const float KP_sync = 2.0;
const int MOTOR_BIAS = 6;  // บวกให้ซ้าย/ลบให้ขวา ถ้าหุ่นเบี้ยวขวาให้เพิ่ม, เบี้ยวซ้ายให้ลด
const int MIN_PWM = 35;
const int MAX_PWM = 255;
const int TARGET_SPEED = 70;
const int RAMP_DURATION = 1500;
const int TURN_MIN_PWM = 125;
const int TURN_MAX_PWM = 255;

// =========================================
// INTERRUPTS & LOW LEVEL
// =========================================
void IRAM_ATTR readEncoderL() { 
  if(digitalRead(ENC_L_B) == LOW) countL++; else countL--; 
}
void IRAM_ATTR readEncoderR() { 
  if(digitalRead(ENC_R_B) == HIGH) countR++; else countR--; 
}

void setSpeed(int L, int R) {
  L = constrain(L, -255, 255); 
  R = constrain(R, -255, 255);
  if (L >= 0) { analogWrite(M1_IN1, L); analogWrite(M1_IN2, 0); } 
  else { analogWrite(M1_IN1, 0); analogWrite(M1_IN2, abs(L)); }
  if (R >= 0) { analogWrite(M2_IN1, 0); analogWrite(M2_IN2, R); } 
  else { analogWrite(M2_IN1, abs(R)); analogWrite(M2_IN2, 0); }
}

void brakeMotors() {
  analogWrite(M1_IN1, 255); analogWrite(M1_IN2, 255);
  analogWrite(M2_IN1, 255); analogWrite(M2_IN2, 255);
}

void forceBrake() {
  setSpeed(-255, -255); 
  delay(50); 
  brakeMotors();
  delay(100);
}

// =========================================
// INIT FUNCTIONS (ใส่ Debug Print กลับมาแล้ว)
// =========================================
void initGyro() {
  I2C_GYRO.begin(SDA_GYRO, SCL_GYRO, 400000);
  I2C_GYRO.beginTransmission(MPU_ADDR);
  I2C_GYRO.write(0x6B); 
  I2C_GYRO.write(0);
  
  // เช็คสถานะการเชื่อมต่อและ Print ผลลัพธ์
  if (I2C_GYRO.endTransmission(true) == 0) {
    Serial.println("✅ Gyro (Bus 1) OK");
    pixels.setPixelColor(0, pixels.Color(0, 255, 0));pixels.show();
  } else {
    Serial.println("❌ Gyro Error!");
    pixels.setPixelColor(0, pixels.Color(255, 255, 0));pixels.show();
  
  }
}

void calibrateGyro() {
  Serial.println("⚠️ Calibrating Gyro...");
  brakeMotors();
  delay(500);
  long sum = 0;
  for (int i = 0; i < 2000; i++) {
    I2C_GYRO.beginTransmission(MPU_ADDR);
    I2C_GYRO.write(0x47);
    I2C_GYRO.endTransmission(false);
    I2C_GYRO.requestFrom(MPU_ADDR, 2, true);
    if (I2C_GYRO.available() >= 2) {
      sum += (int16_t)((I2C_GYRO.read() << 8) | I2C_GYRO.read());
    }
    delay(1);
  }
  gyroZoffset = sum / 2000.0;
  Serial.printf("✅ Gyro Offset: %.2f\n", gyroZoffset);
}

void initDistSensors() {
  Serial.println("--- Init Distance Sensors (Bus 0) ---");
  I2C_DIST.begin(SDA_DIST, SCL_DIST, 100000); 

  // 1. Reset (XSHUT LOW)
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(xshut_pins[i], OUTPUT);
    digitalWrite(xshut_pins[i], LOW);
  }
  delay(10);

  // 2. Init ทีละตัว
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(xshut_pins[i], INPUT);
    delay(10);
    sensors[i].setBus(&I2C_DIST);
    
    // --- จุดที่แก้ไข ---
    sensors[i].init();            // เรียกเฉยๆ ไม่ต้องใส่ if
    sensors[i].configureDefault(); // แนะนำให้เพิ่มบรรทัดนี้เพื่อตั้งค่ามาตรฐาน
    sensors[i].setAddress(0x30 + i);
    sensors[i].setTimeout(500);
    
    // แก้ชื่อฟังก์ชัน startContinuous เป็น startRangeContinuous
    // ใส่เลข 50 คือให้อ่านค่าทุกๆ 50ms (ปรับได้)
    sensors[i].startRangeContinuous(50); 
    
    Serial.printf("✅ Sensor %d OK (Addr: 0x%X)\n", i, 0x30 + i);

    // ------------------
  }
}

void setupRobot() {
  pixels.begin();

  
  
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), readEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), readEncoderR, RISING);
  
  initGyro();
  calibrateGyro();
  initDistSensors();

  delay(1000);
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));pixels.show();
}

// =========================================
// RUNTIME FUNCTIONS
// =========================================
void updateYaw() {
  unsigned long now = micros();
  if (lastGyroTime == 0) { lastGyroTime = now; return; }
  double dt = (now - lastGyroTime) / 1000000.0;
  lastGyroTime = now;

  I2C_GYRO.beginTransmission(MPU_ADDR);
  I2C_GYRO.write(0x47);
  I2C_GYRO.endTransmission(false);
  I2C_GYRO.requestFrom(MPU_ADDR, 2, true);
  
  if (I2C_GYRO.available() >= 2) {
    int16_t rawZ = (I2C_GYRO.read() << 8) | I2C_GYRO.read();
    float rate = (rawZ - gyroZoffset) / 131.0;
    if (abs(rate) > 0.1) {
      currentYaw += (rate * dt * GYRO_SCALE);
    }
  }
}

// ในไฟล์ API.cpp แก้ไขฟังก์ชันนี้
// ใน API.cpp
uint16_t getDistanceMM(int index) {
    if(index < 0 || index >= NUM_SENSORS) return 8190;
    
    // แก้จากอ่าน sensor ตรงๆ เป็นอ่านจากตัวแปรที่ cache ไว้
    uint16_t rawDist = distValues[index]; 
    return rawDist;
}

const int BIAS_RAMP_MS =700;

void moveForwardSync(int targetSpeed) {
  long encError = countL - countR;
  int adjustment = constrain((int)(encError * KP_sync), -MAX_WALL_ADJUST, MAX_WALL_ADJUST);

  unsigned long elapsed = millis() - rampStartTime;
  float t = (elapsed < (unsigned long)BIAS_RAMP_MS) ? (elapsed / (float)BIAS_RAMP_MS) : 1.0f;
  float biasT = (1.0f - cosf(t * M_PI)) / 2.0f;  // smoothstep: slope=0 ที่ t=0 และ t=1
  int scaledBias = (int)(MOTOR_BIAS * biasT);

  setSpeed(targetSpeed - adjustment + scaledBias, targetSpeed + adjustment - scaledBias);
}

void driveDistance(long targetTicks) {
  // --- Reset Encoder ---
  countL = 0; countR = 0;
  rampStartTime = millis();

  const uint32_t CTRL_PERIOD_US = 5000; // 5ms = 200Hz fixed control rate
  uint32_t lastCtrl = micros();
  
  // กำหนดระยะที่จะเริ่มเบรก (หน่วยเป็น Ticks)
  // เช่น 30% ของระยะทางทั้งหมด หรือ Fix ค่าไปเลย เช่น 400 ticks (ประมาณ 10 ซม.)
  long RAMP_DOWN_TICKS = 700; // ปรับค่านี้ตามความเหมาะสม (ยิ่งมาก ยิ่งเบรกยาวนิ่มๆ)
  if (RAMP_DOWN_TICKS > targetTicks / 2) RAMP_DOWN_TICKS = targetTicks / 2; // กันเบรกยาวเกินครึ่งทาง

  Serial.printf("🚗 Drive Distance: %ld ticks (Brake at last %ld)\n", targetTicks, RAMP_DOWN_TICKS); 

  while (abs(countL) < targetTicks) {
    // รอให้ครบ 5ms ก่อน update (fixed 200Hz control rate)
    uint32_t now = micros();
    if (now - lastCtrl < CTRL_PERIOD_US) continue;
    lastCtrl = now;

    unsigned long elapsed = millis() - rampStartTime;
    long remainingDist = targetTicks - abs(countL);

    int speedUp = TARGET_SPEED;
    int speedDown = TARGET_SPEED;

    if (elapsed < RAMP_DURATION) {
      float t = (float)elapsed / RAMP_DURATION;
      speedUp = MIN_PWM + (int)((TARGET_SPEED - MIN_PWM) * sinf(t * M_PI / 2.0f));
    }
    if (remainingDist < RAMP_DOWN_TICKS) {
      float t = (float)remainingDist / RAMP_DOWN_TICKS;
      speedDown = MIN_PWM + (int)((TARGET_SPEED - MIN_PWM) * (1.0f - cosf(t * M_PI)) / 2.0f);
    }

    int currentSpeed = max(min(speedUp, speedDown), MIN_PWM);
    moveForwardSync(currentSpeed);
  }

  // --- จบการทำงาน ---
  brakeMotors(); // เบรกทันทีเมื่อถึงระยะ
  delay(100); 
  setSpeed(0,0);
  Serial.println("✅ Move Done!"); 
}
void turnPID(float targetDeg) {
  currentYaw = 0;
  Serial.printf("🎯 Turn to: %.1f\n", targetDeg); // Print แจ้งเตือนการเลี้ยว

  float prevError = 0, integral = 0;
  bool settled = false;
  unsigned long prevPIDTime = micros();

  while (!settled) {
    updateYaw();
    float error = targetDeg - currentYaw;
    float dt = (micros() - prevPIDTime) / 1000000.0;
    prevPIDTime = micros();
    if(dt <= 0) dt = 0.001;

    if(abs(error) < 30) integral += error * dt; else integral = 0;
    float derivative = (error - prevError) / dt;
    prevError = error;

    float output = (Kp_turn * error) + (Ki_turn * integral) + (Kd_turn * derivative);
    int pwm = constrain((int)output, -MAX_PWM, MAX_PWM);

    if (abs(error) > 1.0) {
      if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;
      if (pwm < 0 && pwm > -MIN_PWM) pwm = -MIN_PWM;
      setSpeed(-pwm, pwm);
    } else {
      brakeMotors();
      settled = true;
    }
    delay(5);
  }
  brakeMotors();
  Serial.printf("✅ Done. Yaw: %.2f\n", currentYaw);
}

void turnPID2(float targetDeg) {
  currentYaw = 0; // รีเซ็ตค่ามุมปัจจุบัน (สมมติว่าใช้ Gyro ที่รีเซ็ตได้ หรือเทียบเป็น Relative)
  Serial.printf("🎯 Turn to: %.1f\n", targetDeg);

  // --- ส่วนที่เพิ่มมา: เลือกค่า PID ตามทิศทาง ---
  float kp, ki, kd; 

  if (targetDeg > 0) {
    // เลี้ยวขวา (ค่าบวก) ใช้ชุด R
    kp = Kp_turn_R;
    ki = Ki_turn_R;
    kd = Kd_turn_R;
    Serial.println("Using RIGHT Turn PID Parameters");
  } else {
    // เลี้ยวซ้าย (ค่าลบ) ใช้ชุด L
    kp = Kp_turn_L;
    ki = Ki_turn_L;
    kd = Kd_turn_L;
    Serial.println("Using LEFT Turn PID Parameters");
  }
  // ----------------------------------------

  float prevError = 0, integral = 0;
  bool settled = false;
  unsigned long prevPIDTime = micros();

  // เพิ่ม Timeout ป้องกันค้าง (Optional แต่แนะนำ)
  unsigned long startTime = millis(); 

  while (!settled) {
    updateYaw(); // อัปเดตค่ามุมจากเซนเซอร์
    
    // คำนวณ Error
    float error = targetDeg - currentYaw; 
    
    // คำนวณ dt
    float dt = (micros() - prevPIDTime) / 1000000.0;
    prevPIDTime = micros();
    if(dt <= 0) dt = 0.001; // กัน Error หาร 0

    // Integral (พร้อม Anti-windup)
    if(abs(error) < 30) integral += error * dt; 
    else integral = 0;

    // Derivative
    float derivative = (error - prevError) / dt;
    prevError = error;

    // คำนวณ Output โดยใช้ kp, ki, kd ที่เลือกมาแล้ว
    float output = (kp * error) + (ki * integral) + (kd * derivative);
    
    int pwm = constrain((int)output, -TURN_MAX_PWM, TURN_MAX_PWM);

    // เงื่อนไขการหยุด (Exit condition)
    if (abs(error) > 1.0) { // ยอมรับความคลาดเคลื่อนได้ +- 1 องศา
      // Deadzone handling (ดันกำลังขั้นต่ำเพื่อให้ล้อหมุน)
      if (pwm > 0 && pwm < TURN_MIN_PWM) pwm = TURN_MIN_PWM;
      if (pwm < 0 && pwm > -TURN_MIN_PWM) pwm = -TURN_MIN_PWM;
      
      setSpeed(-pwm, pwm); // สั่งมอเตอร์ (กลับด้านกันเพื่อให้หมุนตัว)
    } else {
      brakeMotors();
      settled = true;
    }

    // Safety: ถ้าติดลูปนานเกิน 2 วินาทีให้หยุด (ป้องกันหุ่นค้าง)
    if (millis() - startTime > 2000) {
        brakeMotors();
        settled = true;
        Serial.println("⚠️ Turn Timeout!");
    }

    delay(5);
  }
  
  brakeMotors();
  Serial.printf("✅ Done. Yaw: %.2f\n", currentYaw);
  delay(100); // รอให้หุ่นนิ่งสนิทจริงๆ ก่อนไปต่อ
}

void alignFront() {
  const int ALIGN_TOLERANCE = 1;   
  const int ALIGN_TIMEOUT = 3000;  
  const float Kp_align = 2.5;      
  const int MAX_ALIGN_PWM = 37;    
  const int MIN_ALIGN_PWM = 35;    
  const int WALL_THRESHOLD = 305;  

  Serial.println("\n>>> START ALIGNMENT (FL:Pin16 vs FR:Pin6) <<<"); // Debug Start
  setSpeed(0, 0); 
  delay(100);

  unsigned long startTime = millis();

  while (millis() - startTime < ALIGN_TIMEOUT) {
    uint16_t dist_FL = sensors[0].readRangeContinuousMillimeters();
    uint16_t dist_FR = sensors[1].readRangeContinuousMillimeters();

    if (sensors[0].timeoutOccurred() || sensors[1].timeoutOccurred()) {
      Serial.println(" -> Sensor Timeout!"); break; 
    }

    if (dist_FL > WALL_THRESHOLD || dist_FR > WALL_THRESHOLD || dist_FL == 0 || dist_FR == 0) {
      Serial.printf("Skip: Wall invalid (FL:%d, FR:%d)\n", dist_FL, dist_FR);
      break;
    }

    int error = (int)dist_FL - (int)dist_FR;
    Serial.printf("FL: %d | FR: %d | Err: %d", dist_FL, dist_FR, error); // Debug ค่า Realtime

    if (abs(error) <= ALIGN_TOLERANCE) {
      Serial.println(" -> [OK] Aligned!"); // Debug สำเร็จ
      brakeMotors();
      setSpeed(0, 0); 
      break;
    }

    int turnPWM = (int)(error * Kp_align);
    if (turnPWM > 0) turnPWM = constrain(turnPWM, MIN_ALIGN_PWM, MAX_ALIGN_PWM);
    else if (turnPWM < 0) turnPWM = constrain(turnPWM, -MAX_ALIGN_PWM, -MIN_ALIGN_PWM);

    Serial.printf(" | PWM: %d\n", turnPWM); // Debug PWM

    setSpeed(-turnPWM, turnPWM);
    delay(10); 
  }

  setSpeed(0, 0);
  delay(200);
  Serial.println(">>> END ALIGNMENT <<<");
}

// เพิ่มส่วนนี้ใน API.cpp
void readAllDistSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    // อ่านค่าตรงๆ จากเซ็นเซอร์มาเก็บไว้ใน Array
    distValues[i] = sensors[i].readRangeContinuousMillimeters();
    
    // เช็คว่าเซ็นเซอร์ค้าง (Timeout) หรือไม่
    if (sensors[i].timeoutOccurred()) { 
      // หากค้าง ให้ตั้งค่าเป็นค่าสูงสุด เพื่อไม่ให้หุ่นตกใจนึกว่ามีกำแพง
      distValues[i] = 8190; 
    }
  }
}

// Helpers
void turnLeft() { turnPID2(-90.0); }
void turnRight() { turnPID2(90.0); }
void moveForward() { driveDistance(CELL_DISTANCE_TICKS); }

bool wallFront() {  
    // ตอนนี้อ่านได้ ~180-190 แสดงว่าหน้าโล่ง
    // ถ้าต่ำกว่า 130 มม. คือเริ่มใกล้กำแพงหน้าแล้ว
    return (getDistanceMM(0) < 160 && getDistanceMM(1) < 160); 
} 

bool wallLeft()  { 
    // ตอนนี้อ่านได้ 107
    // ดังนั้นต้องตั้งให้มากกว่า 107 เพื่อให้หุ่นรู้ว่า "มีกำแพงซ้าย"
    return (getDistanceMM(2) < 160); // ปรับเป็น 160 มม. (ประมาณครึ่งหนึ่งของช่อง 1 ฟุต)
} 

bool wallRight() { 
    // ตอนนี้อ่านได้ 101
    return (getDistanceMM(3) < 160); // ปรับเป็น 160 มม. เช่นกัน
}

int orientation(int orient, char turning) {
    if (turning == 'L') {
        orient -= 1;
        if (orient < 0) orient = 3;
    } else if (turning == 'R') {
        orient += 1;
        if (orient > 3) orient = 0;
    } else if (turning == 'B') { 
        orient += 2;
        if (orient > 3) orient -= 4;
    }
    return orient;
}