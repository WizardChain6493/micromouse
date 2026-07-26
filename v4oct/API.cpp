#include "esp32-hal-gpio.h"
#include "API.h"
#include <Adafruit_NeoPixel.h>

#define PIN        48 
#define NUMPIXELS   1 
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Objects & Variables
TwoWire I2C_DIST = TwoWire(0);
TwoWire I2C_GYRO = TwoWire(1);
VL6180X sensors[NUM_SENSORS];

// Shared Data (Protected by Mutex)
volatile uint16_t distValues[NUM_SENSORS] = {0,0,0,0}; 
volatile float currentYaw = 0.0;
SemaphoreHandle_t sensorMutex;

// Gyro Internal
float gyroZoffset = 0.0;
unsigned long lastGyroTime = 0;
float GYRO_SCALE = (90.0/69.1); 

// Encoder
volatile long countL = 0;
volatile long countR = 0;

// PID Tuning
const float KP_WALL = 1.35; 
const float KD_WALL = 0.85; 
const int WALL_CENTER_DIST = 45;
const int MAX_WALL_ADJUST = 60;
float prevWallError = 0;

// Turn PID
float Kp_turn_R = 1.7, Ki_turn_R = 0.0, Kd_turn_R = 0.0085;
float Kp_turn_L = 1.7, Ki_turn_L = 0.0, Kd_turn_L = 0.0080;

const int CELL_DISTANCE_TICKS = 1033;
const int xshut_pins[] = {16, 6, 17, 7};

// =========================================
// SENSOR TASK (CORE 0)
// งานเบื้องหลัง: อ่าน Gyro และ Sensor ตลอดเวลา
// =========================================
void SensorTaskCode(void * parameter) {
  unsigned long lastTime = 0;
  for(;;) {
    // 1. Gyro Update (High Frequency ~200Hz)
    unsigned long now = micros();
    double dt = (now - lastGyroTime) / 1000000.0;
    lastGyroTime = now;

    I2C_GYRO.beginTransmission(MPU_ADDR);
    I2C_GYRO.write(0x47);
    I2C_GYRO.endTransmission(false);
    I2C_GYRO.requestFrom(MPU_ADDR, 2, true);
    
    float rate = 0;
    if (I2C_GYRO.available() >= 2) {
      int16_t rawZ = (I2C_GYRO.read() << 8) | I2C_GYRO.read();
      rate = (rawZ - gyroZoffset) / 131.0;
    }

    // 2. Distance Sensors Update (Medium Frequency ~50Hz)
    // อ่านทีละตัวเพื่อลด I2C Bus Contention
    uint16_t tempDist[4];
    for(int i=0; i<NUM_SENSORS; i++) {
        tempDist[i] = sensors[i].readRangeContinuousMillimeters();
        if(sensors[i].timeoutOccurred()) tempDist[i] = 8190;
    }

    // 3. Update Shared Variables (Critical Section)
    if (xSemaphoreTake(sensorMutex, (TickType_t) 5) == pdTRUE) {
      if (abs(rate) > 0.1) currentYaw += (rate * dt * GYRO_SCALE);
      
      for(int i=0; i<4; i++) distValues[i] = tempDist[i];
      
      xSemaphoreGive(sensorMutex);
    }

    // หยุดพักเล็กน้อยเพื่อให้ Task อื่นทำงานได้ (10ms = 100Hz loop)
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

// =========================================
// SAFE ACCESS HELPERS
// =========================================
float getYaw() {
  float y;
  if(xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
    y = currentYaw;
    xSemaphoreGive(sensorMutex);
  }
  return y;
}

uint16_t getDist(int index) {
  if(index < 0 || index >= NUM_SENSORS) return 8190;
  uint16_t d;
  // อ่านค่า int ไม่จำเป็นต้อง lock นาน หรือถ้ากังวลเรื่อง atomic อ่านตรงๆ ได้เลยสำหรับ 16-bit บน 32-bit MCU
  // แต่เพื่อความชัวร์ใช้ค่าล่าสุดที่ Task อัปเดตไว้
  d = distValues[index]; 
  return d;
}

// =========================================
// INTERRUPTS
// =========================================
void IRAM_ATTR readEncoderL() { 
  if(digitalRead(ENC_L_B) == LOW) countL++; else countL--; 
}
void IRAM_ATTR readEncoderR() { 
  if(digitalRead(ENC_R_B) == HIGH) countR++; else countR--; 
}

// =========================================
// MOTOR CONTROL
// =========================================
void setSpeed(int L, int R) {
  L = constrain(L, -255, 255); 
  R = constrain(R, -255, 255);
  // ใช้ ternary operator เพื่อลด Branch prediction miss เล็กน้อย
  analogWrite(M1_IN1, (L >= 0) ? L : 0);
  analogWrite(M1_IN2, (L >= 0) ? 0 : abs(L));
  analogWrite(M2_IN1, (R >= 0) ? 0 : abs(R));
  analogWrite(M2_IN2, (R >= 0) ? R : 0);
}

void brakeMotors() {
  analogWrite(M1_IN1, 255); analogWrite(M1_IN2, 255);
  analogWrite(M2_IN1, 255); analogWrite(M2_IN2, 255);
}

// =========================================
// INIT
// =========================================
void initGyro() {
  I2C_GYRO.begin(SDA_GYRO, SCL_GYRO, 400000); // 400kHz Fast I2C
  I2C_GYRO.beginTransmission(MPU_ADDR); I2C_GYRO.write(0x6B); I2C_GYRO.write(0); I2C_GYRO.endTransmission(true);
}

void calibrateGyro() {
  brakeMotors();
  delay(200);
  long sum = 0;
  for (int i = 0; i < 1000; i++) { // ลดจำนวนรอบลงเพื่อความเร็วตอน Start
    I2C_GYRO.beginTransmission(MPU_ADDR); I2C_GYRO.write(0x47); I2C_GYRO.endTransmission(false);
    I2C_GYRO.requestFrom(MPU_ADDR, 2, true);
    if (I2C_GYRO.available() >= 2) sum += (int16_t)((I2C_GYRO.read() << 8) | I2C_GYRO.read());
    delayMicroseconds(500); // เร็วกว่า delay(1)
  }
  gyroZoffset = sum / 1000.0;
  lastGyroTime = micros(); // Init เวลาเริ่มต้น
}

void initDistSensors() {
  I2C_DIST.begin(SDA_DIST, SCL_DIST, 400000); // 400kHz
  for (int i = 0; i < NUM_SENSORS; i++) { pinMode(xshut_pins[i], OUTPUT); digitalWrite(xshut_pins[i], LOW); }
  delay(10);
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(xshut_pins[i], INPUT); delay(2);
    sensors[i].setBus(&I2C_DIST);
    sensors[i].init();
    sensors[i].configureDefault();
    sensors[i].setAddress(0x30 + i);
    sensors[i].setTimeout(100); // ลด Timeout
    sensors[i].startRangeContinuous(20); // 20ms update rate
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
  
  // Create Mutex
  sensorMutex = xSemaphoreCreateMutex();

  // Create Task on Core 0
  xTaskCreatePinnedToCore(
    SensorTaskCode,   /* Function to implement the task */
    "SensorTask",     /* Name of the task */
    4096,             /* Stack size in words */
    NULL,             /* Task input parameter */
    1,                /* Priority of the task (1 = High) */
    NULL,             /* Task handle */
    0);               /* Core where the task should run (0) */

  pixels.setPixelColor(0, pixels.Color(0, 0, 0)); pixels.show();
}

// =========================================
// RUNTIME LOGIC (Main Loop runs on Core 1)
// =========================================

void moveForwardSync(int targetSpeed) {
  // ไม่ต้องอ่าน Sensor เองแล้ว ดึงค่าตัวแปรได้เลย
  int distL = distValues[3]; 
  int distR = distValues[2]; 

  bool hasLeft = (distL > 10 && distL < 60); // ปรับค่า Threshold ให้แคบลง
  bool hasRight = (distR > 10 && distR < 60);

  float error = 0;
  if (hasLeft && hasRight) error = distL - distR; 
  else if (hasLeft) error = distL - WALL_CENTER_DIST;
  else if (hasRight) error = WALL_CENTER_DIST - distR;
  else {
    prevWallError = 0; 
    long encError = countL - countR;
    int adjustment = (int)(encError * 2.0); 
    adjustment = constrain(adjustment, -MAX_WALL_ADJUST, MAX_WALL_ADJUST);
    setSpeed(targetSpeed - adjustment, targetSpeed + adjustment);
    return;
  }

  int adjustment = (int)(error * KP_WALL + (error - prevWallError) * KD_WALL);
  prevWallError = error;
  adjustment = constrain(adjustment, -MAX_WALL_ADJUST, MAX_WALL_ADJUST);
  setSpeed(targetSpeed - adjustment, targetSpeed + adjustment);
}

void driveDistance(long targetTicks) {
  countL = 0; countR = 0;
  unsigned long rampStartTime = millis();
  
  // ใช้ Loop ที่ไม่อ่าน Sensor หนักๆ
  while (abs(countL) < targetTicks) { 
    // Sensor Task ทำงานเบื้องหลังอยู่แล้ว ไม่ต้องเรียก updateYaw()
    
    unsigned long elapsed = millis() - rampStartTime;
    int currentSpeed = (elapsed < 1000) ? map(elapsed, 0, 1000, 100, 255) : 255;
    
    moveForwardSync(currentSpeed);
    
    // สำคัญ: ใส่ Delay เล็กน้อยเพื่อให้ FreeRTOS จัดการ Task อื่นได้สะดวกขึ้น
    delay(1); 
  }

  // Logic การจอดเหมือนเดิม ... (ตัดย่อ)
  setSpeed(0,0);
}

void turnPID2(float targetDeg) {
  if (xSemaphoreTake(sensorMutex, portMAX_DELAY) == pdTRUE) {
      currentYaw = 0; // Reset Yaw อย่างปลอดภัย
      xSemaphoreGive(sensorMutex);
  }
  
  float kp = (targetDeg > 0) ? Kp_turn_R : Kp_turn_L;
  float ki = (targetDeg > 0) ? Ki_turn_R : Ki_turn_L;
  float kd = (targetDeg > 0) ? Kd_turn_R : Kd_turn_L;

  float prevError = 0, integral = 0;
  unsigned long prevPIDTime = micros();
  unsigned long startTime = millis();

  while (true) {
    float yaw = getYaw(); // อ่านค่า Yaw ล่าสุดจาก Core 0
    float error = targetDeg - yaw;
    
    if (abs(error) <= 1.0) { brakeMotors(); break; }
    if (millis() - startTime > 2000) { brakeMotors(); break; }

    float now = micros();
    float dt = (now - prevPIDTime) / 1000000.0;
    prevPIDTime = now;
    if(dt == 0) dt = 0.001;

    if(abs(error) < 30) integral += error * dt; else integral = 0;
    float output = (kp * error) + (ki * integral) + (kd * (error - prevError) / dt);
    prevError = error;

    int pwm = constrain((int)output, -255, 255);
    // Deadzone boost
    if (pwm > 0 && pwm < 100) pwm = 100;
    if (pwm < 0 && pwm > -100) pwm = -100;

    setSpeed(-pwm, pwm);
    delay(2); // ให้ Task อื่นหายใจ
  }
  delay(100);
}

void turnLeft() { turnPID2(-90.0); }
void turnRight() { turnPID2(90.0); }
void moveForward() { driveDistance(CELL_DISTANCE_TICKS); }

bool wallFront() { return (distValues[0] < 60 && distValues[1] < 60); } 
bool wallLeft()  { return (distValues[2] < 160); } 
bool wallRight() { return (distValues[3] < 160); }

int orientation(int orient, char turning) {
    if (turning == 'L') return (orient - 1 < 0) ? 3 : orient - 1;
    if (turning == 'R') return (orient + 1 > 3) ? 0 : orient + 1;
    if (turning == 'B') {
        orient += 2;
        return (orient > 3) ? orient - 4 : orient;
    }
    return orient;
}