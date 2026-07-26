#ifndef API_H
#define API_H

#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// =========================================
// 1. CONFIGURATION & PINS
// =========================================
#define M1_IN1  4
#define M1_IN2  5
#define M2_IN1  2
#define M2_IN2  1   

#define ENC_L_A  13
#define ENC_L_B  14
#define ENC_R_A  21
#define ENC_R_B  47

#define SDA_DIST 8    
#define SCL_DIST 9    
#define SDA_GYRO 11   
#define SCL_GYRO 12   

#define NUM_SENSORS 4
#define MPU_ADDR 0x68
#define START_BUTTON 3

// =========================================
// 2. EXTERN VARIABLES
// =========================================
extern volatile float currentYaw; // ใช้ volatile เพื่อให้ CPU รู้ว่าค่าเปลี่ยนได้ตลอด
extern volatile uint16_t distValues[NUM_SENSORS];
extern SemaphoreHandle_t sensorMutex; // ตัวจัดการความปลอดภัยข้อมูล

// =========================================
// 3. FUNCTION PROTOTYPES
// =========================================
void setupRobot();

// Helper functions for safe data access
float getYaw();
uint16_t getDist(int index);

// Low-Level
void setSpeed(int L, int R);
void brakeMotors();
void forceBrake();

// Movement
void moveForwardSync(int targetSpeed);
void driveDistance(long targetTicks);
void turnPID(float targetDeg); // ใช้ Version 2 ที่รวมมาแล้ว
void turnLeft();
void turnRight();
void moveForward();

// Logic Helpers
bool wallFront();
bool wallRight();
bool wallLeft();
int orientation(int orient, char turning);

#endif