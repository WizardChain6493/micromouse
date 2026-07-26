#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "Config.h"

// =========================================================
// Sensors - ทุกอย่างที่ "อ่านค่าเข้ามา": ระยะ, gyro, encoder, ปุ่ม
// =========================================================

void sensorsBegin();

// ---------- Distance ----------
// อ่านค่าใหม่เฉพาะตัวที่พร้อม (ไม่ block) เรียกถี่ ๆ ได้ในลูปคุมมอเตอร์
void distUpdate();
uint8_t distMM(int index);          // 255 = ไกลเกิน/ไม่มีกำแพง
bool wallFront();
bool wallLeft();
bool wallRight();

// ---------- Gyro ----------
bool  gyroReady();                  // false = ปลุกชิปไม่สำเร็จ ห้ามให้หุ่นวิ่ง yaw จะเป็น 0 ตลอด
void  yawUpdate();                  // ต้องเรียกถี่ ๆ ไม่งั้นค่ามุมเพี้ยน
void  yawReset();                   // ตั้งมุมปัจจุบันเป็น 0 และล้างนาฬิกา
float yaw();                        // องศา, เพิ่มขึ้นเมื่อหมุนไปทาง turnRight()
float yawRate();                    // dps ล่าสุด -- ถ้าค่าพีคแตะเพดาน range แปลว่าค่าตัน

// ---------- Encoder ----------
void encReset();
long encLeft();
long encRight();
long encAvg();                      // |L|+|R| หาร 2 ใช้วัดระยะทาง

// ---------- Button ----------
bool startPressed();

// ---------- Status LED ----------
void ledColor(uint8_t r, uint8_t g, uint8_t b);

#endif
