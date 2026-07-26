#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>
#include "Config.h"

// =========================================================
// Motion - ทุกอย่างที่ "สั่งออกไป": มอเตอร์, วิ่งตรง, เลี้ยว
// =========================================================

void motionBegin();

// Low level
void setSpeed(int left, int right);  // -255..255, บวก = เดินหน้า
void brake();                        // เบรกหยุด
void coast();                        // ปล่อยฟรี

// วิ่งตรง 1 ช่อง: คุมทิศด้วย encoder คืน false ถ้าเบรกฉุกเฉินเพราะเจอกำแพงหน้าใกล้เกินไป
bool driveCell();

// วิ่งตรงตามจำนวน ticks ที่กำหนด (driveCell เรียกตัวนี้ด้วย CELL_TICKS)
// ใช้ตอน diagnostic วิ่งยาวรวดเดียวเทียบกับวิ่งหลายช่องแยก
bool driveTicks(long targetTicks);

// เลี้ยวอยู่กับที่ด้วย gyro PID
void turnLeft();
void turnRight();
void turnAround();

// จัดหน้าให้ตั้งฉากกับกำแพงหน้า (ทำได้เฉพาะตอนมีกำแพงหน้า)
void alignFront();

#endif
