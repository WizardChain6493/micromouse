#ifndef MAZE_H
#define MAZE_H

#include <Arduino.h>
#include "Config.h"

// =========================================================
// Maze - แผนที่กำแพง + flood fill + ตัดสินใจว่าจะไปทางไหน
// ไม่ยุ่งกับฮาร์ดแวร์เลย ทดสอบแยกได้
// =========================================================

// ทิศสัมบูรณ์ ต้องเรียงแบบนี้เพื่อให้ (dir + 1) % 4 = หมุนขวา
enum Dir { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3 };

// เคลียร์กำแพง ใส่กำแพงขอบสนาม แล้ว flood จากเป้าหมาย
void mazeInit();

// บันทึกกำแพงที่เซ็นเซอร์เห็น ณ ช่อง (x,y) ขณะหันไปทาง dir
void mazeSetWalls(int x, int y, int dir, bool left, bool right, bool front);

// คำนวณระยะทุกช่องใหม่จากเป้าหมาย (BFS ทะลุเฉพาะช่องที่ไม่มีกำแพงกั้น)
void mazeFlood();

// 'F' เดินหน้า / 'L' เลี้ยวซ้าย / 'R' เลี้ยวขวา / 'B' กลับหลัง
char mazeNextMove(int x, int y, int dir);

bool mazeAtGoal(int x, int y);
int  mazeDistance(int x, int y);          // 255 = เข้าไม่ถึง
bool mazeReachable(int x, int y);

// ข้างหน้าไปได้ไหม -- ต้องเช็คก่อนสั่ง driveCell() เสมอ
bool mazeCanGo(int x, int y, int dir);

// อัปเดตทิศหลังหมุน และอัปเดตพิกัดหลังเดินหน้า 1 ช่อง
int  dirAfterTurn(int dir, char move);
bool stepForward(int dir, int *x, int *y);   // false = จะหลุดขอบกระดาน ไม่ขยับให้

void mazePrint(int robotX, int robotY);

#endif
