#ifndef LOG_H
#define LOG_H

#include <Arduino.h>
#include "Config.h"

// =========================================================
// Log - ส่ง log ออกทั้ง USB Serial และ Wi-Fi พร้อมกัน
//
// หุ่นปล่อย Wi-Fi ของตัวเอง (soft AP) ไม่ต้องมีเราเตอร์
//   1. ต่อ Wi-Fi ชื่อ LOG_SSID รหัส LOG_PASS
//   2. telnet 192.168.4.1        (Windows ใช้ PuTTY เลือก Raw/Telnet, mac/linux ใช้ nc 192.168.4.1 23)
//
// logPrintf() ไม่เคย block รอเน็ต -- มันแค่หย่อนข้อความลง buffer แล้วกลับทันที
// ตัว buffer จะถูกปล่อยออกใน logPoll() ซึ่งเรียกจาก loop() ตอนหุ่นไม่ได้อยู่ในลูปคุมมอเตอร์
// ดังนั้น log ระหว่าง driveCell/turnAngle จะโผล่ทีเดียวตอนจบท่านั้น ไม่กวนจังหวะ 200Hz
// =========================================================

// แยกสองขั้นตั้งใจ: เปิด Serial ก่อนเพื่อให้เห็น log ตอน init ฮาร์ดแวร์
// แล้วค่อยเปิดวิทยุทีหลังสุด -- calibrateGyro() จะได้วนในระบบที่เงียบ ไม่มี WiFi task มาแย่งเวลา
// (เปิดวิทยุก่อนแล้วบอร์ดรีเซ็ตด้วย TG1WDT_SYS_RST = interrupt watchdog)
void logBegin();      // Serial อย่างเดียว -- เรียกบรรทัดแรกของ setup()
void logNetBegin();   // เปิด Wi-Fi AP -- เรียกท้าย setup() หลัง init ฮาร์ดแวร์ครบ

void logPoll();                                 // เรียกใน loop() -- รับ client ใหม่ + ปล่อย buffer
void logPrintf(const char *fmt, ...);
bool logClientConnected();

#endif
