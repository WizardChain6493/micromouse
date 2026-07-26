#include <Arduino.h>

#include "Config.h"
#include "Log.h"
#include "Sensors.h"
#include "Motion.h"
#include "Maze.h"

// =========================================================
// v6 - Micromouse flood fill
//
// โครงสร้าง:
//   Config.h  ค่าคงที่/พินทั้งหมด (จูนที่นี่ที่เดียว)
//   Sensors   ระยะ, gyro, encoder, ปุ่ม
//   Motion    มอเตอร์, วิ่งตรง 1 ช่อง, เลี้ยว
//   Maze      แผนที่กำแพง + flood fill (ไม่ยุ่งฮาร์ดแวร์)
//   v6.ino    ลูปหลัก: อ่านกำแพง -> flood -> เลือกทาง -> ขยับ
// =========================================================

static int  robotX = START_X;
static int  robotY = START_Y;
static int  robotDir = START_DIR;
static bool running = false;

// รอให้เซ็นเซอร์มีค่าสด ๆ ก่อนตัดสินใจเรื่องกำแพง
static void settleSensors() {
    for (int i = 0; i < 4; i++) {
        distUpdate();
        delay(RANGE_PERIOD_MS);
    }
}

static void reachGoal() {
    brake();
    running = false;
    ledColor(0, 255, 0);
    logPrintf("\n*** GOAL REACHED ***\n");
    logPrintf("จบที่ (%d, %d)\n", robotX, robotY);
#if DEBUG_MAZE
    mazePrint(robotX, robotY);
#endif
    while (true) { logPoll(); delay(100); }   // ยังปล่อย log ที่ค้างอยู่ต่อได้
}

void setup() {
    logBegin();       // Serial อย่างเดียว -- ยังไม่เปิดวิทยุ

    motionBegin();
    sensorsBegin();   // calibrateGyro() อยู่ในนี้ ต้องได้วนในระบบที่เงียบ
    mazeInit();

    logNetBegin();    // ค่อยเปิด Wi-Fi AP ตอนฮาร์ดแวร์พร้อมหมดแล้ว

    logPrintf("\n=== Micromouse v6 ===\n");
    logPrintf("Start (%d,%d) dir=%d  ->  Goal (%d,%d) %dx%d\n",
              START_X, START_Y, START_DIR, GOAL_X, GOAL_Y, GOAL_W, GOAL_H);

    // gyro ตาย = yaw เป็น 0 ตลอด = turnAngle() หมุนหุ่นจนหมด timeout ทุกครั้ง
    // อย่าปล่อยให้เริ่มวิ่ง ค้างไว้ให้เห็นสาเหตุดีกว่า
    if (!gyroReady()) {
        logPrintf("\n*** หยุด: gyro ไม่พร้อม ห้ามวิ่ง ***\n");
        while (true) { logPoll(); ledColor(255, 0, 0); delay(500); ledColor(0, 0, 0); delay(500); }
    }

    logPrintf("กดปุ่ม START เพื่อเริ่ม\n");
}

void loop() {
    logPoll();        // รับ client telnet ใหม่ + ปล่อย log ที่ค้างใน buffer
    yawUpdate();

#if DEBUG_ENCODER
    // ========== หาสาเหตุหุ่นเดินเบี้ยว ==========
    // มีสองเทส แยกสาเหตุออกจากกัน (counts เริ่มที่ 0 ตอนบูต ไม่ต้องรีเซ็ต)
    //
    // เทส A -- เข็นด้วยมือ (ไม่ต้องกดปุ่ม):
    //   ลากหุ่นด้วยมือไปตามเส้นตรง ~1 เมตร ให้ตรงที่สุด (ล้อหมุนอิสระ มอเตอร์ไม่ทำงาน)
    //   บังคับให้สองล้อเดินระยะเท่ากันเป๊ะ แล้วดูว่า L กับ R นับได้เท่ากันไหม
    //
    // เทส B -- กดปุ่ม START:
    //   สั่งมอเตอร์สองข้าง PWM เท่ากันเป๊ะ ไม่มี encoder แก้ทิศเลย วิ่ง 1.5 วิ
    //   ดูว่าหุ่นเบี้ยวทางไหน + L/R ต่างกันเท่าไหร่
    //
    // แปลผล:
    //   A ต่างกันเยอะ            -> encoder/ล้อสองข้างนับต่อระยะไม่เท่า (เชิงกล) = ต้นเหตุจริง
    //   A เท่า + B ต่าง+เบี้ยว    -> มอเตอร์ข้างนึงแรงกว่า (ล้อที่ count มากกว่าคือข้างที่เร็วกว่า)
    //   A เท่า + B เท่าแต่เบี้ยว  -> ล้อขนาดต่างกัน (นับเท่าแต่ระยะจริงไม่เท่า)
    //   A เท่า + B เท่า + ตรง     -> ฮาร์ดแวร์ปกติ ความเบี้ยวมาจากตัวคุม/ramp
    static bool ranOpenLoop = false;
    if (startPressed() && !ranOpenLoop) {
        delay(800);
        encReset();
        unsigned long t0 = millis();
        while (millis() - t0 < 1500) { setSpeed(DRIVE_SPEED, DRIVE_SPEED); delay(5); }
        brake();
        long l = encLeft(), r = encRight();
        logPrintf("\n[เทส B] PWM สองข้างเท่ากัน 1.5s -> L=%ld R=%ld | diff=%ld (%.1f%%)\n",
                  l, r, l - r, r != 0 ? 100.0 * (l - r) / r : 0.0);
        logPrintf("        หุ่นเบี้ยวทางไหน? (จำไว้เทียบกับ diff)\n");
        ranOpenLoop = true;
    }

    static unsigned long lastEncPr = 0;
    if (millis() - lastEncPr > 200) {
        lastEncPr = millis();
        logPrintf("enc  L=%6ld  R=%6ld  |  L-R=%ld\n", encLeft(), encRight(), encLeft() - encRight());
    }
    return;
#endif

#if DEBUG_SENSORS
    // โหมดจูน: ดูค่าดิบเพื่อตั้ง WALL_FRONT_MM / WALL_SIDE_MM (เกณฑ์ตัดสินว่ามีกำแพง)
    distUpdate();
    logPrintf("FL:%3d FR:%3d | SL:%3d SR:%3d | yaw:%7.2f\n",
              distMM(DIST_FL), distMM(DIST_FR),
              distMM(DIST_SL), distMM(DIST_SR), yaw());
    delay(100);
    return;
#endif

#if DEBUG_STRAIGHT_CONT
    // โหมดหาสาเหตุดริฟต์: วิ่งตรง CALIBRATE_CELLS ช่อง "รวดเดียว" (ออกตัวครั้งเดียว)
    // เทียบกับ DEBUG_CALIBRATE ที่วิ่งแยกทีละช่อง (ออกตัว CALIBRATE_CELLS ครั้ง)
    //   ถ้ารวดเดียวดริฟต์น้อยกว่าแยกมาก -> ดริฟต์มาจากการออกตัว (ทฤษฎีมอเตอร์ start ไม่พร้อม ถูก)
    //   ถ้าดริฟต์เท่ากัน                 -> เชิงกล (ล้อเอียง/toe) start ไม่เกี่ยว
    static bool contDone = false;
    if (!contDone) {
        if (!startPressed()) { delay(10); return; }
        delay(1000);
        settleSensors();
        logPrintf("\n=== วิ่งตรงรวดเดียว %d ช่อง (ออกตัวครั้งเดียว) ===\n", CALIBRATE_CELLS);
        driveTicks((long)CALIBRATE_CELLS * CELL_TICKS);
        brake();
        contDone = true;
        logPrintf("*** วัดดริฟต์ข้าง (ซ้าย/ขวา กี่ cm) เทียบกับตอนวิ่งแยกทีละช่อง ***\n");
    }
    delay(100);
    return;
#endif

#if DEBUG_CALIBRATE
    // โหมดจูน CELL_TICKS: วิ่งตรง CALIBRATE_CELLS ช่องติดกัน แล้วให้วัดระยะจริงด้วยไม้บรรทัด/เทปวัด
    // error สะสมจากหลายช่องจะเห็นชัดกว่าวัดทีละช่องเยอะ
    static bool calibDone = false;
    if (!calibDone) {
        if (!startPressed()) { delay(10); return; }
        delay(1000);
        settleSensors();

        logPrintf("\n=== CALIBRATE: วิ่งตรง %d ช่อง (คาดว่า %dmm) ===\n",
                  CALIBRATE_CELLS, CALIBRATE_CELLS * CELL_SIZE_MM);

        for (int i = 0; i < CALIBRATE_CELLS; i++) {
            if (!driveCell()) {
                logPrintf("[WARN] เจอกำแพงก่อนครบ %d ช่อง (วิ่งได้ %d) -- หาทางตรงยาวกว่านี้\n",
                          CALIBRATE_CELLS, i);
                break;
            }
        }
        brake();
        calibDone = true;

        logPrintf("\n*** วัดระยะจริงจากจุดเริ่ม (เส้น/ผนังที่หน้าหุ่นชนตอนเริ่ม) ถึงหน้าหุ่นตอนนี้ (mm) ***\n");
        logPrintf("สูตรคำนวณ CELL_TICKS ใหม่:\n");
        logPrintf("  CELL_TICKS_ใหม่ = %d * (%d * %d) / ระยะที่วัดได้(mm)\n",
                  CELL_TICKS, CALIBRATE_CELLS, CELL_SIZE_MM);
        logPrintf("  เช่น วัดได้ 1750mm -> CELL_TICKS_ใหม่ = %d * %d / 1750 = %.0f\n",
                  CELL_TICKS, CALIBRATE_CELLS * CELL_SIZE_MM,
                  (float)CELL_TICKS * (CALIBRATE_CELLS * CELL_SIZE_MM) / 1750.0f);
    }
    delay(100);
    return;
#endif

    if (!running) {
        if (!startPressed()) { delay(10); return; }   // logPoll() ข้างบนยังวิ่งอยู่ ต่อ telnet ได้ระหว่างรอ
        delay(1000);              // ปล่อยมือจากปุ่มก่อน
        settleSensors();
        running = true;
        ledColor(0, 0, 32);
    }

    // 1. อ่านกำแพงรอบตัวแล้วบันทึกลงแผนที่
    distUpdate();
    mazeSetWalls(robotX, robotY, robotDir, wallLeft(), wallRight(), wallFront());

    // 2. คำนวณระยะทุกช่องใหม่
    mazeFlood();

    // 3. ถึงเป้าหมายหรือยัง
    if (mazeAtGoal(robotX, robotY)) reachGoal();

    // ช่องปัจจุบันเข้าไม่ถึงเป้าหมายแล้ว = แผนที่พัง (ปกติเกิดจากบันทึกกำแพงผิด)
    // เดินต่อไปก็มั่วเปล่า ๆ หยุดให้เห็นสภาพแผนที่ดีกว่า
    if (!mazeReachable(robotX, robotY)) {
        brake();
        logPrintf("\n*** หยุด: (%d,%d) เข้าไม่ถึงเป้าหมาย -- หุ่นถูกกำแพงในแผนที่ปิดล้อม ***\n",
                  robotX, robotY);
        mazePrint(robotX, robotY);
        ledColor(255, 0, 0);
        while (true) { logPoll(); delay(100); }
    }

    // 4. เลือกทางที่ระยะสั้นที่สุด (เท่ากันให้เดินตรงก่อน)
    char move = mazeNextMove(robotX, robotY, robotDir);

    logPrintf("(%d,%d) dir=%d dist=%d -> %c\n",
              robotX, robotY, robotDir, mazeDistance(robotX, robotY), move);
#if DEBUG_MAZE
    mazePrint(robotX, robotY);
#endif

    // 5. จ่อกำแพงหน้าอยู่ก็ใช้จัดหัวให้ตั้งฉากก่อน แล้วค่อยเลี้ยว มุมจะได้ไม่สะสมความเพี้ยน
    if (wallFront()) alignFront();

    // 6. หมุนตัว
    if (move != 'F') {
        if      (move == 'L') turnLeft();
        else if (move == 'R') turnRight();
        else                  turnAround();
        robotDir = dirAfterTurn(robotDir, move);
        settleSensors();
    }

    // 7. หลังหันเสร็จ ถามเซ็นเซอร์หน้าเลยว่ามีกำแพงไหม -- หุ่นตอนนี้นิ่งและตั้งฉากจากการเลี้ยว
    //    เชื่อค่าได้ เป็นวิธีบันทึกกำแพงหน้าที่แม่นสุด ไม่ต้องรอไปชนแล้วค่อยเดา
    distUpdate();
    if (wallFront()) {
        mazeSetWalls(robotX, robotY, robotDir, false, false, true);
        logPrintf("[wall] (%d,%d) dir=%d เจอกำแพงหน้า บันทึกแล้ว ไม่ขับ\n", robotX, robotY, robotDir);
        return;   // flood รอบหน้าจะหาทางใหม่เอง
    }

    // 8. เช็คแผนที่ก่อนขับ -- ตอนทางตัน mazeNextMove() คืน 'B' โดยไม่ได้บอกว่าหลังหันแล้วไปต่อได้
    if (!mazeCanGo(robotX, robotY, robotDir)) {
        logPrintf("[WARN] (%d,%d) dir=%d แผนที่บอกมีกำแพงข้างหน้า ไม่ขับ\n", robotX, robotY, robotDir);
        return;
    }

    // 9. เดินหน้า 1 ช่อง
    if (!driveCell()) {
        // เซ็นเซอร์หน้าเพิ่งบอกว่าโล่ง (ข้อ 7) แต่มาเบรกฉุกเฉินกลางทาง = มีกำแพงจริงที่ปลายช่อง
        // บันทึกได้เลย ไม่ต้องเดาด้วย ticks อีก (เหตุที่ log เดิมทิ้งกำแพงจริงไปเพราะเดาผิด)
        mazeSetWalls(robotX, robotY, robotDir, false, false, true);
        logPrintf("[wall] (%d,%d) dir=%d เบรกฉุกเฉินกลางทาง บันทึกกำแพงหน้า\n",
                  robotX, robotY, robotDir);
        settleSensors();
        return;   // ไม่ขยับพิกัด
    }

    // 10. อัปเดตพิกัด
    if (!stepForward(robotDir, &robotX, &robotY)) {
        logPrintf("[WARN] เดินหน้าแล้วจะหลุดขอบกระดาน -- พิกัดคงเดิม แผนที่น่าจะเพี้ยน\n");
    }
}
