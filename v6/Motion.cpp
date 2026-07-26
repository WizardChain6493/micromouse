#include "Motion.h"
#include "Sensors.h"
#include "Log.h"

#include <math.h>

// =========================================================
// Low level
// =========================================================
void motionBegin() {
    pinMode(M1_IN1, OUTPUT);
    pinMode(M1_IN2, OUTPUT);
    pinMode(M2_IN1, OUTPUT);
    pinMode(M2_IN2, OUTPUT);
    coast();
}

void setSpeed(int left, int right) {
    left  = constrain(left,  -255, 255);
    right = constrain(right, -255, 255);

    if (left >= 0) { analogWrite(M1_IN1, left);  analogWrite(M1_IN2, 0); }
    else           { analogWrite(M1_IN1, 0);     analogWrite(M1_IN2, -left); }

    if (right >= 0) { analogWrite(M2_IN1, 0);      analogWrite(M2_IN2, right); }
    else            { analogWrite(M2_IN1, -right); analogWrite(M2_IN2, 0); }
}

void brake() {
    analogWrite(M1_IN1, 255); analogWrite(M1_IN2, 255);
    analogWrite(M2_IN1, 255); analogWrite(M2_IN2, 255);
}

void coast() { setSpeed(0, 0); }

// =========================================================
// Straight drive
// =========================================================
// สั่งสองล้อโดย "รักษาผลต่าง" (diff) ไว้เสมอ เพราะ diff คือปริมาณที่กำหนดการเลี้ยวจริง
// ส่วนความเร็วรวมยอมให้ขยับได้
//
// นี่คือจุดที่แก้อาการปัดหัวตอนเร่ง/เบรก ซึ่งมีมาตั้งแต่ v5:
//   ตอน ramp ความเร็ว base ลงมาเหลือ MIN_PWM=35 แต่ diff ยัง swing ได้ ±100
//   v5 สั่ง setSpeed(35-65, 35+65) = setSpeed(-30, 100) -> ล้อนึงถอยหลัง อีกล้อพุ่ง หุ่นปัดหัว
//   ตรงนี้ถ้าล้อช้าจะต่ำกว่าเกณฑ์หมุน ก็เลื่อนทั้งคู่ขึ้นแทน diff ยังเท่าเดิม ไม่มีล้อไหนถอยหรือค้าง
static void driveWithDiff(int base, int diff) {
    diff = constrain(diff, -MAX_DIFF, MAX_DIFF);

    int l = base - diff / 2;
    int r = base + diff / 2;

    int lo = min(l, r);
    if (lo < MIN_PWM) { int shift = MIN_PWM - lo; l += shift; r += shift; }

    int hi = max(l, r);
    if (hi > MAX_PWM) { int shift = hi - MAX_PWM; l -= shift; r -= shift; }

    setSpeed(l, r);
}

static bool frontTooClose() {
    return distMM(DIST_FL) < FRONT_EMERGENCY_MM && distMM(DIST_FR) < FRONT_EMERGENCY_MM;
}

bool driveCell() { return driveTicks(CELL_TICKS); }

bool driveTicks(long targetTicks) {
    encReset();
    yawReset();
    distUpdate();

    const long rampDownTicks = min((long)RAMP_DOWN_TICKS, targetTicks / 2);

    unsigned long startMs   = millis();
    unsigned long lastDist  = millis();
    uint32_t      lastCtrl  = micros();
    bool          emergency = false;

    while (encAvg() < targetTicks) {
        uint32_t now = micros();
        if (now - lastCtrl < CTRL_PERIOD_US) continue;   // ตรึงลูปที่ 200Hz
        lastCtrl = now;

#if USE_GYRO_ASSIST
        yawUpdate();      // v5 ลืมเรียกตรงนี้ -> มุมเพี้ยนสะสมทุกช่องที่วิ่ง
#endif

        // อ่านระยะแค่ตามจังหวะที่เซ็นเซอร์มีค่าใหม่จริง
        // ถ้าอ่านทุก tick: 4 ตัว x I2C 100kHz ~= 3.6ms จากงบลูป 5ms -> ลูปคุมรวน
        if (millis() - lastDist >= RANGE_PERIOD_MS) {
            lastDist = millis();
            distUpdate();
            if (frontTooClose()) { emergency = true; break; }
        }

        unsigned long elapsed   = millis() - startMs;
        long          remaining = targetTicks - encAvg();

        int speedUp = DRIVE_SPEED, speedDown = DRIVE_SPEED;
        if (elapsed < RAMP_UP_MS) {
            // ไต่จาก SOFT_START_PWM (ไม่ใช่ MIN_PWM) เพื่อผ่าน stall นุ่ม ๆ ไม่ให้ตัวแรงกว่าพุ่งนำ
            float t = (float)elapsed / RAMP_UP_MS;
            speedUp = SOFT_START_PWM + (int)((DRIVE_SPEED - SOFT_START_PWM) * sinf(t * M_PI / 2.0f));
        }
        if (remaining < rampDownTicks) {
            float t = (float)remaining / rampDownTicks;
            speedDown = MIN_PWM + (int)((DRIVE_SPEED - MIN_PWM) * (1.0f - cosf(t * M_PI)) / 2.0f);
        }
        // ช่วงต้นยอมให้ base ต่ำกว่า MIN_PWM ได้ (soft start) พ้นช่วงนั้นค่อย floor ที่ MIN_PWM
        int lowFloor = (elapsed < SOFT_START_MS) ? SOFT_START_PWM : MIN_PWM;
        int base = max(min(speedUp, speedDown), lowFloor);

        // ล้อไหนเดินนำ (ตามระยะ) ก็ชะลอล้อนั้น -- scale ชดเชย encoder ขวาที่นับต่อระยะไม่เท่าซ้าย
        float diff = KP_ENC * ((float)encLeft() - (float)encRight() * ENC_BALANCE_SCALE);
#if USE_GYRO_ASSIST
        // ล็อกหัวให้ตรง: driveCell() รีเซ็ต yaw=0 ตอนเริ่ม ตรงนี้ดึงกลับหา 0 ตลอด
        // ถ้าเปิดแล้วเบี้ยวหนักกว่าเดิม = เครื่องหมายผิด สลับบรรทัดนี้เป็น: (yaw() - 0.0f)
        diff += KP_GYRO_ASSIST * (0.0f - yaw());
#endif
        // feedforward เต็มตั้งแต่ t=0 -- การพุ่งของมอเตอร์ที่แรงกว่าเกิดตอนออกตัว
        // ต้องชดเชยตั้งแต่วินาทีแรก (ค่าเล็ก ไม่ต้อง ramp เข้า ไม่กระตุก)
        diff += MOTOR_BIAS_DIFF;

        driveWithDiff(base, (int)diff);
    }

    // ถึงระยะแล้ว -- หยุดแบบคุมทิศ ไม่ slam brake ทันที (เว้นกรณีฉุกเฉินที่ต้องหยุดเดี๋ยวนั้น)
    // ค่อย ๆ ลด PWM ลง 0 พร้อมรักษา count สองข้างเท่ากัน กันมอเตอร์ขวาเบรกแรงกว่าแล้วเตะหัวขวา
    if (!emergency) {
        unsigned long t0 = millis();
        uint32_t lastStop = micros();
        while (millis() - t0 < STOP_DECEL_MS) {
            uint32_t now = micros();
            if (now - lastStop < CTRL_PERIOD_US) continue;
            lastStop = now;

            float k    = 1.0f - (float)(millis() - t0) / STOP_DECEL_MS;   // 1 -> 0
            int   base = (int)(MIN_PWM * k);
            int   d    = constrain((int)(KP_ENC * ((float)encLeft() - (float)encRight() * ENC_BALANCE_SCALE)),
                                   -MAX_DIFF, MAX_DIFF);
            // ใช้ setSpeed ตรง ๆ ให้ลงต่ำกว่า MIN_PWM ได้ (driveWithDiff จะ floor ไว้ที่ MIN_PWM)
            setSpeed(base - d / 2, base + d / 2);
        }
    }

    brake();
    delay(80);
    coast();

    if (emergency) logPrintf("[WARN] emergency stop: กำแพงหน้าใกล้เกินไป\n");
    return !emergency;
}

// =========================================================
// Turning
// =========================================================
static void turnAngle(float targetDeg) {
    yawReset();

    const bool  right = (targetDeg > 0);
    const float kp = right ? KP_TURN_R : KP_TURN_L;
    const float ki = right ? KI_TURN_R : KI_TURN_L;

    float integral = 0;
    int   settle   = 0;
    bool  timedOut = false;
    float peakRate = 0;   // ใช้เช็คว่าชนเพดาน range ของ gyro หรือเปล่า

    unsigned long startMs = millis();
    unsigned long prevT   = micros();

    while (true) {
        yawUpdate();
        float rate = yawRate();
        if (fabsf(rate) > peakRate) peakRate = fabsf(rate);

        unsigned long now = micros();
        float dt = (now - prevT) / 1000000.0f;
        prevT = now;
        if (dt <= 0) dt = 0.001f;

        float err = targetDeg - yaw();

        // จบต่อเมื่อเข้ากรอบ "และนิ่งจริง"
        // v5 ออกทันทีที่ error เข้ากรอบครั้งแรก ทั้งที่ยังเหวี่ยงอยู่ -> ไถเลยเป้าทุกครั้ง
        if (fabsf(err) <= TURN_TOLERANCE && fabsf(rate) <= TURN_SETTLE_DPS) {
            brake();
            if (++settle >= TURN_SETTLE_TICKS) break;
        } else {
            settle = 0;

            if (fabsf(err) < 30.0f) integral += err * dt; else integral = 0;   // anti-windup

            // D เทอมคือความเร็วเชิงมุมตรง ๆ: ยิ่งหมุนเร็ว ยิ่งหักออกมาก = เบรกเข้าเป้าเอง
            float output = kp * err + ki * integral - KD_TURN_RATE * rate;
            int   pwm    = constrain((int)output, -TURN_MAX_PWM, TURN_MAX_PWM);

            // ใส่ floor เฉพาะตอนหุ่นค้างจริง ๆ เท่านั้น
            // ถ้าใส่ตลอดเวลา มันจะฝืนไม่ให้ระบบชะลอตัว -> พุ่งข้ามเป้าแล้วดีดกลับไปมา
            if (fabsf(rate) < TURN_STALL_DPS) {
                if (err > 0 && pwm <  TURN_STALL_PWM) pwm =  TURN_STALL_PWM;
                if (err < 0 && pwm > -TURN_STALL_PWM) pwm = -TURN_STALL_PWM;
            }

            setSpeed(-pwm, pwm);
        }

        if (millis() - startMs > TURN_TIMEOUT_MS) { timedOut = true; break; }
        delay(5);
    }

    brake();
    delay(100);
    coast();

    if (timedOut) {
        // หยุดตรงไหนก็ไม่รู้ = มุมเพี้ยนแน่ ๆ ต้องเห็นเสมอ
        logPrintf("[WARN] turn TIMEOUT: target %.0f, yaw %.1f (ค้างที่ error %.1f)\n",
                  targetDeg, yaw(), targetDeg - yaw());
    }
#if DEBUG_TURN
    // peakRate ใกล้เพดาน range เมื่อไหร่ = ค่าตัน มุมที่ integrate ได้จะขาด ต้องขยับ GYRO_FS_SEL ขึ้น
    const int fsLimit[] = {250, 500, 1000, 2000};
    logPrintf("[turn] target %.0f -> yaw %.2f (%lu ms) | peak %.0f dps (เพดาน %d)\n",
              targetDeg, yaw(), millis() - startMs, peakRate, fsLimit[GYRO_FS_SEL]);
#endif
}

void turnLeft()   { turnAngle(-90.0f); }
void turnRight()  { turnAngle(+90.0f); }
void turnAround() { turnAngle(-90.0f); turnAngle(-90.0f); }   // สอง 90 แม่นกว่า 180 ทีเดียว

// =========================================================
// Front alignment
// =========================================================
void alignFront() {
#if !USE_ALIGN_FRONT
    return;   // ดู Config.h -- ปิดไว้จนกว่าจะวัด FRONT_ALIGN_OFFSET_MM ของหุ่นตัวจริง
#else
    distUpdate();
    if (!wallFront()) return;

    coast();
    delay(50);

    unsigned long startMs = millis();
    while (millis() - startMs < ALIGN_TIMEOUT_MS) {
        distUpdate();
        uint8_t fl = distMM(DIST_FL);
        uint8_t fr = distMM(DIST_FR);

        // หลุดกรอบเมื่อไหร่แปลว่าไม่ได้จ่อกำแพงแล้ว เลิกจัด
        if (fl >= WALL_FRONT_MM || fr >= WALL_FRONT_MM || fl == 0 || fr == 0) break;

        // หักค่า offset ของเซ็นเซอร์ออกก่อน ไม่งั้นจะจัดจนเบี้ยวตาม offset
        int err = ((int)fl - (int)fr) - FRONT_ALIGN_OFFSET_MM;
        if (abs(err) <= ALIGN_TOLERANCE) break;

        // FL ไกลกว่า = หัวเบนซ้าย = ต้องหมุนขวา (pwm บวก)
        int pwm = (int)(err * KP_ALIGN);
        if (pwm > 0) pwm = constrain(pwm,  ALIGN_MIN_PWM,  ALIGN_MAX_PWM);
        else         pwm = constrain(pwm, -ALIGN_MAX_PWM, -ALIGN_MIN_PWM);

        setSpeed(-pwm, pwm);
        delay(10);
    }

    brake();
    delay(80);
    coast();
#endif
}
