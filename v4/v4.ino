#include <Arduino.h>
#include <queue>
#include "API.h" // เรียกใช้ฟังก์ชัน Hardware/Orientation จากไฟล์ที่อัปโหลดมา

// =========================================
// 1. การตั้งค่าเขาวงกตและตัวแปรทั่วไป
// =========================================

#define FORWARD  0
#define RIGHT    1
#define BACKWARD 2
#define LEFT     3

#define MAZE_SIZE 16


// ตัวแปรเก็บทิศทาง
int orient = FORWARD;

// *** แก้ไขจุดเริ่มต้น (Start Point) เป็น x=2 ***
// เพื่อให้มีพื้นที่ทางซ้าย (1,0 และ 0,0) ไม่ให้ array ติดลบ
int current_x = 0;
int current_y = 0;
int previous_x = 0;
int previous_y = 0;

bool started = false;

// Array เก็บกำแพง (Wall Map)
int cellsArray[MAZE_SIZE][MAZE_SIZE] = {0};

// Array เก็บระยะทาง (Flood Map)
int floodArray[MAZE_SIZE][MAZE_SIZE]; 

// =========================================
// 2. ฟังก์ชัน Map Initialization & Logic
// =========================================

// สร้าง Flood Map เริ่มต้นอัตโนมัติ
void initFloodMap(int targetX, int targetY) {
    // 1. เคลียร์ค่าเก่าให้เป็นค่ามากสุด (255)
    for(int i=0; i<MAZE_SIZE; i++) {
        for(int j=0; j<MAZE_SIZE; j++) {
            floodArray[i][j] = 255; 
        }
    }

    // 2. กำหนดจุดเป้าหมาย (Target) ให้เป็น 0
    // เป้าหมายเดิม (4,3) ขยับ x+2 กลายเป็น (6,3)
    floodArray[targetX][targetY] = 0;
    
    // (Optional) กำหนดเป้าหมายให้ครบทั้ง 4 ช่อง (2x2 center)
    // เป้าหมายคือ (6,3), (7,3), (6,4), (7,4)
    if(targetX+1 < MAZE_SIZE) floodArray[targetX+1][targetY] = 0;
    if(targetY+1 < MAZE_SIZE) floodArray[targetX][targetY+1] = 0;
    if(targetX+1 < MAZE_SIZE && targetY+1 < MAZE_SIZE) floodArray[targetX+1][targetY+1] = 0;

    // 3. สร้าง Map ด้วย BFS (Queue) จากเป้าหมายออกมาหาจุดเริ่มต้น
    std::queue<int> q;
    q.push(targetX); q.push(targetY);
    if(targetX+1 < MAZE_SIZE) { q.push(targetX+1); q.push(targetY); } // Push เพื่อนร่วมเป้าหมาย
    
    while(!q.empty()){
        int cx = q.front(); q.pop();
        int cy = q.front(); q.pop();
        int dist = floodArray[cx][cy];
        
        // ลำดับทิศ: N, E, S, W
        int dx[] = {0, 1, 0, -1};
        int dy[] = {1, 0, -1, 0};
        
        for(int i=0; i<4; i++){
            int nx = cx + dx[i];
            int ny = cy + dy[i];
            
            // เช็คขอบกระดาน
            if(nx>=0 && nx<MAZE_SIZE && ny>=0 && ny<MAZE_SIZE){
                // ถ้ายังไม่มีค่า (เป็น 255) ให้ใส่ค่า dist + 1
                if(floodArray[nx][ny] == 255){
                    floodArray[nx][ny] = dist + 1;
                    q.push(nx); q.push(ny);
                }
            }
        }
    }
}

// บันทึกสถานะกำแพงลงใน cellsArray
void updateCells(int x, int y, int orient, bool left, bool right, bool forward) {
    // Bitmasks: North=1, East=2, South=4, West=8
    int wallMask = 0;
    if (forward) {
        if (orient == FORWARD)    wallMask |= 1; // North
        else if (orient == RIGHT) wallMask |= 2; // East
        else if (orient == BACKWARD) wallMask |= 4; // South
        else if (orient == LEFT)     wallMask |= 8; // West
    }
    if (right) {
        if (orient == FORWARD)    wallMask |= 2; // East
        else if (orient == RIGHT) wallMask |= 4; // South
        else if (orient == BACKWARD) wallMask |= 8; // West
        else if (orient == LEFT)     wallMask |= 1; // North
    }
    if (left) {
        if (orient == FORWARD)    wallMask |= 8; // West
        else if (orient == RIGHT) wallMask |= 1; // North
        else if (orient == BACKWARD) wallMask |= 2; // East
        else if (orient == LEFT)     wallMask |= 4; // South
    }

    // บันทึกใส่ช่องปัจจุบัน
    cellsArray[x][y] |= wallMask;

    // อัปเดตกำแพงให้เพื่อนบ้าน (Consistency)
    if (wallMask & 1) { if (y + 1 < MAZE_SIZE) cellsArray[x][y + 1] |= 4; } // N -> S
    if (wallMask & 2) { if (x + 1 < MAZE_SIZE) cellsArray[x + 1][y] |= 8; } // E -> W
    if (wallMask & 4) { if (y - 1 >= 0) cellsArray[x][y - 1] |= 1; } // S -> N
    if (wallMask & 8) { if (x - 1 >= 0) cellsArray[x - 1][y] |= 2; } // W -> E
}

// เช็คว่าเดินจากช่องหนึ่งไปอีกช่องหนึ่งได้ไหม (ติดกำแพงหรือไม่)
bool isAccessible(int current_x, int current_y, int target_x, int target_y) {
    if (abs(current_x - target_x) + abs(current_y - target_y) != 1) return false;

    int walls = cellsArray[current_x][current_y];

    if (target_y > current_y) { return !(walls & 1); } // North
    if (target_x > current_x) { return !(walls & 2); } // East
    if (target_y < current_y) { return !(walls & 4); } // South
    if (target_x < current_x) { return !(walls & 8); } // West
    
    return false;
}

// ดึงพิกัดรอบตัว
void getSurroungings(int current_x, int current_y, int *nX, int *nY, int *eX, int *eY, int *sX, int *sY, int *wX, int *wY) {
    *nX = current_x; *nY = (current_y + 1 >= MAZE_SIZE) ? -1 : current_y + 1;
    *eX = (current_x + 1 >= MAZE_SIZE) ? -1 : current_x + 1; *eY = current_y;
    *sX = current_x; *sY = (current_y - 1 < 0) ? -1 : current_y - 1;
    *wX = (current_x - 1 < 0) ? -1 : current_x - 1; *wY = current_y;
}

// ตรวจสอบความสอดคล้องของค่า Flood
bool isIncrementConsistent(int cx, int cy) {
    if (floodArray[cx][cy] == 0) return true;

    int nX, nY, eX, eY, sX, sY, wX, wY;
    getSurroungings(cx, cy, &nX, &nY, &eX, &eY, &sX, &sY, &wX, &wY);
    int currentValue = floodArray[cx][cy];
    
    if (nY != -1 && isAccessible(cx, cy, nX, nY) && floodArray[nX][nY] == (currentValue - 1)) return true;
    if (eX != -1 && isAccessible(cx, cy, eX, eY) && floodArray[eX][eY] == (currentValue - 1)) return true;
    if (sY != -1 && isAccessible(cx, cy, sX, sY) && floodArray[sX][sY] == (currentValue - 1)) return true;
    if (wX != -1 && isAccessible(cx, cy, wX, wY) && floodArray[wX][wY] == (currentValue - 1)) return true;

    return false;
}

// แก้ไขค่า Flood ในช่องให้ถูกต้อง
void makeCellConsistent(int cx, int cy) {
    if (floodArray[cx][cy] == 0) return;

    int nX, nY, eX, eY, sX, sY, wX, wY;
    getSurroungings(cx, cy, &nX, &nY, &eX, &eY, &sX, &sY, &wX, &wY);
    int minNeighbor = 1000;

    if (nY != -1 && isAccessible(cx, cy, nX, nY)) minNeighbor = min(minNeighbor, floodArray[nX][nY]);
    if (eX != -1 && isAccessible(cx, cy, eX, eY)) minNeighbor = min(minNeighbor, floodArray[eX][eY]);
    if (sY != -1 && isAccessible(cx, cy, sX, sY)) minNeighbor = min(minNeighbor, floodArray[sX][sY]);
    if (wX != -1 && isAccessible(cx, cy, wX, wY)) minNeighbor = min(minNeighbor, floodArray[wX][wY]);

    if (minNeighbor != 1000) {
        floodArray[cx][cy] = minNeighbor + 1;
    }
}

// Flood Fill Algorithm using Queue
void floodFillUsingQueue(int start_x, int start_y, int prev_x, int prev_y) {
    std::queue<int> q;
    
    if (!isIncrementConsistent(start_x, start_y)) {
        q.push(start_x); q.push(start_y);
    }

    while (!q.empty()) {
        int cx = q.front(); q.pop();
        int cy = q.front(); q.pop();

        if (!isIncrementConsistent(cx, cy)) {
            makeCellConsistent(cx, cy);

            int nX, nY, eX, eY, sX, sY, wX, wY;
            getSurroungings(cx, cy, &nX, &nY, &eX, &eY, &sX, &sY, &wX, &wY);

            if (nY != -1 && isAccessible(cx, cy, nX, nY)) { q.push(nX); q.push(nY); }
            if (eX != -1 && isAccessible(cx, cy, eX, eY)) { q.push(eX); q.push(eY); }
            if (sY != -1 && isAccessible(cx, cy, sX, sY)) { q.push(sX); q.push(sY); }
            if (wX != -1 && isAccessible(cx, cy, wX, wY)) { q.push(wX); q.push(wY); }
        }
    }
}

// =========================================
// 4. ฟังก์ชันแสดงผลและตัดสินใจ (ห้ามลบ)
// =========================================

void printWallStatus() {
    // 1. อัปเดตค่าจากเซ็นเซอร์ทุกลำกล้อง
    readAllDistSensors(); 
    
    // 2. เช็คสถานะกำแพง (เรียกใช้จาก API)
    bool F = wallFront();
    bool L = wallLeft();
    bool R = wallRight();
    
    // 3. แสดงผล
    Serial.println("\n========== WALL CHECK ==========");
    
    // แถวหน้า
    if (F) Serial.println("    [ WALL FRONT ]");
    else   Serial.println("    [    OPEN    ]");
    
    // แถวข้าง
    Serial.print(L ? "[WALL L]" : "[ OPEN ]");
    Serial.print("    (o_o)    "); 
    Serial.println(R ? "[WALL R]" : "[ OPEN ]");
    
    // แสดงค่าตัวเลขกำกับ
    Serial.printf("Raw Data -> FL:%d FR:%d | SL:%d SR:%d\n", 
                  distValues[0], distValues[1], distValues[2], distValues[3]);
    
    // สรุปการตัดสินใจ
    Serial.print("Decision: ");
    if (!F)          Serial.println("Go Forward (Priority 1)");
    else if (!L) Serial.println("Turn Left (Priority 2)");
    else if (!R) Serial.println("Turn Right (Priority 3)");
    else         Serial.println("Dead End - Turn Back");
    
    Serial.println("================================");
}

char whereToMove(int cx, int cy, int px, int py, int orient) {
    int minVal = 1000;
    int bestDir = -1;

    // ลำดับ: 0=N, 1=E, 2=S, 3=W (ต้องตรงกับ orient)
    int dx[] = {0, 1, 0, -1};
    int dy[] = {1, 0, -1, 0};

    for (int i = 0; i < 4; i++) {
        int nx = cx + dx[i];
        int ny = cy + dy[i];

        // *** เช็คขอบกระดานก่อนเรียกใช้ array ***
        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
            if (isAccessible(cx, cy, nx, ny)) {
                if (floodArray[nx][ny] < minVal) {
                    minVal = floodArray[nx][ny];
                    bestDir = i;
                }
            }
        }
    }

    if (bestDir == -1) return 'B';

    int diff = bestDir - orient;
    while (diff < 0) diff += 4;
    diff %= 4;

    if (diff == 0) return 'F';
    if (diff == 1) return 'R';
    if (diff == 2) return 'B';
    if (diff == 3) return 'L';
    return 'F';
}

void updateCoordinates(int orient, int *new_x, int *new_y) {
    if (orient == 0)      (*new_y)++; // North
    else if (orient == 1) (*new_x)++; // East
    else if (orient == 2) (*new_y)--; // South
    else if (orient == 3) (*new_x)--; // West
}

void printMyArray() {
  Serial.println("--- Current Grid ---");
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      Serial.print(cellsArray[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// =========================================
// 5. Setup & Loop
// =========================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    setupRobot(); // เรียกจาก API
    
    // *** สร้าง Map เริ่มต้น ***
 
    initFloodMap(8, 8); 

    Serial.println(">>> Robot Mode: Offline (Serial Only) <<<");
    Serial.printf("Start Position: (%d, %d)\n", current_x, current_y);
}

void loop() {
    updateYaw(); // เรียกจาก API
    
    delay(50); 

    // รอการกดปุ่ม Start
    if (!started) {
        if (digitalRead(START_BUTTON) == LOW) { // เช็คขาปุ่มให้ดีว่า HIGH หรือ LOW Active
            started = true;
            delay(1000);
        } else return;
    }
    
    // 1. อ่านค่าและอัปเดตกำแพง
    readAllDistSensors(); // เรียกจาก API
    
    // เรียกฟังก์ชัน wall... จาก API
    bool wall_L = wallLeft();
    bool wall_R = wallRight();
    bool wall_F = wallFront();

    updateCells(current_x, current_y, orient, wall_L, wall_R, wall_F);

    // 2. เช็คว่าถึงเส้นชัย
    if (floodArray[current_x][current_y] == 0) {
        brakeMotors();
        started = false;
        Serial.println(">>> GOAL REACHED! <<<");
        while(1) { updateYaw(); }
    }

    // 3. คำนวณ Flood Fill
    floodFillUsingQueue(current_x, current_y, previous_x, previous_y);

    // 4. เลือกทิศทาง
    char direction = whereToMove(current_x, current_y, previous_x, previous_y, orient);

    // Debug ข้อมูล
    printWallStatus();
    // printMyArray(); 
    Serial.printf("Pos: (%d, %d) -> Dir: %c\n", current_x, current_y, direction);
    Serial.print(orient);

    // 5. สั่งหุ่นยนต์หมุนตัว
    if (direction == 'L') { 
        turnLeft(); // เรียกจาก API
        orient = orientation(orient, 'L'); // เรียกจาก API
    } 
    else if (direction == 'R') { 
        turnRight(); // เรียกจาก API
        orient = orientation(orient, 'R'); // เรียกจาก API
    } 
    else if (direction == 'B') { 
        turnLeft(); // เรียกจาก API
        turnLeft(); 
        orient = orientation(orient, 'B'); // เรียกจาก API
    }

    // 6. เดินหน้า 1 ช่อง
    moveForward(); // เรียกจาก API

    // 7. อัปเดตพิกัด
    previous_x = current_x;
    previous_y = current_y;
    updateCoordinates(orient, &current_x, &current_y);
}

// void loop() {
    
//     updateYaw(); // อัปเดตค่า Gyro ตลอดเวลาเพื่อป้องกันการเลี้ยวเพี้ยน
//     // --- ส่วนสำหรับ Debug ค่า Sensor (วางไว้ใน void loop) ---
//     readAllDistSensors(); // อัปเดตค่าล่าสุดจากเซ็นเซอร์

//     Serial.print("Distances -> ");
//     Serial.printf("FL(0): %4d | ", distValues[0]); // หน้าซ้าย (เอียง 45)
//     Serial.printf("FR(1): %4d | ", distValues[1]); // หน้าขวา (เอียง 45)
//     Serial.printf("SL(2): %4d | ", distValues[2]); // ซ้ายตรง
//     Serial.printf("SR(3): %4d | ", distValues[3]); // ขวาตรง
//     Serial.printf("Yaw: %6.2f", currentYaw);       // มุมปัจจุบันจาก Gyro
//     Serial.println();

//     delay(50); // ใส่หน่วงนิดนึงเพื่อให้เราอ่านค่าใน Serial Monitor ทัน

//     if (!started) {
//         if (digitalRead(START_BUTTON) == LOW) {
//             started = true;
//             delay(1000);
//         } else return;
//     }

//     // 1. ตรวจสอบกำแพง
//     bool wall_L = wallLeft();
//     bool wall_R = wallRight();
//     bool wall_F = wallFront();

//     // 2. อัปเดตแผนที่กำแพง
//     updateCells(current_x, current_y, orient, wall_L, wall_R, wall_F);

//     // 3. เช็คว่าถึงเส้นชัย (เลข 0) หรือยัง
//     if (floodArray[current_x][current_y] == 0) {
//         brakeMotors();
//         started = false;
//         while(1) { updateYaw(); }
//     }

//     // 4. คำนวณ Flood Fill ใหม่ (ถ้าจำเป็น)
//     floodFillUsingQueue(current_x, current_y, previous_x, previous_y);

//     // 5. เลือกทิศทาง
//     char direction = whereToMove(current_x, current_y, previous_x, previous_y, orient);

//     // 6. สั่งหุ่นยนต์หมุนตัว
//     if (direction == 'L') { turnLeft(); orient = orientation(orient, 'L'); } 
//     else if (direction == 'R') { turnRight(); orient = orientation(orient, 'R'); } 
//     else if (direction == 'B') { turnLeft(); turnLeft(); orient = orientation(orient, 'L'); orient = orientation(orient, 'L'); }

//     // 7. เดินหน้า 1 ช่อง
//     moveForward();

//     // 8. อัปเดตพิกัดพิกัดในหน่วยความจำ
//     previous_x = current_x;
//     previous_y = current_y;
//     updateCoordinates(orient, &current_x, &current_y);
// }


// void loop() {
//     updateYaw(); // อัปเดตมุม Gyro เสมอ

//     // เรียกฟังก์ชันเช็คกำแพง
//     printWallStatus();x
    
//     // หน่วงเวลา 1 วินาที เพื่อให้เราอ่านค่าทัน
//     delay(1000); 
    
//     moveForwardSync(100);

// }
//---------------------------------------------------------------------------------------------------------
// void loop() {
//     updateYaw();
//     if(digitalRead(START_BUTTON) == LOW)
//         {
//             delay(1000);
//             turnRight();
//             delay(500);
//             turnLeft();
//             delay(500);
//             turnRight();
//             delay(500);
//             turnLeft();
//             delay(500);
//             turnRight();
//             delay(500);
//             turnLeft();
//             delay(500);
//             turnRight();
//             delay(500);
//             turnLeft();
//             moveForward();
//             moveForward();
//             moveForward();
//             moveForward();
//             moveForward();
//             moveForward();
//             moveForward();
//             moveForward();
//             turnLeft();
//             turnLeft();
//             moveForward();
//             moveForward();
//             moveForward();
//             moveForward();
//             moveForward();
//             moveForward();
//             moveForward();
//             moveForward();
//         }
//     else return;
// }
//---------------------------------------------------------------------------------------------------------------------------------
// void loop()
// {
//     updateYaw();
//     printWallStatus();
//     delay(50);
// }
// void loop()
// {
//     updateYaw();
//     Serial.printf("Yaw: %6.2f", currentYaw);
// }
// void loop(){moveForwardSync(255);}
