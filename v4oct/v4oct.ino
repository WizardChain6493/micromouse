#include <Arduino.h>
#include "API.h"

#define MAZE_SIZE 16
#define FORWARD 0
#define RIGHT 1
#define BACKWARD 2
#define LEFT 3

// Global Variables
int orient = FORWARD;
int current_x = 0;
int current_y = 0;
int previous_x = 0;
int previous_y = 0;
bool started = false;

// Maps
uint8_t cellsArray[MAZE_SIZE][MAZE_SIZE] = {0}; // ใช้ uint8_t ประหยัดเมม
uint8_t floodArray[MAZE_SIZE][MAZE_SIZE];       // ใช้ uint8_t (ค่าสูงสุด 255 พอดี)

// Optimized Queue (Circular Buffer) ไม่ต้อง malloc
typedef struct { uint8_t x, y; } Point;
Point qBuf[512]; 
int qHead = 0, qTail = 0;

void qPush(uint8_t x, uint8_t y) {
    qBuf[qHead] = {x, y};
    qHead = (qHead + 1) % 512;
}
Point qPop() {
    Point p = qBuf[qTail];
    qTail = (qTail + 1) % 512;
    return p;
}
bool qEmpty() { return qHead == qTail; }
void qClear() { qHead = qTail = 0; }

// =========================================
// FLOOD FILL LOGIC (Optimized)
// =========================================
void initFloodMap(int targetX, int targetY) {
    // ใช้ memset เร็วกว่า loop
    memset(floodArray, 255, sizeof(floodArray));
    qClear();

    floodArray[targetX][targetY] = 0;
    qPush(targetX, targetY);

    // Center 2x2 Target
    if(targetX+1 < MAZE_SIZE) { floodArray[targetX+1][targetY] = 0; qPush(targetX+1, targetY); }
    if(targetY+1 < MAZE_SIZE) { floodArray[targetX][targetY+1] = 0; qPush(targetX, targetY+1); }
    if(targetX+1 < MAZE_SIZE && targetY+1 < MAZE_SIZE) { 
        floodArray[targetX+1][targetY+1] = 0; qPush(targetX+1, targetY+1); 
    }

    int dx[] = {0, 1, 0, -1};
    int dy[] = {1, 0, -1, 0};

    while(!qEmpty()){
        Point p = qPop();
        uint8_t dist = floodArray[p.x][p.y];

        for(int i=0; i<4; i++){
            int nx = p.x + dx[i];
            int ny = p.y + dy[i];
            
            if(nx>=0 && nx<MAZE_SIZE && ny>=0 && ny<MAZE_SIZE){
                // สำคัญ: ต้องเช็คกำแพงด้วยว่าเดินผ่านได้ไหม ถ้าไม่เช็ค Flood Fill จะทะลุกำแพง
                // แต่ใน initFloodMap ปกติเราต้องการค่าระยะทางแบบ "สมมติว่าไม่มีกำแพง" หรือ "มีเท่าที่รู้"
                // ถ้าเป็น Logic Micromouse ทั่วไป initFloodMap มักจะ Flood ใหม่ทุกช่อง
                
                if(floodArray[nx][ny] == 255){
                    floodArray[nx][ny] = dist + 1;
                    qPush(nx, ny);
                }
            }
        }
    }
}

// Update Walls (Bitwise logic is already good)
void updateCells(int x, int y, int orient, bool left, bool right, bool forward) {
    uint8_t wallMask = 0;
    // Map relative to absolute (Optimized lookup could be used, but switch is fast enough)
    if (forward) {
        if (orient == FORWARD) wallMask |= 1; else if (orient == RIGHT) wallMask |= 2;
        else if (orient == BACKWARD) wallMask |= 4; else wallMask |= 8;
    }
    if (right) {
        if (orient == FORWARD) wallMask |= 2; else if (orient == RIGHT) wallMask |= 4;
        else if (orient == BACKWARD) wallMask |= 8; else wallMask |= 1;
    }
    if (left) {
        if (orient == FORWARD) wallMask |= 8; else if (orient == RIGHT) wallMask |= 1;
        else if (orient == BACKWARD) wallMask |= 2; else wallMask |= 4;
    }

    cellsArray[x][y] |= wallMask;
    // Update Neighbors
    if ((wallMask & 1) && y+1 < MAZE_SIZE) cellsArray[x][y+1] |= 4;
    if ((wallMask & 2) && x+1 < MAZE_SIZE) cellsArray[x+1][y] |= 8;
    if ((wallMask & 4) && y-1 >= 0)        cellsArray[x][y-1] |= 1;
    if ((wallMask & 8) && x-1 >= 0)        cellsArray[x-1][y] |= 2;
}

bool isAccessible(int cx, int cy, int tx, int ty) {
    if (abs(cx - tx) + abs(cy - ty) != 1) return false;
    uint8_t walls = cellsArray[cx][cy];
    if (ty > cy) return !(walls & 1); // N
    if (tx > cx) return !(walls & 2); // E
    if (ty < cy) return !(walls & 4); // S
    if (tx < cx) return !(walls & 8); // W
    return false;
}

// Flood Fill Run (Modified for Queue Buffer)
void floodFillUsingQueue(int start_x, int start_y) {
    // *Simplified*: Flood fill from Target backwards is usually better, 
    // but consistent with your code:
    // ... (Use qPush / qPop instead of std::queue) ...
    // เนื่องจาก Logic Flood Fill เดิมค่อนข้างซับซ้อน ผมแนะนำให้ใช้ Logic "Flood from Target" 
    // ทุกครั้งที่มีการ update กำแพง จะง่ายและเร็วกว่าสำหรับ ESP32
    
    // 1. Reset Flood Map
    memset(floodArray, 255, sizeof(floodArray));
    qClear();
    
    // 2. Add Target(s)
    int tx = 8, ty = 7; // Example target
    floodArray[tx][ty] = 0;
    qPush(tx, ty);
    
    // 3. Process
    int dx[] = {0, 1, 0, -1};
    int dy[] = {1, 0, -1, 0};
    
    while(!qEmpty()){
        Point p = qPop();
        uint8_t dist = floodArray[p.x][p.y];
        
        for(int i=0; i<4; i++){
            int nx = p.x + dx[i];
            int ny = p.y + dy[i];
            
            if(nx>=0 && nx<MAZE_SIZE && ny>=0 && ny<MAZE_SIZE){
                // Check Access AND Check if shorter path found
                if(floodArray[nx][ny] == 255 && isAccessible(p.x, p.y, nx, ny)){
                    floodArray[nx][ny] = dist + 1;
                    qPush(nx, ny);
                }
            }
        }
    }
}

char whereToMove(int cx, int cy, int orient) {
    int minVal = 255;
    int bestDir = -1;
    int dx[] = {0, 1, 0, -1};
    int dy[] = {1, 0, -1, 0};

    // Check neighbors
    for (int i = 0; i < 4; i++) {
        int nx = cx + dx[i];
        int ny = cy + dy[i];
        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
            if (isAccessible(cx, cy, nx, ny)) {
                if (floodArray[nx][ny] < minVal) {
                    minVal = floodArray[nx][ny];
                    bestDir = i;
                }
            }
        }
    }

    if (bestDir == -1) return 'B'; // Dead end
    
    // Calc relative direction
    int diff = bestDir - orient;
    while (diff < 0) diff += 4;
    diff %= 4;
    
    if (diff == 0) return 'F';
    if (diff == 1) return 'R';
    if (diff == 2) return 'B';
    return 'L';
}

void updateCoordinates(int orient, int *x, int *y) {
    if (orient == 0) (*y)++;
    else if (orient == 1) (*x)++;
    else if (orient == 2) (*y)--;
    else if (orient == 3) (*x)--;
}

// =========================================
// MAIN SETUP & LOOP
// =========================================
void setup() {
    Serial.begin(115200);
    setupRobot(); // Starts Core 0 Task
    
    // Init Maze
    initFloodMap(8, 7); // Initial guess assuming no walls
    
    Serial.println(">>> READY (Dual Core Mode) <<<");
}

void loop() {
    // ไม่ต้องเรียก updateYaw() ที่นี่แล้ว Core 0 ทำให้เอง

    if (!started) {
        if (digitalRead(START_BUTTON) == LOW) {
            started = true;
            delay(1000);
        } else {
            // Debug Mode: Print sensor values when idle
            // อ่านค่าจากตัวแปร global ได้เลย ไม่ต้องสั่งอ่าน I2C
            Serial.printf("FL:%d FR:%d L:%d R:%d Yaw:%.1f\n", 
               distValues[0], distValues[1], distValues[2], distValues[3], currentYaw);
            delay(100);
            return;
        }
    }

    // 1. อ่านสถานะกำแพง (เร็วมาก เพราะอ่านจาก RAM)
    bool wL = wallLeft();
    bool wR = wallRight();
    bool wF = wallFront();

    // 2. Update Map
    updateCells(current_x, current_y, orient, wL, wR, wF);

    // 3. Goal Check
    if (floodArray[current_x][current_y] == 0) {
        brakeMotors();
        started = false;
        Serial.println("GOAL!");
        while(1) delay(100);
    }

    // 4. Recalculate Flood Fill (Fast)
    floodFillUsingQueue(current_x, current_y);

    // 5. Decide Move
    char dir = whereToMove(current_x, current_y, orient);

    // 6. Execute Move
    if (dir == 'L') { turnLeft(); orient = orientation(orient, 'L'); }
    else if (dir == 'R') { turnRight(); orient = orientation(orient, 'R'); }
    else if (dir == 'B') { turnLeft(); turnLeft(); orient = orientation(orient, 'B'); }
    
    moveForward();
    updateCoordinates(orient, &current_x, &current_y);
}