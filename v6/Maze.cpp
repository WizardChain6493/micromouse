#include "Maze.h"
#include "Log.h"

// =========================================================
// State
// =========================================================
// กำแพงเก็บเป็น bitmask ต่อช่อง: N=1, E=2, S=4, W=8
static uint8_t walls[MAZE_SIZE][MAZE_SIZE];
static uint8_t flood[MAZE_SIZE][MAZE_SIZE];

#define WALL_N  0x1
#define WALL_E  0x2
#define WALL_S  0x4
#define WALL_W  0x8
#define UNREACHABLE  255

// dx/dy เรียงตาม enum Dir: N, E, S, W
static const int DX[4] = { 0, +1,  0, -1};
static const int DY[4] = {+1,  0, -1,  0};
static const uint8_t WALL_BIT[4] = {WALL_N, WALL_E, WALL_S, WALL_W};

static inline bool inBounds(int x, int y) {
    return x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE;
}

// =========================================================
// Walls
// =========================================================
// ใส่กำแพงหนึ่งด้านของช่อง (x,y) พร้อมใส่ด้านตรงข้ามให้ช่องข้าง ๆ ด้วย
// ไม่งั้นแผนที่จะขัดกันเอง: ยืนช่องนี้บอกว่ามีกำแพง ยืนช่องโน้นบอกว่าไม่มี
static void addWall(int x, int y, int dir) {
    if (!inBounds(x, y)) return;
    walls[x][y] |= WALL_BIT[dir];

    int nx = x + DX[dir], ny = y + DY[dir];
    if (inBounds(nx, ny)) walls[nx][ny] |= WALL_BIT[(dir + 2) % 4];
}

static bool canGo(int x, int y, int dir) {
    if (!inBounds(x, y)) return false;
    int nx = x + DX[dir], ny = y + DY[dir];
    if (!inBounds(nx, ny)) return false;
    return !(walls[x][y] & WALL_BIT[dir]);
}

bool mazeCanGo(int x, int y, int dir) { return canGo(x, y, dir); }

void mazeSetWalls(int x, int y, int dir, bool left, bool right, bool front) {
    if (front) addWall(x, y, dir);
    if (right) addWall(x, y, (dir + 1) % 4);
    if (left)  addWall(x, y, (dir + 3) % 4);
}

// =========================================================
// Flood fill
// =========================================================
bool mazeAtGoal(int x, int y) {
    return x >= GOAL_X && x < GOAL_X + GOAL_W &&
           y >= GOAL_Y && y < GOAL_Y + GOAL_H;
}

int mazeDistance(int x, int y) {
    return inBounds(x, y) ? flood[x][y] : UNREACHABLE;
}

bool mazeReachable(int x, int y) { return mazeDistance(x, y) != UNREACHABLE; }

// v5 ใช้ incremental flood (isIncrementConsistent / makeCellConsistent) ซึ่งซับซ้อนและพลาดง่าย
// สนาม 5x5 มีแค่ 25 ช่อง BFS ใหม่ทั้งกระดานใช้เวลาไม่ถึงหลักไมโครวินาทีบน ESP32
// -> flood ใหม่ทุกรอบไปเลย ได้ค่าที่ถูกต้องเสมอ ไม่ต้องลุ้น
void mazeFlood() {
    for (int x = 0; x < MAZE_SIZE; x++)
        for (int y = 0; y < MAZE_SIZE; y++)
            flood[x][y] = UNREACHABLE;

    static uint8_t qx[MAZE_SIZE * MAZE_SIZE];
    static uint8_t qy[MAZE_SIZE * MAZE_SIZE];
    int head = 0, tail = 0;

    for (int x = GOAL_X; x < GOAL_X + GOAL_W && x < MAZE_SIZE; x++) {
        for (int y = GOAL_Y; y < GOAL_Y + GOAL_H && y < MAZE_SIZE; y++) {
            flood[x][y] = 0;
            qx[tail] = x; qy[tail] = y; tail++;
        }
    }

    while (head < tail) {
        int cx = qx[head], cy = qy[head];
        head++;

        for (int dir = 0; dir < 4; dir++) {
            if (!canGo(cx, cy, dir)) continue;
            int nx = cx + DX[dir], ny = cy + DY[dir];
            if (flood[nx][ny] != UNREACHABLE) continue;

            flood[nx][ny] = flood[cx][cy] + 1;
            qx[tail] = nx; qy[tail] = ny; tail++;
        }
    }
}

void mazeInit() {
    for (int x = 0; x < MAZE_SIZE; x++)
        for (int y = 0; y < MAZE_SIZE; y++)
            walls[x][y] = 0;

    // กำแพงรอบสนามรู้อยู่แล้วตั้งแต่ต้น ไม่ต้องรอเซ็นเซอร์ไปเจอ
    for (int i = 0; i < MAZE_SIZE; i++) {
        walls[i][MAZE_SIZE - 1] |= WALL_N;
        walls[MAZE_SIZE - 1][i] |= WALL_E;
        walls[i][0]             |= WALL_S;
        walls[0][i]             |= WALL_W;
    }

    mazeFlood();
}

// =========================================================
// Decision
// =========================================================
// จำนวนการเลี้ยวที่ต้องทำเพื่อหันจาก dir ไป target -- ใช้เป็นตัวตัดสินตอนค่า flood เท่ากัน
static int turnCost(int dir, int target) {
    int diff = (target - dir + 4) % 4;
    return (diff == 0) ? 0 : (diff == 2 ? 2 : 1);   // ตรง=0, เลี้ยว=1, กลับหลัง=2
}

char mazeNextMove(int x, int y, int dir) {
    int bestDir  = -1;
    int bestDist = UNREACHABLE;
    int bestCost = 99;

    for (int d = 0; d < 4; d++) {
        if (!canGo(x, y, d)) continue;

        int nx = x + DX[d], ny = y + DY[d];
        int dist = flood[nx][ny];
        int cost = turnCost(dir, d);

        // v5 หยิบทิศแรกที่เจอเสมอ (N ก่อน) ทำให้เลี้ยวทั้งที่เดินตรงก็ได้ระยะเท่ากัน
        if (dist < bestDist || (dist == bestDist && cost < bestCost)) {
            bestDist = dist;
            bestCost = cost;
            bestDir  = d;
        }
    }

    if (bestDir < 0) return 'B';   // ทางตัน

    switch ((bestDir - dir + 4) % 4) {
        case 0:  return 'F';
        case 1:  return 'R';
        case 2:  return 'B';
        default: return 'L';
    }
}

int dirAfterTurn(int dir, char move) {
    switch (move) {
        case 'R': return (dir + 1) % 4;
        case 'B': return (dir + 2) % 4;
        case 'L': return (dir + 3) % 4;
        default:  return dir;
    }
}

// ปฏิเสธถ้าจะหลุดขอบ -- ไม่งั้นพิกัดหลุดไปเป็น (1,5) แล้ว canGo() ไปอ่าน walls[1][5] นอกอาร์เรย์
bool stepForward(int dir, int *x, int *y) {
    int nx = *x + DX[dir], ny = *y + DY[dir];
    if (!inBounds(nx, ny)) return false;
    *x = nx;
    *y = ny;
    return true;
}

// =========================================================
// Debug
// =========================================================
// พิมพ์ให้ y เพิ่มขึ้นด้านบน (เหมือนที่มองสนามจริง) -- v5 พิมพ์ [x][y] ตรง ๆ เลยอ่านแล้วงง
// ประกอบทีละบรรทัดใน buffer แล้วค่อยส่งครั้งเดียว จะได้ไม่ยิง log ทีละตัวอักษร
void mazePrint(int robotX, int robotY) {
    char line[4 * MAZE_SIZE + 8];

    // เส้นกำแพงแนวนอนเหนือแถว y  (ถ้า y < 0 คือขอบล่างสุดของกระดาน)
    auto hLine = [&](int y) {
        int n = snprintf(line, sizeof(line), "+");
        for (int x = 0; x < MAZE_SIZE; x++) {
            bool wall = (y >= 0) ? (walls[x][y] & WALL_N) : (walls[x][0] & WALL_S);
            n += snprintf(line + n, sizeof(line) - n, "%s", wall ? "---+" : "   +");
        }
        logPrintf("%s\n", line);
    };

    logPrintf("\n--- Maze (flood distance) ---\n");
    for (int y = MAZE_SIZE - 1; y >= 0; y--) {
        hLine(y);

        int n = 0;
        for (int x = 0; x < MAZE_SIZE; x++) {
            n += snprintf(line + n, sizeof(line) - n, "%s", (walls[x][y] & WALL_W) ? "|" : " ");
            if (x == robotX && y == robotY)      n += snprintf(line + n, sizeof(line) - n, " R ");
            else if (flood[x][y] == UNREACHABLE) n += snprintf(line + n, sizeof(line) - n, " . ");
            else                                 n += snprintf(line + n, sizeof(line) - n, "%3d", flood[x][y]);
        }
        snprintf(line + n, sizeof(line) - n, "%s", (walls[MAZE_SIZE - 1][y] & WALL_E) ? "|" : " ");
        logPrintf("%s\n", line);
    }
    hLine(-1);
    logPrintf("\n");
}
