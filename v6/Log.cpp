#include "Log.h"

#include <stdarg.h>

#if LOG_WIFI
#include <WiFi.h>

static WiFiServer server(LOG_PORT);
static WiFiClient client;

// ring buffer: ถ้าเต็มให้ทิ้งของเก่าทิ้ง ดีกว่าไปหน่วงลูปคุมมอเตอร์
static char   ring[LOG_BUF_SIZE];
static size_t head = 0;   // ตำแหน่งเขียนถัดไป
static size_t tail = 0;   // ตำแหน่งอ่านถัดไป
static size_t count = 0;

static void ringPush(const char *s, size_t n) {
    for (size_t i = 0; i < n; i++) {
        ring[head] = s[i];
        head = (head + 1) % LOG_BUF_SIZE;
        if (count < LOG_BUF_SIZE) count++;
        else tail = (tail + 1) % LOG_BUF_SIZE;   // เต็มแล้ว ทับของเก่า
    }
}
#endif

#if LOG_WIFI
static bool netUp = false;
#endif

void logBegin() {
    Serial.begin(115200);
    delay(1000);
}

void logNetBegin() {
#if LOG_WIFI
    WiFi.mode(WIFI_AP);
    WiFi.softAP(LOG_SSID, LOG_PASS);
    server.begin();
    server.setNoDelay(true);   // อย่าหน่วงรอรวมแพ็กเก็ต อยากเห็น log ทันที
    netUp = true;

    logPrintf("\n[log] Wi-Fi AP \"%s\"  ->  telnet %s %d\n",
              LOG_SSID, WiFi.softAPIP().toString().c_str(), LOG_PORT);
#endif
}

void logPoll() {
#if LOG_WIFI
    if (!netUp) return;
    // รับ client ใหม่ (ถ้ามีตัวเก่าค้างอยู่ ให้ตัวใหม่แทนที่)
    // ใช้ available() ไม่ใช่ accept() เพราะ accept() มีเฉพาะ esp32 core 3.x
    if (server.hasClient()) {
        if (client && client.connected()) client.stop();
        client = server.available();
        client.setNoDelay(true);
    }

    if (!client || !client.connected()) return;

    // ปล่อย buffer ออกไป -- ส่งเป็นก้อนต่อเนื่องทีละก้อน ไม่ต้องวน byte
    while (count > 0) {
        size_t chunk = count;
        if (chunk > LOG_BUF_SIZE - tail) chunk = LOG_BUF_SIZE - tail;   // ถึงขอบ array แล้วค่อยวนกลับ

        size_t sent = client.write((const uint8_t *)(ring + tail), chunk);
        if (sent == 0) break;                                           // เน็ตยังไม่พร้อม ไว้รอบหน้า

        tail  = (tail + sent) % LOG_BUF_SIZE;
        count -= sent;
    }
#endif
}

void logPrintf(const char *fmt, ...) {
    char buf[256];

    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (n <= 0) return;
    if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;   // ถูกตัด ก็เอาเท่าที่มี

    Serial.write(buf, n);
#if LOG_WIFI
    ringPush(buf, n);
#endif
}

bool logClientConnected() {
#if LOG_WIFI
    return client && client.connected();
#else
    return false;
#endif
}
