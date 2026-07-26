#ifndef WEB_DEBUG_H
#define WEB_DEBUG_H

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebSrv.h> // <-- แก้ไขตรงนี้ให้ตรงกับ Library ที่ติดตั้ง
#include "API.h"

// --- ตั้งค่า Wi-Fi ---
const char* ssid = "Ohh u don't have internet poor u";     // ใส่ชื่อ Wi-Fi
const char* password = "Greg534955";   // ใส่รหัสผ่าน

AsyncWebServer server(80);

// หน้าเว็บ HTML
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <meta charset="UTF-8"><title>Robot Monitor</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: 'Courier New', monospace; background: #1a1a1a; color: #00ff41; padding: 20px; }
    .container { max-width: 600px; margin: auto; border: 1px solid #00ff41; padding: 15px; border-radius: 10px; }
    .data-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 20px; }
    .box { border: 1px solid #333; padding: 10px; background: #222; }
    h2 { text-align: center; color: #fff; text-transform: uppercase; border-bottom: 1px solid #00ff41; }
    .val { font-size: 1.5em; font-weight: bold; color: #fff; }
  </style>
  <script>
    setInterval(function() {
      fetch('/data').then(response => response.json()).then(data => {
        document.getElementById('fl').innerHTML = data.fl;
        document.getElementById('fr').innerHTML = data.fr;
        document.getElementById('sl').innerHTML = data.sl;
        document.getElementById('sr').innerHTML = data.sr;
        document.getElementById('yaw').innerHTML = data.yaw.toFixed(2);
        document.getElementById('pos').innerHTML = "(" + data.x + "," + data.y + ")";
      });
    }, 200); // อัปเดตทุก 200ms
  </script>
</head><body>
  <div class="container">
    <h2>Robot Live Debug</h2>
    <div class="data-grid">
      <div class="box">FL (Front Left)<br><span id="fl" class="val">0</span> mm</div>
      <div class="box">FR (Front R11111ight)<br><span id="fr" class="val">0</span> mm</div>
      <div class="box">SL (Side Left)<br><span id="sl" class="val">0</span> mm</div>
      <div class="box">SR (Side Right)<br><span id="sr" class="val">0</span> mm</div>
      <div class="box">Yaw Angle<br><span id="yaw" class="val">0.00</span> °</div>
      <div class="box">Position (X,Y)<br><span id="pos" class="val">(0,0)</span></div>
    </div>
  </div>
</body></html>)rawliteral";

void initWebDebug() {
    

    WiFi.disconnect(true); // ล้างค่าเก่าก่อน
    delay(100);
    WiFi.mode(WIFI_STA);
    
    // ลดกำลังส่งลงมาที่ 8.5dBm (จากปกติ 20dBm) เพื่อลดการกระชากไฟ 
    WiFi.setTxPower(WIFI_POWER_8_5dBm); 
    
    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi...");
    unsigned long startAttemptTime = millis();

    // ป้องกันหุ่นค้างถ้าหา WiFi ไม่เจอ (Timeout 10 วินาที)
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ WiFi Connected!");
        Serial.print("🌐 IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\n❌ WiFi Failed (Proceeding to Robot Setup)");
    }

    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
        // ส่งเป็น JSON เพื่อให้จัดการง่ายขึ้น
        String json = "{";
        json += "\"fl\":" + String(distValues[0]) + ",";
        json += "\"fr\":" + String(distValues[1]) + ",";
        json += "\"sl\":" + String(distValues[2]) + ",";
        json += "\"sr\":" + String(distValues[3]) + ",";
        json += "\"yaw\":" + String(currentYaw) + ",";
        json += "\"x\":" + String(current_x) + ","; // ต้องแน่ใจว่าตัวแปรนี้เป็น global ใน .ino
        json += "\"y\":" + String(current_y);
        json += "}";
        request->send(200, "application/json", json);
    });

    server.begin();
}

#endif