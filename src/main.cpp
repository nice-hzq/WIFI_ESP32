#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_wifi.h"
#include "esp_netif.h"

#include "config.h"
#include "wt901_parser.h"
#include "session_manager.h"
#include "TCP_service/tcp_service.h"

// ==========================
// WiFi AP 配置
// ==========================
IPAddress local_IP(AP_IP);
IPAddress gateway(AP_GATEWAY);
IPAddress subnet(AP_SUBNET);

// ==========================
// UDP 配置
// ==========================
WiFiUDP udp;
uint8_t udpBuffer[UDP_BUFFER_SIZE];

// ==========================
// TCP 服务器
// ==========================
TCPDataServer tcpServer(TCP_PORT);

// ==========================
// 状态变量
// ==========================
uint32_t udpPacketCount     = 0;
uint32_t totalFrameCount    = 0;
uint32_t udpDroppedEstimate = 0;  // 预估丢包数（基于 parsePacket 返回大小与实际读取的差异）
uint32_t lastStatusPrintTime = 0;

// Per-sensor frame counters for drop detection
uint32_t sensorFrameCount[MAX_SENSORS] = {0};

// ==========================
// CSV 输出一行
// ==========================
void outputCSVRow(const WT901Data& data)
{
  // 格式化时间戳: "20YY-MM-DD HH:MM:SS.mmm"
  char timestamp[48];
  snprintf(timestamp, sizeof(timestamp),
           "20%02d-%02d-%02d %02d:%02d:%02d.%03d",
           (int)data.year, (int)data.month, (int)data.day,
           (int)data.hour, (int)data.minute, (int)data.second, (int)data.millisecond);

  // 格式化 CSV 行（14 列）
  // device_id,timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,angle_x,angle_y,angle_z,mag_x,mag_y,mag_z
  char csvLine[256];
  snprintf(csvLine, sizeof(csvLine),
           "%s,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
           data.deviceId, timestamp,
           data.accX, data.accY, data.accZ,
           data.gyroX, data.gyroY, data.gyroZ,
           data.angleX, data.angleY, data.angleZ,
           data.magX, data.magY, data.magZ);

  // TCP 发送（主通道）
  if (tcpServer.isConnected()) {
    tcpServer.sendLine(csvLine);
  }

  // Serial 发送（带 [DATA] 前缀标记，方便上位机过滤 CSV 行）
  Serial.print("[DATA]");
  Serial.println(csvLine);
}

// ==========================
// 打印连接站信息
// ==========================
void printConnectedStations()
{
  wifi_sta_list_t wifi_sta_list;
  esp_netif_sta_list_t netif_sta_list;

  esp_wifi_ap_get_sta_list(&wifi_sta_list);
  esp_netif_get_sta_list(&wifi_sta_list, &netif_sta_list);

  Serial.println("Connected station detail:");

  for (int i = 0; i < netif_sta_list.num; i++) {
    esp_netif_sta_info_t station = netif_sta_list.sta[i];

    Serial.print("STA ");
    Serial.print(i + 1);
    Serial.print(" MAC: ");

    for (int j = 0; j < 6; j++) {
      if (station.mac[j] < 0x10) Serial.print("0");
      Serial.print(station.mac[j], HEX);
      if (j < 5) Serial.print(":");
    }

    Serial.print(" IP: ");
    Serial.println(IPAddress(station.ip.addr));
  }
}

// ==========================
// 启动 ESP32 AP
// ==========================
void startSoftAP()
{
  Serial.println();
  Serial.println("========== ESP32 WT901WIFI Gateway ==========");

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  delay(200);

  bool configOK = WiFi.softAPConfig(local_IP, gateway, subnet);
  if (configOK) {
    Serial.println("[AP] Static IP configured.");
  } else {
    Serial.println("[AP] Static IP config failed.");
  }

  bool apOK = WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, AP_MAX_CLIENTS);

  if (apOK) {
    Serial.println("[AP] Started successfully.");
    Serial.print("[AP] SSID: ");
    Serial.println(AP_SSID);
    Serial.print("[AP] Password: ");
    Serial.println(AP_PASSWORD);
    Serial.print("[AP] IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("[AP] Start failed.");
  }
}

// ==========================
// 启动 UDP 服务器
// ==========================
void startUdpServer()
{
  if (udp.begin(UDP_PORT)) {
    Serial.print("[UDP] Listening on port ");
    Serial.println(UDP_PORT);
  } else {
    Serial.println("[UDP] Start failed.");
  }
}

// ==========================
// 接收并分发 UDP 数据（批量排空 + ESP32 统一时间戳）
// ==========================
void handleUdp()
{
  // 批量排空所有积压的 UDP 包 —— 50Hz 双传感器不会积压
  while (true) {
    int packetSize = udp.parsePacket();
    if (packetSize <= 0) break;  // 无更多包

    IPAddress remoteIp  = udp.remoteIP();

    int len = udp.read(udpBuffer, sizeof(udpBuffer));
    if (len <= 0) continue;

    udpPacketCount++;

    // 找到此 IP 对应的 session
    SensorSession* s = findOrCreateSession(remoteIp);
    int sessionIdx = s - getSessions();  // 用于 per-sensor 计数

    // 逐字节喂入该传感器对应的帧状态机
    for (int i = 0; i < len; i++) {
      bool frameReady = feedSensorByte(s, udpBuffer[i]);

      if (frameReady) {
        WT901Data parsed;
        if (parseWT901Frame(s->frameBuffer, parsed)) {
          totalFrameCount++;
          if (sessionIdx >= 0 && sessionIdx < getMaxSessions()) {
            sensorFrameCount[sessionIdx]++;
          }

          // ---- ESP32 统一时间戳 ---- //
          // 用 ESP32 millis() 覆盖传感器原始时间，使多传感器共享同一时间基准
          uint32_t t = millis();
          parsed.hour        = (t / 3600000UL) % 24;
          parsed.minute      = (t / 60000UL) % 60;
          parsed.second      = (t / 1000UL) % 60;
          parsed.millisecond  = t % 1000;
          // year/month/day 保留传感器值（ESP32 无 RTC）

          // 首次成功解析时缓存 deviceId
          if (!s->deviceIdKnown) {
            strncpy(s->deviceId, parsed.deviceId, sizeof(s->deviceId) - 1);
            s->deviceIdKnown = true;
            Serial.print("[Session] IP ");
            Serial.print(s->ip);
            Serial.print(" -> Device ID: ");
            Serial.println(s->deviceId);
          }

          // 输出 CSV 行
          outputCSVRow(parsed);
        }
      }
    }
  }
}

// ==========================
// 打印网关状态
// ==========================
void printGatewayStatus()
{
  Serial.println();
  Serial.println("---------- Gateway Status ----------");
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  Serial.print("Connected WiFi clients: ");
  Serial.println(WiFi.softAPgetStationNum());

  Serial.print("Active sensor sessions: ");
  Serial.println(getActiveSessionCount());

  Serial.print("UDP port: ");
  Serial.println(UDP_PORT);

  Serial.print("TCP port: ");
  Serial.println(TCP_PORT);

  Serial.print("TCP client: ");
  if (tcpServer.isConnected()) {
    Serial.println(tcpServer.clientIP());
  } else {
    Serial.println("(none)");
  }

  Serial.print("UDP packets received: ");
  Serial.println(udpPacketCount);

  Serial.print("Frames parsed: ");
  Serial.println(totalFrameCount);

  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());

  printConnectedStations();

  // 列出各 session 详情（含帧数和丢包率）
  SensorSession* sessions = getSessions();
  int maxSessions = getMaxSessions();
  for (int i = 0; i < maxSessions; i++) {
    if (sessions[i].active) {
      Serial.print("  Session[");
      Serial.print(i);
      Serial.print("] IP=");
      Serial.print(sessions[i].ip);
      Serial.print(" Device=");
      if (sessions[i].deviceIdKnown) {
        Serial.print(sessions[i].deviceId);
      } else {
        Serial.print("(unknown)");
      }
      Serial.print(" Frames=");
      Serial.print(sensorFrameCount[i]);
      Serial.print(" (");
      // 与此 session 对应的丢包估算
      int activeCount = getActiveSessionCount();
      if (activeCount > 0) {
        uint32_t expected = totalFrameCount / activeCount;
        if (expected > 0 && sensorFrameCount[i] < expected) {
          int dropPct = 100 - (sensorFrameCount[i] * 100 / expected);
          Serial.print(dropPct);
          Serial.print("% less than avg");
        } else {
          Serial.print("OK");
        }
      }
      Serial.println(")");
    }
  }

  Serial.println("------------------------------------");
}

// ==========================
// 初始化
// ==========================
void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  initSessions();

  startSoftAP();
  startUdpServer();
  tcpServer.setDebugOutput(&Serial);
  tcpServer.begin();
  printGatewayStatus();
}

// ==========================
// 主循环
// ==========================
void loop()
{
  // 接收 UDP（传感器数据）
  handleUdp();

  // TCP 连接管理
  tcpServer.handle();

  // LED 指示
  int clients = WiFi.softAPgetStationNum();
  digitalWrite(LED_PIN, clients > 0 ? HIGH : LOW);

  // 定期打印状态
  uint32_t now = millis();
  if (now - lastStatusPrintTime >= STATUS_INTERVAL_MS) {
    lastStatusPrintTime = now;
    printGatewayStatus();
  }
}
