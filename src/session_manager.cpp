#include "session_manager.h"
#include "config.h"

// ==========================
// 全局会话数组
// ==========================
static SensorSession sessions[MAX_SENSORS];
static int sessionCount = 0;

// ==========================
// 初始化一个会话槽位
// ==========================
static void initSessionSlot(SensorSession& s, const IPAddress& ip)
{
  s.ip = ip;
  s.frameIndex = 0;
  s.lastActivityMs = millis();
  s.deviceIdKnown = false;
  memset(s.deviceId, 0, sizeof(s.deviceId));
  s.active = true;
}

// ==========================
// 公共 API
// ==========================
void initSessions()
{
  for (int i = 0; i < MAX_SENSORS; i++) {
    sessions[i].active = false;
    sessions[i].frameIndex = 0;
  }
  sessionCount = 0;
}

SensorSession* findOrCreateSession(const IPAddress& ip)
{
  // 先查找已有 session
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (sessions[i].active && sessions[i].ip == ip) {
      return &sessions[i];
    }
  }

  // 查找空闲槽位
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (!sessions[i].active) {
      initSessionSlot(sessions[i], ip);
      sessionCount++;
      Serial.print("[Session] New sensor from IP: ");
      Serial.println(ip);
      return &sessions[i];
    }
  }

  // 已满，复用最旧的 session
  Serial.println("[Session] Slot full, reusing oldest session.");
  int oldest = 0;
  uint32_t oldestTime = sessions[0].lastActivityMs;
  for (int i = 1; i < MAX_SENSORS; i++) {
    if (sessions[i].lastActivityMs < oldestTime) {
      oldestTime = sessions[i].lastActivityMs;
      oldest = i;
    }
  }
  initSessionSlot(sessions[oldest], ip);
  return &sessions[oldest];
}

bool feedSensorByte(SensorSession* s, uint8_t b)
{
  s->lastActivityMs = millis();

  // 第 1 个字节必须是 'W' = 0x57
  if (s->frameIndex == 0) {
    if (b == WT901_HEADER_0) {
      s->frameBuffer[s->frameIndex++] = b;
    }
    return false;
  }

  // 第 2 个字节必须是 'T' = 0x54
  if (s->frameIndex == 1) {
    if (b == WT901_HEADER_1) {
      s->frameBuffer[s->frameIndex++] = b;
    } else {
      // 若又是 'W'，作为新帧开始
      s->frameIndex = 0;
      if (b == WT901_HEADER_0) {
        s->frameBuffer[s->frameIndex++] = b;
      }
    }
    return false;
  }

  // 后续字节直接存入
  s->frameBuffer[s->frameIndex++] = b;

  // 满 54 字节，解析一帧
  if (s->frameIndex >= WT901_FRAME_LEN) {
    s->frameIndex = 0;
    return true;  // 帧就绪
  }

  return false;
}

int getActiveSessionCount()
{
  return sessionCount;
}

SensorSession* getSessions()
{
  return sessions;
}

int getMaxSessions()
{
  return MAX_SENSORS;
}
