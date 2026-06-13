#ifndef SESSION_MANAGER_H
#define SESSION_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include "wt901_parser.h"

// ==========================
// 传感器会话 — 按源 IP 管理独立帧组装缓冲区
// ==========================
struct SensorSession {
  IPAddress ip;
  uint8_t  frameBuffer[WT901_FRAME_LEN];
  int      frameIndex;
  uint32_t lastActivityMs;
  char     deviceId[13];
  bool     deviceIdKnown;
  bool     active;
};

// ==========================
// 会话管理 API
// ==========================

/// 初始化所有会话槽位为未激活状态
void initSessions();

/// 查找或创建指定 IP 的会话；槽位满时复用最旧的
SensorSession* findOrCreateSession(const IPAddress& ip);

/// 向会话喂入一个字节，返回 true 表示完整帧已就绪（可解析）
bool feedSensorByte(SensorSession* s, uint8_t b);

/// 返回当前活跃会话数
int getActiveSessionCount();

/// 获取会话数组（用于状态遍历）
SensorSession* getSessions();
int getMaxSessions();

#endif  // SESSION_MANAGER_H
