#ifndef WT901_PARSER_H
#define WT901_PARSER_H

#include <Arduino.h>

// WT901WIFI 一帧完整解析数据
struct WT901Data {
  char deviceId[13];       // 设备 ID，以 null 结尾
  uint16_t year;
  uint16_t month;
  uint16_t day;
  uint16_t hour;
  uint16_t minute;
  uint16_t second;
  uint16_t millisecond;
  float accX;              // 加速度 X (g)
  float accY;              // 加速度 Y (g)
  float accZ;              // 加速度 Z (g)
  float gyroX;             // 角速度 X (°/s)
  float gyroY;             // 角速度 Y (°/s)
  float gyroZ;             // 角速度 Z (°/s)
  float magX;              // 磁场 X
  float magY;              // 磁场 Y
  float magZ;              // 磁场 Z
  float angleX;            // 角度 X (°)
  float angleY;            // 角度 Y (°)
  float angleZ;            // 角度 Z (°)
  float temperature;       // 温度 (°C)
  uint16_t batteryRaw;     // 电量原始值
  int batteryPercent;      // 电量百分比
  int16_t rssi;            // 信号强度
  int16_t version;         // 固件版本
};

// 帧总长度（含 12 字节设备 ID + 42 字节数据）
#define WT901_FRAME_LEN 54

// 帧头校验
#define WT901_HEADER_0 0x57  // 'W'
#define WT901_HEADER_1 0x54  // 'T'

/// 将 uint16_t 转为有符号 int16_t（补码转换）
int16_t getSignInt16(uint16_t num);

/// 小端字节序读取有符号 int16_t（低字节在 lowIndex，高字节在 lowIndex+1）
int16_t readInt16LE(const uint8_t* data, int lowIndex);

/// 小端字节序读取无符号 uint16_t（低字节在 lowIndex，高字节在 lowIndex+1）
uint16_t readUInt16LE(const uint8_t* data, int lowIndex);

/// 根据原始电量值计算电量百分比
int getElectricPercentage(uint16_t quantity);

/// 解析 54 字节 WT901WIFI 帧，填充 WT901Data 结构体
/// 返回 true 表示解析成功（帧头校验通过），false 表示无效帧
bool parseWT901Frame(const uint8_t* data, WT901Data& out);

/// 调试用：以十六进制打印字节数组到指定输出（默认 Serial）
void printHex(const uint8_t* data, int len, Print& output = Serial);

#endif  // WT901_PARSER_H
