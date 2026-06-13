#include "wt901_parser.h"

// ==========================
// 缩放因子常量
// ==========================
static constexpr float ACC_SCALE   = 16.0f / 32768.0f;
static constexpr float GYRO_SCALE  = 2000.0f / 32768.0f;
static constexpr float ANGLE_SCALE = 180.0f / 32768.0f;
static constexpr float MAG_SCALE   = 100.0f / 1024.0f;

// ==========================
// int16 有符号转换
// 对应 Python getSignInt16
// ==========================
int16_t getSignInt16(uint16_t num)
{
  if (num >= 32768) {
    return (int16_t)(num - 65536);
  }
  return (int16_t)num;
}

// ==========================
// 小端字节转 int16
// Python 写法：data[21] << 8 | data[20]
// ==========================
int16_t readInt16LE(const uint8_t* data, int lowIndex)
{
  uint16_t raw = ((uint16_t)data[lowIndex + 1] << 8) | data[lowIndex];
  return getSignInt16(raw);
}

uint16_t readUInt16LE(const uint8_t* data, int lowIndex)
{
  return ((uint16_t)data[lowIndex + 1] << 8) | data[lowIndex];
}

// ==========================
// 电量百分比计算（查找表）
// 对应 Python DeviceModel
// ==========================
int getElectricPercentage(uint16_t quantity)
{
  static const struct {
    uint16_t threshold;
    int      percent;
  } BATTERY_MAP[] = {
    {396, 100}, {393, 90}, {387, 75}, {382, 60}, {379, 50},
    {377,  40}, {373, 30}, {370, 20}, {368, 15}, {350, 10},
    {340,   5}, {  0,  0}
  };

  for (const auto& entry : BATTERY_MAP) {
    if (quantity > entry.threshold) {
      return entry.percent;
    }
  }
  return 0;
}

// ==========================
// 打印 HEX，调试用
// ==========================
void printHex(const uint8_t* data, int len, Print& output)
{
  for (int i = 0; i < len; i++) {
    if (data[i] < 0x10) output.print("0");
    output.print(data[i], HEX);
    output.print(" ");
  }
  output.println();
}

// ==========================
// 解析一帧 WT901WIFI 数据
// 对应 Python DeviceModel.onDataReceived()
// ==========================
bool parseWT901Frame(const uint8_t* data, WT901Data& out)
{
  // 校验设备 ID 是否 WT 开头
  if (data[0] != WT901_HEADER_0 || data[1] != WT901_HEADER_1) {
    return false;
  }

  // 设备 ID（前 12 字节）
  memcpy(out.deviceId, data, 12);
  out.deviceId[12] = '\0';

  // 时间戳各字段
  out.year        = data[12];
  out.month       = data[13];
  out.day         = data[14];
  out.hour        = data[15];
  out.minute      = data[16];
  out.second      = data[17];
  out.millisecond = readUInt16LE(data, 18);

  // 加速度，单位 g（量程 ±16g）
  out.accX = readInt16LE(data, 20) * ACC_SCALE;
  out.accY = readInt16LE(data, 22) * ACC_SCALE;
  out.accZ = readInt16LE(data, 24) * ACC_SCALE;

  // 角速度，单位 °/s（量程 ±2000°/s）
  out.gyroX = readInt16LE(data, 26) * GYRO_SCALE;
  out.gyroY = readInt16LE(data, 28) * GYRO_SCALE;
  out.gyroZ = readInt16LE(data, 30) * GYRO_SCALE;

  // 磁场
  out.magX = readInt16LE(data, 32) * MAG_SCALE;
  out.magY = readInt16LE(data, 34) * MAG_SCALE;
  out.magZ = readInt16LE(data, 36) * MAG_SCALE;

  // 角度，单位 °（±180°）
  out.angleX = readInt16LE(data, 38) * ANGLE_SCALE;
  out.angleY = readInt16LE(data, 40) * ANGLE_SCALE;
  out.angleZ = readInt16LE(data, 42) * ANGLE_SCALE;

  // 温度
  out.temperature = readInt16LE(data, 44) / 100.0f;

  // 电量
  out.batteryRaw     = readUInt16LE(data, 46);
  out.batteryPercent = getElectricPercentage(out.batteryRaw);

  // RSSI 信号强度
  out.rssi = readInt16LE(data, 48);

  // 固件版本
  out.version = readInt16LE(data, 50);

  return true;
}
