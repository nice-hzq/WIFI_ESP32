#ifndef TCP_SERVICE_H
#define TCP_SERVICE_H

#include <Arduino.h>
#include <WiFi.h>

/// 非阻塞 TCP 服务器 —— 接受一个上位机客户端，向其发送 CSV 数据行
class TCPDataServer {
public:
  /// 构造并指定监听端口，默认 8888
  TCPDataServer(uint16_t port = 8888);

  /// 析构时自动清理资源
  ~TCPDataServer();

  /// 启动 TCP 服务器
  void begin();

  /// 停止 TCP 服务器并断开客户端
  void stop();

  /// 在 loop() 中定期调用：接受新连接 / 检测断线
  void handle();

  /// 向已连接的客户端发送一行文本（自动追加换行）
  /// 未连接时静默丢弃，返回 true 表示成功写入
  bool sendLine(const char* line);

  /// 当前是否有客户端连接
  bool isConnected();

  /// 获取当前连接的客户端 IP 字符串
  const char* clientIP() const;

  /// 设置调试输出目标（默认 nullptr = 静默），可注入 &Serial
  void setDebugOutput(Print* output);

private:
  WiFiServer   _server;
  WiFiClient   _client;
  uint16_t     _port;
  bool         _connected;
  char         _clientIPStr[20];
  Print*       _debug;
};

#endif  // TCP_SERVICE_H
