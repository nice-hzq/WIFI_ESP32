#ifndef CONFIG_H
#define CONFIG_H

// ==========================
// WiFi AP 配置
// ==========================
#define AP_SSID          "ESP32_Gait_Gateway"
#define AP_PASSWORD      "12345678"
#define AP_IP            192, 168, 10, 1
#define AP_GATEWAY       192, 168, 10, 1
#define AP_SUBNET        255, 255, 255, 0
#define AP_MAX_CLIENTS   4
#define AP_CHANNEL       1

// ==========================
// UDP 配置
// ==========================
#define UDP_PORT         1399
#define UDP_BUFFER_SIZE  1024

// ==========================
// TCP 配置
// ==========================
#define TCP_PORT         8888

// ==========================
// 传感器配置
// ==========================
#define MAX_SENSORS      4

// ==========================
// 时序配置
// ==========================
#define STATUS_INTERVAL_MS  3000

// ==========================
// 硬件配置
// ==========================
#define LED_PIN          2
#define SERIAL_BAUD      921600

#endif  // CONFIG_H
