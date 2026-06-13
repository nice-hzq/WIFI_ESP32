#include "tcp_service.h"

TCPDataServer::TCPDataServer(uint16_t port)
  : _server(port)
  , _port(port)
  , _connected(false)
  , _debug(nullptr)
{
  memset(_clientIPStr, 0, sizeof(_clientIPStr));
}

TCPDataServer::~TCPDataServer()
{
  stop();
}

void TCPDataServer::setDebugOutput(Print* output)
{
  _debug = output;
}

void TCPDataServer::begin()
{
  _server.begin();
  if (_debug) {
    _debug->print("[TCP] Server started on port ");
    _debug->println(_port);
  }
}

void TCPDataServer::stop()
{
  if (_connected) {
    _client.stop();
    _connected = false;
  }
  _server.stop();
  if (_debug) {
    _debug->println("[TCP] Server stopped.");
  }
}

void TCPDataServer::handle()
{
  // 有新连接请求且当前未连接
  if (!_connected && _server.hasClient()) {
    _client = _server.accept();
    _connected = true;
    IPAddress ip = _client.remoteIP();
    snprintf(_clientIPStr, sizeof(_clientIPStr), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    if (_debug) {
      _debug->print("[TCP] Client connected: ");
      _debug->println(_clientIPStr);
    }
  }

  // 检测客户端断线
  if (_connected && !_client.connected()) {
    _client.stop();
    _connected = false;
    if (_debug) {
      _debug->println("[TCP] Client disconnected.");
    }
    memset(_clientIPStr, 0, sizeof(_clientIPStr));
  }
}

bool TCPDataServer::sendLine(const char* line)
{
  if (!_connected || !_client.connected()) {
    return false;
  }

  size_t len = strlen(line);
  size_t written = _client.write((const uint8_t*)line, len);

  // 追加换行
  written += _client.write('\n');

  return written == len + 1;
}

bool TCPDataServer::isConnected()
{
  return _connected && _client.connected();
}

const char* TCPDataServer::clientIP() const
{
  return _clientIPStr;
}
