#pragma once

#include <helpers/SensorManager.h>

#ifdef NRF52_PLATFORM
#include <bluefruit.h>

#define BM6_SERVICE_UUID        0xFFF0
#define BM6_WRITE_CHAR_UUID     0xFFF3
#define BM6_NOTIFY_CHAR_UUID    0xFFF4

#define BM6_READ_INTERVAL_DEFAULT  60000
#define BM6_SCAN_TIMEOUT           10     // seconds
#define BM6_CONNECT_TIMEOUT        5000
#define BM6_RESPONSE_TIMEOUT       3000
#define BM6_MAX_MAC_LEN            18

enum BM6State {
  BM6_IDLE,
  BM6_SCANNING,
  BM6_CONNECTING,
  BM6_DISCOVERING,
  BM6_READING
};

class BM6SensorManager : public SensorManager {
public:
  BM6SensorManager();
  bool begin() override;
  bool querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) override;
  void loop() override;

  int getNumSettings() const override;
  const char* getSettingName(int i) const override;
  const char* getSettingValue(int i) const override;
  bool setSettingValue(const char* name, const char* value) override;

  // Flags set by callbacks — must be public for static callback access
  volatile bool     _scanFoundDevice;
  volatile bool     _connectDone;
  volatile bool     _disconnected;
  volatile bool     _gotNotify;
  volatile bool     _gotSecondNotify;
  volatile bool     _connected;
  volatile uint16_t _connHandle;
  volatile int8_t   _pendingRssi;
  ble_gap_addr_t    _pendingReport;
  uint8_t           _notifyData[16];


  uint8_t  _targetMac[6];
  bool     _hasMac;

private:
  BLEClientService        _service;
  BLEClientCharacteristic _writeChar;
  BLEClientCharacteristic _notifyChar;

  void encrypt(const uint8_t* input, uint8_t* output);
  void decrypt(const uint8_t* input, uint8_t* output);
  bool parseMac(const char* str, uint8_t* mac);
  void macToString(const uint8_t* mac, char* str);

  bool     _enabled;
  char     _macStr[BM6_MAX_MAC_LEN];

  float    _voltage;
  float    _temperature;
  bool     _hasData;

  uint32_t _readInterval;
  unsigned long _lastReadTime;
  unsigned long _stateStartTime;

  BM6State _state;
  int _channel;
};

#endif // NRF52_PLATFORM
