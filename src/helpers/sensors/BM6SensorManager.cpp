#include "BM6SensorManager.h"

#ifdef NRF52_PLATFORM

#include <AES.h>

static const uint8_t BM6_AES_KEY[16] = {
  108, 101, 97, 103, 101, 110, 100, 255,
  254, 48, 49, 48, 48, 48, 48, 57
};

static const uint8_t BM6_COMMAND[16] = {
  0xd1, 0x55, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static BM6SensorManager* _instance = nullptr;

// BM6 iBeacon UUID prefix in manufacturer data: 4c000215 + 3ba29cd9a42c894856badaf2606ef777
static const uint8_t BM6_IBEACON_PREFIX[] = {
  0x4c, 0x00, 0x02, 0x15,
  0x3b, 0xa2, 0x9c, 0xd9, 0xa4, 0x2c, 0x89, 0x48,
  0x56, 0xba, 0xda, 0xf2, 0x60, 0x6e, 0xf7, 0x77
};

static bool isBM6Advertisement(ble_gap_evt_adv_report_t* report) {
  uint8_t* data = report->data.p_data;
  uint16_t len = report->data.len;

  // Search for manufacturer data AD type (0xFF) containing iBeacon prefix
  uint8_t pos = 0;
  while (pos < len) {
    uint8_t ad_len = data[pos];
    if (ad_len == 0 || pos + ad_len >= len) break;
    uint8_t ad_type = data[pos + 1];
    if (ad_type == 0xFF && ad_len >= 21) {
      // Check if manufacturer data matches BM6 iBeacon UUID
      if (memcmp(&data[pos + 2], BM6_IBEACON_PREFIX, sizeof(BM6_IBEACON_PREFIX)) == 0) {
        return true;
      }
    }
    pos += ad_len + 1;
  }
  return false;
}

static volatile uint8_t _scanCount = 0;

// Callbacks — minimal work, just set flags. NO Serial, NO connect calls.
static void scan_callback(ble_gap_evt_adv_report_t* report) {
  if (!_instance) return;
  _scanCount++;

  _scanCount++;

  // Check MAC match in callback (safe — only reads volatile/public fields)
  if (_instance->_hasMac) {
    bool match = true;
    for (int i = 0; i < 6; i++) {
      if (report->peer_addr.addr[5 - i] != _instance->_targetMac[i]) {
        match = false;
        break;
      }
    }
    if (!match) {
      Bluefruit.Scanner.resume();
      return;
    }
  } else if (!isBM6Advertisement(report)) {
    Bluefruit.Scanner.resume();
    return;
  }

  memcpy(&_instance->_pendingReport, &report->peer_addr, sizeof(ble_gap_addr_t));
  _instance->_pendingRssi = report->rssi;
  _instance->_scanFoundDevice = true;
  Bluefruit.Scanner.stop();
}

static void connect_callback(uint16_t conn_handle) {
  if (!_instance) return;
  _instance->_connHandle = conn_handle;
  _instance->_connectDone = true;
}

static void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  if (!_instance) return;
  _instance->_connected = false;
  _instance->_disconnected = true;
}

static void notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
  if (!_instance || len != 16) return;

  // BM6 sends the command echo first, then actual data
  // Skip if first byte matches command signature (0xD1 after decrypt = echo)
  // Store in second slot if first is already filled
  if (!_instance->_gotNotify) {
    memcpy(_instance->_notifyData, data, 16);
    _instance->_gotNotify = true;
  } else {
    // Second notification — this is likely the real data
    memcpy(_instance->_notifyData, data, 16);
    _instance->_gotSecondNotify = true;
  }
}

BM6SensorManager::BM6SensorManager()
  : _service(BM6_SERVICE_UUID),
    _writeChar(BM6_WRITE_CHAR_UUID),
    _notifyChar(BM6_NOTIFY_CHAR_UUID),
    _connected(false),
    _hasMac(false),
    _voltage(0),
    _temperature(0),
    _hasData(false),
    _readInterval(BM6_READ_INTERVAL_DEFAULT),
    _lastReadTime(0),
    _channel(TELEM_CHANNEL_SELF + 1),
    _state(BM6_IDLE),
    _scanFoundDevice(false),
    _connectDone(false),
    _disconnected(false),
    _gotNotify(false),
    _gotSecondNotify(false),
    _connHandle(BLE_CONN_HANDLE_INVALID),
    _pendingRssi(0),
    _stateStartTime(0)
{
  _instance = this;
  memset(_targetMac, 0, 6);
  memset(_notifyData, 0, 16);
  memset(&_pendingReport, 0, sizeof(_pendingReport));
  _macStr[0] = 0;

#ifdef BM6_ENABLED_DEFAULT
  _enabled = true;
#else
  _enabled = false;
#endif

#ifdef BM6_TARGET_MAC
  if (parseMac(BM6_TARGET_MAC, _targetMac)) {
    _hasMac = true;
    macToString(_targetMac, _macStr);
  }
#endif
}

bool BM6SensorManager::begin() {
  if (!_enabled) return false;

  _notifyChar.setNotifyCallback(::notify_callback);
  _service.begin();
  _writeChar.begin();
  _notifyChar.begin();

  Bluefruit.Central.setConnectCallback(::connect_callback);
  Bluefruit.Central.setDisconnectCallback(::disconnect_callback);

  MESH_DEBUG_PRINTLN("BM6: initialized, enabled=%d hasMac=%d", _enabled, _hasMac);
  if (_hasMac) {
    MESH_DEBUG_PRINTLN("BM6: target %s", _macStr);
  }

  return true;
}

void BM6SensorManager::loop() {
  if (!_enabled) return;

  unsigned long now = millis();

  switch (_state) {

  case BM6_IDLE:
    if (now - _lastReadTime >= _readInterval) {
      _lastReadTime = now;
      _scanFoundDevice = false;
      _connectDone = false;
      _disconnected = false;
      _gotNotify = false;
      _gotSecondNotify = false;

      Bluefruit.Scanner.setRxCallback(::scan_callback);
      Bluefruit.Scanner.clearFilters();
      Bluefruit.Scanner.useActiveScan(false);
      Bluefruit.Scanner.setIntervalMS(100, 100);  // 100% duty cycle
      Bluefruit.Scanner.start(BM6_SCAN_TIMEOUT * 100);  // parameter is in 10ms units

      _state = BM6_SCANNING;
      _stateStartTime = now;
      MESH_DEBUG_PRINTLN("BM6: scanning...");
    }
    break;

  case BM6_SCANNING:
    if (_scanFoundDevice) {
      _scanFoundDevice = false;

      // Auto-bond if no MAC configured
      if (!_hasMac) {
        for (int i = 0; i < 6; i++) {
          _targetMac[i] = _pendingReport.addr[5 - i];
        }
        _hasMac = true;
        macToString(_targetMac, _macStr);
        MESH_DEBUG_PRINTLN("BM6: bonded %s rssi=%d", _macStr, _pendingRssi);
      }

      MESH_DEBUG_PRINTLN("BM6: found, connecting...");
      _connectDone = false;
      Bluefruit.Central.connect(&_pendingReport);
      _state = BM6_CONNECTING;
      _stateStartTime = now;
    }
    else if (now - _stateStartTime > (BM6_SCAN_TIMEOUT * 1000 + 2000)) {
      MESH_DEBUG_PRINTLN("BM6: scan done, %d devices, no match", _scanCount);
      _scanCount = 0;
      _state = BM6_IDLE;
    }
    break;

  case BM6_CONNECTING:
    if (_connectDone) {
      _connected = true;
      MESH_DEBUG_PRINTLN("BM6: connected, discovering...");
      _state = BM6_DISCOVERING;
      _stateStartTime = now;
    }
    else if (now - _stateStartTime > BM6_CONNECT_TIMEOUT) {
      MESH_DEBUG_PRINTLN("BM6: connect timeout");
      _state = BM6_IDLE;
    }
    break;

  case BM6_DISCOVERING: {
    bool ok = true;

    if (!_service.discover(_connHandle)) {
      MESH_DEBUG_PRINTLN("BM6: service not found");
      ok = false;
    }

    if (ok && (!_writeChar.discover() || !_notifyChar.discover())) {
      MESH_DEBUG_PRINTLN("BM6: chars not found");
      ok = false;
    }

    if (ok && !_notifyChar.enableNotify()) {
      MESH_DEBUG_PRINTLN("BM6: notify failed");
      ok = false;
    }

    if (ok) {
      uint8_t encrypted[16];
      encrypt(BM6_COMMAND, encrypted);
      _writeChar.write(encrypted, 16);
      MESH_DEBUG_PRINTLN("BM6: command sent");
      _gotNotify = false;
      _state = BM6_READING;
      _stateStartTime = now;
    } else {
      if (_connected) Bluefruit.disconnect(_connHandle);
      _connected = false;
      _state = BM6_IDLE;
    }
    break;
  }

  case BM6_READING:
    if (_gotSecondNotify || (_gotNotify && (now - _stateStartTime > 1500))) {
      uint8_t decrypted[16];
      decrypt(_notifyData, decrypted);

      // Try primary parse: bytes 7-8
      uint16_t rawV = (decrypted[7] << 8) | decrypted[8];
      float v = rawV / 100.0f;

      if (v < 8.0f || v > 20.0f) {
        rawV = ((decrypted[1] << 8) | decrypted[2]) >> 4;
        v = rawV / 100.0f;
      }

      if (v >= 8.0f && v <= 20.0f) {
        _voltage = v;

        // Try multiple temp parse positions
        // BM6 protocol: byte 3 = sign (01=neg), bytes 4-5 = temp*100
        // Some firmwares: bytes 9-10 or 10-11
        bool neg = (decrypted[3] == 0x01);
        uint16_t rawT = (decrypted[4] << 8) | decrypted[5];
        float t = rawT / 100.0f;
        if (t < 1.0f || t > 80.0f) {
          // try alternate positions
          rawT = (decrypted[9] << 8) | decrypted[10];
          t = rawT / 100.0f;
          neg = false;
        }
        if (t < 1.0f || t > 80.0f) {
          rawT = (decrypted[10] << 8) | decrypted[11];
          t = rawT / 100.0f;
        }
        if (neg) t = -t;
        _temperature = t;
        _hasData = true;
        MESH_DEBUG_PRINTLN("BM6: V=%.2f T=%.1f", _voltage, _temperature);
      } else {
        MESH_DEBUG_PRINTLN("BM6: parse failed (v=%.2f)", v);
      }

      if (_connected) Bluefruit.disconnect(_connHandle);
      _connected = false;
      _state = BM6_IDLE;
    }
    else if (now - _stateStartTime > BM6_RESPONSE_TIMEOUT) {
      MESH_DEBUG_PRINTLN("BM6: response timeout");
      if (_connected) Bluefruit.disconnect(_connHandle);
      _connected = false;
      _state = BM6_IDLE;
    }
    break;
  }
}

bool BM6SensorManager::querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) {
  if (!_enabled || !_hasData) return false;

  if (requester_permissions & TELEM_PERM_ENVIRONMENT) {
    telemetry.addVoltage(_channel, _voltage);
    telemetry.addTemperature(_channel, _temperature);
  }

  return true;
}

// --- AES ---

void BM6SensorManager::encrypt(const uint8_t* input, uint8_t* output) {
  AES128 aes;
  aes.setKey(BM6_AES_KEY, 16);
  aes.encryptBlock(output, input);
}

void BM6SensorManager::decrypt(const uint8_t* input, uint8_t* output) {
  AES128 aes;
  aes.setKey(BM6_AES_KEY, 16);
  aes.decryptBlock(output, input);
}

// --- Settings ---

int BM6SensorManager::getNumSettings() const {
  return _enabled ? 2 : 1;
}

const char* BM6SensorManager::getSettingName(int i) const {
  if (i == 0) return "bm6";
  if (i == 1 && _enabled) return "bm6_mac";
  return NULL;
}

const char* BM6SensorManager::getSettingValue(int i) const {
  if (i == 0) return _enabled ? "1" : "0";
  if (i == 1 && _enabled) {
    return _hasMac ? _macStr : "auto";
  }
  return NULL;
}

bool BM6SensorManager::setSettingValue(const char* name, const char* value) {
  if (strcmp(name, "bm6") == 0) {
    _enabled = (strcmp(value, "1") == 0 || strcmp(value, "on") == 0);
    if (_enabled && !_service.discovered()) {
      begin();
    }
    return true;
  }
  if (strcmp(name, "bm6_mac") == 0) {
    if (strcmp(value, "auto") == 0 || strcmp(value, "0") == 0) {
      _hasMac = false;
      _macStr[0] = 0;
    } else {
      if (parseMac(value, _targetMac)) {
        _hasMac = true;
        macToString(_targetMac, _macStr);
      } else {
        return false;
      }
    }
    return true;
  }
  return false;
}

// --- Helpers ---

bool BM6SensorManager::parseMac(const char* str, uint8_t* mac) {
  if (strlen(str) != 17) return false;
  int vals[6];
  if (sscanf(str, "%x:%x:%x:%x:%x:%x",
             &vals[0], &vals[1], &vals[2],
             &vals[3], &vals[4], &vals[5]) != 6) {
    return false;
  }
  for (int i = 0; i < 6; i++) mac[i] = (uint8_t)vals[i];
  return true;
}

void BM6SensorManager::macToString(const uint8_t* mac, char* str) {
  sprintf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

#endif // NRF52_PLATFORM
