#pragma once

#include <MeshCore.h>
#include <Arduino.h>

namespace mesh {

class PowerManager {
private:
  unsigned long _last_check;
  uint32_t _check_interval_ms;

protected:
  virtual void checkPower() {}

public:
  PowerManager(uint32_t check_interval_ms = 300000)  // 5 minutes default
    : _check_interval_ms(check_interval_ms), _last_check(0)
  {}

  virtual ~PowerManager() {}

  void loop() {
    unsigned long now = millis();
    if (now - _last_check >= _check_interval_ms) {
      _last_check = now;
      checkPower();
      MESH_DEBUG_PRINTLN("PowerManager::loop() check complete");
    }
  }
};

}
