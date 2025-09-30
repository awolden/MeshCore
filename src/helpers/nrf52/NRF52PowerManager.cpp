#include "NRF52PowerManager.h"
#include <Arduino.h>

namespace mesh {

void NRF52PowerManager::checkPower() {
  MESH_DEBUG_PRINTLN("NRF52PowerManager::checkPower() START");

  MESH_DEBUG_PRINTLN("  Reading battery voltage...");
  uint16_t vbat = _board->getBattMilliVolts();
  MESH_DEBUG_PRINTLN("  Battery voltage: %umV", vbat);

  MESH_DEBUG_PRINTLN("  Checking USB status...");
  bool usb_connected = false; //(NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk);
  MESH_DEBUG_PRINTLN("  USB connected: %s", usb_connected ? "yes" : "no");

  MESH_DEBUG_PRINTLN("  Power check: battery=%umV, usb=%s, cutoff=%umV", vbat, usb_connected ? "yes" : "no", _cutoff_voltage);

  // Skip brownout check if reading is invalid (0) or USB connected
  if (vbat == 0) {
    MESH_DEBUG_PRINTLN(" Skipping brownout check: invalid voltage reading");
    return;
  }

  if (usb_connected) {
    MESH_DEBUG_PRINTLN("  Skipping brownout check: USB connected");
    return;
  }

  if (vbat < _cutoff_voltage) {
    MESH_DEBUG_PRINTLN("  Battery below cutoff! Entering brownout mode...");
    enterBrownoutMode();
  }

  MESH_DEBUG_PRINTLN("NRF52PowerManager::checkPower() END");
}

void NRF52PowerManager::enterBrownoutMode() {
  uint16_t vbat = _board->getBattMilliVolts();

  MESH_DEBUG_PRINTLN("Battery critically low (%umV < %umV), entering deep sleep until recharged", vbat, _cutoff_voltage);

  NRF_LPCOMP->ENABLE = LPCOMP_ENABLE_ENABLE_Enabled;
  NRF_LPCOMP->PSEL = _comparator_input;
  NRF_LPCOMP->REFSEL = _wakeup_threshold;
  NRF_LPCOMP->ANADETECT = LPCOMP_ANADETECT_ANADETECT_Up;
  NRF_LPCOMP->TASKS_START = 1;

  delay(100);

  NRF_POWER->SYSTEMOFF = 1;
}

}
