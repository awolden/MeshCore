#pragma once

#include <helpers/PowerManager.h>
#include <nrf_lpcomp.h>

namespace mesh {

class NRF52PowerManager : public PowerManager {
private:
  MainBoard* _board;
  uint16_t _cutoff_voltage;
  nrf_lpcomp_ref_t _wakeup_threshold;
  nrf_lpcomp_input_t _comparator_input;

  void enterBrownoutMode();

protected:
  void checkPower() override;

public:
  NRF52PowerManager(MainBoard* board,
                     uint16_t cutoff_voltage = 3100,
                     nrf_lpcomp_ref_t wakeup_threshold = NRF_LPCOMP_REF_SUPPLY_3_16,
                     nrf_lpcomp_input_t comparator_input = NRF_LPCOMP_INPUT_1,
                     uint32_t check_interval_ms = 15000)  // 5 minutes
    : PowerManager(check_interval_ms),
      _board(board),
      _cutoff_voltage(cutoff_voltage),
      _wakeup_threshold(wakeup_threshold),
      _comparator_input(comparator_input)
  {}
};

}
