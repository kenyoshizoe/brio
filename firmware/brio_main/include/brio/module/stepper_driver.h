#ifndef BRIO_MAIN_MODULE_STEPPER_DRIVER_H_
#define BRIO_MAIN_MODULE_STEPPER_DRIVER_H_

#include <Arduino.h>
#include <SPI.h>

#include "brio/proto/stepper_driver.h"

namespace brio {
class StepperDriver {
 public:
  StepperDriver(SPIClass* spi) : spi_(spi) {}
  void Init();
  void Communicate();
  void Write(const StepperDriver2Main& msg) { rx_ = msg; }
  Main2StepperDriver Read() { return tx_; }

 private:
  const uint32_t kSpiFrequency = 4000000;
  SPIClass* spi_;
  Main2StepperDriver tx_;
  StepperDriver2Main rx_;
};
}  // namespace brio
#endif  // BRIO_MAIN_MODULE_STEPPER_DRIVER_H_
