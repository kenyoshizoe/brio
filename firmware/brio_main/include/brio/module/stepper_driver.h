#ifndef BRIO_MAIN_MODULE_STEPPER_DRIVER_H_
#define BRIO_MAIN_MODULE_STEPPER_DRIVER_H_

#include <Arduino.h>
#include <SPI.h>

#include "brio/proto/stepper_driver.h"

namespace brio {
class StepperDriver {
 public:
  StepperDriver(SPIClass* spi) : spi_(spi) {
    for (int i = 0; i < sizeof(Main2StepperDriver); i++) tx_.bin[0] = 0;
    for (int i = 0; i < sizeof(StepperDriver2Main); i++) rx_.bin[0] = 0;
  }
  void Init();
  void Communicate();
  void Write(const Main2StepperDriver& msg) { tx_ = msg; }
  StepperDriver2Main Read() { return rx_; }

 private:
  const uint32_t kSpiFrequency = 20000;
  SPIClass* spi_;
  Main2StepperDriver tx_;
  StepperDriver2Main rx_;
};
}  // namespace brio
#endif  // BRIO_MAIN_MODULE_STEPPER_DRIVER_H_
