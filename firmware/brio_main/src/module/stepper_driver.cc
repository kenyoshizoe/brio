#include "brio/module/stepper_driver.h"

#include "etl/crc.h"

namespace brio {
void StepperDriver::Init() {
  spi_->begin();
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);
}

void StepperDriver::Communicate() {
  digitalWrite(SS, LOW);
  spi_->beginTransaction(SPISettings(kSpiFrequency, MSBFIRST, SPI_MODE0));
  spi_->transferBytes(tx_.bin, rx_.bin, 36);
  delayMicroseconds(100);
  digitalWrite(SS, HIGH);
  spi_->endTransaction();
}
}  // namespace brio
