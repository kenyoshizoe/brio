#include "brio/module/stepper_driver.h"

#include "etl/crc.h"

namespace brio {
void StepperDriver::Init() {
  spi_->begin();
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);
}

void StepperDriver::Communicate() {
  // Calc CRC
  etl::crc8_ccitt crc;
  for (int i = 0; i < sizeof(Main2StepperDriver) - 1; i++) {
    crc.add(tx_.bin[i]);
  }
  tx_.bin[sizeof(Main2StepperDriver) - 1] = crc.value();
  crc.reset();
  // Communicate
  digitalWrite(SS, LOW);
  spi_->beginTransaction(SPISettings(kSpiFrequency, MSBFIRST, SPI_MODE0));
  spi_->transferBytes(
      tx_.bin, rx_.bin,
      std::max(sizeof(Main2StepperDriver), sizeof(StepperDriver2Main)));
  delayMicroseconds(50);
  digitalWrite(SS, HIGH);
  spi_->endTransaction();
  // Check CRC
  for (int i = 0; i < sizeof(StepperDriver2Main) - 1; i++) {
    crc.add(rx_.bin[i]);
  }
  if (crc.value() != rx_.bin[sizeof(StepperDriver2Main) - 1]) {
    Serial.println("CRC Error");
    return;
  }
}
}  // namespace brio
