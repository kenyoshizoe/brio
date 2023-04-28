#include "brio/module/buzzer.h"

namespace brio {
void Buzzer::PlayStartUp() {
  tone(pin_, 3000);
  delay(200);
  tone(pin_, 3000);
  delay(200);
  noTone(pin_);
}
}  // namespace brio
