#include "brio/module/buzzer.h"

namespace brio {
void Buzzer::PlayStartUp() {
  tone(pin_, 3000);
  delay(200);
  tone(pin_, 3000);
  delay(200);
  noTone(pin_);
}
void Buzzer::Play(int freq) { tone(pin_, freq); }
}  // namespace brio
