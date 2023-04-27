#include <Arduino.h>

#include "brio/pin_assign.hpp"

void setup() {
  Serial.begin(115200);
  delay(1000);
  tone(brio::pin_assign::kSpeeker, 3000);
  delay(200);
  tone(brio::pin_assign::kSpeeker, 3000);
  delay(200);
  noTone(brio::pin_assign::kSpeeker);
}

void loop() {}
