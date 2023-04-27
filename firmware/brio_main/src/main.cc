#include <Arduino.h>

#include "brio/pin_assign.h"
#include "brio/buzzer.h"

brio::Buzzer buzzer(brio::pin_assign::kBuzzer);

void setup() {
  Serial.begin(115200);
  delay(1000);
  buzzer.PlayStartUp();
}

void loop() {}
