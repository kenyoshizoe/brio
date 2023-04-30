#include <Arduino.h>
#include <SPI.h>

#include "brio/module/buzzer.h"
#include "brio/module/stepper_driver.h"
#include "brio/pin_assign.h"

brio::Buzzer buzzer(brio::pin_assign::kBuzzer);
brio::StepperDriver stepper_driver(new SPIClass(VSPI));

hw_timer_t* stepper_driver_timer = NULL;
void IRAM_ATTR Communicate() { stepper_driver.Communicate(); }

void setup() {
  Serial.begin(115200);
  Serial.println("Brio Main Board Start Up");
  delay(1000);
  stepper_driver.Init();
  buzzer.PlayStartUp();

  stepper_driver_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(stepper_driver_timer, &Communicate, true);
  timerAlarmWrite(stepper_driver_timer, 1000000, true);
  timerAlarmEnable(stepper_driver_timer);
}

void loop() {}
