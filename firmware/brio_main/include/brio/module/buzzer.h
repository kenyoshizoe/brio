#ifndef BRIO_MAIN_MODULE_BUZZER_H_
#define BRIO_MAIN_MODULE_BUZZER_H_

#include <Arduino.h>

namespace brio {
class Buzzer {
 public:
  Buzzer(int pin) : pin_(pin){};
  void PlayStartUp();

 private:
  int pin_;
};
}  // namespace brio

#endif  // BRIO_MAIN_MODULE_BUZZER_H_
