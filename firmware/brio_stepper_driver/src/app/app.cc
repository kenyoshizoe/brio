#include "brio/app/app.h"

#include <stdint.h>
#include <stdlib.h>

// C includes
#include "SEGGER_RTT.h"
#include "iwdg.h"
#include "tim.h"
// C++ includes
#include "brio/module/a4988.h"
#include "brio/util/utils.h"
#include "etl/queue.h"

// Stepper motor
brio::A4988* stepper1;
brio::A4988* stepper2;
brio::A4988* stepper3;

void MainTask() {
  // Initialize RTT
  SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0,
                            SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
  // Initialize stepper motor
  stepper1 = new brio::A4988(STEPPER1_STEP_GPIO_Port, STEPPER1_STEP_Pin,
                             STEPPER1_DIR_GPIO_Port, STEPPER1_DIR_Pin,
                             STEPPER1_SENS_GPIO_Port, STEPPER1_SENS_Pin, &htim3,
                             TIM_CHANNEL_3);
  stepper1->SetMicrostep(16);
  stepper1->SetGearRatio(5);
  stepper1->SetInitialSpeed(0.1 * brio::kPI);
  stepper1->SetDefaultSpeed(1 * brio::kPI);
  stepper1->SetAccel(0.5 * brio::kPI);
  stepper1->SetMinRad(-2 * brio::kPI);
  stepper1->SetMaxRad(2 * brio::kPI);
  stepper1->SetSensReverse(true);
  stepper2 = new brio::A4988(STEPPER2_STEP_GPIO_Port, STEPPER2_STEP_Pin,
                             STEPPER2_DIR_GPIO_Port, STEPPER2_DIR_Pin,
                             STEPPER2_SENS_GPIO_Port, STEPPER2_SENS_Pin, &htim2,
                             TIM_CHANNEL_1);
  stepper2->SetMicrostep(4);
  stepper2->SetDefaultSpeed(12 * brio::kPI);
  stepper2->SetAccel(10 * brio::kPI);
  stepper2->SetMinRad(0);
  stepper2->SetMaxRad(40 * brio::kPI);
  stepper3 = new brio::A4988(STEPPER3_STEP_GPIO_Port, STEPPER3_STEP_Pin,
                             STEPPER3_DIR_GPIO_Port, STEPPER3_DIR_Pin,
                             STEPPER3_SENS_GPIO_Port, STEPPER3_SENS_Pin,
                             &htim15, TIM_CHANNEL_1);
  stepper3->SetMotorSteps(200);
  stepper3->SetMicrostep(16);
  stepper3->SetGearRatio(3);
  stepper3->SetInitialSpeed(0.2 * brio::kPI);
  stepper3->SetDefaultSpeed(2 * brio::kPI);
  stepper3->SetAccel(2 * brio::kPI);
  stepper3->SetSensReverse(true);
  HAL_GPIO_WritePin(STEPPER_RESET_GPIO_Port, STEPPER_RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(STEPPER_RESET_GPIO_Port, STEPPER_RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(STEPPER_SLEEP_GPIO_Port, STEPPER_SLEEP_Pin, GPIO_PIN_SET);
  HAL_Delay(100);
  // Start Timers
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim15);
  // Return to origin
  SEGGER_RTT_printf(0, "Stepper motor 1 return to origin...");
  stepper1->ReturnToOrigin();
  SEGGER_RTT_printf(0, "done.\r\n");
  SEGGER_RTT_printf(0, "Stepper motor 2 return to origin...");
  stepper2->ReturnToOrigin();
  SEGGER_RTT_printf(0, "done.\r\n");
  SEGGER_RTT_printf(0, "Stepper motor 3 return to origin...");
  stepper3->ReturnToOrigin();
  SEGGER_RTT_printf(0, "done.\r\n");
  SEGGER_RTT_printf(0, "Stepper motor driver initialized.\r\n");

  HAL_Delay(1000);

  etl::queue<char, 256> cmd_queue;
  cmd_queue.clear();

  while (true) {
    int input = SEGGER_RTT_GetKey();
    if (input < 0) {
      continue;
    }
    if (cmd_queue.full()) {
      cmd_queue.pop();
    }
    cmd_queue.push(input);

    if (cmd_queue.back() == ';') {
      if (cmd_queue.size() < 3) {
        cmd_queue.clear();
        continue;
      }

      // Parse command
      char cmd = cmd_queue.front();
      cmd_queue.pop();
      char target = cmd_queue.front();
      cmd_queue.pop();
      char value_char[16] = "";
      for (int i = 0; i < cmd_queue.size() - 1; i++) {
        strcat(value_char, &cmd_queue.front());
        cmd_queue.pop();
      }
      int value = atoi(value_char);
      cmd_queue.clear();

      SEGGER_RTT_printf(0, "Command: %c, Target: %c, Value: %d\r\n", cmd,
                        target, value);

      brio::A4988* stepper;
      if (target == '1') {
        stepper = stepper1;
      } else if (target == '2') {
        stepper = stepper2;
      } else if (target == '3') {
        stepper = stepper3;
      } else {
        continue;
      }

      if (cmd == 'm') {
        // Move to
        stepper->MoveTo(0.1 * value * brio::kPI);
      }
      if (cmd == 's') {
        // Set speed
        stepper->SetDefaultSpeed(0.1 * value * brio::kPI);
      }
      if (cmd == 'a') {
        // Acceleration
        stepper->SetAccel(0.1 * value * brio::kPI);
      }
      if (cmd == 'r') {
        HAL_NVIC_SystemReset();
      }
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim == &htim6) {
    HAL_IWDG_Refresh(&hiwdg);
    stepper1->Update();
    stepper2->Update();
    stepper3->Update();
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim) {
  if (htim == &htim3) {
    stepper1->PWMPulseFinishedCallback();
  } else if (htim == &htim2) {
    stepper2->PWMPulseFinishedCallback();
  } else if (htim == &htim15) {
    stepper3->PWMPulseFinishedCallback();
  }
}
