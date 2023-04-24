#include "brio/app/app.h"

#include <stdint.h>

// C includes
#include "SEGGER_RTT.h"
#include "tim.h"
// C++ includes
#include "brio/module/a4988.h"

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
  stepper1->SetMicrostep(4);
  stepper2 = new brio::A4988(STEPPER2_STEP_GPIO_Port, STEPPER2_STEP_Pin,
                             STEPPER2_DIR_GPIO_Port, STEPPER2_DIR_Pin,
                             STEPPER2_SENS_GPIO_Port, STEPPER2_SENS_Pin, &htim2,
                             TIM_CHANNEL_1);
  stepper2->SetMicrostep(4);
  stepper2->SetMaxStepCount(24000);
  stepper3 = new brio::A4988(STEPPER3_STEP_GPIO_Port, STEPPER3_STEP_Pin,
                             STEPPER3_DIR_GPIO_Port, STEPPER3_DIR_Pin,
                             STEPPER3_SENS_GPIO_Port, STEPPER3_SENS_Pin,
                             &htim15, TIM_CHANNEL_1);
  stepper3->SetMicrostep(4);
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

  SEGGER_RTT_printf(0, "Stepper motor driver initialized.\r\n");

  SEGGER_RTT_printf(0, "Stepper motor 2 return to origin...");
  stepper2->ReturnToOrigin();
  SEGGER_RTT_printf(0, "done.\r\n");
  stepper2->Run(4 * kPI, 8 * kPI);

  HAL_Delay(1000);

  while (true) {
    SEGGER_RTT_printf(0, "Stepper motor 2 CW.\r\n");
    stepper2->Run(20 * M_PI, 8 * M_PI);
    while (stepper2->IsRunning()) {
      HAL_Delay(1);
    }
    HAL_Delay(1000);
    SEGGER_RTT_printf(0, "Stepper motor 2 CCW.\r\n");
    stepper2->Run(-20 * M_PI, 8 * M_PI);
    while (stepper2->IsRunning()) {
      HAL_Delay(1);
    }
    HAL_Delay(1000);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if (htim == &htim6) {
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
