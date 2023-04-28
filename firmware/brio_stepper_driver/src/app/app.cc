#include "brio/app/app.h"

#include <stdint.h>
#include <stdlib.h>

// C includes
#include "SEGGER_RTT.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
// C++ includes
#include "brio/module/a4988.h"
#include "brio/proto/stepper_driver.h"
#include "brio/util/utils.h"
#include "etl/queue.h"

// Stepper motor
brio::A4988* stepper1;
brio::A4988* stepper2;
brio::A4988* stepper3;
// SPI buffer
Main2StepperDriver spi_rx_buffer;
StepperDriver2Main spi_tx_buffer;

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
  stepper1->SetDefaultAccel(0.5 * brio::kPI);
  stepper1->SetMinRad(-2 * brio::kPI);
  stepper1->SetMaxRad(2 * brio::kPI);
  stepper1->SetSensReverse(true);
  stepper2 = new brio::A4988(STEPPER2_STEP_GPIO_Port, STEPPER2_STEP_Pin,
                             STEPPER2_DIR_GPIO_Port, STEPPER2_DIR_Pin,
                             STEPPER2_SENS_GPIO_Port, STEPPER2_SENS_Pin, &htim2,
                             TIM_CHANNEL_1);
  stepper2->SetMicrostep(4);
  stepper2->SetDefaultSpeed(12 * brio::kPI);
  stepper2->SetDefaultAccel(10 * brio::kPI);
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
  stepper3->SetDefaultAccel(2 * brio::kPI);
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
  // Start SPI
  HAL_SPI_TransmitReceive_DMA(&hspi1, spi_tx_buffer.bin, spi_rx_buffer.bin, 2);
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

  while (true) HAL_Delay(1);
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

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
  SEGGER_RTT_printf(0, "SPI Received: %u, %u\r\n", spi_rx_buffer.bin,
                    spi_rx_buffer.bin);
  stepper1->MoveTo(spi_rx_buffer.cmd.j1_position, spi_rx_buffer.cmd.j1_velocity,
                   spi_rx_buffer.cmd.j1_acceleration);
  stepper2->MoveTo(spi_rx_buffer.cmd.j2_position, spi_rx_buffer.cmd.j2_velocity,
                   spi_rx_buffer.cmd.j2_acceleration);
  stepper3->MoveTo(spi_rx_buffer.cmd.j3_position, spi_rx_buffer.cmd.j3_velocity,
                   spi_rx_buffer.cmd.j3_acceleration);
  spi_tx_buffer.status.j1_position = stepper1->GetAngle();
  spi_tx_buffer.status.j1_velocity = stepper1->GetVelocity();
  spi_tx_buffer.status.j1_acceleration = stepper1->GetAcceleration();
  spi_tx_buffer.status.j2_position = stepper2->GetAngle();
  spi_tx_buffer.status.j2_velocity = stepper2->GetVelocity();
  spi_tx_buffer.status.j2_acceleration = stepper2->GetAcceleration();
  spi_tx_buffer.status.j3_position = stepper3->GetAngle();
  spi_tx_buffer.status.j3_velocity = stepper3->GetVelocity();
  spi_tx_buffer.status.j3_acceleration = stepper3->GetAcceleration();
  HAL_SPI_TransmitReceive_DMA(
      &hspi1, spi_tx_buffer.bin, spi_rx_buffer.bin,
      std::max(sizeof(StepperDriver2Main), sizeof(Main2StepperDriver)));
}
