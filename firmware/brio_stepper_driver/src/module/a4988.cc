#include "brio/module/a4988.h"

#include "SEGGER_RTT.h"

namespace brio {
A4988::A4988(GPIO_TypeDef *step_port, uint16_t step_pin, GPIO_TypeDef *dir_port,
             uint16_t dir_pin, GPIO_TypeDef *sens_port, uint16_t sens_pin,
             TIM_HandleTypeDef *timer, uint64_t timer_channel)
    : step_port_(step_port),
      step_pin_(step_pin),
      dir_port_(dir_port),
      dir_pin_(dir_pin),
      sens_port_(sens_port),
      sens_pin_(sens_pin),
      timer_(timer),
      timer_channel_(timer_channel) {}

void A4988::ReturnToOrigin() {
  HAL_GPIO_WritePin(dir_port_, dir_pin_,
                    (reverse_direction_ ? GPIO_PIN_SET : GPIO_PIN_RESET));
  uint16_t period = (uint32_t)(kBaseFreq / initial_speed_);
  __HAL_TIM_SET_AUTORELOAD(timer_, period);
  __HAL_TIM_SET_COMPARE(timer_, timer_channel_, period / 2);
  HAL_TIM_PWM_Start(timer_, timer_channel_);
  while (HAL_GPIO_ReadPin(sens_port_, sens_pin_) ==
         (reverse_sens_ ? GPIO_PIN_SET : GPIO_PIN_RESET)) {
    HAL_Delay(1);
  }
  HAL_TIM_PWM_Stop(timer_, timer_channel_);
  step_count_ = 0;
}

void A4988::Run(float rad, float speed) {
  HAL_TIM_PWM_Stop(timer_, timer_channel_);
  // Set direction
  if (rad > 0.0f) {
    HAL_GPIO_WritePin(dir_port_, dir_pin_,
                      reverse_direction_ ? GPIO_PIN_RESET : GPIO_PIN_SET);
    rotate_forward_ = true;
  } else {
    HAL_GPIO_WritePin(dir_port_, dir_pin_,
                      reverse_direction_ ? GPIO_PIN_SET : GPIO_PIN_RESET);
    rotate_forward_ = false;
  }
  // Calculate target step count
  step_count_target_ =
      step_count_ + rad * motor_steps_ * microstep_ * gear_ratio_ / 2 / kPI;
  step_count_target_ = std::clamp((int64_t)step_count_target_, (int64_t)0,
                                  (int64_t)max_step_count_);
  if (step_count_target_ == step_count_) {
    return;
  }
  // Calculate max speed
  max_speed_ = std::min(
      (float)(speed * motor_steps_ * microstep_ * gear_ratio_ / 2 / kPI),
      std::sqrt(std::abs(step_count_target_ - step_count_) * accel_ +
                initial_speed_ * initial_speed_));
  SEGGER_RTT_printf(0, "max_spped: %d\n", (int)max_speed_ * 10);
  // Calculate initial period
  uint16_t period = (uint32_t)(kBaseFreq / initial_speed_);
  // Set state
  state_ = State::kAccel;
  current_speed_ = kBaseFreq / period;
  // Set period
  __HAL_TIM_SET_AUTORELOAD(timer_, period);
  // Set duty cycle
  __HAL_TIM_SET_COMPARE(timer_, timer_channel_, period / 2);
  // Start timer
  HAL_TIM_PWM_Start_IT(timer_, timer_channel_);

  SEGGER_RTT_printf(0, "HAL_TIM_PWM_Start period: %u\n", period);
  SEGGER_RTT_printf(0, "step_count_target_: %d\n", step_count_target_);
}

void A4988::Update() {
  if (state_ == State::kIdle) {
    return;
  } else if (state_ == State::kAccel) {
    current_speed_ += accel_ / kUpdateHz;
    if (current_speed_ > max_speed_) {
      current_speed_ = max_speed_;
      state_ = State::kCruise;
    }
  } else if (state_ == State::kDecel) {
    // Calculate new period
    current_speed_ -= accel_ / kUpdateHz;
    if (current_speed_ < initial_speed_) {
      current_speed_ = initial_speed_;
    }
  } else if (state_ == State::kCruise) {
    current_speed_ = max_speed_;
    if (abs(step_count_target_ - step_count_) <=
        (max_speed_ * max_speed_ - initial_speed_ * initial_speed_) / accel_ /
            2) {
      state_ = State::kDecel;
    }
  }

  uint32_t period = (uint32_t)(kBaseFreq / current_speed_);
  __HAL_TIM_SET_AUTORELOAD(timer_, period);
  __HAL_TIM_SET_COMPARE(timer_, timer_channel_, period / 2);
}

void A4988::PWMPulseFinishedCallback() {
  // Update step count
  if (rotate_forward_) {
    step_count_++;
  } else {
    step_count_--;
  }
  // Stop timer if target step count is reached
  if (step_count_ == step_count_target_) {
    HAL_TIM_PWM_Stop(timer_, timer_channel_);
    HAL_GPIO_WritePin(step_port_, step_pin_, GPIO_PIN_RESET);
    state_ = State::kIdle;

    SEGGER_RTT_printf(0, "step count reached target: %d\n", step_count_);
  }
}
}  // namespace brio
