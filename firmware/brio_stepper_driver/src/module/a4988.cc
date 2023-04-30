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
  }
  HAL_TIM_PWM_Stop(timer_, timer_channel_);
  step_count_ = 0;
}

void A4988::MoveTo(float rad, float speed, float accel) {
  // Calculate target step count
  uint32_t new_step_count_target =
      std::clamp((int32_t)Rad2Pulse(rad), min_step_count_, max_step_count_);
  if ((uint32_t)step_count_target_ == new_step_count_target) {
    return;
  } else {
    step_count_target_ = new_step_count_target;
  }
  SEGGER_RTT_printf(0, "step_count_target_: %d\n", step_count_target_);
  // Set accel
  current_accel_ = accel <= 0 ? default_accel_ : Rad2Pulse(accel);
  // Set speed
  speed = speed == 0 ? default_speed_ : std::abs(Rad2Pulse(speed));
  target_velocity_ = std::min(
      std::abs(speed),
      std::sqrt(abs(step_count_target_ - step_count_) * current_accel_ +
                initial_speed_ * initial_speed_));
  target_velocity_ *= step_count_ < step_count_target_ ? 1 : -1;

  if (step_count_ < step_count_target_) {
    if (0 <= current_velocity_ && current_velocity_ < target_velocity_) {
      current_velocity_ = initial_speed_;
    }
    state_ = State::kAccel;
  } else if (step_count_ > step_count_target_) {
    if (0 >= current_velocity_ && current_velocity_ > target_velocity_) {
      current_velocity_ = -initial_speed_;
    }
    state_ = State::kDecel;
  } else {
    return;
  }

  // Calculate period
  uint16_t period = (uint32_t)(kBaseFreq / std::abs(current_velocity_));
  // Set direction
  if (step_count_target_ - step_count_ > 0.0f) {
    HAL_GPIO_WritePin(dir_port_, dir_pin_,
                      reverse_direction_ ? GPIO_PIN_RESET : GPIO_PIN_SET);
    rotate_forward_ = true;
  } else {
    HAL_GPIO_WritePin(dir_port_, dir_pin_,
                      reverse_direction_ ? GPIO_PIN_SET : GPIO_PIN_RESET);
    rotate_forward_ = false;
  }
  // Set period
  __HAL_TIM_SET_AUTORELOAD(timer_, period);
  // Set duty cycle
  __HAL_TIM_SET_COMPARE(timer_, timer_channel_, period / 2);
  // Start timer
  HAL_TIM_PWM_Start_IT(timer_, timer_channel_);
}

void A4988::Update() {
  if (state_ == State::kIdle) {
    current_velocity_ = 0;
    return;
  } else if (state_ == State::kAccel) {
    current_velocity_ += current_accel_ / kUpdateHz;
    if (current_velocity_ > target_velocity_) {
      current_velocity_ = target_velocity_;
      state_ = State::kCruise;
    }
  } else if (state_ == State::kDecel) {
    current_velocity_ -= current_accel_ / kUpdateHz;
    if (current_velocity_ < target_velocity_) {
      current_velocity_ = target_velocity_;
      state_ = State::kCruise;
    }
  } else if (state_ == State::kCruise) {
    current_velocity_ = target_velocity_;
    if (std::abs(step_count_target_ - step_count_) <=
        (target_velocity_ * target_velocity_ -
         initial_speed_ * initial_speed_) /
            current_accel_ / 2) {
      target_velocity_ =
          current_velocity_ > 0 ? initial_speed_ : -initial_speed_;
      if (target_velocity_ > current_velocity_) {
        state_ = State::kAccel;
      } else if (target_velocity_ < current_velocity_) {
        state_ = State::kDecel;
      }
    }
  }
  uint32_t period = (uint32_t)(kBaseFreq / std::abs(current_velocity_));
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
    current_velocity_ = 0;
    target_velocity_ = 0;
    state_ = State::kIdle;

    SEGGER_RTT_printf(0, "step count reached target: %d\n", step_count_);
  }
}
}  // namespace brio
