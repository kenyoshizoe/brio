/**
 * @file a4988.h
 * @author Ken Yoshizoe (kenyoshizoe@gmail.com)
 */

#ifndef BRIO_A4988_H_
#define BRIO_A4988_H_

#include <algorithm>
#include <cmath>
#include <limits>

#include "brio/util/utils.h"
#include "main.h"

namespace brio {
/**
 * @brief
 * Class to control A4988 stepper motor driver by trapezoidal acceleration.
 * @note datasheet
 * https://www.pololu.com/file/download/a4988_DMOS_microstepping_driver_with_translator.pdf?file_id=0J450
 */
class A4988 {
 public:
  A4988(GPIO_TypeDef *step_port, uint16_t step_pin, GPIO_TypeDef *dir_port,
        uint16_t dir_pin, GPIO_TypeDef *sens_port, uint16_t sens_pin,
        TIM_HandleTypeDef *timer, uint64_t timer_channel);

  // Setters
  void SetMotorSteps(uint16_t motor_steps) { motor_steps_ = motor_steps; }
  void SetMicrostep(uint8_t microstep) { microstep_ = microstep; }
  void SetGearRatio(uint8_t gear_ratio) { gear_ratio_ = gear_ratio; }
  /**
   * @brief Set max step count
   * @param max_step_count max step count in pulse
   */
  void SetMaxRad(float max_rad) {
    max_step_count_ = std::max((int32_t)Rad2Pulse(max_rad), min_step_count_);
  }
  /**
   * @brief Set min step count
   * @param min_step_count min step count in pulse
   */
  void SetMinRad(float min_rad) {
    min_step_count_ = std::min((int32_t)Rad2Pulse(min_rad), max_step_count_);
  }
  /**
   * @brief Reverse direction of rotation
   */
  void SetReverse(bool reverse) { reverse_direction_ = reverse; }
  /**
   * @brief Set initial speed
   * @param initial_speed initial speed in rad/s
   */
  void SetInitialSpeed(float initial_speed) {
    initial_speed_ = Rad2Pulse(std::max(initial_speed, 0.0f));
  }
  /**
   * @brief Set default speed
   * @param default_speed default speed in rad/s
   */
  void SetDefaultSpeed(float default_speed) {
    default_speed_ = Rad2Pulse(std::max(default_speed, 0.0f));
  }
  /**
   * @brief Set acceleration
   * @param accel acceleration in rad/s^2
   */
  void SetDefaultAccel(float accel) { default_accel_ = Rad2Pulse(accel); }
  /**
   * @brief Reverse origin sensor signal
   */
  void SetSensReverse(bool reverse_sens) { reverse_sens_ = reverse_sens; }

  // Getters
  /**
   * @brief Return stepper is running or not
   */
  bool IsRunning() { return state_ != State::kIdle; }
  /**
   * @brief Return current angle in rad
   */
  float GetAngle() {
    return step_count_ * 2 * kPI / (motor_steps_ * microstep_ * gear_ratio_);
  }
  int64_t GetStepCount() { return step_count_; }
  /**
   * @brief Return current Velocity in rad/s
   */
  float GetVelocity() { return Pulse2Rad(current_velocity_); }
  /**
   * @brief Return current Acceleration in rad/s^2
   */
  float GetAcceleration() {
    return Pulse2Rad(current_accel_) * (rotate_forward_ ? 1 : -1);
  }

  // Control
  /**
   * @brief Return to origin
   * @note This function is blocking
   */
  void ReturnToOrigin();
  /**
   * @brief Move stepper motor to specified angle
   * @param rad angle in rad
   */
  void MoveTo(float rad, float speed = -1, float accel = -1);
  /**
   * @brief Update state of stepper motor
   * @note Call this function in HAL_TIM_PeriodElapsedCallback
   */
  void Update();
  /**
   * @brief Call this function in HAL_TIM_PWM_PulseFinishedCallback
   */
  void PWMPulseFinishedCallback();

 private:
  constexpr static int kBaseFreq = 8000000;  // 8MHz
  constexpr static float kUpdateHz = 1000;   // 1kHz
  // GPIO pins
  GPIO_TypeDef *step_port_;
  uint16_t step_pin_;
  GPIO_TypeDef *dir_port_;
  uint16_t dir_pin_;
  GPIO_TypeDef *sens_port_;
  uint16_t sens_pin_;
  // Timer
  TIM_HandleTypeDef *timer_;
  uint64_t timer_channel_;
  // Parameters
  uint16_t motor_steps_ = 400;  // pulse/rev
  uint8_t microstep_ = 1;       // based on ms1 ms2 ms3
  float gear_ratio_ = 1;
  int32_t max_step_count_ = std::numeric_limits<int32_t>::max();  // pulse
  int32_t min_step_count_ = std::numeric_limits<int32_t>::min();  // pulse
  bool reverse_direction_ = false;
  float initial_speed_ = 1600;  // pulse/s
  float default_speed_ = 6400;  // pulse/s
  float default_accel_ = 3200;  // pulse/s^2
  bool reverse_sens_ = false;
  // State
  volatile enum class State {
    kIdle,
    kAccel,
    kDecel,
    kCruise
  } state_ = State::kIdle;
  volatile bool rotate_forward_ = false;
  volatile int32_t step_count_ = 0;
  volatile int32_t step_count_target_ = 0;
  volatile float target_velocity_ = 0;   // pulse/s
  volatile float current_velocity_ = 0;  // pulse/s
  volatile float current_accel_ = 0;     // pulse/s

  float Pulse2Rad(int pulse) {
    return pulse * 2 * kPI / (motor_steps_ * microstep_ * gear_ratio_);
  }
  float Rad2Pulse(float rad) {
    return rad * motor_steps_ * microstep_ * gear_ratio_ / 2 / kPI;
  }
};
}  // namespace brio
#endif  // BRIO_A4988_H_
