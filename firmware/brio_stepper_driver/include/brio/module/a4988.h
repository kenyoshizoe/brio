/**
 * @file a4988.h
 * @author Ken Yoshizoe (kenyoshizoe@gmail.com)
 */

#ifndef BRIO_A4988_H_
#define BRIO_A4988_H_

#include <algorithm>
#include <cmath>

#include "main.h"

constexpr double kPI = 3.141592653589793;

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

  void SetMotorSteps(uint16_t motor_steps) { motor_steps_ = motor_steps; }
  void SetMicrostep(uint8_t microstep) { microstep_ = microstep; }
  /**
   * @brief Set initial speed
   * @param initial_speed initial speed in rad/s
   */
  void SetInitialSpeed(float initial_speed) {
    initial_speed_ =
        Rad2Pulse(std::clamp(initial_speed, 0.0f, (float)max_speed_));
  }
  /**
   * @brief Set default speed
   * @param default_speed default speed in rad/s
   */
  void SetDefaultSpeed(float default_speed) {
    default_speed_ =
        Rad2Pulse(std::clamp(default_speed, 0.0f, (float)max_speed_));
  }
  /**
   * @brief Set acceleration
   * @param accel acceleration in rad/s^2
   */
  void SetAccel(float accel) { accel_ = Rad2Pulse(accel); }
  void SetMaxStepCount(int64_t max_step_count) {
    max_step_count_ = max_step_count;
  }
  /**
   * @brief Reverse direction of rotation
   */
  void SetReverse(bool reverse) { reverse_direction_ = reverse; }
  void SetGearRatio(uint8_t gear_ratio) { gear_ratio_ = gear_ratio; }
  /**
   * @brief Reverse origin sensor signal
   */
  void SetSensReverse(bool reverse_sens) { reverse_sens_ = reverse_sens; }
  /**
   * @brief Return current angle in rad
   */
  float GetAngle() {
    return step_count_ * 2 * kPI / (motor_steps_ * microstep_ * gear_ratio_);
  }
  int64_t GetStepCount() { return step_count_; }
  /**
   * @brief Return stepper is running or not
   */
  bool IsRunning() { return state_ != State::kIdle; }

  /**
   * @brief Return to origin
   * @note This function is blocking
   */
  void ReturnToOrigin();
  /**
   * @brief Move stepper motor to specified angle
   * @param rad angle in rad
   */
  void MoveTo(float rad, float speed = -1);
  /**
   * @brief Run stepper motor with const acceleration
   * @param rad angle in rad
   * @param speed max speed in rad/s
   */
  void Run(float rad, float speed = -1);
  /**
   * @brief Update state of stepper motor
   * @note Call this function in HAL_TIM_PeriodElapsedCallback
   * @param hz timer frequency in Hz
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
  float initial_speed_ = 1600;  // pulse/s
  float default_speed_ = 6400;  // pulse/s
  float accel_ = 3200;          // pulse/s^2
  int64_t max_step_count_ = 0;  // pulse
  bool reverse_direction_ = false;
  uint8_t gear_ratio_ = 1;
  bool reverse_sens_ = false;
  // State
  volatile enum class State {
    kIdle,
    kAccel,
    kDecel,
    kCruise
  } state_ = State::kIdle;
  volatile bool rotate_forward_ = false;
  volatile float max_speed_ = 0;      // pulse/s
  volatile float current_speed_ = 0;  // pulse/s
  volatile int64_t step_count_ = 0;
  volatile int64_t step_count_target_ = 0;

  float Pulse2Rad(int pulse) {
    return pulse * 2 * kPI / (motor_steps_ * microstep_ * gear_ratio_);
  }
  float Rad2Pulse(float rad) {
    return rad * motor_steps_ * microstep_ * gear_ratio_ / 2 / kPI;
  }
};
}  // namespace brio
#endif  // BRIO_A4988_H_
