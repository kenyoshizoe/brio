#ifndef BRIO_STEPPER_DRIVER_PROTO_STEPPER_DRIVER_H_
#define BRIO_STEPPER_DRIVER_PROTO_STEPPER_DRIVER_H_

union Main2StepperDriver {
  struct {
    float j1_position;
    float j1_velocity;
    float j1_acceleration;
    float j2_position;
    float j2_velocity;
    float j2_acceleration;
    float j3_position;
    float j3_velocity;
    float j3_acceleration;
    uint8_t crc;
  } cmd;
  uint8_t bin[sizeof(float) * 9 + 1];
};

union StepperDriver2Main {
  struct {
    float j1_position;
    float j1_velocity;
    float j1_acceleration;
    float j2_position;
    float j2_velocity;
    float j2_acceleration;
    float j3_position;
    float j3_velocity;
    float j3_acceleration;
    uint8_t crc;
  } status;
  uint8_t bin[sizeof(float) * 9 + 1];
};

#endif  // BRIO_STEPPER_DRIVER_PROTO_STEPPER_DRIVER_H_
