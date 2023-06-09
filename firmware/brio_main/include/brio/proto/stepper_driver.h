#ifndef BRIO_MAIN_PROTO_STEPPER_DRIVER_H_
#define BRIO_MAIN_PROTO_STEPPER_DRIVER_H_

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
  } cmd;
  uint8_t bin[sizeof(cmd)];
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
  } status;
  uint8_t bin[sizeof(status)];
};

#endif  // BRIO_MAIN_PROTO_STEPPER_DRIVER_H_
