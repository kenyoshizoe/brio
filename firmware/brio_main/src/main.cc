#include <Arduino.h>
#include <SPI.h>

#include "brio/module/buzzer.h"
#include "brio/module/stepper_driver.h"
#include "brio/pin_assign.h"
#include "brio/proto/stepper_driver.h"
#include "brio/utils/cobs.h"
#include "etl/queue.h"
#include "main.pb.h"
#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"

// Modules
brio::Buzzer buzzer(brio::pin_assign::kBuzzer);
brio::StepperDriver stepper_driver(new SPIClass(VSPI));
// Timer
hw_timer_t* stepper_driver_timer = NULL;
void IRAM_ATTR Communicate() { stepper_driver.Communicate(); }

void setup() {
  Serial.begin(115200);
  Serial.println("Brio Main Board Start Up");
  Serial2.begin(115200);
  Serial2.println("Brio Main Board Start Up");
  delay(1000);
  stepper_driver.Init();
  buzzer.PlayStartUp();

  stepper_driver_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(stepper_driver_timer, &Communicate, true);
  timerAlarmWrite(stepper_driver_timer, 100000, true);
  timerAlarmEnable(stepper_driver_timer);
}

void loop() {
  etl::queue<uint8_t, 256> rx_queue;
  while (true) {
    StepperDriver2Main rx_msg = stepper_driver.Read();
    // Encode to Protobuf message
    brio_Robot2PC msg = brio_Robot2PC_init_zero;
    msg.has_j1 = true;
    msg.j1.position = rx_msg.status.j1_position;
    msg.j1.velocity = rx_msg.status.j1_velocity;
    msg.j1.acceleration = rx_msg.status.j1_acceleration;
    msg.has_j2 = true;
    msg.j2.position = rx_msg.status.j2_position;
    msg.j2.velocity = rx_msg.status.j2_velocity;
    msg.j2.acceleration = rx_msg.status.j2_acceleration;
    msg.has_j3 = true;
    msg.j3.position = rx_msg.status.j3_position;
    msg.j3.velocity = rx_msg.status.j3_velocity;
    msg.j3.acceleration = rx_msg.status.j3_acceleration;
    uint8_t* buf = new uint8_t[256];
    pb_ostream_t ostream = pb_ostream_from_buffer(buf, 256);
    if (!pb_encode(&ostream, brio_Robot2PC_fields, &msg)) return;
    uint8_t checksum = 0;
    for (int i = 0; i < ostream.bytes_written; i++) {
      checksum += buf[i];
    }
    buf[ostream.bytes_written] = checksum;
    // COBS Encode
    size_t encoded_size =
        brio::COBS::GetEncodedBufferSize(ostream.bytes_written + 1);
    uint8_t* encoded_data = new uint8_t[encoded_size];
    brio::COBS::Encode(buf, ostream.bytes_written + 1, encoded_data);
    // Send
    Serial.write(encoded_data, encoded_size);
    delete[] buf;
    delete[] encoded_data;

    while (Serial.available()) {
      rx_queue.push(Serial.read());
      if (rx_queue.full()) {
        rx_queue.pop();
      }
      if (rx_queue.back() != 0x00) {
        continue;
      }
      // Decode
      size_t size = rx_queue.size();
      uint8_t* data = new uint8_t[size];
      for (int i = 0; i < size; i++) {
        data[i] = rx_queue.front();
        rx_queue.pop();
      }
      size_t decoded_size = brio::COBS::GetEncodedBufferSize(size);
      uint8_t* decoded_data = new uint8_t[decoded_size];
      decoded_size = brio::COBS::Decode(data, size, decoded_data);
      // Check Checksum
      uint8_t checksum = 0;
      for (int i = 0; i < decoded_size - 1; i++) {
        checksum += decoded_data[i];
      }
      if (checksum != decoded_data[decoded_size - 1]) {
        rx_queue.clear();
        delete[] data;
        delete[] decoded_data;
        continue;
      }
      // Parse
      pb_istream_t istream =
          pb_istream_from_buffer(decoded_data, decoded_size - 1);
      brio_PC2Robot msg = brio_PC2Robot_init_zero;
      if (!pb_decode(&istream, brio_PC2Robot_fields, &msg)) {
        rx_queue.clear();
        delete[] data;
        delete[] decoded_data;
        continue;
      }
      // Execute
      // Send to Stepper Driver
      Main2StepperDriver stepper_driver_msg;
      if (msg.has_j1) {
        stepper_driver_msg.cmd.j1_position = msg.j1.position;
        stepper_driver_msg.cmd.j1_velocity = msg.j1.velocity;
        stepper_driver_msg.cmd.j1_acceleration = msg.j1.acceleration;
      }
      if (msg.has_j2) {
        stepper_driver_msg.cmd.j2_position = msg.j2.position;
        stepper_driver_msg.cmd.j2_velocity = msg.j2.velocity;
        stepper_driver_msg.cmd.j2_acceleration = msg.j2.acceleration;
      }
      if (msg.has_j3) {
        stepper_driver_msg.cmd.j3_position = msg.j3.position;
        Serial2.println(msg.j3.position);
        stepper_driver_msg.cmd.j3_velocity = msg.j3.velocity;
        stepper_driver_msg.cmd.j3_acceleration = msg.j3.acceleration;
      }
      stepper_driver.Write(stepper_driver_msg);
      // Send to Buzzer
      buzzer.Play(msg.buzzer_freq);
      // Send to LED
      // TODO(@kenyoshizoe)
      // LED配線後に実装
      rx_queue.clear();
      delete[] data;
      delete[] decoded_data;
    }
  }
}
