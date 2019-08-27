/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/canbus/vehicle/ctrek/protocol/steering_215.h"
#include "modules/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ctrek {

const int32_t Steering215::ID = 0x215;

uint32_t Steering215::GetPeriod() const {
  // receive rate??
  // receive timeout would trigger fault, letting en=0 and etc.
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Steering215::UpdateData(uint8_t *data) {
	set_steering_null(data);
	set_steering_angle_p(data, steering_angle_);
	set_enable_p(data, steering_enable_);
	set_steering_angle_speed_p(data, steering_angle_speed_);
}

void Steering215::Reset() {
  steering_angle_ = 0.0;
  steering_enable_ = 4;
  steering_angle_speed_ = 0.0;
  watchdog_counter_ = 0;
}


Steering215 *Steering215::set_steering_angle(double angle) {
  steering_angle_ = angle;
  return this;
}

Steering215 *Steering215::set_enable() {
  steering_enable_ = 1;
  return this;
}

Steering215 *Steering215::set_disable() {
  steering_enable_ = 4;
  return this;
}

Steering215 *Steering215::set_driver_override() {
  steering_enable_ = 5;
  return this;
}

Steering215 *Steering215::set_steering_angle_speed(double angle_speed) {
  steering_angle_speed_ = angle_speed;
  return this;
}

// private


void Steering215::set_steering_null(uint8_t *data) {
	Byte startbyte0(data);
	startbyte0.set_value(0);

	Byte startbyte1(data + 1);
	startbyte1.set_value_high_4_bits(0);

	Byte startbyte5(data + 5);
	startbyte5.set_value(0);

	Byte startbyte6(data + 6);
	startbyte6.set_value(0);

	Byte startbyte7(data + 7);
	startbyte7.set_value(0);

}

void Steering215::set_steering_angle_p(uint8_t *data, double angle) {
  angle = ProtocolData::BoundedValue(-510.0, 510.0, angle);
  int32_t x = angle / 0.100000;
  x += 8700;
  std::uint8_t t = 0;
  t = x & 0xFF;  // low
  Byte frame_low(data + 3);
  frame_low.set_value(t, 0, 8);

  x >>= 8;  // high
  t = x & 0xFF;
  Byte frame_high(data + 2);
  frame_high.set_value(t, 0, 8);
}

void Steering215::set_enable_p(uint8_t *bytes, uint8_t enable) {
  Byte frame(bytes + 1);
  if (enable != 5) {
    uint8_t t = enable;
    frame.set_value(t, 0, 4);
    t = 1;
    frame.set_value(t, 4, 1);
    frame.set_value(t, 5, 1);
    Byte frame2(bytes + 7);
    frame2.set_value(0, 0, 1);
  } else {
    uint8_t t = enable;
    frame.set_value(t, 0, 4);
    t = 1;
    frame.set_value(t, 4, 1);
    frame.set_value(t, 5, 1);
    Byte frame2(bytes + 7);
    frame2.set_value(0, 0, 1);
  }
}

void Steering215::set_steering_angle_speed_p(uint8_t *data, double angle_speed) {
  angle_speed = ProtocolData::BoundedValue(0.0, 1016.0, angle_speed);

  //���þ���
  int32_t x = angle_speed / 4.000000;

  Byte frame(data + 4);
  frame.set_value(x, 0, 8);
}

void Steering215::set_watchdog_counter_p(uint8_t *data, int32_t count) {
  count = ProtocolData::BoundedValue(0, 15, count);
  Byte frame(data + 0);
  frame.set_value(count, 4, 4);
}


}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo
