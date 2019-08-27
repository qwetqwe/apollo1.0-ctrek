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

#include "modules/canbus/vehicle/ctrek/protocol/throttle_216.h"
#include "modules/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ctrek {

// public

const int32_t Throttle216::ID = 0x216;

uint32_t Throttle216::GetPeriod() const {
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Throttle216::UpdateData(uint8_t *data) {
  for (int i=0;i<=7;i++)
    data[i]=0;
  set_pedal_p(data, pedal_cmd_);
  set_enable_p(data, pedal_enable_);
//  set_clear_driver_override_flag_p(data, clear_driver_override_flag_);
//  set_ignore_driver_override_p(data, ignore_driver_override_);
//  set_watchdog_counter_p(data, watchdog_counter_);
}

void Throttle216::Reset() {
  pedal_cmd_ = 0.0;
  pedal_enable_ = false;
  clear_driver_override_flag_ = false;
  ignore_driver_override_ = false;
  watchdog_counter_ = 0;
}

Throttle216 *Throttle216::set_pedal(double pedal) {
  pedal_cmd_ = pedal;
  return this;
}

Throttle216 *Throttle216::set_enable() {
  pedal_enable_ = true;
  return this;
}

Throttle216 *Throttle216::set_disable() {
  pedal_enable_ = false;
  return this;
}

// private

void Throttle216::set_pedal_p(uint8_t *data, double pedal) {
  // change from [0-100] to [0.00-1.00]
  // and a rough mapping
  pedal /= 100.0;
  pedal = ProtocolData::BoundedValue(0.0, 0.99, pedal);
  int32_t x = pedal * 100 / 0.39;

  Byte frame(data + 1);
  frame.set_value(x, 0, 8);

}

void Throttle216::set_enable_p(uint8_t *bytes, bool enable) {
  Byte frame(bytes + 0);
  if (enable) {
    frame.set_value(1, 0, 2);
  } else {
    frame.set_value(2, 0, 2);
  }
}

void Throttle216::set_clear_driver_override_flag_p(uint8_t *bytes, bool clear) {
  Byte frame(bytes + 3);
  if (clear) {
    frame.set_bit_1(1);
  } else {
    frame.set_bit_0(1);
  }
}

void Throttle216::set_ignore_driver_override_p(uint8_t *bytes, bool ignore) {
  Byte frame(bytes + 3);
  if (ignore) {
    frame.set_bit_1(2);
  } else {
    frame.set_bit_0(2);
  }
}

void Throttle216::set_watchdog_counter_p(uint8_t *data, int32_t count) {
  count = ProtocolData::BoundedValue(0, 255, count);
  Byte frame(data + 7);
  frame.set_value(count, 0, 8);
}

}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo
