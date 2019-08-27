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

#include "modules/canbus/vehicle/ctrek/protocol/brake_214.h"
#include "modules/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ctrek {
// public

const int32_t Brake214::ID = 0x214;
uint32_t Brake214::GetPeriod() const {
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Brake214::UpdateData(uint8_t *data) {
  for (int i=0;i<=7;i++)
    data[i]=0;
  set_pedal_p(data, pedal_cmd_);
  set_enable_p(data, pedal_enable_);
}

void Brake214::Reset() {
  pedal_cmd_ = 0.0;
  pedal_enable_ = 0;
}

Brake214 *Brake214::set_pedal(double pedal) {
  pedal_cmd_ = pedal;
  return this;
}

Brake214 *Brake214::set_enable() {
  pedal_enable_ = 1;
  return this;
}

Brake214 *Brake214::set_disable() {
  pedal_enable_ = 2;
  return this;
}
Brake214 *Brake214::set_driver_override(){
  pedal_enable_ = 3;
  return this;
}
void Brake214::set_pedal_p(uint8_t *data, double pedal) {
  // change from [0-100] to [0.00-1.00]
  // and a rough mapping
  pedal /= 100.;
  pedal = ProtocolData::BoundedValue(0.0, 1.0, pedal); //限定输出在0~1
  int32_t x = pedal / 0.005;  //精度，计算精度的多少倍【0~1】->[0~65535]
  std::uint8_t t = 0;

  t = x & 0xFF;
  Byte frame_low(data + 0);
  frame_low.set_value(t, 0, 8);
}

void Brake214::set_enable_p(uint8_t *bytes, uint8_t enable) {
  Byte frame(bytes + 1);
  frame.set_value(enable, 0, 2);
}


}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo
