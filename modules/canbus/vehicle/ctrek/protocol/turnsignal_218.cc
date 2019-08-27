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

#include "modules/canbus/vehicle/ctrek/protocol/turnsignal_218.h"
#include "modules/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ctrek {



const int32_t Turnsignal218::ID = 0x218;

uint32_t Turnsignal218::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

int32_t Turnsignal218::turn_cmd() const { return turn_cmd_; }

void Turnsignal218::UpdateData(uint8_t *data) {
  for (int i=0;i<=7;i++)
    data[i]=0;
  set_enable_p(data);
  set_turn_cmd_p(data, turn_cmd_);
}

Turnsignal218 *Turnsignal218::set_enable(){
    enable_=true;
    return this;
}

Turnsignal218 *Turnsignal218::set_disable(){
    enable_=false;
    return this;
}

void Turnsignal218::Reset() { 
  turn_cmd_ = 0; 
  enable_=false;
}

Turnsignal218 *Turnsignal218::set_turn_none() {
  turn_cmd_ = 0x00;
  return this;
}

Turnsignal218 *Turnsignal218::set_turn_left() {
  turn_cmd_ = 0x01;
  return this;
}

Turnsignal218 *Turnsignal218::set_turn_right() {
  turn_cmd_ = 0x02;
  return this;
}

// 0x03 not used

// private
void Turnsignal218::set_turn_cmd_p(uint8_t *data, int32_t turn_cmd) {
  turn_cmd = ProtocolData::BoundedValue(0, 3, turn_cmd);
  Byte frame(data + 1);
  frame.set_value(turn_cmd, 4, 2);
}
void Turnsignal218::set_enable_p(uint8_t *data){
  Byte frame(data + 1);
  uint8_t t;
  if (enable_ == true)
    t=0x01;
  else
    t=0x02;
  frame.set_value(t);
}
}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo
