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

#include "modules/canbus/vehicle/ctrek/protocol/gear_217.h"
#include "modules/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ctrek {

// public
const int32_t Gear217::ID = 0x217;

uint32_t Gear217::GetPeriod() const {
  // on event, so value nonsense
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Gear217::UpdateData(uint8_t *data) {
  for (int i=0;i<=7;i++)
    data[i]=0;
  set_gear_p(data, gear_);
  set_enable_p(data, gear_enable_);
}

void Gear217::Reset() { 
  gear_ = 0;
	gear_enable_ = 2;
}

Gear217 *Gear217::set_gear_none() {
  gear_ = 0x00;
  return this;
}

Gear217 *Gear217::set_gear_park() {
  gear_ = 0x01;
  return this;
}

Gear217 *Gear217::set_gear_reverse() {
  gear_ = 0x02;
  return this;
}

Gear217 *Gear217::set_gear_neutral() {
  gear_ = 0x03;
  return this;
}

Gear217 *Gear217::set_gear_drive() {
  gear_ = 0x04;
  return this;
}

Gear217 *Gear217::set_gear_low() {
  gear_ = 0x05;
  return this;
}

Gear217 *Gear217::set_enable() {
	gear_enable_ = 1;
	return this;
}

Gear217 *Gear217::set_disable() {
	gear_enable_ = 2;
	return this;
}
Gear217 *Gear217::set_driver_override() {
  gear_enable_ = 3;
	return this;
}

// private
void Gear217::set_enable_p(uint8_t *bytes, uint8_t enable) {
	Byte frame(bytes + 0);
  frame.set_value(enable, 0, 2);
}

void Gear217::set_gear_p(uint8_t *data, int32_t gear) {
  gear = ProtocolData::BoundedValue(0, 7, gear);
  Byte frame(data + 1);
  frame.set_value(gear, 0, 4);
}


}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo
