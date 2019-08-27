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

#include "modules/canbus/vehicle/ctrek/protocol/gear_324.h"

#include "modules/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ctrek {


const int32_t Gear324::ID = 0x324;

void Gear324::Parse(const std::uint8_t *bytes, int32_t length,
                   ChassisDetail *chassis_detail) const {
  int32_t gear = gear_state(bytes, length);
  switch (gear) {
    case 0:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_INVALID);
      break;
    case 1:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_PARKING);
      break;
    case 2:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_REVERSE);
      break;
    case 3:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_NEUTRAL);
      break;
    case 4:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_DRIVE);
      break;
    default:
      chassis_detail->mutable_gear()->set_gear_state(Chassis::GEAR_INVALID);
      break;
  }    
  if (is_driver_override(bytes, length)) {
    // last shift requested by driver
    chassis_detail->mutable_gear()->set_is_shift_position_valid(false);
  } else {
    // last shift requested by-wire
    chassis_detail->mutable_gear()->set_is_shift_position_valid(true);
  }
  chassis_detail->mutable_gear()->set_driver_override(
      is_driver_override(bytes, length));

  chassis_detail->mutable_gear()->set_canbus_fault(
      is_canbus_fault(bytes, length));
  chassis_detail->mutable_check_response()->set_is_gear_online(
      !is_driver_override(bytes, length));
}

int32_t Gear324::gear_state(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 1);
  int32_t x = frame.get_byte(0, 4);
  return x;
}

bool Gear324::is_driver_override(const std::uint8_t *bytes,
                                int32_t length) const {
  Byte frame(bytes + 0);
  int32_t x = frame.get_byte(0, 2);
  if (x==3)
    return true;
  else
    return false;
}

int32_t Gear324::reported_gear_cmd(const std::uint8_t *bytes,
                                  int32_t length) const {
  Byte frame(bytes + 0);
  int32_t x = frame.get_byte(4, 3);
  return x;
}

bool Gear324::is_canbus_fault(const std::uint8_t *bytes, int32_t length) const {
  Byte frame(bytes + 0);
  uint8_t t = frame.get_byte(3,2);
  if (t==2) 
    return true;
  else
    return false;
}

}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo
