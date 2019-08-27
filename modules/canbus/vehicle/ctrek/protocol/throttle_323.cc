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

#include "modules/canbus/vehicle/ctrek/protocol/throttle_323.h"
#include "modules/common/log.h"
#include "modules/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ctrek {

const int32_t Throttle323::ID = 0x323;

void Throttle323::Parse(const std::uint8_t* bytes, int32_t length,
                       ChassisDetail* car_status) const {
  car_status->mutable_gas()->set_throttle_input(pedal_input(bytes, length));
  car_status->mutable_gas()->set_throttle_cmd(pedal_output(bytes, length));
  car_status->mutable_gas()->set_throttle_output(pedal_output(bytes, length));
  car_status->mutable_gas()->set_throttle_enabled(is_enabled(bytes, length));
  car_status->mutable_gas()->set_driver_override(false);
  car_status->mutable_gas()->set_driver_activity(false);
  car_status->mutable_gas()->set_watchdog_fault(false);
  car_status->mutable_gas()->set_channel_1_fault(false);
  car_status->mutable_gas()->set_channel_2_fault(false);
  car_status->mutable_gas()->set_connector_fault(
      is_connector_fault(bytes, length));
  car_status->mutable_check_response()->set_is_vcu_online(
      !is_connector_fault(bytes, length));
}

double Throttle323::pedal_input(const std::uint8_t* bytes,
                               int32_t length) const {
  // Pedal Input from the physical pedal
  Byte frame(bytes + 2);
  int32_t raw = frame.get_byte(0, 8);
  // control needs a value in range [0, 100] %
  double output = raw * 0.39;
  output = ProtocolData::BoundedValue(0.0, 100.0, output);
  return output;
}



double Throttle323::pedal_output(const std::uint8_t* bytes,
                                int32_t length) const {
  // Pedal Output is the maximum of PI and PC
  Byte frame(bytes + 1);
  int32_t raw = frame.get_byte(0, 8);
  // control needs a value in range [0, 100] %
  double output = raw * 0.39;
  output = ProtocolData::BoundedValue(0.0, 100.0, output);
  return output;
}

bool Throttle323::is_enabled(const std::uint8_t* bytes, int32_t length) const {
  Byte frame(bytes + 0);
  int32_t raw = frame.get_byte(0, 2);
  if (raw == 1)
    return true;
  else
    return false;
  
}

bool Throttle323::is_connector_fault(const std::uint8_t* bytes,
                                    int32_t length) const {
  Byte frame(bytes + 0);
  int32_t raw = frame.get_byte(2, 2);
  if (raw == 2)
    return true;
  else
    return false;
}

}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo
