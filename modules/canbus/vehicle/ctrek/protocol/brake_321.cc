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

#include "modules/canbus/vehicle/ctrek/protocol/brake_321.h"
#include "modules/canbus/common/byte.h"

namespace apollo
{
namespace canbus
{
namespace ctrek
{

const int32_t Brake321::ID = 0x321;

void Brake321::Parse(const std::uint8_t *bytes, int32_t length,
                     ChassisDetail *chassis_detail) const
{
    /*  chassis_detail->mutable_brake()->set_brake_input(pedal_input(bytes, length));
  chassis_detail->mutable_brake()->set_brake_cmd(pedal_cmd(bytes, length));
  chassis_detail->mutable_brake()->set_brake_output(
      pedal_output(bytes, length));
  chassis_detail->mutable_brake()->set_boo_input(boo_input(bytes, length));
  chassis_detail->mutable_brake()->set_boo_cmd(boo_cmd(bytes, length));
  chassis_detail->mutable_brake()->set_boo_output(boo_output(bytes, length));
  chassis_detail->mutable_brake()->set_watchdog_applying_brakes(
      is_watchdog_counter_applying_brakes(bytes, length));
  chassis_detail->mutable_brake()->set_watchdog_source(
      watchdog_counter_source(bytes, length));
  
  chassis_detail->mutable_brake()->set_driver_override(
      is_driver_override(bytes, length));
  chassis_detail->mutable_brake()->set_driver_activity(
      is_driver_activity(bytes, length));
  chassis_detail->mutable_brake()->set_boo_fault(
      is_boo_switch_fault(bytes, length));

  
  */
    chassis_detail->mutable_brake()->set_watchdog_fault(false);
    chassis_detail->mutable_brake()->set_channel_1_fault(false);
    chassis_detail->mutable_brake()->set_channel_2_fault(false);
    chassis_detail->mutable_brake()->set_connector_fault(false);
    chassis_detail->mutable_brake()->set_brake_enabled(is_enabled(bytes, length));
    chassis_detail->mutable_brake()->set_boo_fault(
        is_boo_switch_fault(bytes, length));
    chassis_detail->mutable_brake()->set_driver_override(
        is_driver_override(bytes, length));
    chassis_detail->mutable_check_response()->set_is_esp_online(
        !is_driver_override(bytes, length));
}

bool Brake321::is_enabled(const std::uint8_t *bytes, int32_t length) const
{
    Byte frame(bytes + 1);
    uint8_t t = frame.get_byte(0, 3);
    if (t == 1)
        return true;
    else
        return false;
}

bool Brake321::is_driver_override(const std::uint8_t *bytes,
                                  int32_t length) const
{
    Byte frame(bytes + 1);
    uint8_t t = frame.get_byte(0, 3);;
    if (t == 3)
        return true;
    else
        return false;
}

bool Brake321::is_boo_switch_fault(const std::uint8_t *bytes,
                                   int32_t length) const
{
    Byte frame(bytes + 1);
    uint8_t t = frame.get_byte(3, 2);
    if (t == 2)
        return true;
    else
        return false;
}

} // namespace ctrek
} // namespace canbus
} // namespace apollo
