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

#include "modules/canbus/vehicle/ctrek/protocol/vehiclespd_4A0.h"
#include<stdio.h>
#include "modules/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ctrek {



const int32_t Vehiclespd4A0::ID = 0x4a0;

void Vehiclespd4A0::Parse(const std::uint8_t *bytes, int32_t length,
                       ChassisDetail *chassis_detail) const{


  // vehicle speed from steering, kph -> mps
  //chassis_detail->mutable_eps()->set_vehicle_speed(
   //   vehicle_speed(bytes, length) / 3.6);

  // speed, as it has a higher accuracy
  // kph -> mps
  double fl,fr,rl,rr;
  fl =front_left_wheel_speed(bytes, length); 
  chassis_detail->mutable_vehicle_spd()->set_wheel_spd_fl(fl);
  
  chassis_detail->mutable_vehicle_spd()->set_is_wheel_spd_fl_valid(true);
  // front right

  fr = front_right_wheel_speed(bytes, length);
  chassis_detail->mutable_vehicle_spd()->set_wheel_spd_fr(fr);
  chassis_detail->mutable_vehicle_spd()->set_is_wheel_spd_fr_valid(true);
  // rear left
  rl = rear_left_wheel_speed(bytes, length);
  chassis_detail->mutable_vehicle_spd()->set_wheel_spd_rl(rl);
  chassis_detail->mutable_vehicle_spd()->set_is_wheel_spd_rl_valid(true);
  // rear right
  rr = rear_right_wheel_speed(bytes, length);
  chassis_detail->mutable_vehicle_spd()->set_wheel_spd_rr(rr);
  chassis_detail->mutable_vehicle_spd()->set_is_wheel_spd_rr_valid(true);
  
  chassis_detail->mutable_vehicle_spd()->set_vehicle_spd((fl+fr+rl+rr)/4.0/3.6); //m/s
  chassis_detail->mutable_vehicle_spd()->set_is_vehicle_spd_valid(true);
}

void Vehiclespd4A0::Parse(const std::uint8_t *bytes, int32_t length,
                       const struct timeval &timestamp,
                       ChassisDetail *chassis_detail)  const{
  Parse(bytes, length, chassis_detail);
}
double Vehiclespd4A0::front_left_wheel_speed(const std::uint8_t *bytes,
                                int32_t length) const{
  Byte frame_high(bytes+1),frame_low(bytes+0);
  uint8_t low = frame_low.get_byte(1, 7);
  uint8_t high = frame_high.get_byte(0, 8);
  uint32_t value = (high<<7) | low;
  return value*0.01;
}

double Vehiclespd4A0::front_right_wheel_speed(const std::uint8_t *bytes,
                                int32_t length) const{
  Byte frame_high(bytes+3),frame_low(bytes+2);
  uint8_t low = frame_low.get_byte(1, 7);
  uint8_t high = frame_high.get_byte(0, 8);
  uint32_t value = (high<<7) | low;
  return value*0.01;
}

double Vehiclespd4A0::rear_left_wheel_speed(const std::uint8_t *bytes,
                                int32_t length)  const{
  Byte frame_high(bytes+5),frame_low(bytes+4);
  uint8_t low = frame_low.get_byte(1, 7);
  uint8_t high = frame_high.get_byte(0, 8);
  uint32_t value = (high<<7) | low;
  return value*0.01;
  
}

double Vehiclespd4A0::rear_right_wheel_speed(const std::uint8_t *bytes,
                                int32_t length) const{
  Byte frame_high(bytes+7),frame_low(bytes+6);
  uint8_t low = frame_low.get_byte(1, 7);
  uint8_t high = frame_high.get_byte(0, 8);
  uint32_t value = (high<<7) | low;
  return value*0.01;
}


}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo
