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

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace ctrek {

TEST(Wheelspeed6aTest, General) {
  Vehiclespd4A0 vehiclespd4a0;
  uint8_t data[8] = {0x80, 0x4C, 0x80, 0x19, 0x80, 0x19, 0x40, 0x26};
  int32_t length = 8;
  ChassisDetail cd;
  struct timeval timestamp;
  vehiclespd4a0.Parse(data, length, timestamp, &cd);

  EXPECT_TRUE(cd.vehicle_spd().is_wheel_spd_fl_valid());
  EXPECT_NEAR(cd.vehicle_spd().wheel_spd_rr(), 97,2);
  EXPECT_TRUE(cd.vehicle_spd().is_wheel_spd_fr_valid());
  EXPECT_NEAR(cd.vehicle_spd().wheel_spd_rl(), 32,2);
  EXPECT_TRUE(cd.vehicle_spd().is_wheel_spd_rl_valid());
  EXPECT_NEAR(cd.vehicle_spd().wheel_spd_fr(), 326.39,2);
  EXPECT_TRUE(cd.vehicle_spd().is_wheel_spd_rr_valid());
  EXPECT_NEAR(cd.vehicle_spd().wheel_spd_fl(), 326.39,2);
  EXPECT_NEAR(cd.vehicle_spd().vehicle_spd(),326.39/3.6,2);
}

}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo
