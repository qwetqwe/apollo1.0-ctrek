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

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace ctrek {
TEST(Brake321Test, General) {
  uint8_t data[8] = {0x01, 0x13, 0x13, 0x04, 0x11, 0x12, 0x13, 0x14};
  int32_t length = 8;
  ChassisDetail cd;
  Brake321 brake;
  brake.Parse(data, length, &cd);

  EXPECT_EQ(data[0], 0b00000001);
  EXPECT_EQ(data[1], 0b00010011);
  EXPECT_EQ(data[2], 0b00010011);
  EXPECT_EQ(data[3], 0b00000100);
  EXPECT_EQ(data[4], 0b00010001);
  EXPECT_EQ(data[5], 0b00010010);
  EXPECT_EQ(data[6], 0b00010011);
  EXPECT_EQ(data[7], 0b00010100);

  auto &brakeinfo = cd.brake();
  


  EXPECT_FALSE(brakeinfo.brake_enabled());
  EXPECT_TRUE(brakeinfo.driver_override());
  EXPECT_TRUE(brakeinfo.boo_fault());
  EXPECT_FALSE(cd.check_response().is_esp_online());
}

}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo
