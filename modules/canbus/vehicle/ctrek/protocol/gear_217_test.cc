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

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace ctrek {

class Accel6bTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(Accel6bTest, Parse) {
  Gear217 gear;
  uint8_t data[8] = {0x67, 0x62, 0x63, 0x64, 0x51, 0x52, 0x53, 0x54};
  EXPECT_EQ(gear.GetPeriod(), 10 * 1000);
  gear.set_enable();
  gear.set_gear_reverse();
  gear.UpdateData(data);
  EXPECT_EQ(data[0], 0b00000001);
  EXPECT_EQ(data[1], 0b00000010);
  EXPECT_EQ(data[2], 0b00000000);
  EXPECT_EQ(data[3], 0b00000000);
  EXPECT_EQ(data[4], 0b00000000);
  EXPECT_EQ(data[5], 0b00000000);
  EXPECT_EQ(data[6], 0b00000000);
  EXPECT_EQ(data[7], 0b00000000);
  gear.set_driver_override();
  gear.UpdateData(data);
  EXPECT_EQ(data[0], 0b00000011);

}

}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo
