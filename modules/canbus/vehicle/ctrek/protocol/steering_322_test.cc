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

#include "modules/canbus/vehicle/ctrek/protocol/steering_322.h"

#include "gtest/gtest.h"

namespace apollo {
namespace canbus {
namespace ctrek {

TEST(Steering322Test, General) {
  uint8_t data[8] = {0x43, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
  int32_t length = 8;
  ChassisDetail cd;
  Steering322 steering;
  steering.Parse(data, length, &cd);
  
  EXPECT_TRUE(cd.eps().steering_enabled());
  EXPECT_NEAR(cd.eps().steering_angle(), 870, 1.0);
  EXPECT_FALSE(cd.eps().driver_override());
  EXPECT_TRUE(cd.check_response().is_eps_online());
  EXPECT_FALSE(cd.eps().connector_fault());
  data[7]=0x04;
  steering.Parse(data, length, &cd);
  EXPECT_FALSE(cd.eps().steering_enabled());
  EXPECT_FALSE(cd.check_response().is_eps_online());
  data[7]=0x07;
  steering.Parse(data, length, &cd);
  EXPECT_TRUE(cd.eps().connector_fault());


}

}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo
