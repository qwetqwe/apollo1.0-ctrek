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

#include "modules/canbus/vehicle/ctrek/ctrek_message_manager.h"
#include "modules/canbus/vehicle/protocol_data.h"
#include "gtest/gtest.h"
#include "modules/canbus/vehicle/ctrek/protocol/brake_214.h"
#include "modules/canbus/vehicle/ctrek/protocol/gear_217.h"
#include "modules/canbus/vehicle/ctrek/protocol/gear_324.h"
#include "modules/canbus/vehicle/ctrek/protocol/steering_215.h"
#include "modules/canbus/vehicle/ctrek/protocol/steering_322.h"
#include "modules/canbus/vehicle/ctrek/protocol/throttle_216.h"
#include "modules/canbus/vehicle/ctrek/protocol/throttle_323.h"
#include "modules/canbus/vehicle/ctrek/protocol/light_218.h"
#include "modules/canbus/vehicle/ctrek/protocol/vehiclespd_4A0.h"
namespace apollo {
namespace canbus {
namespace ctrek {


class ctrekMessageManagerTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};



TEST_F(ctrekMessageManagerTest, Brake214) {
  ctrekMessageManager manager;
  ProtocolData *pd =
      manager.GetMutableProtocolDataById(Brake214::ID);
  EXPECT_TRUE(pd != nullptr);
  EXPECT_EQ(static_cast<Brake214 *>(pd)->ID, Brake214::ID);
}


TEST_F(ctrekMessageManagerTest, Gear217) {
  ctrekMessageManager manager;
  ProtocolData  *pd =
      manager.GetMutableProtocolDataById(Gear217::ID);
  EXPECT_TRUE(pd != nullptr);
  EXPECT_EQ(static_cast<Gear217 *>(pd)->ID, Gear217::ID);
}

TEST_F(ctrekMessageManagerTest, Gear324) {
  ctrekMessageManager manager;
  ProtocolData  *pd =
      manager.GetMutableProtocolDataById(Gear324::ID);
  EXPECT_TRUE(pd != nullptr);
  EXPECT_EQ(static_cast<Gear324 *>(pd)->ID, Gear324::ID);
}


TEST_F(ctrekMessageManagerTest, Steering215) {
  ctrekMessageManager manager;
  ProtocolData  *pd =
      manager.GetMutableProtocolDataById(Steering215::ID);
  EXPECT_TRUE(pd != nullptr);
  EXPECT_EQ(static_cast<Steering215 *>(pd)->ID, Steering215::ID);
}

TEST_F(ctrekMessageManagerTest, Steering322) {
  ctrekMessageManager manager;
  ProtocolData  *pd =
      manager.GetMutableProtocolDataById(Steering322::ID);
  EXPECT_TRUE(pd != nullptr);
  EXPECT_EQ(static_cast<Steering322 *>(pd)->ID, Steering322::ID);
}
TEST_F(ctrekMessageManagerTest, Vehiclespd4A0) {
  ctrekMessageManager manager;
  ProtocolData  *pd =
      manager.GetMutableProtocolDataById(Vehiclespd4A0::ID);
  EXPECT_TRUE(pd != nullptr);
  EXPECT_EQ(static_cast<Vehiclespd4A0 *>(pd)->ID, Vehiclespd4A0::ID);
}
TEST_F(ctrekMessageManagerTest, Throttle216) {
  ctrekMessageManager manager;
  ProtocolData  *pd =
      manager.GetMutableProtocolDataById(Throttle216::ID);
  EXPECT_TRUE(pd != nullptr);
  EXPECT_EQ(static_cast<Throttle216 *>(pd)->ID, Throttle216::ID);
}

TEST_F(ctrekMessageManagerTest, Light218) {
  ctrekMessageManager manager;
  ProtocolData  *pd =
      manager.GetMutableProtocolDataById(Light218::ID);
  EXPECT_TRUE(pd != nullptr);
  EXPECT_EQ(static_cast<Light218 *>(pd)->ID, Light218::ID);
}


}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo
