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

#include "modules/canbus/can_client/usb/usb_can_client.h"

#include <vector>

#include "gtest/gtest.h"

#include "modules/canbus/proto/can_card_parameter.pb.h"

namespace apollo {
namespace canbus {
namespace can {

using apollo::common::ErrorCode;

TEST(usbCanClientTest, simple_test) {
  CANCardParameter param;
  param.set_brand(CANCardParameter::usb_CAN);
  param.set_channel_id(CANCardParameter::CHANNEL_ID_ZERO);

  usbCanClient usb_can_client;
  EXPECT_TRUE(usb_can_client.Init(param));
  EXPECT_EQ(usb_can_client.Start(), ErrorCode::OK);
  std::vector<CanFrame> frames;
  int32_t num = 0;
  EXPECT_EQ(usb_can_client.Send(frames, &num),
      ErrorCode::OK);
  EXPECT_EQ(usb_can_client.Receive(&frames, &num),
      ErrorCode::OK);
  CanFrame can_frame;
  frames.push_back(can_frame);
  EXPECT_EQ(usb_can_client.SendSingleFrame(frames),
      ErrorCode::OK);
  usb_can_client.Stop();
}

}  // namespace can
}  // namespace canbus
}  // namespace apollo
