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

#include "modules/canbus/vehicle/ctrek/protocol/light_218.h"
#include "modules/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ctrek {
// public

const int32_t Light218::ID = 0x218;
uint32_t Light218::GetPeriod() const {
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Light218::UpdateData(uint8_t *data) {
  for (int i=0;i<=7;i++)
    data[i]=0;
  set_blink_p(data, blink_cmd_);
  set_warnblink_p(data, warnblink_cmd_);
  set_enable_p(data, enable_cmd_);
}

void Light218::Reset() {
  blink_cmd_ = 0;
  warnblink_cmd_ = false;
  enable_cmd_ = 2;

}
Light218 *Light218::set_blink(int turnlight)
{
  blink_cmd_ = turnlight;
  return this;
}
Light218 *Light218::set_warnblink(bool warnblink)
{
  warnblink_cmd_=warnblink;
  return this;
}
Light218 *Light218::set_light_enable()
{
  enable_cmd_=1;
  return this;
}
Light218 *Light218::set_light_disable()
{
  enable_cmd_=2;
  return this;
}
Light218 *Light218::set_driver_override()
{
  enable_cmd_=3;
  return this;
}


void Light218::set_enable_p(uint8_t *bytes, uint8_t enable_cmd_)
{
  if (enable_cmd_!=3)
  {
    Byte frame1(bytes + 1);
    uint8_t t = enable_cmd_;
    frame1.set_value(t, 0, 2);
    Byte frame2(bytes + 2);
    frame2.set_value(t, 0, 2);
  }
  else
  {
    Byte frame1(bytes + 1);
    uint8_t t = 2;
    frame1.set_value(t, 0, 2);
    t = enable_cmd_;
    Byte frame2(bytes + 2);
    frame2.set_value(t, 0, 2);
  }
}
void Light218::set_warnblink_p(uint8_t *data, bool warnblink_cmd_)
{
  uint8_t t;
  Byte frame1(data + 2);
  if (warnblink_cmd_)
    t = 1;
  else
    t = 2;
  frame1.set_value(t, 4, 2);
}
void Light218::set_blink_p(uint8_t *data, uint8_t blink_cmd_)
{
  Byte frame1(data + 1);
  uint8_t t = blink_cmd_;
  frame1.set_value(t, 4, 2);
}


}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo
