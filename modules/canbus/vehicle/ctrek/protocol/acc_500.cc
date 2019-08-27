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

#include "modules/canbus/vehicle/ctrek/protocol/acc_500.h"
#include "modules/canbus/common/byte.h"

namespace apollo
{
namespace canbus
{
namespace ctrek
{
using apollo::localization::LocalizationEstimate;
const int32_t Acc500::ID = 0x500;

void Acc500::Parse(const std::uint8_t *bytes, int32_t length,
                     LocalizationEstimate *localization_) const
{
    //vehicle coordinate (Right/Forward/Up)
    localization_->mutable_pose()->mutable_linear_acceleration_vrf()->set_x(-acc_left(bytes,length));
    localization_->mutable_pose()->mutable_linear_acceleration_vrf()->set_y(acc_front(bytes,length));
    localization_->mutable_pose()->mutable_linear_acceleration_vrf()->set_z(acc_up(bytes,length));
}



double Acc500::acc_front(const std::uint8_t *bytes,
                                  int32_t length) const
{
    Byte frame_low(bytes + 1);
    uint8_t low = frame_low.get_byte(0,8);
    Byte frame_high(bytes + 0);
    uint8_t high = frame_high.get_byte(0,8);
    uint32_t val = (high<<8) | low;
    return (val*0.0001220703125-4)*9.8;
}

double Acc500::acc_left(const std::uint8_t *bytes,
                                  int32_t length) const
{
    Byte frame_low(bytes + 3);
    uint8_t low = frame_low.get_byte(0,8);
    Byte frame_high(bytes + 2);
    uint8_t high = frame_high.get_byte(0,8);
    uint32_t val = (high<<8) | low;
    return (val*0.0001220703125-4)*9.8;
}

double Acc500::acc_up(const std::uint8_t *bytes,
                                  int32_t length) const
{
    Byte frame_low(bytes + 5);
    uint8_t low = frame_low.get_byte(0,8);
    Byte frame_high(bytes + 4);
    uint8_t high = frame_high.get_byte(0,8);
    uint32_t val = (high<<8) | low;
    return (val*0.0001220703125-4)*9.8;
}



} // namespace ctrek
} // namespace canbus
} // namespace apollo
