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

#include "modules/canbus/vehicle/ctrek/protocol/vel_505.h"
#include "modules/canbus/common/byte.h"

namespace apollo
{
namespace canbus
{
namespace ctrek
{
using apollo::localization::LocalizationEstimate;
const int32_t Vel505::ID = 0x505;

void Vel505::Parse(const std::uint8_t *bytes, int32_t length,
                     LocalizationEstimate *localization_) const
{
    //ENU 
    localization_->mutable_pose()->mutable_linear_velocity()->set_x(east(bytes,length));
    localization_->mutable_pose()->mutable_linear_velocity()->set_y(north(bytes,length));
    localization_->mutable_pose()->mutable_linear_velocity()->set_z(-ground(bytes,length));
}



double Vel505::north(const std::uint8_t *bytes,
                                  int32_t length) const
{
    Byte frame1(bytes + 1);
    uint8_t t1 = frame1.get_byte(0,8);
    Byte frame2(bytes + 0);
    uint8_t t2 = frame2.get_byte(0,8);
    uint32_t val = (t2<<8) | t1;
    return val*0.0030517-100;
}

double Vel505::east(const std::uint8_t *bytes,
                                  int32_t length) const
{
    Byte frame1(bytes + 3);
    uint8_t t1 = frame1.get_byte(0,8);
    Byte frame2(bytes + 2);
    uint8_t t2 = frame2.get_byte(0,8);
    uint32_t val = (t2<<8) | t1;
    return val*0.0030517-100;
}

double Vel505::ground(const std::uint8_t *bytes,
                                  int32_t length) const
{
    Byte frame1(bytes + 5);
    uint8_t t1 = frame1.get_byte(0,8);
    Byte frame2(bytes + 4);
    uint8_t t2 = frame2.get_byte(0,8);
    uint32_t val = (t2<<8) | t1;
    return val*0.0030517-100;
}





} // namespace ctrek
} // namespace canbus
} // namespace apollo
