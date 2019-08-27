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

#include "modules/canbus/vehicle/ctrek/protocol/height_503.h"
#include "modules/canbus/common/byte.h"

namespace apollo
{
namespace canbus
{
namespace ctrek
{
using apollo::localization::LocalizationEstimate;
const int32_t Height503::ID = 0x503;

void Height503::Parse(const std::uint8_t *bytes, int32_t length,
                     LocalizationEstimate *localization_) const
{
    //ENU 
    localization_->mutable_pose()->mutable_position()->set_z(height(bytes,length));
    localization_->set_measurement_time(time(bytes,length));
}



double Height503::height(const std::uint8_t *bytes,
                                  int32_t length) const
{
    Byte frame1(bytes + 3);
    uint8_t t1 = frame1.get_byte(0,8);
    Byte frame2(bytes + 2);
    uint8_t t2 = frame2.get_byte(0,8);
    Byte frame3(bytes + 1);
    uint8_t t3 = frame3.get_byte(0,8);
    Byte frame4(bytes + 0);
    uint8_t t4 = frame4.get_byte(0,8);
    uint32_t val = (((t4<<8) | t3 )<<8 | t2)<<8 | t1;
    return val*0.001-10000;
}

double Height503::time(const std::uint8_t *bytes,
                                  int32_t length) const
{
    Byte frame1(bytes + 7);
    uint8_t t1 = frame1.get_byte(0,8);
    Byte frame2(bytes + 6);
    uint8_t t2 = frame2.get_byte(0,8);
    Byte frame3(bytes + 5);
    uint8_t t3 = frame3.get_byte(0,8);
    Byte frame4(bytes + 4);
    uint8_t t4 = frame4.get_byte(0,8);
    uint32_t val = (((t4<<8) | t3 )<<8 | t2)<<8 | t1;
    return val/1000;
}


} // namespace ctrek
} // namespace canbus
} // namespace apollo
