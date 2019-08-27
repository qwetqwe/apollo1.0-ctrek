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

#include "modules/canbus/vehicle/ctrek/protocol/status_506.h"
#include "modules/canbus/common/byte.h"

namespace apollo
{
namespace canbus
{
namespace ctrek
{

const int32_t Status506::ID = 0x506;

void Status506::Parse(const std::uint8_t *bytes, int32_t length,
                     uint8_t *status) const
{
    //ENU 
    Byte frame1(bytes + 6);
    uint8_t t1 = frame1.get_byte(0,8);
    *status = t1;
}

} // namespace ctrek
} // namespace canbus
} // namespace apollo
