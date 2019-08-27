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
#include "modules/canbus/vehicle/ctrek/protocol/brake_214.h"
#include "modules/canbus/vehicle/ctrek/protocol/gear_217.h"
#include "modules/canbus/vehicle/ctrek/protocol/gear_324.h"
#include "modules/canbus/vehicle/ctrek/protocol/steering_215.h"
#include "modules/canbus/vehicle/ctrek/protocol/brake_321.h"
#include "modules/canbus/vehicle/ctrek/protocol/steering_322.h"
#include "modules/canbus/vehicle/ctrek/protocol/throttle_216.h"
#include "modules/canbus/vehicle/ctrek/protocol/throttle_323.h"
#include "modules/canbus/vehicle/ctrek/protocol/light_218.h"
#include "modules/canbus/vehicle/ctrek/protocol/vehiclespd_4A0.h"
#include "modules/canbus/vehicle/ctrek/protocol/acc_500.h"
#include "modules/canbus/vehicle/ctrek/protocol/gyro_501.h"
#include "modules/canbus/vehicle/ctrek/protocol/heading_502.h"
#include "modules/canbus/vehicle/ctrek/protocol/height_503.h"
#include "modules/canbus/vehicle/ctrek/protocol/lonlat_504.h"
#include "modules/canbus/vehicle/ctrek/protocol/status_506.h"
#include "modules/canbus/vehicle/ctrek/protocol/vel_505.h"


namespace apollo {
namespace canbus {
namespace ctrek {

ctrekMessageManager::ctrekMessageManager() {
  // TODO(Authors): verify which one is recv/sent
  AddSendProtocolData<Brake214, true>();
  AddSendProtocolData<Throttle216, true>();
  AddSendProtocolData<Steering215, true>();
  AddSendProtocolData<Gear217, true>();
  AddSendProtocolData<Light218, true>();
  AddRecvProtocolData<Brake321, true>();
  AddRecvProtocolData<Steering322, true>();
  AddRecvProtocolData<Throttle323, true>();
  AddRecvProtocolData<Gear324, true>();
  AddRecvProtocolData<Vehiclespd4A0, true>();
  AddRecvProtocolData<Acc500, true>();
  AddRecvProtocolData<Gyro501, true>();
  AddRecvProtocolData<Heading502, true>();
  AddRecvProtocolData<Height503, true>();
  AddRecvProtocolData<Lonlat504, true>();
  AddRecvProtocolData<Status506, true>();
  AddRecvProtocolData<Vel505, true>();

}

ctrekMessageManager::~ctrekMessageManager() {}

}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo
