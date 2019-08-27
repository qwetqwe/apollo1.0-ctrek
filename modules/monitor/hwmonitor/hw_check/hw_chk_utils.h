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

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "modules/hmi/utils/hmi_status_helper.h"
#include "modules/monitor/common/interface/hw_checker.h"
#include "modules/monitor/common/interface/hw_status.h"

namespace apollo {
namespace platform {
namespace hw {

void set_hmi_status(apollo::hmi::HardwareStatus *hs,
                    const std::string &name, int status,
                    const std::string &mssg) {
  hs->set_name(name);
  hs->set_status(status);
  hs->set_message(mssg);
}

void hw_chk_result_to_hmi_status(
    const std::vector<apollo::platform::HwCheckResult> &rslt,
    std::vector<apollo::hmi::HardwareStatus> *v_hs) {
  for (const auto &el : rslt) {
    apollo::hmi::HardwareStatus hs;
    set_hmi_status(&hs, el.name, el.status, el.mssg);
    v_hs->emplace_back(std::move(hs));
  }
}

}  // namespace hw
}  // namespace platform
}  // namespace apollo
