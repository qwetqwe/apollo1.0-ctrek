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

/**
 * @file
 */

#ifndef MODULES_CANBUS_CANBUS_H_
#define MODULES_CANBUS_CANBUS_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "third_party/ros/include/ros/ros.h"

#include "modules/canbus/can_client/can_client.h"
#include "modules/canbus/can_comm/can_receiver.h"
#include "modules/canbus/can_comm/can_sender.h"
#include "modules/canbus/proto/can_card_parameter.pb.h"
#include "modules/canbus/vehicle/message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/common/monitor/monitor.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/hmi/utils/hmi_status_helper.h"
#include <cmath>
#include <proj_api.h>
#include "modules/localization/proto/localization.pb.h"
/**
 * @namespace apollo::canbus
 * @brief apollo::canbus
 */
namespace apollo {
namespace canbus {

/**
* @class Canbus
*
* @brief canbus module main class.
* It processes the control data to send protocol messages to can card.
*/
class Canbus : public apollo::common::ApolloApp {
 public:
  Canbus() : monitor_(apollo::common::monitor::MonitorMessageItem::CANBUS) {}

  /**
  * @brief obtain module name
  * @return module name
  */
  std::string Name() const override;

  /**
  * @brief module initialization function
  * @return initialization status
  */
  apollo::common::Status Init() override;

  /**
  * @brief module start function
  * @return start status
  */
  apollo::common::Status Start() override;

  /**
  * @brief module stop function
  */
  void Stop() override;

 private:
  void PublishChassis();
  void PublishLocalization();
  void PublishChassisDetail();
  void OnTimer(const ros::TimerEvent& event);
  void OnControlCommand(const apollo::control::ControlCommand& control_command);
  apollo::common::Status OnError(const std::string& error_msg);
  void RegisterCanClients();
  void CheckStatus();
  CanbusConf canbus_conf_;
  std::unique_ptr<CanClient> can_client_;
  CanSender can_sender_;
  CanReceiver can_receiver_;
  std::unique_ptr<MessageManager> message_manager_;
  std::unique_ptr<VehicleController> vehicle_controller_;

  int64_t last_timestamp_ = 0;
  ros::Timer timer_;
  apollo::common::monitor::Monitor monitor_;
  projPJ _wgs84pj_source;
  projPJ _utm_target;
};

}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_CANBUS_H_
