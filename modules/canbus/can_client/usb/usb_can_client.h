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
 * @brief Defines the usbCanClient class which inherites CanClient.
 */

#ifndef MODULES_CANBUS_CAN_CLIENT_CLIENT_usb_CAN_CLIENT_H_
#define MODULES_CANBUS_CAN_CLIENT_CLIENT_usb_CAN_CLIENT_H_

#include <string>
#include <vector>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include "gflags/gflags.h"
#include "modules/canbus/can_client/can_client.h"
#include "modules/canbus/common/canbus_consts.h"
#include "modules/canbus/proto/can_card_parameter.pb.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/canbus/can_client/usb/controlcan.h"

/**
 * @namespace apollo::canbus::can
 * @brief apollo::canbus::can
 */
namespace apollo {
namespace canbus {
namespace can {

/**
 * @class usbCanClient
 * @brief The class which defines a usb CAN client which inherites CanClient.
 */
class usbCanClient : public CanClient {
 public:
  /**
   * @brief Initialize the usb CAN client by specified CAN card parameters.
   * @param parameter CAN card parameters to initialize the CAN client.
   * @return If the initialization is successful.
   */
  bool Init(const CANCardParameter& parameter) override;

  /**
   * @brief Destructor
   */
  virtual ~usbCanClient() = default;

  /**
   * @brief Start the usb CAN client.
   * @return The status of the start action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Start() override;

  /**
   * @brief Stop the usb CAN client.
   */
  void Stop() override;

  /**
   * @brief Send messages
   * @param frames The messages to send.
   * @param frame_num The amount of messages to send.
   * @return The status of the sending action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Send(const std::vector<CanFrame>& frames,
                                 int32_t* const frame_num) override;

  /**
   * @brief Receive messages
   * @param frames The messages to receive.
   * @param frame_num The amount of messages to receive.
   * @return The status of the receiving action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Receive(std::vector<CanFrame>* const frames,
                                    int32_t* const frame_num) override;

  /**
   * @brief Get the error string.
   * @param status The status to get the error string.
   */
  std::string GetErrorString(const int32_t status) override;

 private:
 
  CANCardParameter::CANChannelId port_;
};

}  // namespace can
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_CANCARD_CLIENT_usb_CAN_CLIENT_H_
