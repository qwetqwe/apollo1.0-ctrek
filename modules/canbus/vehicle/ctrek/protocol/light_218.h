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
 * @file brake_214.h
 * @brief the class of Brake214 (for ctrek vehicle)
 */

#ifndef MODULES_CANBUS_VEHICLE_ctrek_PROTOCOL_LIGHT_218_H_
#define MODULES_CANBUS_VEHICLE_ctrek_PROTOCOL_LIGHT_218_H_

#include "modules/canbus/vehicle/protocol_data.h"
/**
 * @namespace apollo::canbus::ctrek
 * @brief apollo::canbus::ctrek
 */
namespace apollo {
namespace canbus {
namespace ctrek {

/**
 * @class Brake214
 *
 * @brief one of the protocol data of ctrek vehicle
 */
class Light218 : public ProtocolData {
 public:
  static const int32_t ID;
  /**
   * @brief get the data period
   * @return the value of data period
   */
  uint32_t GetPeriod() const override;

  virtual void UpdateData(uint8_t *data);

  void Reset() override;

  Light218 *set_blink(int turnlight);

  Light218 *set_warnblink(bool warnblink);

  Light218 *set_light_enable();

  Light218 *set_light_disable();

  Light218 *set_driver_override();

 private:

  void set_blink_p(uint8_t *data, uint8_t blink_cmd_);

  void set_warnblink_p(uint8_t *data, bool warnblink_cmd_);

  void set_enable_p(uint8_t *bytes, uint8_t enable_cmd_);


 private:
  uint8_t blink_cmd_ = 0;
  bool warnblink_cmd_ = false;
  uint8_t enable_cmd_ = 0;
};

}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICLE_ctrek_PROTOCOL_BRAKE_214_H_
