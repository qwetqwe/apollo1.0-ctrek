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

#ifndef MODULES_CANBUS_VEHICLE_ctrek_PROTOCOL_BRAKE_214_H_
#define MODULES_CANBUS_VEHICLE_ctrek_PROTOCOL_BRAKE_214_H_

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
class Brake214 : public ProtocolData {
 public:
  static const int32_t ID;
  /**
   * @brief get the data period
   * @return the value of data period
   */
  uint32_t GetPeriod() const override;

  /**
   * @brief update the data
   * @param data a pointer to the data to be updated
   * 设置数据
   */
  void UpdateData(uint8_t *data) override;

  /**
   * @brief reset the private variables
   */
  void Reset() override;

  /**
   * @brief set pedal based on pedal command
   * @return a this pointer to the instance itself
   */
  Brake214 *set_pedal(double pcmd);

  /**
   * @brief set pedal_enable_ to true
   * @return a this pointer to the instance itself
   */
  Brake214 *set_enable();

  /**
   * @brief set pedal_enable_ to false
   * @return a this pointer to the instance itself
   */
  Brake214 *set_disable();

  Brake214 *set_driver_override();

 private:

  void set_pedal_p(uint8_t *data, double pedal);

  void set_enable_p(uint8_t *bytes, uint8_t enable);

 private:
  double pedal_cmd_ = 0.0;
  uint8_t pedal_enable_ = 0;

};

}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICLE_ctrek_PROTOCOL_BRAKE_214_H_
