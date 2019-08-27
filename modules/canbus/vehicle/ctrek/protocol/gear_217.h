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
 * @file gear_217.h
 * @brief the class of Gear217 (for ctrek vehicle)
 */

#ifndef MODULES_CANBUS_VEHICLE_ctrek_PROTOCOL_GEAR_217_H_
#define MODULES_CANBUS_VEHICLE_ctrek_PROTOCOL_GEAR_217_H_

#include "modules/canbus/vehicle/protocol_data.h"

/**
 * @namespace apollo::canbus::ctrek
 * @brief apollo::canbus::ctrek
 */
namespace apollo {
namespace canbus {
namespace ctrek {

/**
 * @class Gear217
 *
 * @brief one of the protocol data of ctrek vehicle
 */
class Gear217 : public ProtocolData {
 public:
  static const int32_t ID;

  Gear217 *set_enable();

  Gear217 *set_disable();

  virtual uint32_t GetPeriod() const;

  virtual void UpdateData(uint8_t *data);

  virtual void Reset();

  Gear217 *set_gear_none();

  Gear217 *set_gear_park();

  Gear217 *set_gear_reverse();

  Gear217 *set_gear_neutral();

  Gear217 *set_gear_drive();

  Gear217 *set_gear_low();

  Gear217 *set_driver_override();

 private:

  void set_gear_p(uint8_t *data, int32_t gear);

  void set_enable_p(uint8_t *bytes, uint8_t enable);

 private:
  int32_t gear_ = 0;
  uint8_t gear_enable_ = false;
  bool update_ = false;
};

}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICLE_ctrek_PROTOCOL_GEAR_217_H_
