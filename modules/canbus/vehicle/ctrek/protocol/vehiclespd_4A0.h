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
 * @file vehiclespd_4A0.h
 * @brief the class of vehiclespd_4A0.h (for ctrek vehicle)
 */

#ifndef MODULES_CANBUS_VEHICL_ctrek_PROTOCOL_Vehiclespd_4A0_H_
#define MODULES_CANBUS_VEHICL_ctrek_PROTOCOL_Vehiclespd_4A0_H_

#include <sys/time.h>

#include "modules/canbus/vehicle/protocol_data.h"

/**
 * @namespace apollo::canbus::ctrek
 * @brief apollo::canbus::ctrek
 */
namespace apollo {
namespace canbus {
namespace ctrek {

/**
 * @class vehiclespd4A0
 *
 * @brief one of the protocol data of ctrek vehicle
 */
class Vehiclespd4A0 : public ProtocolData  {
   public:
   static const int32_t ID;

  /*
   * @brief parse received data
   * @param bytes a pointer to the input bytes
   * @param length the length of the input bytes
   * @param chassis_detail the parsed chassis_detail
   */
   virtual void Parse(const std::uint8_t *bytes, int32_t length,
                     ChassisDetail *chassis_detail) const;

  /*
   * @brief parse received data
   * @param bytes a pointer to the input bytes
   * @param length the length of the input bytes
   * @param timestamp the timestamp of input data
   * @param chassis_detail the parsed chassis_detail
   */
   virtual void Parse(const std::uint8_t *bytes, int32_t length,
                     const struct timeval &timestamp,
                     ChassisDetail *chassis_detail) const;

 
   double vehicle_speed(const std::uint8_t *bytes, int32_t length);
   double front_left_wheel_speed(const std::uint8_t *bytes,
                                int32_t length) const;

  /**
   * @brief calculate front right wheel speed based on byte array.
   * config detail: {'name': 'fr', 'offset': 0.0, 'precision': 0.01, 'len': 16,
   * 'f_type': 'value', 'is_signed_var': True, 'physical_range': '[0|0]', 'bit':
   * 16, 'type': 'double', 'order': 'intel', 'physical_unit': '"km/h"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of front right wheel speed
   */
   double front_right_wheel_speed(const std::uint8_t *bytes,
                                 int32_t length) const;

  /**
   * @brief calculate rear left wheel speed based on byte array.
   * config detail: {'name': 'rl', 'offset': 0.0, 'precision': 0.01, 'len': 16,
   * 'f_type': 'value', 'is_signed_var': True, 'physical_range': '[0|0]', 'bit':
   * 32, 'type': 'double', 'order': 'intel', 'physical_unit': '"km/h"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of rear left wheel speed
   */
   double rear_left_wheel_speed(const std::uint8_t *bytes, int32_t length) const;

  /**
   * @brief calculate rear right wheel speed based on byte array.
   * config detail: {'name': 'rr', 'offset': 0.0, 'precision': 0.01, 'len': 16,
   * 'f_type': 'value', 'is_signed_var': True, 'physical_range': '[0|0]', 'bit':
   * 48, 'type': 'double', 'order': 'intel', 'physical_unit': '"km/h"'}
   * @param bytes a pointer to the byte array
   * @param length the length of the byte array
   * @return the value of rear right wheel speed
   */
   double rear_right_wheel_speed(const std::uint8_t *bytes,
                                int32_t length) const;

};

}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ctrek_PROTOCOL_STEERING_322_H_
