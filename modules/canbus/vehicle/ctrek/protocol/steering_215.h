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
 * @file steering_215.h
 * @brief the class of Steering215 (for ctrek vehicle)
 */

#ifndef MODULES_CANBUS_VEHICL_ctrek_PROTOCOL_STEERING_215_H_
#define MODULES_CANBUS_VEHICL_ctrek_PROTOCOL_STEERING_215_H_

#include "modules/canbus/vehicle/protocol_data.h"

/**
 * @namespace apollo::canbus::ctrek
 * @brief apollo::canbus::ctrek
 */
namespace apollo {
namespace canbus {
namespace ctrek {

/**
 * @class Steering215
 *
 * @brief one of the protocol data of ctrek vehicle
 */
class Steering215 : public ProtocolData {
 public:
  static const int32_t ID;

  /**
   * @brief get the data period
   * @return the value of data period
   */
  virtual uint32_t GetPeriod() const;

  /**
   * @brief update the data
   * @param data a pointer to the data to be updated
   */
  virtual void UpdateData(uint8_t *data);

  /**
   * @brief reset the private variables
   */
  virtual void Reset();

  /**
   * @brief set steering request enable to true
   * @return a this pointer to the instance itself
   */
  Steering215 *set_enable();

  /**
   * @brief set steering request disable to true
   * @return a this pointer to the instance itself
   */
  Steering215 *set_disable();

  /**
   * @brief set steering angle
   * @return a this pointer to the instance itself
   */
  Steering215 *set_steering_angle(double angle);

  /**
   * @brief set steering angle speed
   * @return a this pointer to the instance itself
   */
  Steering215 *set_steering_angle_speed(double angle_speed);

  Steering215 *set_driver_override();
 private:

	 void set_steering_null(uint8_t *data);

  void set_steering_angle_p(uint8_t *data, double angle);

  void set_enable_p(uint8_t *bytes, uint8_t enable);

  void set_steering_angle_speed_p(uint8_t *data, double angle_speed);

  void set_watchdog_counter_p(uint8_t *data, int32_t count);

 private:
  double steering_angle_ = 0.0;
  uint8_t steering_enable_ = 0;
  double steering_angle_speed_ = 0.0;
  int32_t watchdog_counter_ = 0;
};

}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_ctrek_PROTOCOL_STEERING_215_H_
