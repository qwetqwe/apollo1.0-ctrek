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

#include "modules/canbus/vehicle/ctrek/ctrek_controller.h"

#include "modules/common/log.h"

#include "modules/canbus/can_comm/can_sender.h"
#include "modules/canbus/vehicle/ctrek/ctrek_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/time/time.h"

namespace apollo {
namespace canbus {
namespace ctrek {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;
using apollo::localization::LocalizationEstimate;
namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}

ErrorCode ctrekController::Init(const VehicleParameter& params,
                                  CanSender* const can_sender,
                                  MessageManager* const message_manager) {
  if (is_initialized_) {
    AINFO << "ctrekController has already been initiated.";
    return ErrorCode::CANBUS_ERROR;
  }

  params_.CopyFrom(params);
  if (!params_.has_driving_mode()) {
    AERROR << "Vehicle conf pb not set driving_mode.";
    return ErrorCode::CANBUS_ERROR;
  }

  if (can_sender == nullptr) {
    return ErrorCode::CANBUS_ERROR;
  }
  can_sender_ = can_sender;

  if (message_manager == nullptr) {
    AERROR << "protocol manager is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  message_manager_ = message_manager;

  // sender part

  brake_214_ = dynamic_cast<Brake214*>(
      message_manager_->GetMutableProtocolDataById(Brake214::ID));
  if (brake_214_ == nullptr) {
    AERROR << "Brake214 does not exist in the ctrekMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  throttle_216_ = dynamic_cast<Throttle216*>(
      message_manager_->GetMutableProtocolDataById(Throttle216::ID));
  if (throttle_216_ == nullptr) {
    AERROR << "Throttle216 does not exist in the ctrekMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  steering_215_ = dynamic_cast<Steering215*>(
      message_manager_->GetMutableProtocolDataById(Steering215::ID));
  if (steering_215_ == nullptr) {
    AERROR << "Steering215 does not exist in the ctrekMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  gear_217_ = dynamic_cast<Gear217*>(
      message_manager_->GetMutableProtocolDataById(Gear217::ID));
  if (gear_217_ == nullptr) {
    AERROR << "Gear217 does not exist in the ctrekMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }
  light_218_ = dynamic_cast<Light218*>(
      message_manager_->GetMutableProtocolDataById(Light218::ID));
  if (light_218_ == nullptr) {
    AERROR << "Light218 does not exist in the ctrekMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Brake214::ID, brake_214_, false);
  can_sender_->AddMessage(Throttle216::ID, throttle_216_, false);
  can_sender_->AddMessage(Steering215::ID, steering_215_, false);
  can_sender_->AddMessage(Gear217::ID, gear_217_, false);
  can_sender_->AddMessage(Light218::ID, light_218_, false);

  // need sleep to ensure all messages received
  AINFO << "Controller is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

bool ctrekController::Start() {
  if (!is_initialized_) {
    AERROR << "ctrekController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void ctrekController::Stop() {
  if (!is_initialized_) {
    AERROR << "ctrekController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "ctrekController stopped.";
  }
}

Chassis ctrekController::chassis() {
  chassis_.Clear();

  ChassisDetail chassis_detail;
  message_manager_->GetChassisDetail(&chassis_detail);

  // 21, 22, previously 1, 2
  if (driving_mode() == Chassis::EMERGENCY_MODE) {
    set_chassis_error_code(Chassis::NO_ERROR);
  }

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3
  chassis_.set_engine_started(true);
  // 4
  if (chassis_detail.ems().has_engine_rpm()) {
    chassis_.set_engine_rpm(chassis_detail.ems().engine_rpm());
  } else {
    chassis_.set_engine_rpm(0);
  }
  // 5
  if (chassis_detail.vehicle_spd().has_vehicle_spd()) {
    chassis_.set_speed_mps(chassis_detail.vehicle_spd().vehicle_spd());
  } else {
    chassis_.set_speed_mps(0);
  }
  // 6
  chassis_.set_odometer_m(0);
  // 7
  // ctrek only has fuel percentage
  // to avoid confusing, just don't set
  chassis_.set_fuel_range_m(0);
  // 8
  if (chassis_detail.gas().has_throttle_output()) {
    chassis_.set_throttle_percentage(chassis_detail.gas().throttle_output());
  } else {
    chassis_.set_throttle_percentage(0);
  }
  // 9
  if (chassis_detail.brake().has_brake_output()) {
    chassis_.set_brake_percentage(chassis_detail.brake().brake_output());
  } else {
    chassis_.set_brake_percentage(0);
  }
  // 23, previously 10
  if (chassis_detail.gear().has_gear_state()) {
    chassis_.set_gear_location(chassis_detail.gear().gear_state());
  } else {
    chassis_.set_gear_location(Chassis::GEAR_NONE);
  }
  // 11
  if (chassis_detail.eps().has_steering_angle()) {
    chassis_.set_steering_percentage(chassis_detail.eps().steering_angle() *
                                     100.0 / params_.max_steer_angle());
  } else {
    AINFO<<"NO STEERING CHASSISDETAIL";
    chassis_.set_steering_percentage(0);
  }
  // 12
  if (chassis_detail.eps().has_epas_torque()) {
    chassis_.set_steering_torque_nm(chassis_detail.eps().epas_torque());
  } else {
    chassis_.set_steering_torque_nm(0);
  }
  // 13
  if (chassis_detail.epb().has_parking_brake_status()) {
    chassis_.set_parking_brake(chassis_detail.epb().parking_brake_status() ==
                               Epb::PBRAKE_ON);
  } else {
    chassis_.set_parking_brake(false);
  }
  // TODO: ctrek beam
  // 14, 15

  // 16, 17
  if (chassis_detail.light().has_turn_light_type() &&
      chassis_detail.light().turn_light_type() != Light::TURN_LIGHT_OFF) {
    if (chassis_detail.light().turn_light_type() == Light::TURN_LEFT_ON) {
      chassis_.mutable_signal()->set_turn_signal(Signal::TURN_LEFT);
    } else if (chassis_detail.light().turn_light_type() ==
               Light::TURN_RIGHT_ON) {
      chassis_.mutable_signal()->set_turn_signal(Signal::TURN_RIGHT);
    } else {
      chassis_.mutable_signal()->set_turn_signal(Signal::TURN_NONE);
    }
  } else {
    chassis_.mutable_signal()->set_turn_signal(Signal::TURN_NONE);
  }
  // 18
  if (chassis_detail.light().has_is_horn_on() &&
      chassis_detail.light().is_horn_on()) {
    chassis_.mutable_signal()->set_horn(true);
  } else {
    chassis_.mutable_signal()->set_horn(false);
  }

  // 19, ctrek wiper is too complicated
  // 24
  if (chassis_detail.eps().has_timestamp_65()) {
    chassis_.set_steering_timestamp(chassis_detail.eps().timestamp_65());
  }
  // 26
  if (chassis_error_mask_) {
    chassis_.set_chassis_error_mask(chassis_error_mask_);
  }

  return chassis_;
}
LocalizationEstimate ctrekController::localization()
{
  LocalizationEstimate localization;
  message_manager_->GetLocalization(&localization);
  return localization;
}
uint8_t ctrekController::getstatus()
{
  uint8_t status;
  message_manager_->GetStatus(&status);
  return status;
}
void ctrekController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
  clear_driver_override();
  set_chassis_error_code(Chassis::CHASSIS_ERROR);
}

ErrorCode ctrekController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  brake_214_->set_enable();
  throttle_216_->set_enable();
  steering_215_->set_enable();
  gear_217_->set_enable();
  light_218_->set_light_enable();
  can_sender_->Update();
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG;
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
    Emergency();
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
    AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
    return ErrorCode::OK;
  }
}

ErrorCode ctrekController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode ctrekController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode";
    return ErrorCode::OK;
  }
  brake_214_->set_disable();
  throttle_216_->set_disable();
  steering_215_->set_enable();
  gear_217_->set_disable();
  light_218_->set_light_enable();
  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Switch to AUTO_STEER_ONLY mode ok.";
    return ErrorCode::OK;
  }
}

ErrorCode ctrekController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  brake_214_->set_enable();
  throttle_216_->set_enable();
  steering_215_->set_disable();
  gear_217_->set_enable();
  light_218_->set_light_enable();
  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Switch to AUTO_SPEED_ONLY mode ok.";
    return ErrorCode::OK;
  }
}

// NEUTRAL, REVERSE, DRIVE
void ctrekController::Gear(Chassis::GearPosition gear_position) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "this drive mode no need to set gear.";
    return;
  }
  // enable steering to enable shifting
  // actually, if we wanna shift from parking
  // to some other state
  // we need to apply a brake
  // which needs to be done by human or
  // some canbus cmd
  // TODO
  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      gear_217_->set_gear_neutral();
      break;
    }
    case Chassis::GEAR_REVERSE: {
      gear_217_->set_gear_reverse();
      break;
    }
    case Chassis::GEAR_DRIVE: {
      gear_217_->set_gear_drive();
      break;
    }
    case Chassis::GEAR_PARKING: {
      gear_217_->set_gear_park();
      break;
    }
    case Chassis::GEAR_LOW: {
      gear_217_->set_gear_low();
      break;
    }
    case Chassis::GEAR_NONE: {
      gear_217_->set_gear_none();
      break;
    }
    case Chassis::GEAR_INVALID: {
      AERROR << "Gear command is invalid!";
      gear_217_->set_gear_none();
      break;
    }
    default: {
      gear_217_->set_gear_none();
      break;
    }
  }
}

// brake with new acceleration
// acceleration:0.00~99.99, unit:%
// acceleration:0.0 ~ 7.0, unit:m/s^2
// acceleration_spd:60 ~ 100, suggest: 90
// -> pedal
void ctrekController::Brake(double pedal) {
  // double real_value = params_.max_acc() * acceleration / 100;
  // TODO

  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  brake_214_->set_pedal(pedal);
}

// drive with old acceleration
// gas:0.00~99.99 unit:%
void ctrekController::Throttle(double pedal) {

  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  throttle_216_->set_pedal(pedal);
}

// ctrek default, -470 ~ 470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:%, left:-, right:+
void ctrekController::Steer(double angle) {
 

  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle = params_.max_steer_angle() * angle / 100.0;
  // reverse sign
  steering_215_->set_steering_angle(real_angle)->set_steering_angle_speed(200);
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:%, left:-, right:+
// angle_spd:0.00~99.99, unit:deg/s
void ctrekController::Steer(double angle, double angle_spd) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle = params_.max_steer_angle() * angle / 100.0;
  const double real_angle_spd = ProtocolData::BoundedValue(
      params_.min_steer_angle_spd(), params_.max_steer_angle_spd(),
      params_.max_steer_angle_spd() * angle_spd / 100.0);
  steering_215_->set_steering_angle(real_angle)
      ->set_steering_angle_speed(real_angle_spd);
}

void ctrekController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    // None
  } else {
    // None
  }
}

void ctrekController::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    // None
  } else if (command.signal().low_beam()) {
    // None
  } else {
    // None
  }
}

void ctrekController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    // None
  } else {
    // None
  }
}

void ctrekController::SetTurningSignal(const ControlCommand& command) {
  // Set Turn Signal
  auto signal = command.signal().turn_signal();
  if (signal == Signal::TURN_LEFT) {
    light_218_->set_blink(1);
  } else if (signal == Signal::TURN_RIGHT) {
    light_218_->set_blink(2);
  } else {
    light_218_->set_blink(0);
  }
  if (command.signal().has_emergency_light())
  {
   auto emergency_light = command.signal().emergency_light();
   light_218_->set_warnblink(emergency_light);
  }
  
}

void ctrekController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool ctrekController::CheckChassisError() {
  // steer fault
  ChassisDetail chassis_detail;
  message_manager_->GetChassisDetail(&chassis_detail);

  int32_t error_cnt = 0;
  int32_t chassis_error_mask = 0;
  bool steer_fault = chassis_detail.eps().watchdog_fault() |
                     chassis_detail.eps().channel_1_fault() |
                     chassis_detail.eps().channel_2_fault() |
                     chassis_detail.eps().calibration_fault() |
                     chassis_detail.eps().connector_fault();

  chassis_error_mask |=
      ((chassis_detail.eps().watchdog_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.eps().channel_1_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.eps().channel_2_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.eps().calibration_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.eps().connector_fault()) << (error_cnt++));

  // brake fault
  bool brake_fault = chassis_detail.brake().watchdog_fault() |
                     chassis_detail.brake().channel_1_fault() |
                     chassis_detail.brake().channel_2_fault() |
                     chassis_detail.brake().boo_fault() |
                     chassis_detail.brake().connector_fault();

  chassis_error_mask |=
      ((chassis_detail.brake().watchdog_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.brake().channel_1_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.brake().channel_2_fault()) << (error_cnt++));
  chassis_error_mask |= ((chassis_detail.brake().boo_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.brake().connector_fault()) << (error_cnt++));

  // throttle fault
  bool throttle_fault = chassis_detail.gas().watchdog_fault() |
                        chassis_detail.gas().channel_1_fault() |
                        chassis_detail.gas().channel_2_fault() |
                        chassis_detail.gas().connector_fault();

  chassis_error_mask |=
      ((chassis_detail.gas().watchdog_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.gas().channel_1_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.gas().channel_2_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.gas().connector_fault()) << (error_cnt++));

  // gear fault
  bool gear_fault = chassis_detail.gear().canbus_fault();

  chassis_error_mask |=
      ((chassis_detail.gear().canbus_fault()) << (error_cnt++));

  set_chassis_error_mask(chassis_error_mask);

  if (steer_fault) {
    AERROR << "Steering fault detected: "
           << chassis_detail.eps().watchdog_fault() << ", "
           << chassis_detail.eps().channel_1_fault() << ", "
           << chassis_detail.eps().channel_2_fault() << ", "
           << chassis_detail.eps().calibration_fault() << ", "
           << chassis_detail.eps().connector_fault();
  }

  if (brake_fault) {
    AERROR << "Brake fault detected: "
           << chassis_detail.brake().watchdog_fault() << ", "
           << chassis_detail.brake().channel_1_fault() << ", "
           << chassis_detail.brake().channel_2_fault() << ", "
           << chassis_detail.brake().boo_fault() << ", "
           << chassis_detail.brake().connector_fault();
  }

  if (throttle_fault) {
    AERROR << "Throttle fault detected: "
           << chassis_detail.gas().watchdog_fault() << ", "
           << chassis_detail.gas().channel_1_fault() << ", "
           << chassis_detail.gas().channel_2_fault() << ", "
           << chassis_detail.gas().connector_fault();
  }

  if (gear_fault) {
    AERROR << "Gear fault detected: " << chassis_detail.gear().canbus_fault();
  }

  if (steer_fault || brake_fault || throttle_fault) {
    return true;
  }

  return false;
}

void ctrekController::SecurityDogThreadFunc() {
  if (can_sender_ == nullptr) {
    AERROR << "Fail to run SecurityDogThreadFunc() because can_sender_ is "
              "nullptr.";
    return;
  }
  while (!can_sender_->IsRunning()) {
    std::this_thread::yield();
  }

  std::chrono::duration<double, std::micro> default_period{50000};
  int64_t start =
      ::apollo::common::time::AsInt64<::apollo::common::time::micros>(
          ::apollo::common::time::Clock::Now());

  int32_t speed_ctrl_fail = 0;
  int32_t steer_ctrl_fail = 0;

  while (can_sender_->IsRunning()) {
    const Chassis::DrivingMode mode = driving_mode();
    bool emergency_mode = false;

    // 1. steer control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false) == false) {
      ++steer_ctrl_fail;
      if (steer_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      steer_ctrl_fail = 0;
    }

    // 2. speed control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_SPEED_ONLY) &&
        CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, false) == false) {
      ++speed_ctrl_fail;
      if (speed_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      speed_ctrl_fail = 0;
    }
    if (CheckChassisError()) {
      set_chassis_error_code(Chassis::CHASSIS_ERROR);
      emergency_mode = true;
    }

    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      Emergency();
    }
    int64_t end =
        ::apollo::common::time::AsInt64<::apollo::common::time::micros>(
            ::apollo::common::time::Clock::Now());
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
      start = ::apollo::common::time::AsInt64<::apollo::common::time::micros>(
            ::apollo::common::time::Clock::Now());
    } else {
      AERROR
          << "Too much time consumption in ctrekController looping process:"
          << elapsed.count();
      start = end;
    }
  }
}

bool ctrekController::CheckResponse(const int32_t flags, bool need_wait) {
  // for ctrek, CheckResponse commonly takes 300ms. We leave a 100ms buffer
  // for it.
  int32_t retry_num = 20;
  ChassisDetail chassis_detail;
  bool is_eps_online = false;
  bool is_vcu_online = false;
  bool is_esp_online = false;
  bool is_gear_online = false;
  bool check_ok = false;

  do {
    if (message_manager_->GetChassisDetail(&chassis_detail) != ErrorCode::OK) {
      AERROR << "get chassis detail failed.";
      return false;
    }
    check_ok = true;
    if (flags & CHECK_RESPONSE_STEER_UNIT_FLAG) {
      is_eps_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_eps_online() &&
                      chassis_detail.check_response().is_eps_online();
      check_ok = check_ok && is_eps_online;
    }

    if (flags & CHECK_RESPONSE_SPEED_UNIT_FLAG) {
      is_vcu_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_vcu_online() &&
                      chassis_detail.check_response().is_vcu_online();
      is_esp_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_esp_online() &&
                      chassis_detail.check_response().is_esp_online();
      is_gear_online= chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_gear_online() &&
                      chassis_detail.check_response().is_gear_online();
      check_ok = check_ok && is_vcu_online && is_esp_online && is_gear_online;
    }
    if (check_ok) {
      return true;
    } else {
      AINFO << "Need to check response again.";
    }
    if (need_wait) {
      --retry_num;
      std::this_thread::sleep_for(
          std::chrono::duration<double, std::milli>(20));
    }
  } while (need_wait && retry_num);

  AINFO << "check_response fail: is_eps_online:" << is_eps_online
        << ", is_vcu_online:" << is_vcu_online
        << ", is_esp_online:" << is_esp_online
        << ", is_gear_online:"<<is_gear_online;
  return false;
}

void ctrekController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t ctrekController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode ctrekController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void ctrekController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}
void ctrekController::clear_driver_override()
{
  gear_217_->set_driver_override();
  brake_214_->set_driver_override();
  light_218_->set_driver_override();
  steering_215_->set_driver_override();
}

}  // namespace ctrek
}  // namespace canbus
}  // namespace apollo
