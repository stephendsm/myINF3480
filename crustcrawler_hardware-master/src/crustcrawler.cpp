#include "crustcrawler_hardware/crustcrawler.hpp"
#include <iostream>

namespace crustcrawler_hardware {
Crustcrawler::Crustcrawler(ros::NodeHandle &robot_nh) {
  ROS_DEBUG("Initializing Crustcrawler");
  // Initialize dynamixel driver
  std::string path = robot_nh.param<std::string>("device_name", "/dev/ttyUSB0");
  int baud = robot_nh.param<int>("baud_rate", 1000000);
  bool full_arm = robot_nh.param<bool>("full_arm", false);
  _comm_stats.device = path;
  _comm_stats.baud_rate = baud;
  if (full_arm) {
    ROS_WARN("Using all joints");
    _ids = {0, 1, 2, 3, 4, 5, 6};
  } else {
    ROS_WARN("Using truncated arm");
    _ids = {0, 1, 2, 3};
  }
  ROS_DEBUG("Creating handles 'ros_control'");
  for (auto id : _ids) {
    joints[id].ctrl = dynamixel::CtrlType::None;
    // Joint 1 is a combination of ID 1 & 2, only register it once
    if (id != 2) {
      std::string name("joint_" + std::to_string((id <= 1 ? id + 1 : id)));
      hardware_interface::JointStateHandle state_handle(
          name, &(joints[id].position), &(joints[id].velocity),
          &(joints[id].effort));
      hardware_interface::JointHandle handle(state_handle,
                                             &(joints[id].command));
      joint_states_.registerHandle(state_handle);
      effort_int_.registerHandle(handle);
      velocity_int_.registerHandle(handle);
      position_int_.registerHandle(handle);
    }
  }
  ROS_DEBUG("Registering interfaces");
  registerInterface(&joint_states_);
  registerInterface(&effort_int_);
  registerInterface(&velocity_int_);
  registerInterface(&position_int_);
  ROS_DEBUG("Initializing Dynamixel driver, path: %s, baud rate: %d",
            path.c_str(), baud);
  _comm = std::make_unique<dynamixel::Comm>(path, baud);
  _status_read = _comm->make_mx_reader(_ids);
  _torque_read = _comm->make_reader(dynamixel::Addr::TorqueEnable, _ids);
  _mode_read = _comm->make_reader(dynamixel::Addr::OperatingMode, _ids);
  _hardware_error_read =
      _comm->make_reader(dynamixel::Addr::HardwareError, _ids);
  // Check that we can detect the required number of servos
  const auto res = _torque_read->read_data();
  const auto expected_size = _ids.size();
  if (res.size() != expected_size) {
    const auto scan = _comm->scan();
    for (const auto &servo : scan) {
      ROS_WARN("Found servo with ID: %u, type: %u", servo.first, servo.second);
    }
    // This is very hacky, but since `dynamixel_sdk` can't properly ping ID 0
    // we allow for one less servo to be found
    ROS_FATAL("Too few servos connected, found %zu, expected %zu", scan.size(),
              expected_size - 1);
    throw std::runtime_error("Too few servos connected, found " +
                             std::to_string(scan.size()) + ", expected " +
                             std::to_string(expected_size - 1));
  }
  // const auto res_torque = enable(true);
  // if (res_torque != dynamixel::Result::Success) {
  // ROS_FATAL_STREAM("Could not enable torque: " << res_torque);
  // throw std::runtime_error("Could not enable torque during initialization");
  //}
  ROS_INFO("Crustcrawler hardware initialized");
}

Crustcrawler::~Crustcrawler() {
  if (_comm != nullptr) {
    const auto res = enable(false);
    if (res != dynamixel::Result::Success) {
      // Use `std::cerr` here instead of ROS_* since ROS will be shutdown
      // once we get here.
      //
      // Note the strange sequence below is to get color, this is read and
      // '\033[0m' resets the colors
      std::cerr << "\033[31mFATAL: Error sending torque disable, error: \033[0m"
                << res << std::endl;
    } else {
      // Output in beautiful blue to ease users
      std::cout << "\033[34mCrustcrawler shutdown successfully \033[0m\n";
    }
  }
}

dynamixel::Result Crustcrawler::enable(bool enable) {
  if (enable) {
    ROS_WARN("Enabling torque on all Crustcrawler joints!");
  } else {
    ROS_INFO("Disabling torque");
  }
  std::vector<dynamixel::IdAddrVal> stop_update;
  for (const auto &id : _ids) {
    dynamixel::IdAddrVal stop;
    stop.id = id;
    stop.addr = dynamixel::Addr::TorqueEnable;
    stop.value = static_cast<uint8_t>(enable);
    stop_update.push_back(stop);
  }
  const auto res = _comm->bulk_write(stop_update);
  if (res != dynamixel::Result::Success) {
    ROS_WARN_STREAM("Error writing torque update: " << res);
  }
  return res;
}

void Crustcrawler::read(const ros::Time &time, const ros::Duration &period) {
  const auto status = _status_read->read_data();
  _comm_stats.state.reads += 1;
  if (status.size() != _ids.size()) {
    ROS_WARN("Not all servos replied to state read: %zu (expected: %zu)",
             status.size(), _ids.size());
    _comm_stats.state.errors += 1;
  }
  for (const auto &stat : status) {
    joints[stat.id].position = dynamixel::from_pos(stat.position);
    joints[stat.id].velocity = dynamixel::from_vel(stat.velocity);
    joints[stat.id].effort = dynamixel::from_eff(stat.current);
    joints[stat.id].pwm = dynamixel::from_pwm(stat.pwm);
    joints[stat.id].voltage = dynamixel::from_voltage(stat.input_voltage);
    joints[stat.id].temperature = dynamixel::from_temp(stat.temperature);
  }
}

void Crustcrawler::write(const ros::Time &, const ros::Duration &) {
  std::vector<dynamixel::IdAddrVal> updates;
  updates.reserve(_ids.size());
  for (auto id : _ids) {
    if (!joints[id].enabled) {
      // If the joint is not enabled ignore
      continue;
    }
    dynamixel::IdAddrVal val;
    val.id = id;
    // Special case, this ensures that ID 1 == 2
    // for commands, NOTE the `val.id = id` above
    if (id == 2) {
      id = 1;
    }
    switch (joints[id].ctrl) {
    case dynamixel::CtrlType::None:
      continue;
    case dynamixel::CtrlType::Position:
      val.addr = dynamixel::Addr::GoalPosition;
      val.value = dynamixel::to_pos(joints[id].command);
      break;
    case dynamixel::CtrlType::Velocity:
      val.addr = dynamixel::Addr::GoalVelocity;
      val.value = dynamixel::to_vel(joints[id].command);
      break;
    case dynamixel::CtrlType::Effort:
      val.addr = dynamixel::Addr::GoalCurrent;
      val.value = dynamixel::to_eff(joints[id].command);
      break;
    default:
      ROS_WARN_THROTTLE(0.5, "Unsupported operating mode: %i",
                        static_cast<uint8_t>(joints[id].ctrl));
    }
    updates.push_back(val);
  }
  if (updates.size() > 0) {
    const auto res = _comm->bulk_write(updates);
    _comm_stats.writes.reads += 1;
    if (res != dynamixel::Result::Success) {
      ROS_WARN_STREAM("Error writing command to servos, error:" << res);
      _comm_stats.writes.errors += 1;
    }
  }
}

void Crustcrawler::doSwitch(
    const std::list<hardware_interface::ControllerInfo> &start,
    const std::list<hardware_interface::ControllerInfo> &stop) {
  std::vector<dynamixel::IdAddrVal> mode_updates;
  for (auto info : start) {
    dynamixel::CtrlType new_ctrl = dynamixel::CtrlType::None;
    if (info.type == "position_controllers/JointPositionController") {
      new_ctrl = dynamixel::CtrlType::Position;
    } else if (info.type == "joint_state_controller/JointStateController") {
      // Ignore by default!
      continue;
    } else if (info.type == "velocity_controllers/JointVelocityController") {
      new_ctrl = dynamixel::CtrlType::Velocity;
    } else if (info.type == "effort_controllers/JointEffortController") {
      new_ctrl = dynamixel::CtrlType::Effort;
    } else if (info.type == "position_controllers/JointTrajectoryController") {
      new_ctrl = dynamixel::CtrlType::Position;
    }
    if (new_ctrl == dynamixel::CtrlType::None) {
      ROS_ERROR("No compatible control mode found for %s", info.type.c_str());
      continue;
    }
    for (auto claim : info.claimed_resources) {
      for (auto res : claim.resources) {
        ROS_DEBUG("Changing %s to controller: %i", res.c_str(),
                  static_cast<int>(new_ctrl));
        dynamixel::IdAddrVal msg;
        msg.addr = dynamixel::Addr::OperatingMode;
        msg.value = static_cast<uint8_t>(new_ctrl);
        if (res == "joint_1") {
          msg.id = 0;
        } else if (res == "joint_2") {
          msg.id = 1;
          // NOTE this is a special case for the double joint
          dynamixel::IdAddrVal m;
          m.addr = dynamixel::Addr::OperatingMode;
          m.value = static_cast<uint8_t>(new_ctrl);
          m.id = 2;
          mode_updates.push_back(m);
        } else if (res == "joint_3") {
          msg.id = 3;
        } else if (res == "joint_4") {
          msg.id = 4;
        } else if (res == "joint_5") {
          msg.id = 5;
        } else if (res == "joint_6") {
          msg.id = 6;
        } else {
          ROS_WARN("Unknown resource claimed: %s", res.c_str());
          continue;
        }
        if (!joints[msg.id].enabled) {
          mode_updates.push_back(msg);
        } else {
          ROS_ERROR(
              "Cannot change mode to '%i' for joint %s while it is enabled",
              static_cast<int>(new_ctrl), res.c_str());
        }
      }
    }
  }
  if (mode_updates.size() > 0) {
    const auto res = _comm->bulk_write(mode_updates);
    _comm_stats.writes.reads += 1;
    if (res != dynamixel::Result::Success) {
      ROS_WARN_STREAM("Error changing mode for servos, error: " << res);
      _comm_stats.writes.errors += 1;
    } else if (first_update) {
      // If we changed servos and this is the first update then change servo
      // mode
      first_update = false;
      const auto res_torque = enable(true);
      if (res_torque != dynamixel::Result::Success) {
        ROS_FATAL_STREAM("Could not enable torque: " << res_torque);
      }
    }
  }
}

void Crustcrawler::readStatus(const ros::Time &, const ros::Duration &) {
  const auto torque = _torque_read->read_data();
  const auto hw_error = _hardware_error_read->read_data();
  const auto mode = _mode_read->read_data();
  _comm_stats.torque.reads += 1;
  _comm_stats.hw_error.reads += 1;
  _comm_stats.mode.reads += 1;
  if (torque.size() == _ids.size()) {
    for (const auto &pair : torque) {
      joints[pair.first].enabled = static_cast<bool>(pair.second);
    }
  } else {
    ROS_ERROR("Not all servos responded to torque read!");
    _comm_stats.torque.errors += 1;
  }
  if (hw_error.size() == _ids.size()) {
    for (const auto &pair : torque) {
      joints[pair.first].err.raw = pair.second;
    }
  } else {
    ROS_ERROR("Not all servos responded to HW error read!");
    _comm_stats.hw_error.errors += 1;
  }
  if (mode.size() == _ids.size()) {
    for (const auto &pair : mode) {
      joints[pair.first].ctrl = static_cast<dynamixel::CtrlType>(pair.second);
    }
  } else {
    ROS_ERROR("Not all servos responded to Operating Mode read!");
    _comm_stats.mode.errors += 1;
  }
}

void Crustcrawler::servo_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat,
    const dynamixel::ServoID &id) {
  // Initial summary of joint
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Joint is good");
  const auto torque_str = joints[id].enabled ? "enabled" : "disabled";
  std::string mode = "none";
  switch (joints[id].ctrl) {
  case dynamixel::CtrlType::None:
    break;
  case dynamixel::CtrlType::Position:
    mode = "position";
    break;
  case dynamixel::CtrlType::Velocity:
    mode = "velocity";
    break;
  case dynamixel::CtrlType::Effort:
    mode = "effort";
    break;
  default:
    mode = "unknown";
    break;
  }
  // Add data not published other places
  stat.addf("torque", "%s", torque_str);
  stat.addf("mode", "%s", mode.c_str());
  stat.addf("pwm", "%.3f", joints[id].pwm);
  stat.addf("voltage", "%.1f", joints[id].voltage);
  stat.addf("temperature", "%.1f", joints[id].temperature);
  // Check Mode, 'none' and unknown are bad
  if (mode == "none") {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN,
                      "Control mode set to 'none'");
  } else if (mode == "unknown") {
    stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                       "Unknown control mode (%i)!",
                       static_cast<uint8_t>(joints[id].ctrl));
  }
  // Check temperature, if high we warn users
  if (joints[id].temperature > 50) {
    stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::WARN,
                       "Temperature is large %.1fC", joints[id].temperature);
  }
  if (joints[id].temperature > 65) {
    stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
                       "Temperature is very large %.1fC",
                       joints[id].temperature);
  }
  if (11.1 > joints[id].voltage || joints[id].voltage > 14.8) {
    stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::WARN,
                       "Voltage is outside operating range: %.1f [11.1, 14.8]",
                       joints[id].voltage);
  }
  // Check hardware error and update summary with higher value
  // status if necessary
  if (joints[id].err.bits.overload) {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
                      "Overload detected");
  }
  if (joints[id].err.bits.electrical_shock) {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
                      "Electrical shock detected");
  }
  if (joints[id].err.bits.encoder_error) {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
                      "Encoder error detected");
  }
  if (joints[id].err.bits.overheating) {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
                      "Overheating detected");
  }
  if (joints[id].err.bits.input_voltage) {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
                      "Input voltage error detected");
  }
}

void Crustcrawler::connection_diagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Connection stable");
  // Add static information about connection
  stat.add("device path", _comm_stats.device);
  stat.addf("baud rate", "%i", _comm_stats.baud_rate);
  // Add data about connection
  stat.addf("state", "%li (errors: %li)", _comm_stats.state.reads,
            _comm_stats.state.errors);
  stat.addf("writes", "%li (errors: %li)", _comm_stats.writes.reads,
            _comm_stats.writes.errors);
  stat.addf("torque", "%li (errors: %li)", _comm_stats.torque.reads,
            _comm_stats.torque.errors);
  stat.addf("hw_error", "%li (errors: %li)", _comm_stats.hw_error.reads,
            _comm_stats.hw_error.errors);
  stat.addf("mode", "%li (errors: %li)", _comm_stats.mode.reads,
            _comm_stats.mode.errors);
}
} // namespace crustcrawler_hardware
