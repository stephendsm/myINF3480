#include <controller_manager/controller_manager.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "crustcrawler_hardware/crustcrawler.hpp"

int main(int argc, char **argv) {
  // Initialize ROS:
  ros::init(argc, argv, "crustcrawler");
  // Initialize controller manager and arm hardware interface
  ros::NodeHandle nh("");
  ros::NodeHandle priv("~");
  crustcrawler_hardware::Crustcrawler arm(priv);
  controller_manager::ControllerManager cm(&arm);
  // Create diagnostic update which will be used to publish
  // diagnostic information
  diagnostic_updater::Updater diag;
  diag.setHardwareID("crustcrawler");
  for (const auto &id : arm.connected_servos()) {
    const auto name = "servo " + std::to_string(id);
    diag.add(name,
             [&arm, id](diagnostic_updater::DiagnosticStatusWrapper &stat) {
               arm.servo_diagnostics(stat, id);
             });
  }
  diag.add("connection", &arm,
           &crustcrawler_hardware::Crustcrawler::connection_diagnostics);
  double min_read_freq = 130;
  double max_read_freq = 150;
  diagnostic_updater::TopicDiagnostic read_freq(
      "read", diag,
      diagnostic_updater::FrequencyStatusParam(&min_read_freq, &max_read_freq),
      diagnostic_updater::TimeStampStatusParam(-1.0, 1. / 50.));
  // Force first update to get diagnostic before any read or write below
  diag.force_update();

  // This spinner will deal with callbacks from ROS, since message
  // handling can take time it is very useful to have an extra thread
  // so that we don't have to use `ros::spin_once` in controller loop
  // below
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Close-realtime control loop:
  ros::Time prev_time = ros::Time::now();
  ros::Time last_status = ros::Time::now() - ros::Duration(5.0);
  ROS_DEBUG("Starting Crustcrawler hardware loop");
  while (ros::ok()) {
    // Calculate time used
    const ros::Time time = ros::Time::now();
    const ros::Duration period = time - prev_time;
    prev_time = time;

    // Read state from Crustcrawler hardware
    arm.read(time, period);
    read_freq.tick(time);
    // Sometime read internal state
    if (time - last_status > ros::Duration(0.1)) {
      arm.readStatus(time, period);
      last_status = time;
    }
    cm.update(time, period);
    // Write back updates as needed
    arm.write(time, period);
    // Lastly read diagnostics (this is internally rate-limited)
    diag.update();
  }
  return 0;
}
