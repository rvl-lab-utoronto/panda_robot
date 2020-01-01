// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>

namespace panda_robot_controllers {

class CartesianVelocityExampleController : public controller_interface::MultiInterfaceController<
                                               franka_hw::FrankaVelocityCartesianInterface,
                                               franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;
  void target_velocity_callback(const geometry_msgs::Twist::ConstPtr& msg);

 private:
  franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
  franka_hw::FrankaStateInterface* state_interface;
  std::string arm_id;

  bool was_in_contact;
  std::array<double, 6> ee_twist_cause_of_contact;
  ros::Time time_since_last_target_velocity;
  
  std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;
  ros::Duration elapsed_time_;
  
  geometry_msgs::Twist target_velocity;
  geometry_msgs::Twist current_velocity;
  
  ros::Subscriber target_velocity_subscriber;

  ros::Publisher current_velocity_publisher;
  
};

}  
