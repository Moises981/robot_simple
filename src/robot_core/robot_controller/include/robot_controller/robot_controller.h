#pragma once

#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace robot_controller {
class RobotController : public controller_interface::Controller<
                            hardware_interface::VelocityJointInterface> {
public:
  RobotController();

  bool init(hardware_interface::VelocityJointInterface *hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);

  void update(const ros::Time &time, const ros::Duration &period);

  void starting(const ros::Time &time);

  void stopping(const ros::Time &time);

private:
  // Joints handles
  hardware_interface::JointHandle left_wheel_joint;
  hardware_interface::JointHandle right_wheel_joint;

  // Topics
  ros::Subscriber sub_cmd;

  // Commands CmdVel
  struct Commands {
    double lin;
    double ang;
    ros::Time stamp;

    Commands() : lin(0.0), ang(0.0), stamp(0.0) {}
  };
  Commands command_struct_;

  // Realtime
  realtime_tools::RealtimeBuffer<Commands> _command;

public:
  void cmdvelCallback(const geometry_msgs::Twist &command);
};
} // namespace robot_controller
