#include <realtime_tools/realtime_buffer.h>
#include <robot_controller/robot_controller.h>

namespace robot_controller {
RobotController::RobotController() : command_struct_(){};

bool RobotController::init(hardware_interface::VelocityJointInterface *hw,
                           ros::NodeHandle &root_nh,
                           ros::NodeHandle &controller_nh) {

  std::string leftWheel;
  std::string rightWheel;
  controller_nh.getParam("left_wheel", leftWheel);
  controller_nh.getParam("right_wheel", rightWheel);
  left_wheel_joint = hw->getHandle(leftWheel);
  right_wheel_joint = hw->getHandle(rightWheel);
  sub_cmd = controller_nh.subscribe("cmd_vel", 1,
                                    &RobotController::cmdvelCallback, this);
  return true;
}

void RobotController::update(const ros::Time &time,
                             const ros::Duration &period) {
  ROS_INFO_STREAM(left_wheel_joint.getPosition());
}

void RobotController::starting(const ros::Time &time) {}

void RobotController::stopping(const ros::Time &time) {}

void RobotController::cmdvelCallback(const geometry_msgs::Twist &command) {
  if (isRunning()) {
    command_struct_.lin = command.linear.x;
    command_struct_.ang = command.angular.z;
    _command.writeFromNonRT(command_struct_);
  }
}

} // namespace robot_controller

PLUGINLIB_EXPORT_CLASS(robot_controller::RobotController,
                       controller_interface::ControllerBase);