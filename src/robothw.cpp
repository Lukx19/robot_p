#include <robot_p/robothw.h>

using namespace robotp;

RobotHW::RobotHW()
  : jnt_state_interface(), jnt_vel_interface(), io_(), port_(io_)

{
  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_left("LEFT", &pos_[0],
                                                         &vel_[0], &eff_[0]);
  jnt_state_interface.registerHandle(state_handle_left);

  hardware_interface::JointStateHandle state_handle_right("RIGHT", &pos_[1],
                                                          &vel_[1], &eff_[1]);
  jnt_state_interface.registerHandle(state_handle_right);

  registerInterface(&jnt_state_interface);

  // connect and register the joint position interface
  hardware_interface::JointHandle vel_handle_left(
      jnt_state_interface.getHandle("LEFT"), &cmd_[0]);
  jnt_vel_interface.registerHandle(vel_handle_left);

  hardware_interface::JointHandle vel_handle_right(
      jnt_state_interface.getHandle("RIGHT"), &cmd_[1]);
  jnt_vel_interface.registerHandle(vel_handle_right);

  registerInterface(&jnt_vel_interface);

  port_.open("COM3");
  port_.set_option(boost::asio::serial_port_base::baud_rate(115200));
}

void RobotHW::read(const ros::Time &time, const ros::Duration &period)
{
  pos_[0] = 0;
  vel_[0] = 0;
  eff_[0] = 0;

  pos_[1] = 0;
  vel_[1] = 0;
  eff_[1] = 0;
}

void RobotHW::write(const ros::Time &time, const ros::Duration &period)
{
}
