#include <math.h>
#include <robot_p/robothw.h>
#include <boost/math/constants/constants.hpp>

using namespace robotp;

RobotHW::RobotHW(ros::NodeHandle &nh_private)
  : jnt_state_interface()
  , jnt_vel_interface()
  , serial_()
  , full_speed_(0)
  , input_buff_()

{
  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_left(
      "front_left_wheel", &pos_[LEFT], &vel_[LEFT], &eff_[LEFT]);
  jnt_state_interface.registerHandle(state_handle_left);

  hardware_interface::JointStateHandle state_handle_right(
      "front_right_wheel", &pos_[RIGHT], &vel_[RIGHT], &eff_[RIGHT]);
  jnt_state_interface.registerHandle(state_handle_right);

  registerInterface(&jnt_state_interface);

  // connect and register the joint position interface
  hardware_interface::JointHandle vel_handle_left(
      jnt_state_interface.getHandle("front_left_wheel"), &cmd_[LEFT]);
  jnt_vel_interface.registerHandle(vel_handle_left);

  hardware_interface::JointHandle vel_handle_right(
      jnt_state_interface.getHandle("front_right_wheel"), &cmd_[RIGHT]);
  jnt_vel_interface.registerHandle(vel_handle_right);

  registerInterface(&jnt_vel_interface);

  // initialize parameters
  std::string port;
  int serial_baudrate = 0;

  nh_private.param<double>("full_speed", full_speed_, 3.14);
  nh_private.param<std::string>("serial_port", port, "/dev/ttyS0");
  nh_private.param<int>("serial_baudrate", serial_baudrate, 57600);

  serial_.setPort(port);
  serial_.setBaudrate(serial_baudrate);
  auto timeout = serial::Timeout::simpleTimeout(serial::Timeout::max());
  serial_.setTimeout(timeout);
  try {
    serial_.open();
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("[ROBOTHW]: Serial exception: " << e.what());
    return;
  }
}

robotp::RobotHW::~RobotHW()
{
  sendMsg(createVelocityMsg(0., 0.));
}

void RobotHW::read(const ros::Time &time, const ros::Duration &period)
{
  // we don't read odom from controller
  if (serial_.available() > 0) {
    ROS_INFO_STREAM(
        "ROBOT_P_CONTROL: serial: " << serial_.read(serial_.available()));
  }

  for (size_t i = 0; i < 2; ++i) {
    vel_[i] = 0;  // rad/s
    pos_[i] += 0;
    eff_[i] = 0;
  }
}

void RobotHW::write()
{
  sendMsg(createVelocityMsg(cmd_[0], cmd_[1]));
}

inline static int radiansToPWM(double vel)
{
  double speed_factor = vel / full_speed;
  int pwm = (int)(speed_factor * 255);
  // ensure to stay in boundaries
  return std::max(std::min(pwm, 255), -255);
}

robotp::RobotHW::Msg32
robotp::RobotHW::createVelocityMsg(double left_vel, double right_vel) const
{
  Msg32 msg;
  msg.id = 'V';
  msg.crc = 0;
  // input speeds are in rad/s
  msg.left = radiansToPWM(left_vel);
  msg.right = radiansToPWM(right_vel);

  ROS_INFO_STREAM("ROBOT_P_CONTROL: speed: " << msg.left << " " << msg.right);
  return msg;
}
