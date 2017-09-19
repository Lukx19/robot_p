#include <math.h>
#include <robot_p/robothw.h>
#include <boost/math/constants/constants.hpp>

using namespace robotp;

RobotHW::RobotHW(ros::NodeHandle &nh_private)
  : jnt_state_interface()
  , jnt_vel_interface()
  , serial_()
  , ticks_per_revolution_(258)
  , input_buff_()
  , alarm_(false)

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
  float Kp, Ki, Kd;

  nh_private.param<double>("ticks_per_revolution", ticks_per_revolution_, 258);

  nh_private.param<std::string>("serial_port", port, "/dev/ttyS0");
  nh_private.param<float>("Kp", Kp, 1);
  nh_private.param<float>("Ki", Kd, 0.3);
  nh_private.param<float>("Kd", Ki, 0.5);

  serial_.setPort(port);
  // serial_.setBaudrate(115200);
  serial_.setBaudrate(57600);
  auto timeout = serial::Timeout::simpleTimeout(serial::Timeout::max());
  serial_.setTimeout(timeout);
  try {
    serial_.open();
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("[ROBOTHW]: Serial exception: " << e.what());
    return;
  }
  // serial_.setRTS(false);
  // serial_.setDTR(false);
  sendMsg(createStartMsg());
  sendMsg(createPIDMsg('P', Kp));
  sendMsg(createPIDMsg('I', Ki));
  sendMsg(createPIDMsg('D', Kd));
}

robotp::RobotHW::~RobotHW()
{
  sendMsg(createStopMsg());
}

void RobotHW::read(const ros::Time &time, const ros::Duration &period)
{
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
  if (alarm_) {
    sendMsg(createStartMsg());
    alarm_ = false;
  } else {
    sendMsg(createVelocityMsg(cmd_[0], cmd_[1]));
  }
}

robotp::RobotHW::Msg32 robotp::RobotHW::createPIDMsg(char letter,
                                                     float data) const
{
  Msg32 msg;
  msg.id = letter;
  msg.pid_param = static_cast<uint16_t>(std::trunc(data * FLOAT_MULTIPLIER));
  msg.padding = 0;
  msg.crc = 0;
  return msg;
}

robotp::RobotHW::Msg32
robotp::RobotHW::createVelocityMsg(double left_vel, double right_vel) const
{
  Msg32 msg;
  msg.id = 'V';
  msg.crc = 0;
  // rad/s to ticks/s
  double conversion = ticks_per_revolution_ / 2 * M_PI;
  auto calcVelocity = [this, conversion](double vel) -> int16_t {
    return static_cast<int16_t>(std::round(vel * conversion)) /
           TICKS_TIME_MULTIPLIER;
  };

  msg.left = calcVelocity(left_vel);
  msg.right = calcVelocity(right_vel);
  return msg;
}

robotp::RobotHW::Msg32 robotp::RobotHW::createStartMsg() const
{
  Msg32 msg;
  msg.id = 'S';
  return msg;
}

robotp::RobotHW::Msg32 robotp::RobotHW::createStopMsg() const
{
  Msg32 msg;
  msg.id = 'E';
  return msg;
}
