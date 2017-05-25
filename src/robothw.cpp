#include <robot_p/robothw.h>
#include <boost/math/constants/constants.hpp>
using namespace robotp;

RobotHW::RobotHW(ros::NodeHandle &nh_private)
  : jnt_state_interface()
  , jnt_vel_interface()
  , serial_()
  , max_wheel_velocity_(2)
  , ticks_per_meter_(100)
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
  float Kp, Ki, Kd;

  nh_private.param<double>("ticks_per_meter", ticks_per_meter_, 100);
  // radians/s
  nh_private.param<double>("max_wheel_velocity", max_wheel_velocity_, 2);
  max_wheel_velocity_ = std::abs(max_wheel_velocity_);

  nh_private.param<std::string>("serial_port", port, "/dev/ttyS0");
  nh_private.param<float>("Kp", Kp, 1);
  nh_private.param<float>("Ki", Kd, 0.3);
  nh_private.param<float>("Kd", Ki, 0.5);

  serial_.setPort(port);
  serial_.setBaudrate(57600);
  try {
    serial_.open();
  } catch (serial::SerialException e) {
    ROS_ERROR_STREAM("[ROBOTHW]: Serial exception: " << e.what());
    return;
  } catch (serial::IOException e) {
    ROS_ERROR_STREAM("ROBOT_P_CONTROL: Serial IO exception: " << e.what());
    return;
  }

  sendMsg(createPIDMsg('P', Kp));
  sendMsg(createPIDMsg('I', Ki));
  sendMsg(createPIDMsg('D', Kd));
}

void RobotHW::read(const ros::Time &time, const ros::Duration &period)
{
  size_t message_count = serial_.available() / 10;
  std::array<double, 2> ticks = {0, 0};
  for (size_t i = 0; i < message_count; ++i) {
    if (serial_.read(input_buff_.data(), 10) != 10) {
      ROS_ERROR_STREAM(
          "ROBOT_P_CONTROL:unable to read 10 byte message from serial "
          "input");
      return;
    }
    InMsg *msg = reinterpret_cast<InMsg *>(input_buff_.data());
    switch (msg->id) {
      case 'T':
        ticks[0] += msg->left;
        ticks[1] += msg->right;
        break;
      default:
        ROS_ERROR_STREAM(
            "ROBOT_P_CONTROL: Unknown input message type: " << msg->id);
        break;
    }
  }

  for (size_t i = 0; i < 2; ++i) {
    pos_[i] = 0;
    vel_[i] = (ticks[i] / ticks_per_meter_) / period.toSec();  // m/s
    eff_[i] = 0;
  }
}

void RobotHW::write()
{
  sendMsg(createVelocityMsg(cmd_[0], cmd_[1]));
}

robotp::RobotHW::OutMsg robotp::RobotHW::createPIDMsg(char letter,
                                                      float data) const
{
  OutMsg msg;
  msg.id = letter;
  msg.param = static_cast<uint32_t>(std::trunc(data * 100000));
  msg.crc = 0;
  return msg;
}

robotp::RobotHW::OutMsg
robotp::RobotHW::createVelocityMsg(double left_vel, double right_vel) const
{
  OutMsg msg;
  msg.id = 'V';
  msg.crc = 0;
  auto calcVelocity = [this](double vel) -> int16_t {
    double trunc_vel = std::abs(vel) > max_wheel_velocity_ ?
                           max_wheel_velocity_ :
                           std::abs(vel);
    int16_t velocity = static_cast<int16_t>((trunc_vel / max_wheel_velocity_) *
                                            VELOCITY_SCALE);
    return vel < 0 ? velocity * -1 : velocity;
  };

  msg.left = calcVelocity(left_vel);
  msg.right = calcVelocity(right_vel);
  return msg;
}
