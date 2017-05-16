#include <robot_p/robothw.h>
#include <boost/math/constants/constants.hpp>
using namespace robotp;

RobotHW::RobotHW(ros::NodeHandle &nh_private)
  : jnt_state_interface()
  , jnt_vel_interface()
  , serial_()
  , max_velocity_(2)
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

  nh_private.param<double>("max_angular_velocity", max_velocity_, 2);
  max_velocity_ = std::abs(max_velocity_);
  nh_private.param<std::string>("serial_port", port, "/dev/ttyS0");
  nh_private.param<float>("Kp", Kp, 1);
  nh_private.param<float>("Ki", Kd, 0.3);
  nh_private.param<float>("Kd", Ki, 0.5);

  serial_.setPort(port);
  serial_.setBaudrate(115200);
  try {
    serial_.open();
  } catch (serial::SerialException e) {
    ROS_ERROR_STREAM("[ROBOTHW]: Serial exception: " << e.what());
    return;
  } catch (serial::IOException e) {
    ROS_ERROR_STREAM("[ROBOTHW]: Serial IO exception: " << e.what());
    return;
  }

  // send PID constants to robot
  // these messages are in format P/I/D 3bytes of encoded float. To decode this
  // float divide the value stored in these bytes by 1000. The data bytes are
  // send in MSB first
  sendFloat('P', Kp);
  sendFloat('I', Ki);
  sendFloat('D', Kd);
}

void RobotHW::read(const ros::Time &time, const ros::Duration &period)
{
  /* reads 4 byte long messages
   * [0-3600][0-3600]
   *    -> left and right wheel CCW position represented with 2 bytes holding
   *       values in range [0-3600] stored MSB first
   */
  if (serial_.available() > 4) {
    serial_.flushInput();
  }
  if (serial_.read(input_buff_.data(), 4) != 4) {
    ROS_ERROR_STREAM("[ROBOTHW]:unable to read 6 bytes from serial input");
    return;
  }
  auto parsed_data = parseData(input_buff_);
  if (!std::get<1>(parsed_data)) {
    return;
  }

  double conversion_rate = boost::math::constants::pi<double>() / 1800;
  for (size_t i = 0; i < 2; ++i) {
    double old_pos = pos_[i];
    pos_[i] = std::get<0>(parsed_data)[i] * conversion_rate;
    vel_[i] = (pos_[i] - old_pos) / period.toSec();
    eff_[i] = 0;
  }
}

void RobotHW::write()
{
  /* sends 2x4 bytes message
   * V L P [0-255]
   *      -> angular_velocity left_wheel CW in range from 0 to 255
   * V R N [0-255]
   *      -> angular_velocity right_wheel CCW  in range from 0 to 255
   */
  for (size_t i = 0; i < 2; ++i) {
    unsigned char dir = cmd_[i] < 0 ? 'N' : 'P';
    unsigned char wheel = i % 2 == 0 ? 'L' : 'R';
    double trunc_vel =
        std::abs(cmd_[i]) > max_velocity_ ? max_velocity_ : std::abs(cmd_[i]);
    uint8_t velocity = static_cast<uint8_t>((trunc_vel / max_velocity_) * 255);
    std::vector<uint8_t> data = {'V', wheel, dir, velocity};
    serial_.write(data);
  }
}

std::tuple<std::array<uint16_t, 2>, bool>
RobotHW::parseData(const std::array<uint8_t, 4> &data)
{
  std::array<uint16_t, 2> poses;
  for (size_t i = 0; i < 2; ++i) {
    poses[i] = (static_cast<uint16_t>(data[i * 2]) << 8) &
               static_cast<uint16_t>(data[i * 2 + 1]);
    if (poses[i] > 3600) {
      std::string wheel = i % 2 == 0 ? "Left" : "Right";
      ROS_ERROR_STREAM("[ROBOTHW]: " << wheel
                                     << "wheel pose is out of bounds [0:3600]"
                                     << poses[i]);
      return std::make_tuple(poses, false);
    }
  }
  return std::make_tuple(poses, true);
}

std::array<uint8_t, 3> robotp::RobotHW::floatTo3Bytes(float val) const
{
  std::array<uint8_t, 3> data;
  uint32_t val_2b = static_cast<uint32_t>(std::trunc(val * 1000));
  data[0] = static_cast<uint8_t>((val_2b & 0x00FF0000) >> 16);
  data[1] = static_cast<uint8_t>((val_2b & 0x0000FF00) >> 8);
  data[2] = static_cast<uint8_t>(val_2b & 0x000000FF);
  return data;
}

void robotp::RobotHW::sendFloat(unsigned char letter, float val)
{
  auto data = floatTo3Bytes(val);
  std::vector<uint8_t> data_v = {letter, data[0], data[1], data[2]};
  serial_.write(data_v);
}
