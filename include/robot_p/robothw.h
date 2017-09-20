#ifndef ROBOT_P_CONTROL_ROBOTHW_H
#define ROBOT_P_CONTROL_ROBOTHW_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <serial/serial.h>

#include <array>
#include <cstdint>
#include <tuple>

namespace robotp
{
class RobotHW : public hardware_interface::RobotHW
{
  struct Msg32 {
    char id;
    union {
      struct {
        int16_t pid_param;
        int16_t padding;
      };
      struct {
        int16_t left;
        int16_t right;
      };
      struct {
        char data[4];
      };
    };
    uint8_t crc;
  } __attribute__((__packed__));

public:
  /**
   * @brief RobotHW
   * @param port serial port
   * @param max_velocity in rad/s
   */
  RobotHW(ros::NodeHandle& nh_private);
  ~RobotHW();
  /**
    * Reads data from the robot HW
    *
    * \param time The current time
    * \param period The time passed since the last call to \ref read
    */
  virtual void read(const ros::Time& time, const ros::Duration& period);

  /**
   * Writes data to the robot HW
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref write
   */
  virtual void write();

private:
  const size_t LEFT = 0;
  const size_t RIGHT = 1;
  const static constexpr size_t VELOCITY_SCALE = 255;
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  serial::Serial serial_;
  double full_speed_;
  std::array<uint8_t, 10> input_buff_;
  double cmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];

  Msg32 createVelocityMsg(double left_vel, double right_vel) const;
  int radiansToPWM(double vel) const;

  template <typename T>
  void sendMsg(const T& msg)
  {
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&msg);
    size_t size = sizeof(T);
    ROS_INFO_STREAM("sending size of mesasge " << size);
    size_t c = 0;
    if ((c = serial_.write(data, size)) != size) {
      ROS_WARN_STREAM("ROBOT_P_CONTROL: sending message was not successful " << c);
    }
  }
};
}

#endif  // ROBOT_P_CONTROL_ROBOTHW_H
