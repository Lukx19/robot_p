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
public:
  /**
   * @brief RobotHW
   * @param port serial port
   * @param max_velocity in rad/s
   */
  RobotHW(ros::NodeHandle& nh_private);
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
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  serial::Serial serial_;
  std::array<uint8_t, 4> input_buff_;
  double max_velocity_;
  double cmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];

  std::tuple<std::array<uint16_t, 2>, bool>
  parseData(const std::array<uint8_t, 4>& data);
  std::array<uint8_t, 3> floatTo3Bytes(float val) const;
  void sendFloat(unsigned char letter, float val);
};
}

#endif  // ROBOT_P_CONTROL_ROBOTHW_H
