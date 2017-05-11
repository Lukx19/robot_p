#ifndef ROBOT_P_CONTROL_ROBOTHW_H
#define ROBOT_P_CONTROL_ROBOTHW_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

namespace robotp
{
class RobotHW : public hardware_interface::RobotHW
{
public:
  RobotHW();
  /**
    * Reads data from the robot HW
    *
    * \param time The current time
    * \param period The time passed since the last call to \ref read
    */
  virtual void read(const ros::Time& time,
                    const ros::Duration& period) override;

  /**
   * Writes data to the robot HW
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref write
   */
  virtual void write(const ros::Time& time,
                     const ros::Duration& period) override;

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  boost::asio::io_service io_;
  boost::asio::serial_port port_;
  double cmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];
};
}

#endif  // ROBOT_P_CONTROL_ROBOTHW_H
