#include <controller_manager/controller_manager.h>
#include <robot_p/robothw.h>
#include <ros/ros.h>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>

typedef boost::chrono::steady_clock time_source;

void controlThread(ros::NodeHandle* controller_nh, ros::Rate rate,
                   robotp::RobotHW* robot,
                   controller_manager::ControllerManager* cm)
{
  time_source::time_point last_time = time_source::now();

  while (1) {
    // Calculate monotonic time elapsed
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    // read odom from encoder
    robot->read(ros::Time::now(), elapsed);
    cm->update(ros::Time::now(), elapsed);
    // execute movement
    robot->write(ros::Time::now(), elapsed);
    rate.sleep();
  }
}

int main(int argc, char* argv[])
{
  // Initialize ROS node.
  ros::init(argc, argv, "robot_p_node");
  robotp::RobotHW robot;

  // Background thread for the controls callback.
  ros::NodeHandle controller_nh("");
  controller_manager::ControllerManager cm(&robot, controller_nh);
  boost::thread(
      boost::bind(controlThread, &controller_nh, ros::Rate(50), &robot, &cm));

  // Foreground ROS spinner for ROS callbacks
  ros::spin();

  return 0;
}
