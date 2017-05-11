#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <utility>

// publisher in HAL for laser
ros::Publisher publisher;

// constants for sensor
#define VREP_MIN_DIST_DETECTION 0.0550
#define VREP_MAX_DIST_DETECTION 19.9450

void laser(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::LaserScan laser_msg = *msg;

  float min = std::numeric_limits<float>::max();
  for (auto value : laser_msg.ranges) {
    std::min(min, value);
  }

  ROS_INFO("%f\n", (double)min);

  publisher.publish(std::move(laser_msg));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_filter");
  ros::NodeHandle n("~"); // we want relative namespace

  ros::Subscriber laser_subsriber = n.subscribe("/scan_raw", 1000, laser);

  // will publish the laser mesages to the rest of the stack
  publisher = n.advertise<sensor_msgs::LaserScan>("/scan", 1000);

  ROS_INFO("laser_filter node initialized");

  // runs event loop
  ros::spin();

  return 0;
}
