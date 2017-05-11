#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <utility>

/* removes poles (from robot construction) obstructing laser beam */

ros::Publisher publisher;

// robot poles obstructiong laser bean
constexpr float pole_distance = 0.35;
constexpr float dist_tolerance = 0.01;
// position tolerance for poles
constexpr size_t pole_tolerance = 1;
// how many scans 1 pole
constexpr size_t pole_size = 6;
// positions in scan where poles are
constexpr std::initializer_list<size_t> pole_starts{60, 138, 217, 294};

void laser(const sensor_msgs::LaserScan::ConstPtr &msg) {
  sensor_msgs::LaserScan laser_msg = *msg;

  for (auto idx : pole_starts) {
    for (size_t i = std::max(size_t(0), idx - pole_tolerance);
         i < idx + pole_size && i < laser_msg.ranges.size(); ++i) {
      if (laser_msg.ranges[i] < pole_distance + dist_tolerance) {
        laser_msg.ranges[i] = std::numeric_limits<float>::infinity();
      }
    }
  }

  publisher.publish(std::move(laser_msg));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_filter");
  ros::NodeHandle n("~");
  ros::Subscriber laser_subsriber = n.subscribe("/scan_raw", 1000, laser);

  // will publish the laser mesages to the rest of the stack
  publisher = n.advertise<sensor_msgs::LaserScan>("/scan", 1000);

  ROS_INFO("laser_filter node initialized");

  ros::spin();
  return 0;
}
