//
// Created by zhanghairong on 22-9-7.
//

#include <ros/ros.h>

#include <nodelet/loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apriltag_localization");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;

  nodelet.load(ros::this_node::getName(),
               "apriltag_localization/ApriltagLocalization",
               remap, nargv);

  ros::spin();
  return 0;
}