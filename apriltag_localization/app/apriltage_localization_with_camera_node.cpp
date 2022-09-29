//
// Created by zhanghairong on 22-9-7.
//

#include <ros/ros.h>

#include <nodelet/loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apriltag_localization_with_camera");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;

  nodelet.load(ros::this_node::getName() + "_image",
               "apriltag_localization/ImageNodeLet",
               remap, nargv);
  nodelet.load(ros::this_node::getName(),
               "apriltag_localization/ApriltagLocalization",
               remap, nargv);

  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
//  ros::spin();
  return 0;
}