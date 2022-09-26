//
// Created by zhanghairong on 22-9-6.
//

#include <ros/ros.h>

#include <nodelet/loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apriltag_localization_camera_node");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;

  nodelet.load(ros::this_node::getName(),
               "apriltag_localization/ImageNodeLet",
               remap, nargv);
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}