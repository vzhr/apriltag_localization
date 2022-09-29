//
// Created by zhanghairong on 22-9-28.
//
#include <thread>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <nav_msgs/Odometry.h>
namespace apriltag_localization
{
class FakeOdomNodeLet : public nodelet::Nodelet
{
public:
  FakeOdomNodeLet() = default;
  void onInit() override{
    mt_nh_ = getMTNodeHandle();
    odom_pub_ = mt_nh_.advertise<nav_msgs::Odometry>("/jarvis/odom", 10, true);
    timer_ = mt_nh_.createTimer(ros::Rate(100), &FakeOdomNodeLet::publish, this, false, false);
    timer_.start();
  }
  void publish(const ros::TimerEvent& event){
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "baselink";
    msg.pose.pose.orientation.w = 1;
    odom_pub_.publish(msg);
  }

private:
  ros::NodeHandle mt_nh_;
  ros::Publisher odom_pub_;
  ros::Timer timer_;
};
}  // namespace apriltag_localization

PLUGINLIB_EXPORT_CLASS(apriltag_localization::FakeOdomNodeLet, nodelet::Nodelet);