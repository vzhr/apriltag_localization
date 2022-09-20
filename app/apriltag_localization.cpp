//
// Created by zhanghairong on 22-9-6.
//
#include <condition_variable>
#include <memory>
#include <thread>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <tf2_ros/transform_listener.h>
#include <apriltag_ros/common_functions.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <pluginlib/class_list_macros.hpp>
#include "camera_models/include/CameraFactory.h"
#include "utils/ros_utils.hpp"
#include "apriltag_localization/pose_estimator.h"
namespace apriltag_localization
{
using namespace apriltag_ros;
class ApriltagLocalizationNodeLet : public nodelet::Nodelet
{
public:
  ApriltagLocalizationNodeLet() : tf_buffer_(), tf_listener_(tf_buffer_)
  {
  }
  void onInit() override
  {
    nh_ = getNodeHandle();
    mt_nh_ = getMTNodeHandle();
    pnh_ = getPrivateNodeHandle();

    process_th_ = std::thread(&ApriltagLocalizationNodeLet::processImg, this);

    active_ = false;
    cam_name_sub_ = nh_.subscribe("tag_cam_name", 10, &ApriltagLocalizationNodeLet::camNameCallback, this,
                                  ros::TransportHints().tcp());
    odom_sub_ = nh_.subscribe("odometry", 10, &ApriltagLocalizationNodeLet::odomCallback, this);
    pose_pub_ = nh_.advertise<nav_msgs::Odometry>("pose_in_tag", 1);
    if (pnh_.hasParam("cam_config"))
    {
      pnh_.getParam("cam_config", cam_config_path_);
      ROS_WARN("cam_config: %s\n", cam_config_path_.c_str());
      if (cam_config_path_.back() != '/')
      {
        cam_config_path_.push_back('/');
      }
    }
    else
    {
      ROS_WARN("Does not has cam_config!!\n");
    }

    tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh_));
    draw_tag_detections_image_ = getAprilTagOption<bool>(pnh_, "publish_tag_detections_image", false);
    it_ = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh_));
    tag_detections_publisher_ = nh_.advertise<AprilTagDetectionArray>("tag_detections", 1);
    if (draw_tag_detections_image_)
    {
      tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
    }
    pose_estimator_ = std::make_unique<PoseEstimator>();
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    if (!active_)
    {
      {
        std::lock_guard<std::mutex> lk(odom_mutex_);
        odom_queue_.clear();
      }
      return;
    }
    else
    {
      std::lock_guard<std::mutex> lk(odom_mutex_);
      odom_queue_.push_back(msg);
      while (!odom_queue_.empty() &&
             odom_queue_.back()->header.stamp.toSec() - odom_queue_.front()->header.stamp.toSec() > 5.0)
      {
        odom_queue_.pop_front();
      }
    }
  }

  void camNameCallback(const std_msgs::StringConstPtr& msg)
  {
    // start
    active_ = true;
    // setup camera model and image callback
    std::string msg_string = msg->data;
    std::string cam_name;
    auto split = msg_string.find_first_of('@');
    if (split == std::string::npos)
    {
      tag_name_.clear();
      cam_name = msg_string;
      ROS_INFO("set tag name to none.");
    }
    else
    {
      cam_name = msg_string.substr(0, split);
      tag_name_ = msg_string.substr(split + 1);
      ROS_INFO("set tag name: %s.", tag_name_.c_str());
    }

    std::string cam_config_file = cam_config_path_ + cam_name + ".yaml";
    ROS_INFO("reading cam_config_file: %s.", cam_config_file.c_str());
    cv::FileStorage fs(cam_config_file, cv::FileStorage::READ);
    if (!fs.isOpened() || fs["image_topic"].isNone())
    {
      ROS_WARN("open file: %s error!", cam_config_file.c_str());
      return;
    }
    std::string image_topic = fs["image_topic"];
    fs.release();

    // setup camera model
    CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(cam_config_file);
    tag_detector_->setCam(camera);

    // setup subscribe
    std::string transport_hint;
    pnh_.param<std::string>("transport_hint", transport_hint, "raw");
    camera_image_subscriber_ = it_->subscribe(image_topic, 1, &ApriltagLocalizationNodeLet::imageCallback, this,
                                              image_transport::TransportHints(transport_hint));
    ROS_INFO("camera_image_subscriber setup on topic: %s", image_topic.c_str());

    // renew estimator
    pose_estimator_ = std::make_unique<PoseEstimator>();
  }
  void imageCallback(const sensor_msgs::ImageConstPtr& image_rect)
  {
    if (!active_)
    {
      return;
    }
    // Lazy updates:
    // When there are no subscribers _and_ when tf is not published,
    // skip detection.
    if (pose_pub_.getNumSubscribers() == 0 && tag_detections_publisher_.getNumSubscribers() == 0 &&
        tag_detections_image_publisher_.getNumSubscribers() == 0 && !tag_detector_->get_publish_tf())
    {
      ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
      camera_image_subscriber_.shutdown();
      active_ = false;
      return;
    }

    // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
    // AprilTag 2 on the image
    try
    {
      cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    {
      std::lock_guard<std::mutex> lk(image_mutex_);
      image_queue_.push_back(cv_image_);
    }
    img_con_.notify_one();
  }

protected:
  void processImg()
  {
    while (ros::ok())
    {
      std::unique_lock<std::mutex> lk(image_mutex_);
      img_con_.wait(lk, [&]() { return !image_queue_.empty(); });
      while (!image_queue_.empty())
      {
        if (!active_)
        {
          image_queue_.clear();
          lk.unlock();
          break;
        }

        cv_bridge::CvImagePtr cv_image = image_queue_.front();
        image_queue_.pop_front();
        lk.unlock();
        std::vector<std::string> detection_names;
        AprilTagDetectionArray tag_result = tag_detector_->detectTagsWithModel(cv_image, detection_names);
        if (tag_result.detections.empty())
        {
          continue;
        }
        Eigen::Isometry3d T_cam_tag;
        bool find_tag = false;
        if (tag_name_.empty())
        {
          // tag no specify, get nearest tag
          double least_distance = std::numeric_limits<double>::max();
          std::string select_tag;

          for (size_t i = 0; i < tag_result.detections.size(); ++i)
          {
            auto& tag_pose = tag_result.detections.at(i).pose.pose.pose.position;
            double distance = tag_pose.x * tag_pose.x + tag_pose.y * tag_pose.y + tag_pose.z * tag_pose.z;
            if (distance < least_distance)
            {
              least_distance = distance;
              T_cam_tag = pose2isometry(tag_result.detections.at(i).pose.pose.pose);
              select_tag = detection_names.at(i);
              find_tag = true;
            }
          }
          ROS_DEBUG("select tag: %s.", select_tag.c_str());
        }
        else
        {
          for (size_t i = 0; i < detection_names.size(); ++i)
          {
            auto item = std::find(detection_names.begin(), detection_names.end(), tag_name_);
            if (item != detection_names.end())
            {
              T_cam_tag = pose2isometry(tag_result.detections.at(i).pose.pose.pose);
              find_tag = true;
              break;
            }
          }
        }
        if (!find_tag)
        {
          continue;
        }

        // select odom to predict
        // predict odom with v and w, use advanced euler or rk4?

        double time0 = pose_estimator_->stamp();
        double time1 = cv_image->header.stamp.toSec();
        std::vector<nav_msgs::OdometryConstPtr> odom_data = select_odom(time0, time1);
        if (odom_data.size() > 1)
        {
          for (int i = 0; i < odom_data.size() - 1; ++i)
          {
            pose_estimator_->predict_va_rk4();
          }
        }

        // measurement

        // Publish detected tags in the image by AprilTag 2
        tag_detections_publisher_.publish(tag_result);

        // Publish the camera image overlaid by outlines of the detected tags and
        // their payload values
        if (draw_tag_detections_image_)
        {
          tag_detector_->drawDetections(cv_image);
          tag_detections_image_publisher_.publish(cv_image->toImageMsg());
        }
      }
    }
  }

  std::vector<nav_msgs::OdometryConstPtr> select_odom(double time0, double time1)
  {
    std::vector<nav_msgs::OdometryConstPtr> odom_data;
    if (odom_queue_.empty())
    {
      return odom_data;
    }
    if (odom_queue_.front()->header.stamp.toSec() > time0)
    {
      return odom_data;
    }

    auto it = odom_queue_.begin();
    for (size_t i = 0; i < odom_queue_.size() - 1; ++i)
    {
      auto it0 = it;
      auto it1 = ++it;

      if ((*it0)->header.stamp.toSec() < time0 && (*it1)->header.stamp.toSec() > time0)
      {
        auto data = interpolate_data(*it0, *it1, time0);
        odom_data.push_back(data);
        continue;
      }
      if ((*it0)->header.stamp.toSec() >= time0 && (*it1)->header.stamp.toSec() <= time1)
      {
        odom_data.push_back(*it0);
        continue ;
      }

      if ((*it1)->header.stamp.toSec() > time1){
        if ((*it0)->header.stamp.toSec() > time1 && i == 0){
          // this happened in start time when first odom come in after image
          break;
        } else if ((*it0)->header.stamp.toSec() <= time1){
          odom_data.push_back(*it0);
          auto data = interpolate_data(*it0, *it1, time1);
          break ;
        }
      }
    }
    if (time1 - odom_data.back()->header.stamp.toSec() > 1e-3){
      auto data = interpolate_data(odom_data.at(odom_data.size() - 2), odom_data.back(), time1);
      odom_data.push_back(data);
    }

    // Loop through and ensure we do not have an zero dt values
    for (size_t i = 0; i < odom_data.size() - 1; ++i)
    {
      if (std::abs(odom_data.at(i + 1)->header.stamp.toSec() - odom_data.at(i)->header.stamp.toSec()) < 1e-12){
        ROS_WARN("odom data time messed up!!");
        odom_data.erase(odom_data.begin() + i);
        --i;
      }
    }
  }
  nav_msgs::OdometryConstPtr interpolate_data(const nav_msgs::OdometryConstPtr& left,
                                              const nav_msgs::OdometryConstPtr& right, const double& timestamp)
  {
    nav_msgs::OdometryPtr data = boost::make_shared<::nav_msgs::Odometry>();
    double lambda =
        (timestamp - left->header.stamp.toSec()) / (right->header.stamp.toSec() - left->header.stamp.toSec());
    data->header.stamp.fromSec(timestamp);
    data->pose.pose.position.x = (1 - lambda) * left->pose.pose.position.x + lambda * right->pose.pose.position.x;
    data->pose.pose.position.y = (1 - lambda) * left->pose.pose.position.y + lambda * right->pose.pose.position.y;
    data->pose.pose.position.z = (1 - lambda) * left->pose.pose.position.z + lambda * right->pose.pose.position.z;

    Eigen::Quaterniond q_left(left->pose.pose.orientation.w, left->pose.pose.orientation.x,
                              left->pose.pose.orientation.y, left->pose.pose.orientation.z);
    Eigen::Quaterniond q_right(right->pose.pose.orientation.w, right->pose.pose.orientation.x,
                               right->pose.pose.orientation.y, right->pose.pose.orientation.z);
    q_left.normalize();
    q_right.normalize();

    // slerp or use angle axis
    Eigen::Quaterniond interpolate_q = q_left.slerp(lambda, q_right);  // slerp

    // Eigen::AngleAxisd diff_q(q_left.conjugate() * q_right);
    // Eigen::AngleAxisd lambda_diff(lambda * diff_q.angle(), diff_q.axis());
    // Eigen::Quaterniond interpolate_q = q_left * lambda_diff;

    data->pose.pose.orientation.w = interpolate_q.w();
    data->pose.pose.orientation.x = interpolate_q.x();
    data->pose.pose.orientation.y = interpolate_q.y();
    data->pose.pose.orientation.z = interpolate_q.z();
    data->twist.twist.angular.x = (1 - lambda) * left->twist.twist.angular.x + lambda * right->twist.twist.angular.x;
    data->twist.twist.angular.y = (1 - lambda) * left->twist.twist.angular.y + lambda * right->twist.twist.angular.y;
    data->twist.twist.angular.z = (1 - lambda) * left->twist.twist.angular.z + lambda * right->twist.twist.angular.z;
    data->twist.twist.linear.x = (1 - lambda) * left->twist.twist.linear.x + lambda * right->twist.twist.linear.x;
    data->twist.twist.linear.y = (1 - lambda) * left->twist.twist.linear.y + lambda * right->twist.twist.linear.y;
    data->twist.twist.linear.z = (1 - lambda) * left->twist.twist.linear.z + lambda * right->twist.twist.linear.z;
    return data;
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle mt_nh_;
  ros::NodeHandle pnh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::shared_ptr<TagDetector> tag_detector_;
  bool draw_tag_detections_image_;
  cv_bridge::CvImagePtr cv_image_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber camera_image_subscriber_;
  image_transport::Publisher tag_detections_image_publisher_;
  ros::Publisher tag_detections_publisher_;
  ros::Publisher pose_pub_;
  ros::Subscriber cam_name_sub_;
  ros::Subscriber odom_sub_;
  std::string cam_config_path_;

  std::mutex odom_mutex_;
  std::mutex image_mutex_;
  std::deque<cv_bridge::CvImagePtr> image_queue_;
  std::deque<nav_msgs::OdometryConstPtr> odom_queue_;

  std::atomic<bool> thread_update_running_;
  std::atomic<bool> active_;
  std::condition_variable img_con_;
  std::thread process_th_;

  std::string tag_name_;
  int tag_id_;

  std::unique_ptr<PoseEstimator> pose_estimator_;
  Eigen::Isometry3d T_tag_odom_;  // result: odom to tag frame coordinate
};

}  // namespace apriltag_localization

PLUGINLIB_EXPORT_CLASS(apriltag_localization::ApriltagLocalizationNodeLet, nodelet::Nodelet);
