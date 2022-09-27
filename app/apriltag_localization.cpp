//
// Created by zhanghairong on 22-9-6.
//
#include <condition_variable>
#include <memory>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nodelet/nodelet.h>
#include <tf2_ros/transform_listener.h>
#include <apriltag_ros/common_functions.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <pluginlib/class_list_macros.hpp>
#include <opencv2/core/eigen.hpp>
#include "camera_models/include/CameraFactory.h"
#include "utils/ros_utils.hpp"
#include "apriltag_localization/pose_estimator.h"
namespace apriltag_localization
{
using namespace apriltag_ros;
class ApriltagLocalizationNodeLet : public nodelet::Nodelet
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ApriltagLocalizationNodeLet() : tf_buffer_(), tf_listener_(tf_buffer_), T_correct_tag_odom_(boost::none)
  {
  }
  void onInit() override
  {
    nh_ = getNodeHandle();
    mt_nh_ = getMTNodeHandle();
    pnh_ = getPrivateNodeHandle();
    mt_pnh_ = getMTPrivateNodeHandle();
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
    it_ = std::make_shared<image_transport::ImageTransport>(mt_nh_);
    tag_detections_publisher_ = mt_pnh_.advertise<AprilTagDetectionArray>("tag_detections", 1);
    if (draw_tag_detections_image_)
    {
      tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
    }
    cam_name_sub_ = mt_nh_.subscribe("tag_cam_name", 10, &ApriltagLocalizationNodeLet::camNameCallback, this,
                                     ros::TransportHints().tcp());
    stop_sub_ = mt_nh_.subscribe("/stop_docking", 10, &ApriltagLocalizationNodeLet::stopCallback, this,
                                 ros::TransportHints().tcp());

    odom_sub_ = mt_nh_.subscribe("odom", 10, &ApriltagLocalizationNodeLet::odomCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_in_tag", 1);
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    ROS_DEBUG("odom_income");
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
             odom_queue_.back()->header.stamp.toSec() - odom_queue_.front()->header.stamp.toSec() > 50.0)
      {
        odom_queue_.pop_front();
      }
    }
    // publish pose
    if (T_correct_tag_odom_)
    {
      auto& pose = msg->pose.pose.position;
      auto& orientation = msg->pose.pose.orientation;
      Eigen::Isometry3d T_odom_baselink = Eigen::Isometry3d::Identity();
      T_odom_baselink.translation() = Eigen::Vector3d(pose.x, pose.y, pose.z);
      T_odom_baselink.linear() = Eigen::Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z).toRotationMatrix();
      Eigen::Isometry3d T_tag_baselink = T_correct_tag_odom_.get() * T_odom_baselink;
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.pose = isometry2pose(T_tag_baselink);
      pose_msg.header.frame_id = select_tag_name_;
      pose_msg.header.stamp = msg->header.stamp;
      pose_pub_.publish(pose_msg);

      Eigen::Matrix3d rotation = T_tag_baselink.linear();
      double theta = std::atan2(rotation(1,0), rotation(0,0));
      ROS_INFO("theta: %f", theta);
      ROS_INFO_STREAM("pose: " << T_tag_baselink.translation().transpose());
    }
  }
  void cleanOldOdomData(double stamp)
  {

    std::lock_guard<std::mutex> lk(odom_mutex_);
    while (!odom_queue_.empty() && odom_queue_.front()->header.stamp.toSec() < stamp)
    {
      odom_queue_.pop_front();
    }
  }

  void stopCallback(const std_msgs::StringConstPtr& msg){
    auto m = boost::make_shared<std_msgs::String>();
    m->data = "stop";
    camNameCallback(m);
  }

  void camNameCallback(const std_msgs::StringConstPtr& msg)
  {
    stop();
    if (msg->data == "stop"){
      ROS_INFO("stop");
      return ;
    }
    // setup camera model and image callback
    std::string msg_string = msg->data;
    std::string cam_name;
    auto split = msg_string.find_first_of('@');
    if (split == std::string::npos)
    {
      select_tag_name_.clear();
      cam_name = msg_string;
      ROS_INFO("set tag name to none.");
    }
    else
    {
      cam_name = msg_string.substr(0, split);
      select_tag_name_ = msg_string.substr(split + 1);
      ROS_INFO("set tag name: %s.", select_tag_name_.c_str());
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
    image_topic_ = image_topic;
    cv::Mat cv_T;
    fs["T_base_cam"] >> cv_T;
    Eigen::Matrix4d T;
    cv::cv2eigen(cv_T, T);
    fs.release();
    std::cout << T <<std::endl;
    T_baselink_cam_.translation() = T.block<3,1>(0,3);
    T_baselink_cam_.linear() = T.block<3,3>(0,0);
    // setup camera model
    CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(cam_config_file);
    tag_detector_->setCam(camera);
    start();
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
    /*
     if (pose_pub_.getNumSubscribers() == 0 && tag_detections_publisher_.getNumSubscribers() == 0 &&
         tag_detections_image_publisher_.getNumSubscribers() == 0 && !tag_detector_->get_publish_tf())
     {
       ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
       stop();
       return;
     }
     */

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
  virtual ~ApriltagLocalizationNodeLet()
  {
    stop();
  }

  void processImg()
  {
    while (ros::ok() && active_)
    {
      std::unique_lock<std::mutex> lk(image_mutex_);
      img_con_.wait(lk);
      if (!active_){
        // judge active here for out loop notify
        return ;
      }
      if (image_queue_.empty()){
        continue ;
      }
      while (!image_queue_.empty())
      {
        if (!active_)
        {
          image_queue_.clear();
          return;
        }

        cv_bridge::CvImagePtr cv_image = image_queue_.front();
        lk.unlock();
        // in the start time, the image is old than odom, pop
        if (!pose_estimator_->isInit()){
          double odom_time_left = -1.0;
          {
            std::lock_guard<std::mutex> odom_lk(odom_mutex_);
            if (!odom_queue_.empty()){
              odom_time_left = odom_queue_.front()->header.stamp.toSec();
            }
          }
          if (cv_image->header.stamp.toSec() < odom_time_left || odom_time_left < 0){
            lk.lock();
            image_queue_.pop_front();
            ROS_INFO("pop image");
            continue ;
          }
        }
        // on running, image timestamp new than odom, waiting for 10ms
        else{
          double odom_time_right = std::numeric_limits<double>::max();
          {
            std::lock_guard<std::mutex> odom_lk(odom_mutex_);
            if (!odom_queue_.empty()){
              odom_time_right = odom_queue_.back()->header.stamp.toSec();
            }
          }
          if (cv_image->header.stamp.toSec() > odom_time_right){
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(10ms);
            lk.lock();
            break ;
          }
        }

        lk.lock();
        image_queue_.pop_front();
        lk.unlock();
        std::vector<std::string> detection_names;
        AprilTagDetectionArray tag_result = tag_detector_->detectTagsWithModel(cv_image, detection_names);

        // Publish detected tags in the image by AprilTag 2
        tag_detections_publisher_.publish(tag_result);
        // visualize
        // Publish the camera image overlaid by outlines of the detected tags and
        // their payload values
        if (draw_tag_detections_image_ && tag_detections_image_publisher_.getNumSubscribers() > 0)
        {
          tag_detector_->drawDetections(cv_image);
          tag_detections_image_publisher_.publish(cv_image->toImageMsg());
        }
        if (tag_result.detections.empty())
        {
          lk.lock();
          continue;
        }

        Eigen::VectorXf measure(6);
        bool find_tag = getMeasurement(tag_result, detection_names, measure);
        if (find_tag)
        {
          update(tag_result.header.stamp.toSec(), measure);
        }
        lk.lock();
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

  void update(double stamp, const Eigen::VectorXf& measure)
  {
    if (!pose_estimator_)
    {
      ROS_WARN("pose_estimator_ not construct!");
      return;
    }
    // if we don't have initial pose, use correct to set
    if (!pose_estimator_->isInit())
    {
      ROS_INFO("pose_estimator init correct");
      pose_estimator_->correct(stamp, measure);
      return;
    }

    /// do predict and correct

    // select odom to predict
    // predict odom with v and w, use rk4.

    double time0 = pose_estimator_->lastCorrection();
    double time1 = stamp;
    std::vector<nav_msgs::OdometryConstPtr> odom_data = select_odom(time0, time1);
    if (odom_data.size() < 2)
    {
      // restart estimator
      ROS_INFO("update: odom_data < 2, stop");
//      pose_estimator_->stop();
      pose_estimator_->correct(stamp, measure);
      return;
    }

    for (int i = 0; i < odom_data.size() - 1; ++i)
    {
      auto& data0 = odom_data.at(i);
      auto& data1 = odom_data.at(i + 1);
      double dt = data1->header.stamp.toSec() - data0->header.stamp.toSec();
      auto& v0 = data0->twist.twist.linear;
      auto& w0 = data0->twist.twist.angular;
      auto& v1 = data1->twist.twist.linear;
      auto& w1 = data1->twist.twist.angular;

      Eigen::Vector3f velocity0(v0.x, v0.y, v0.z);
      Eigen::Vector3f angular0(w0.x, w0.y, w0.z);
      Eigen::Vector3f velocity1(v1.x, v1.y, v1.z);
      Eigen::Vector3f angular1(w1.x, w1.y, w1.z);
      Eigen::VectorXf control(12);
      control.middleRows<3>(0) = velocity0;
      control.middleRows<3>(3) = angular0;
      control.middleRows<3>(6) = velocity1;
      control.middleRows<3>(9) = angular1;
      pose_estimator_->predict_va_rk4(dt, control);
      pose_estimator_->stamp() = data1->header.stamp.toSec();
    }
    pose_estimator_->correct(stamp, measure);
    cleanOldOdomData(stamp - 1.0);  // for interpolate data
    /// get correction result
    Eigen::Isometry3d trans = pose_estimator_->matrix();
    Eigen::Isometry3d odom_pose = Eigen::Isometry3d::Identity();
    auto& pose = odom_data.back()->pose.pose;
    odom_pose.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    odom_pose.linear() =
        Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
            .toRotationMatrix();

    T_correct_tag_odom_ = trans * odom_pose.inverse();
  }

  bool getMeasurement(const AprilTagDetectionArray& tag_result, const std::vector<std::string>& detection_names,
                      Eigen::VectorXf& measure)
  {
    bool find_tag = false;
    Eigen::Isometry3d T_cam_tag;
    if (select_tag_name_.empty())
    {
      /// tag no specify, get nearest tag
      double least_distance = std::numeric_limits<double>::max();
      std::string select_tag;

      // check and get tag name
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
      select_tag_name_ = select_tag;
      ROS_INFO("select tag: %s.", select_tag.c_str());
    }
    else
    {
      auto item = std::find(detection_names.begin(), detection_names.end(), select_tag_name_);
      if (item != detection_names.end())
      {
        auto dist = std::distance(detection_names.begin(), item);
        T_cam_tag = pose2isometry(tag_result.detections.at(dist).pose.pose.pose);
        find_tag = true;
      }
    }
    if (!find_tag){return false;}

    measure.resize(6);
    // matrix coordinate
    Eigen::Matrix4d matrix_tag_front;
    // clang-format off
    matrix_tag_front <<
         0, -1, 0, 0,
         0,  0, 1, 0,
        -1,  0, 0, 0,
         0,  0, 0, 1;
    // clang-format on

    Eigen::Isometry3d T_tag_front(matrix_tag_front);
    Eigen::Isometry3d T_baselink_cam_tag_front = T_baselink_cam_ * T_cam_tag * T_tag_front;
    Eigen::Isometry3d T_front_baselink = T_baselink_cam_tag_front.inverse();
    Eigen::Vector3d translation = T_front_baselink.translation();
    Eigen::Matrix3d rotation = T_front_baselink.linear();
    Eigen::AngleAxisf angle_axisf(rotation.cast<float>());
    measure.head(3) = angle_axisf.angle() * angle_axisf.axis();
    measure.tail(3) = translation.cast<float>();
    return find_tag;
  }

  std::vector<nav_msgs::OdometryConstPtr> select_odom(double time0, double time1)
  {
    std::vector<nav_msgs::OdometryConstPtr> odom_data;
    std::lock_guard<std::mutex> lk(odom_mutex_);
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
        continue;
      }

      if ((*it1)->header.stamp.toSec() > time1)
      {
        if ((*it0)->header.stamp.toSec() > time1 && i == 0)
        {
          // this happened in start time when first odom come in after image
          break;
        }
        else if ((*it0)->header.stamp.toSec() <= time1)
        {
          odom_data.push_back(*it0);
          auto data = interpolate_data(*it0, *it1, time1);
          odom_data.push_back(data);
          break;
        }
      }
    }
    if (odom_data.empty()){
      return odom_data;
    }

    if (odom_data.size() == 1){
      nav_msgs::OdometryPtr data = boost::make_shared<::nav_msgs::Odometry>();
      data->header=odom_data.back()->header;
      data->header.stamp.fromSec(time1);
      odom_data.push_back(data);
    }

    // this case for solve latest odom time < time1
    if (std::abs(time1 - odom_data.back()->header.stamp.toSec()) > 1e-3)
    {
      auto data = interpolate_data(odom_data.at(odom_data.size() - 2), odom_data.back(), time1);
      odom_data.push_back(data);
    }

    // Loop through and ensure we do not have an zero dt values
    for (size_t i = 0; i < odom_data.size() - 1; ++i)
    {
      if (std::abs(odom_data.at(i + 1)->header.stamp.toSec() - odom_data.at(i)->header.stamp.toSec()) < 1e-12)
      {
        ROS_WARN("odom data time messed up!!");
        odom_data.erase(odom_data.begin() + i);
        --i;
      }
    }
    return odom_data;
  }
  void stop()
  {
    ROS_INFO("stop...");
    active_ = false;
    camera_image_subscriber_.shutdown();
    select_tag_name_.clear();
    pose_estimator_.reset();
    T_correct_tag_odom_ = boost::none;
    img_con_.notify_all();  // for out process loop
    if (process_th_)
    {
      ROS_INFO("[apriltag localization]: process join");
      process_th_->join();
    }
    ROS_INFO("[apriltag localization]: stop0");
    process_th_.reset();
    ROS_INFO("[apriltag localization]: stop1");
  }
  void start()
  {
    ROS_INFO("start...");
    // start
    active_ = true;
    // setup subscribe
    std::string transport_hint;
    pnh_.getParam("transport_hint", transport_hint);
    camera_image_subscriber_ = it_->subscribe(image_topic_, 1, &ApriltagLocalizationNodeLet::imageCallback, this,
                                              image_transport::TransportHints(transport_hint));
    ROS_INFO("camera_image_subscriber setup on topic: %s", image_topic_.c_str());

    // reset estimator
    pose_estimator_ = std::make_unique<PoseEstimator>();
    process_th_ = std::make_unique<std::thread>(&ApriltagLocalizationNodeLet::processImg, this);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle mt_nh_;
  ros::NodeHandle pnh_;
  ros::NodeHandle mt_pnh_;

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
  ros::Subscriber stop_sub_;
  ros::Subscriber odom_sub_;
  std::string cam_config_path_;

  std::mutex odom_mutex_;
  std::mutex image_mutex_;
  std::deque<cv_bridge::CvImagePtr> image_queue_;
  std::deque<nav_msgs::OdometryConstPtr> odom_queue_;

  std::atomic<bool> active_;
  std::condition_variable img_con_;
  std::unique_ptr<std::thread> process_th_;

  /// camera info
  std::string image_topic_;
  std::string select_tag_name_;
  Eigen::Isometry3d T_baselink_cam_;  // camera to robot extrinsic

  std::unique_ptr<PoseEstimator> pose_estimator_;
  boost::optional<Eigen::Isometry3d> T_correct_tag_odom_;  // result: odom to tag frame coordinate
};

}  // namespace apriltag_localization

PLUGINLIB_EXPORT_CLASS(apriltag_localization::ApriltagLocalizationNodeLet, nodelet::Nodelet);
