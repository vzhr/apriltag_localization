//
// Created by zhanghairong on 22-9-6.
//

#include <memory>
#include <thread>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
namespace apriltag_localization
{

class ImageNodeLet : public nodelet::Nodelet
{
public:
  ImageNodeLet()
  {
  }
  void onInit() override
  {
    nh_ = getNodeHandle();
    mt_nh_ = getMTNodeHandle();
    pnh_ = getPrivateNodeHandle();
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

    it_ = std::make_shared<image_transport::ImageTransport>(nh_);
    start_sub_ = mt_nh_.subscribe("tag_cam_name", 1, &ImageNodeLet::camCallback, this, ros::TransportHints().tcp());
  }

  void camCallback(const std_msgs::StringConstPtr& msg)
  {
    stop();
    // setup camera model and image callback
    std::string msg_string = msg->data;
    std::string cam_name;
    auto split = msg_string.find_first_of('@');
    if (split == std::string::npos)
    {
      cam_name = msg_string;
      ROS_INFO("set tag name to none.");
    }
    else
    {
      cam_name = msg_string.substr(0, split);
    }
    std::string cam_config_file = cam_config_path_ + cam_name + ".yaml";
    ROS_INFO("reading cam_config_file: %s.", cam_config_file.c_str());
    start(cam_config_file);
  }
  void start(std::string cam_config_file)
  {
    cv::FileStorage fs(cam_config_file, cv::FileStorage::READ);
    if (!fs.isOpened() || fs["image_topic"].isNone())
    {
      ROS_WARN("open file: %s error!", cam_config_file.c_str());
      stop();
      return;
    }
    std::string image_topic = fs["image_topic"];
    image_topic_ = image_topic;
    ROS_INFO("image nodelet: pub image topic %s", image_topic_.c_str());
    if (fs["pub_freq"].isNone())
    {
      pub_freq_ = 30;
    }
    else
    {
      pub_freq_ = fs["pub_freq"];
    }
    if (fs["video_dev"].isNone())
    {
      video_dev_ = 0;
    }
    else
    {
      video_dev_ = fs["video_dev"];
    }
    active_ = true;
    image_publisher_.shutdown();
    image_publisher_ = it_->advertise(image_topic_, 1);

    pub_th_ = std::make_unique<std::thread>(&ImageNodeLet::publishImg, this);
  }
  void stop()
  {
    active_ = false;
    image_publisher_.shutdown();
    if (pub_th_)
    {
      pub_th_->join();
    }
  }
  void publishImg()
  {
    // open camera
    cv::VideoCapture capture(video_dev_);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    using namespace std::chrono;
    using namespace std::chrono_literals;
    auto first_img_time = system_clock::now();
    bool first = true;
    bool pub;
    int pub_count = 0;
    cv::Mat frame;
    while (ros::ok() && active_)
    {
      std::this_thread::sleep_for(10ms);
      if (capture.isOpened())
      {
        if (!capture.read(frame)){continue;}
        auto now = system_clock::now();
        if (first){
          first = false;
          first_img_time = now;
          publish(first_img_time.time_since_epoch().count(), frame);
          ++pub_count;
          continue ;
        }
        // freq control
        std::chrono::duration<double> dur_time = now - first_img_time;
        double diff_time = dur_time.count();
        if (std::round(1.0 * pub_count/(diff_time)) <= pub_freq_){
          ++pub_count;
          if (std::abs(1.0 * pub_count/(diff_time) - pub_freq_) < 0.01 * pub_freq_ || pub_count > 10000){
            pub_count = 1;
            first_img_time = now;
          }
          publish(now.time_since_epoch().count(), frame);
        }
      }
      else
      {
        first = true;
        pub_count＝0;
        capture.release();
        capture.open(video_dev_);
        capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
      }
    }
  }
  void publish(uint64_t stamp, cv::Mat img){
    std_msgs::Header header;
    header.stamp.fromNSec(stamp);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
    image_publisher_.publish(msg);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle mt_nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber start_sub_;
  ros::Subscriber stop_sub_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher image_publisher_;

  double pub_freq_;  // frequency of publish image
  std::string cam_config_path_;
  std::string image_topic_;
  int video_dev_;

  std::unique_ptr<std::thread> pub_th_;

  bool active_ = false;
};

}  // namespace apriltag_localization

PLUGINLIB_EXPORT_CLASS(apriltag_localization::ImageNodeLet, nodelet::Nodelet);
