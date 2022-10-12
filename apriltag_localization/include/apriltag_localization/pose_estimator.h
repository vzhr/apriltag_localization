//
// Created by zhanghairong on 22-9-6.
//

#ifndef APRILTAG_ROS_SRC_POSE_ESTIMATOR_H_
#define APRILTAG_ROS_SRC_POSE_ESTIMATOR_H_
#include <chrono>
#include <memory>
#include <kkl/alg/unscented_kalman_filter.hpp>
#include "pose_system.h"
namespace apriltag_localization
{

class PoseEstimator
{
public:
  PoseEstimator();
  void stop(){
    init_ = false; // wait for correction to start
  }
  bool isInit(){
    return init_;
  }
  void predict_va(double dt, Eigen::Vector3f& velocity0, Eigen::Vector3f& angular0,
                  Eigen::Vector3f& velocity1, Eigen::Vector3f& angular1);
  void predict_va_rk4(double dt, Eigen::VectorXf& control);
  void correct(double timestamp, const Eigen::VectorXf& measure);

  double& stamp() {return stamp_;}
  double stamp() const {return stamp_;}
  double lastCorrection() const{return last_correction_;}

  Eigen::Isometry3d matrix();
  Eigen::MatrixXf cov(){return ukf->cov;};
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  double stamp_;
  double last_correction_;
  bool init_ = false;
  std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>> ukf;
  Eigen::MatrixXf process_noise_;
  Eigen::MatrixXf measure_noise_;
};
}  // namespace apriltag_localization

#endif  // APRILTAG_ROS_SRC_POSE_ESTIMATOR_H_
