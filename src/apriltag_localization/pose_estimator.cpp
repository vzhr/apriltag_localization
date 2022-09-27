//
// Created by zhanghairong on 22-9-6.
//

#include "apriltag_localization/pose_estimator.h"

#include <memory>

namespace apriltag_localization
{
PoseEstimator::PoseEstimator() : init_(false)
{

  last_correction_ = -1;
  stamp_ = -1;

  process_noise_ = Eigen::MatrixXf::Identity(6, 6);
  process_noise_.middleRows(0, 3) *= 0.01;
  process_noise_.middleRows(3, 3) *= 0.1;

  measure_noise_ = Eigen::MatrixXf::Identity(6, 6);
  measure_noise_.middleRows(0, 3) *= 0.001;
  measure_noise_.middleRows(3, 3) *= 0.01;

  Eigen::VectorXf mean(6);
  mean.setZero();

  Eigen::MatrixXf cov = Eigen::MatrixXf::Identity(6, 6) * 0.01;

  PoseSystem system;
  ukf = std::make_unique<kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>>(system, 6, 6, 6, process_noise_, measure_noise_,
                                                                    mean, cov);
}

void PoseEstimator::predict_va(double dt, Eigen::Vector3f& velocity0, Eigen::Vector3f& angular0,
                               Eigen::Vector3f& velocity1, Eigen::Vector3f& angular1)
{
  if (!init_){
    return ;
  }
//  Eigen::MatrixXf process_noise_ = Eigen::MatrixXf::Identity(6, 6);
//  process_noise_.middleRows(0, 3) *= 0.001;
//  process_noise_.middleRows(3, 3) *= 0.0005;
//
//  ukf->setProcessNoiseCov(process_noise_ * dt);
//  ukf->system.dt = dt;

  // use euler
  // TODO
}
void PoseEstimator::predict_va_rk4(double dt, Eigen::VectorXf& control)
{
  //control: [v0, w0, v1, w1]'
  assert(control.rows() == 12);
  if (!init_){
    return ;
  }

  ukf->setProcessNoiseCov(process_noise_ * dt);
  ukf->system.dt = dt;
  ukf->predict(control);
}
void PoseEstimator::correct(double timestamp, const Eigen::VectorXf& measure)
{
  assert(measure.rows() == 6);

  last_correction_ = timestamp;
  if (!init_){
    init_ = true;
    ukf->cov = measure_noise_;
    ukf->mean = measure;
    return ;
  }
  ukf->correct(measure);
}
Eigen::Isometry3d PoseEstimator::matrix()
{
  Eigen::Isometry3d trans = Eigen::Isometry3d::Identity();
  Eigen::AngleAxisd angle_axisd;
  Eigen::Vector3d theta = ukf->mean.head<3>().cast<double>();
  angle_axisd.angle() = theta.norm();
  angle_axisd.axis() = theta.normalized();
  trans.linear() = angle_axisd.toRotationMatrix();
  trans.translation() = ukf->mean.tail<3>().cast<double>();
  return trans;
}

}  // namespace apriltag_localization