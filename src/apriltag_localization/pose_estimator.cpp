//
// Created by zhanghairong on 22-9-6.
//

#include "apriltag_localization/pose_estimator.h"
#include "camera_models/include/Camera.h"
namespace apriltag_localization
{

PoseEstimator::PoseEstimator() : init_(false)
{

  last_correction_ = -1;
  stamp_ = -1;

  process_noise = Eigen::MatrixXf::Identity(6, 6);
  process_noise.middleRows(0, 3) *= 1.0;
  process_noise.middleRows(3, 3) *= 1.0;
  process_noise.middleRows(6, 3) *= 0.5;

  Eigen::MatrixXf measurement_noise = Eigen::MatrixXf::Identity(6, 6);
  measurement_noise.middleRows(0, 3) *= 0.01;
  measurement_noise.middleRows(3, 3) *= 0.001;

  Eigen::VectorXf mean(6);
  mean.setZero();

  Eigen::MatrixXf cov = Eigen::MatrixXf::Identity(6, 6) * 0.01;

  PoseSystem system;
  ukf.reset(new kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>(system, 6, 6, 6, process_noise, measurement_noise,
                                                                    mean, cov));
}

void PoseEstimator::predict_va(double dt, Eigen::Vector3f& velocity0, Eigen::Vector3f& angular0,
                               Eigen::Vector3f& velocity1, Eigen::Vector3f& angular1)
{
  if (!init_){
    return ;
  }
  Eigen::MatrixXf process_noise = Eigen::MatrixXf::Identity(6, 6);
  process_noise.middleRows(0, 3) *= 0.001;
  process_noise.middleRows(3, 3) *= 0.0005;

  ukf->setProcessNoiseCov(process_noise * dt);
  ukf->system.dt = dt;

  // use euler
  // TODO

}
void PoseEstimator::predict_va_rk4(double dt, Eigen::Vector3f& velocity0, Eigen::Vector3f& angular0,
                                   Eigen::Vector3f& velocity1, Eigen::Vector3f& angular1)
{
  if (!init_){
    return ;
  }

  Eigen::MatrixXf process_noise = Eigen::MatrixXf::Identity(6, 6);
  process_noise.middleRows(0, 3) *= 0.001;
  process_noise.middleRows(3, 3) *= 0.0005;

  ukf->setProcessNoiseCov(process_noise * dt);
  ukf->system.dt = dt;
  Eigen::Vector3f dd;
  dd.x() = 5;
  Eigen::VectorXf control(12);// [v0, w0, v1, w1]'
  control.middleRows<3>(0) = velocity0;
  control.middleRows<3>(3) = angular0;
  control.middleRows<3>(6) = velocity1;
  control.middleRows<3>(9) = angular1;

  ukf->predict(control);


}
void PoseEstimator::correct(double timestamp, Eigen::VectorXf& measure)
{
  if (!init_){
    init_ = true;
    last_correction_ = timestamp;
    stamp_ = timestamp;
    ukf->mean = measure;
    return ;
  }
  // TODO: correct measure


}

}  // namespace apriltag_localization