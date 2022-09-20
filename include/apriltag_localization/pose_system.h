//
// Created by zhanghairong on 22-9-13.
//

#ifndef APRILTAG_LOCALIZATION_POSESYSTEM_H_
#define APRILTAG_LOCALIZATION_POSESYSTEM_H_
#include <iostream>
#include <Eigen/Eigen>
namespace apriltag_localization
{
/**
 * @brief Definition of system to be estimated by ukf
 * @note state = [theta1, theta2, theta3, px, py, pz], theta is axis angle
 */
class PoseSystem
{
public:
  typedef double T;
  typedef Eigen::Matrix<T, 3, 1> Vector3t;
  typedef Eigen::Matrix<T, 4, 1> Vector4t;
  typedef Eigen::Matrix<T, 4, 4> Matrix4t;
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;
  typedef Eigen::Quaternion<T> Quaterniont;

  static Eigen::Matrix3d skew_x(Eigen::Vector3d& w)
  {
    Eigen::Matrix<double, 3, 3> w_x;
    w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
    return w_x;
  }

  static Eigen::Matrix4d Omega(Eigen::Vector3d w)
  {
    Eigen::Matrix<double, 4, 4> mat;
    mat.block(0, 0, 3, 3) = -skew_x(w);
    mat.block(3, 0, 1, 3) = -w.transpose();
    mat.block(0, 3, 3, 1) = w;
    mat(3, 3) = 0;
    return mat;
  }

public:
  PoseSystem()
  {
    dt = 0.01;
  }

  // system equation
  Eigen::VectorXd f(const Eigen::VectorXd& state, const Eigen::VectorXd& control) const
  {
    Eigen::VectorXd next_state(6);
    // rk4
    // pre-compute things
    Eigen::Vector3d v_hat1 = control.middleRows<3>(0);
    Eigen::Vector3d w_hat1 = control.middleRows<3>(3);
    Eigen::Vector3d v_hat2 = control.middleRows<3>(6);
    Eigen::Vector3d w_hat2 = control.middleRows<3>(9);
    Eigen::Vector3d v_a = (v_hat2 - v_hat1)/ dt;
    Eigen::Vector3d w_alpha = (w_hat2 - w_hat1)/ dt;
    Eigen::Vector3d v_hat = v_hat1;
    Eigen::Vector3d w_hat = w_hat1;

    // y0 ======================
    Eigen::Vector3d theta_0 = state.middleRows<3>(0);
    Eigen::AngleAxisd axis(theta_0.norm(), theta_0.normalized());
    Eigen::Quaterniond q_0(axis);
    q_0.normalize();
    Eigen::Vector3d p_0 = state.middleRows<3>(3);

    // q = [x, y, z, w]
    // k1 ================
    Eigen::Vector4d dq_0(0, 0, 0, 1);
    Eigen::Vector4d q0_dot = 0.5 * Omega(w_hat) * dq_0;
    Eigen::Quaterniond qx = q_0 * Eigen::Quaterniond(dq_0(3), dq_0(0),dq_0(1),dq_0(2));
    Eigen::Vector3d p0_dot = qx.toRotationMatrix() * v_hat;

    Eigen::Vector4d k1_q = q0_dot * dt;
    Eigen::Vector3d k1_p = p0_dot * dt;

    // k2 ================
    w_hat += 0.5 * w_alpha * dt;
    v_hat += 0.5 * v_a * dt;

    Eigen::Vector4d dq_1 = (dq_0 + 0.5 * k1_q).normalized();
    if (dq_1(3) < 0)
    {
      dq_1 *= -1;
    }
    Eigen::Vector3d p_1 = p_0 + 0.5 * k1_p;

    Eigen::Vector4d q1_dot = 0.5 * Omega(w_hat) * dq_1;
    qx = q_0 * Eigen::Quaterniond(dq_1(3), dq_1(0),dq_1(1),dq_1(2));
    Eigen::Vector3d p1_dot = qx.toRotationMatrix() * v_hat;

    Eigen::Vector4d k2_q = q1_dot * dt;
    Eigen::Vector3d k2_p = p1_dot * dt;

    // k3 ================
    Eigen::Vector4d dq_2 = (dq_0 + 0.5 * k2_q).normalized();
    if (dq_2(3) < 0)
    {
      dq_2 *= -1;
    }
    Eigen::Vector3d p_2 = p_0 + 0.5 * k2_p;

    Eigen::Vector4d q2_dot = 0.5 * Omega(w_hat) * dq_2;
    qx = q_0 * Eigen::Quaterniond(dq_2(3), dq_2(0),dq_2(1),dq_2(2));
    Eigen::Vector3d p2_dot = qx.toRotationMatrix() * v_hat;

    Eigen::Vector4d k3_q = q2_dot * dt;
    Eigen::Vector3d k3_p = p2_dot * dt;

    // k4 ================
    w_hat += 0.5 * w_alpha * dt;
    v_hat += 0.5 * v_a * dt;

    Eigen::Vector4d dq_3 = (dq_0 + k3_q).normalized();
    if (dq_3(3) < 0)
    {
      dq_3 *= -1;
    }
    Eigen::Vector3d p_3 = p_0 + k3_p;

    Eigen::Vector4d q3_dot = 0.5 * Omega(w_hat) * dq_3;
    qx = q_0 * Eigen::Quaterniond(dq_2(3), dq_2(0),dq_2(1),dq_2(2));
    Eigen::Vector3d p3_dot = qx.toRotationMatrix() * v_hat;

    Eigen::Vector4d k4_q = q3_dot * dt;
    Eigen::Vector3d k4_p = p3_dot * dt;

    // y+dt ==================
    Eigen::Vector4d dq = (dq_0 + (1.0 / 6.0) * k1_q + (1.0 / 3.0) * k2_q
                          + (1.0 / 3.0) * k3_q + (1.0 / 6.0) * k4_q).normalized();
    if (dq(3) < 0)
    {
      dq *= -1;
    }
    Eigen::Quaterniond _dq, new_q;
    _dq.coeffs() = dq;
    new_q = q_0 * _dq;
    Eigen::Vector3d new_p =  p_0 + (1.0 / 6.0) * k1_p + (1.0 / 3.0) * k2_p
                          + (1.0 / 3.0) * k3_p + (1.0 / 6.0) * k4_p;

    Eigen::AngleAxisd new_axis(new_q);
    Eigen::Vector3d theta = new_axis.angle() * new_axis.axis();
    next_state.middleRows<3>(0) = theta;
    next_state.middleRows<3>(3) = new_p;
    return next_state;
  }

  // observation equation
  Eigen::VectorXd h(const Eigen::VectorXd& state) const
  {
    Eigen::VectorXd observation(6);
    observation.middleRows(0, 3) = state.middleRows(0, 3);
    observation.middleRows(3, 6) = state.middleRows(3, 3);

    return observation;
  }

  double dt;
};
}  // namespace apriltag_localization
#endif  // APRILTAG_LOCALIZATION_POSESYSTEM_H_
