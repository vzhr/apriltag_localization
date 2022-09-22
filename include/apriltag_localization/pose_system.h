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
  typedef float T;
  typedef Eigen::Matrix<T, 3, 1> Vector3t;
  typedef Eigen::Matrix<T, 4, 1> Vector4t;
  typedef Eigen::Matrix<T, 4, 4> Matrix4t;
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;
  typedef Eigen::Quaternion<T> Quaterniont;
  typedef Eigen::AngleAxis<T> AngleAxist;

  template <class T>
  Eigen::Matrix<T, 3, 3> skew_x(Eigen::Matrix<T, 3, 1>& w) const
  {
    Eigen::Matrix<T, 3, 3> w_x;
    w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
    return w_x;
  }
  template <class T>
  Eigen::Matrix<T, 4, 4> Omega(Eigen::Matrix<T, 3, 1> w) const
  {
    Eigen::Matrix<T, 4, 4> mat;
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
  VectorXt f(const VectorXt& state, const VectorXt& control) const
  {
    VectorXt next_state(6);
    // rk4
    // pre-compute things
    Vector3t v_hat1 = control.middleRows<3>(0);
    Vector3t w_hat1 = control.middleRows<3>(3);
    Vector3t v_hat2 = control.middleRows<3>(6);
    Vector3t w_hat2 = control.middleRows<3>(9);
    Vector3t v_a = (v_hat2 - v_hat1) / dt;
    Vector3t w_alpha = (w_hat2 - w_hat1) / dt;
    Vector3t v_hat = v_hat1;
    Vector3t w_hat = w_hat1;

    // y0 ======================
    Vector3t theta_0 = state.middleRows<3>(0);
    AngleAxist axis(theta_0.norm(), theta_0.normalized());
    Quaterniont q_0(axis);
    q_0.normalize();
    Vector3t p_0 = state.middleRows<3>(3);

    // q = [x, y, z, w]
    // k1 ================
    Vector4t dq_0(0, 0, 0, 1);
    Vector4t q0_dot = 0.5 * Omega(w_hat) * dq_0;
    Quaterniont qx = q_0 * Quaterniont(dq_0(3), dq_0(0), dq_0(1), dq_0(2));
    Vector3t p0_dot = qx.toRotationMatrix() * v_hat;

    Vector4t k1_q = q0_dot * dt;
    Vector3t k1_p = p0_dot * dt;

    // k2 ================
    w_hat += 0.5 * w_alpha * dt;
    v_hat += 0.5 * v_a * dt;

    Vector4t dq_1 = (dq_0 + 0.5 * k1_q).normalized();
    if (dq_1(3) < 0)
    {
      dq_1 *= -1;
    }
    Vector3t p_1 = p_0 + 0.5 * k1_p;

    Vector4t q1_dot = 0.5 * Omega(w_hat) * dq_1;
    qx = q_0 * Quaterniont(dq_1(3), dq_1(0), dq_1(1), dq_1(2));
    Vector3t p1_dot = qx.toRotationMatrix() * v_hat;

    Vector4t k2_q = q1_dot * dt;
    Vector3t k2_p = p1_dot * dt;

    // k3 ================
    Vector4t dq_2 = (dq_0 + 0.5 * k2_q).normalized();
    if (dq_2(3) < 0)
    {
      dq_2 *= -1;
    }
    Vector3t p_2 = p_0 + 0.5 * k2_p;

    Vector4t q2_dot = 0.5 * Omega(w_hat) * dq_2;
    qx = q_0 * Quaterniont(dq_2(3), dq_2(0), dq_2(1), dq_2(2));
    Vector3t p2_dot = qx.toRotationMatrix() * v_hat;

    Vector4t k3_q = q2_dot * dt;
    Vector3t k3_p = p2_dot * dt;

    // k4 ================
    w_hat += 0.5 * w_alpha * dt;
    v_hat += 0.5 * v_a * dt;

    Vector4t dq_3 = (dq_0 + k3_q).normalized();
    if (dq_3(3) < 0)
    {
      dq_3 *= -1;
    }
    Vector3t p_3 = p_0 + k3_p;

    Vector4t q3_dot = 0.5 * Omega(w_hat) * dq_3;
    qx = q_0 * Quaterniont(dq_2(3), dq_2(0), dq_2(1), dq_2(2));
    Vector3t p3_dot = qx.toRotationMatrix() * v_hat;

    Vector4t k4_q = q3_dot * dt;
    Vector3t k4_p = p3_dot * dt;

    // y+dt ==================
    Vector4t dq =
        (dq_0 + (1.0 / 6.0) * k1_q + (1.0 / 3.0) * k2_q + (1.0 / 3.0) * k3_q + (1.0 / 6.0) * k4_q).normalized();
    if (dq(3) < 0)
    {
      dq *= -1;
    }
    Quaterniont _dq, new_q;
    _dq.coeffs() = dq;
    new_q = q_0 * _dq;
    Vector3t new_p = p_0 + (1.0 / 6.0) * k1_p + (1.0 / 3.0) * k2_p + (1.0 / 3.0) * k3_p + (1.0 / 6.0) * k4_p;

    AngleAxist new_axis(new_q);
    Vector3t theta = new_axis.angle() * new_axis.axis();
    next_state.middleRows<3>(0) = theta;
    next_state.middleRows<3>(3) = new_p;
    return next_state;
  }

  // observation equation
  VectorXt h(const VectorXt& state) const
  {
    VectorXt observation(6);
    observation.middleRows(0, 3) = state.middleRows(0, 3);
    observation.middleRows(3, 6) = state.middleRows(3, 3);

    return observation;
  }
  double dt;
};
}  // namespace apriltag_localization
#endif  // APRILTAG_LOCALIZATION_POSESYSTEM_H_
